// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_LINEAR_SOLVER_CHOLMOD
#define G2O_LINEAR_SOLVER_CHOLMOD

#include "g2o/core/linear_solver.h"
#include "g2o/core/marginal_covariance_cholesky.h"
#include "g2o/core/batch_stats.h"
#include "g2o/stuff/timeutil.h"
#include "g2o/stuff/sparse_helper.h"

#include <cholmod.h>

namespace g2o {

/**
 * \brief Our extension of the CHOLMOD matrix struct
 */
struct CholmodExt : public cholmod_sparse
{
  CholmodExt()
  {
    nzmax = 0;
    nrow  = 0;
    ncol  = 0;
    p     = 0;
    i     = 0;
    nz    = 0;
    x     = 0;
    z     = 0;
    stype = 1; // upper triangular block only
    itype = CHOLMOD_INT;
    xtype = CHOLMOD_REAL;
    dtype = CHOLMOD_DOUBLE;
    sorted = 1;
    packed = 1;
    columnsAllocated = 0;
  }
  ~CholmodExt()
  {
    delete[] (int*)p; p = 0;
    delete[] (double*)x; x = 0;
    delete[] (int*)i; i = 0;
  }
  size_t columnsAllocated;
};

/**
 * \brief basic solver for Ax = b which has to reimplemented for different linear algebra libraries
 */
template <typename MatrixType>
class LinearSolverCholmod : public LinearSolverCCS<MatrixType>
{
  public:
    LinearSolverCholmod() :
      LinearSolverCCS<MatrixType>()
    {
      _writeDebug = true;
      _blockOrdering = false;
      _cholmodSparse = new CholmodExt();
      _cholmodFactor = 0;
      cholmod_start(&_cholmodCommon);

      // setup ordering strategy
      _cholmodCommon.nmethods = 1 ;
      _cholmodCommon.method[0].ordering = CHOLMOD_AMD; //CHOLMOD_COLAMD
      //_cholmodCommon.postorder = 0;

      _cholmodCommon.supernodal = CHOLMOD_AUTO; //CHOLMOD_SUPERNODAL; //CHOLMOD_SIMPLICIAL;
    }

    virtual ~LinearSolverCholmod()
    {
      delete _cholmodSparse;
      if (_cholmodFactor != 0) {
        cholmod_free_factor(&_cholmodFactor, &_cholmodCommon);
        _cholmodFactor = 0;
      }
      cholmod_finish(&_cholmodCommon);
    }

    virtual bool init()
    {
      if (_cholmodFactor != 0) {
        cholmod_free_factor(&_cholmodFactor, &_cholmodCommon);
        _cholmodFactor = 0;
      }
      return true;
    }

    bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
    {
      //cerr << __PRETTY_FUNCTION__ << " using cholmod" << endl;
      fillCholmodExt(A, _cholmodFactor != 0); // _cholmodFactor used as bool, if not existing will copy the whole structure, otherwise only the values

      if (_cholmodFactor == 0) {
        computeSymbolicDecomposition(A);
        assert(_cholmodFactor != 0 && "Symbolic cholesky failed");
      }
      double t=get_monotonic_time();

      // setting up b for calling cholmod
      cholmod_dense bcholmod;
      bcholmod.nrow  = bcholmod.d = _cholmodSparse->nrow;
      bcholmod.ncol  = 1;
      bcholmod.x     = b;
      bcholmod.xtype = CHOLMOD_REAL;
      bcholmod.dtype = CHOLMOD_DOUBLE;

      cholmod_factorize(_cholmodSparse, _cholmodFactor, &_cholmodCommon);
      if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF) {
        if (_writeDebug) {
          std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by Octave)" << std::endl;
          saveMatrix("debug.txt");
        }
        return false;
      }

      cholmod_dense* xcholmod = cholmod_solve(CHOLMOD_A, _cholmodFactor, &bcholmod, &_cholmodCommon);
      memcpy(x, xcholmod->x, sizeof(double) * bcholmod.nrow); // copy back to our array
      cholmod_free_dense(&xcholmod, &_cholmodCommon);

      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats){
        globalStats->timeNumericDecomposition = get_monotonic_time() - t;
        globalStats->choleskyNNZ = static_cast<size_t>(_cholmodCommon.method[0].lnz);
      }

      return true;
    }

    bool solveBlocks(double**& blocks, const SparseBlockMatrix<MatrixType>& A)
    {
      //cerr << __PRETTY_FUNCTION__ << " using cholmod" << endl;
      fillCholmodExt(A, _cholmodFactor != 0); // _cholmodFactor used as bool, if not existing will copy the whole structure, otherwise only the values

      if (_cholmodFactor == 0) {
        computeSymbolicDecomposition(A);
        assert(_cholmodFactor && "Symbolic cholesky failed");
      }

      if (! blocks){
        blocks=new double*[A.rows()];
        double **block=blocks;
        for (size_t i = 0; i < A.rowBlockIndices().size(); ++i){
          int dim=A.rowsOfBlock(i)*A.colsOfBlock(i);
          *block = new double [dim];
          block++;
        }
      }

      cholmod_factorize(_cholmodSparse, _cholmodFactor, &_cholmodCommon);
      if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF)
        return false;

      // convert the factorization to LL, simplical, packed, monotonic
      int change_status = cholmod_change_factor(CHOLMOD_REAL, 1, 0, 1, 1, _cholmodFactor, &_cholmodCommon);
      if (! change_status) {
        return false;
      }
      assert(_cholmodFactor->is_ll && !_cholmodFactor->is_super && _cholmodFactor->is_monotonic && "Cholesky factor has wrong format");

      // invert the permutation
      int* p = (int*)_cholmodFactor->Perm;
      VectorXI pinv; pinv.resize(_cholmodSparse->ncol);
      for (size_t i = 0; i < _cholmodSparse->ncol; ++i)
        pinv(p[i]) = i;

      // compute the marginal covariance
      MarginalCovarianceCholesky mcc;
      mcc.setCholeskyFactor(_cholmodSparse->ncol, (int*)_cholmodFactor->p, (int*)_cholmodFactor->i,
          (double*)_cholmodFactor->x, pinv.data());
      mcc.computeCovariance(blocks, A.rowBlockIndices());

      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats) {
        globalStats->choleskyNNZ = static_cast<size_t>(_cholmodCommon.method[_cholmodCommon.selected].lnz);
      }

      return true;
    }

    virtual bool solvePattern(SparseBlockMatrix<MatrixX>& spinv, const std::vector<std::pair<int, int> >& blockIndices, const SparseBlockMatrix<MatrixType>& A)
    {
      //cerr << __PRETTY_FUNCTION__ << " using cholmod" << endl;
      fillCholmodExt(A, _cholmodFactor != 0); // _cholmodFactor used as bool, if not existing will copy the whole structure, otherwise only the values

      if (_cholmodFactor == 0) {
        computeSymbolicDecomposition(A);
        assert(_cholmodFactor && "Symbolic cholesky failed");
      }

      cholmod_factorize(_cholmodSparse, _cholmodFactor, &_cholmodCommon);
      if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF)
        return false;

      // convert the factorization to LL, simplical, packed, monotonic
      int change_status = cholmod_change_factor(CHOLMOD_REAL, 1, 0, 1, 1, _cholmodFactor, &_cholmodCommon);
      if (! change_status) {
        return false;
      }
      assert(_cholmodFactor->is_ll && !_cholmodFactor->is_super && _cholmodFactor->is_monotonic && "Cholesky factor has wrong format");

      // invert the permutation
      int* p = (int*)_cholmodFactor->Perm;
      VectorXI pinv; pinv.resize(_cholmodSparse->ncol);
      for (size_t i = 0; i < _cholmodSparse->ncol; ++i)
        pinv(p[i]) = i;

      // compute the marginal covariance
      MarginalCovarianceCholesky mcc;
      mcc.setCholeskyFactor(_cholmodSparse->ncol, (int*)_cholmodFactor->p, (int*)_cholmodFactor->i,
          (double*)_cholmodFactor->x, pinv.data());
      mcc.computeCovariance(spinv, A.rowBlockIndices(), blockIndices);

      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats) {
        globalStats->choleskyNNZ = static_cast<size_t>(_cholmodCommon.method[_cholmodCommon.selected].lnz);
      }

      return true;
    }

    //! do the AMD ordering on the blocks or on the scalar matrix
    bool blockOrdering() const { return _blockOrdering;}
    void setBlockOrdering(bool blockOrdering) { _blockOrdering = blockOrdering;}

    //! write a debug dump of the system matrix if it is not SPD in solve
    virtual bool writeDebug() const { return _writeDebug;}
    virtual void setWriteDebug(bool b) { _writeDebug = b;}

    virtual bool saveMatrix(const std::string& fileName) {
      writeCCSMatrix(fileName, _cholmodSparse->nrow, _cholmodSparse->ncol, (int*)_cholmodSparse->p, (int*)_cholmodSparse->i, (double*)_cholmodSparse->x, true);
      return true;
    }

  protected:
    // temp used for cholesky with cholmod
    cholmod_common _cholmodCommon;
    CholmodExt* _cholmodSparse;
    cholmod_factor* _cholmodFactor;
    bool _blockOrdering;
    MatrixStructure _matrixStructure;
    VectorXI _scalarPermutation, _blockPermutation;
    bool _writeDebug;

    void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A)
    {
      double t = get_monotonic_time();
      if (! _blockOrdering) {
        // setup ordering strategy
        _cholmodCommon.nmethods = 1;
        _cholmodCommon.method[0].ordering = CHOLMOD_AMD; //CHOLMOD_COLAMD
        _cholmodFactor = cholmod_analyze(_cholmodSparse, &_cholmodCommon); // symbolic factorization
      } else {

        A.fillBlockStructure(_matrixStructure);

        // get the ordering for the block matrix
        if (_blockPermutation.size() == 0)
          _blockPermutation.resize(_matrixStructure.n);
        if (_blockPermutation.size() < _matrixStructure.n) // double space if resizing
          _blockPermutation.resize(2*_matrixStructure.n);
 
        // prepare AMD call via CHOLMOD
        cholmod_sparse auxCholmodSparse;
        auxCholmodSparse.nzmax = _matrixStructure.nzMax();
        auxCholmodSparse.nrow = auxCholmodSparse.ncol = _matrixStructure.n;
        auxCholmodSparse.p = _matrixStructure.Ap;
        auxCholmodSparse.i = _matrixStructure.Aii;
        auxCholmodSparse.nz = 0;
        auxCholmodSparse.x = 0;
        auxCholmodSparse.z = 0;
        auxCholmodSparse.stype = 1;
        auxCholmodSparse.xtype = CHOLMOD_PATTERN;
        auxCholmodSparse.itype = CHOLMOD_INT;
        auxCholmodSparse.dtype = CHOLMOD_DOUBLE;
        auxCholmodSparse.sorted = 1;
        auxCholmodSparse.packed = 1;
        int amdStatus = cholmod_amd(&auxCholmodSparse, NULL, 0, _blockPermutation.data(), &_cholmodCommon);
        if (! amdStatus) {
          return;
        }

        // blow up the permutation to the scalar matrix
        if (_scalarPermutation.size() == 0)
          _scalarPermutation.resize(_cholmodSparse->ncol);
        if (_scalarPermutation.size() < (int)_cholmodSparse->ncol)
          _scalarPermutation.resize(2*_cholmodSparse->ncol);
        size_t scalarIdx = 0;
        for (int i = 0; i < _matrixStructure.n; ++i) {
          const int& p = _blockPermutation(i);
          int base  = A.colBaseOfBlock(p);
          int nCols = A.colsOfBlock(p);
          for (int j = 0; j < nCols; ++j)
            _scalarPermutation(scalarIdx++) = base++;
        }
        assert(scalarIdx == _cholmodSparse->ncol);

        // apply the ordering
        _cholmodCommon.nmethods = 1 ;
        _cholmodCommon.method[0].ordering = CHOLMOD_GIVEN;
        _cholmodFactor = cholmod_analyze_p(_cholmodSparse, _scalarPermutation.data(), NULL, 0, &_cholmodCommon);

      }
      G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
      if (globalStats)
        globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;

      //const int& bestIdx = _cholmodCommon.selected;
      //cerr << "# Number of nonzeros in L: " << (int)_cholmodCommon.method[bestIdx].lnz << " by "
        //<< (_cholmodCommon.method[bestIdx].ordering == CHOLMOD_GIVEN ? "block" : "scalar") << " AMD ordering " << endl;

    }

    void fillCholmodExt(const SparseBlockMatrix<MatrixType>& A, bool onlyValues)
    {
      if (! onlyValues)
        this->initMatrixStructure(A);
      size_t m = A.rows();
      size_t n = A.cols();
      assert(m > 0 && n > 0 && "Hessian has 0 rows/cols");

      if (_cholmodSparse->columnsAllocated < n) {
        //std::cerr << __PRETTY_FUNCTION__ << ": reallocating columns" << std::endl;
        _cholmodSparse->columnsAllocated = _cholmodSparse->columnsAllocated == 0 ? n : 2 * n; // pre-allocate more space if re-allocating
        delete[] (int*)_cholmodSparse->p;
        _cholmodSparse->p = new int[_cholmodSparse->columnsAllocated+1];
      }
      if (! onlyValues) {
        size_t nzmax = A.nonZeros();
        if (_cholmodSparse->nzmax < nzmax) {
          //std::cerr << __PRETTY_FUNCTION__ << ": reallocating row + values" << std::endl;
          _cholmodSparse->nzmax = _cholmodSparse->nzmax == 0 ? nzmax : 2 * nzmax; // pre-allocate more space if re-allocating
          delete[] (double*)_cholmodSparse->x;
          delete[] (int*)_cholmodSparse->i;
          _cholmodSparse->i = new int[_cholmodSparse->nzmax];
          _cholmodSparse->x = new double[_cholmodSparse->nzmax];
        }
      }
      _cholmodSparse->ncol = n;
      _cholmodSparse->nrow = m;

      if (onlyValues)
        this->_ccsMatrix->fillCCS((double*)_cholmodSparse->x, true);
      else
        this->_ccsMatrix->fillCCS((int*)_cholmodSparse->p, (int*)_cholmodSparse->i, (double*)_cholmodSparse->x, true);
    }

};

} // end namespace

#endif
