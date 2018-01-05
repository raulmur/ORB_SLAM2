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

#include "linear_solver_cholmod.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_factory.h"

#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_dogleg.h"

#include "g2o/stuff/macros.h"

//#define ADD_SCALAR_ORDERING
using namespace std;

namespace g2o {

  namespace
  {
    template<int p, int l, bool blockorder>
    std::unique_ptr<BlockSolverBase> AllocateSolver()
    {
      std::cerr << "# Using CHOLMOD poseDim " << p << " landMarkDim " << l << " blockordering " << blockorder << std::endl;
      auto linearSolver = g2o::make_unique<LinearSolverCholmod<typename BlockSolverPL<p, l>::PoseMatrixType>>();
      linearSolver->setBlockOrdering(blockorder);
      return g2o::make_unique<BlockSolverPL<p, l>>(std::move(linearSolver));
    }
  }

  static OptimizationAlgorithm* createSolver(const std::string& fullSolverName)
  {
    static const std::map<std::string, std::function<std::unique_ptr<BlockSolverBase>()>> solver_factories{
      { "var_cholmod", &AllocateSolver<-1, -1, true> },
      { "fix3_2_cholmod", &AllocateSolver<3, 2, true> },
      { "fix6_3_cholmod", &AllocateSolver<6, 3, true> },
      { "fix7_3_cholmod", &AllocateSolver<7, 3, true> },
#ifdef ADD_SCALAR_ORDERING
      { "fix3_2_cholmod_scalar", &AllocateSolver<3, 2, false> },
      { "fix6_3_cholmod_scalar", &AllocateSolver<6, 3, false> },
      { "fix7_3_cholmod_scalar", &AllocateSolver<7, 3, false> },
#endif
    };

    string solverName = fullSolverName.substr(3);
    auto solverf = solver_factories.find(solverName);
    if (solverf == solver_factories.end())
      return nullptr;

    string methodName = fullSolverName.substr(0, 2);

    if (methodName == "gn")
    {
      return new OptimizationAlgorithmGaussNewton(solverf->second());
    }
    else if (methodName == "lm")
    {
      return new OptimizationAlgorithmLevenberg(solverf->second());
    }
    else if (methodName == "dl")
    {
      return new OptimizationAlgorithmDogleg(solverf->second());
    }

    return nullptr;
  }

  class CholmodSolverCreator : public AbstractOptimizationAlgorithmCreator
  {
    public:
      CholmodSolverCreator(const OptimizationAlgorithmProperty& p) : AbstractOptimizationAlgorithmCreator(p) {}
      virtual OptimizationAlgorithm* construct()
      {
        return createSolver(property().name);
      }
  };

  G2O_REGISTER_OPTIMIZATION_LIBRARY(cholmod);

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_var_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_var_cholmod", "Gauss-Newton: Cholesky solver using CHOLMOD (variable blocksize)", "CHOLMOD", false, Eigen::Dynamic, Eigen::Dynamic)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix3_2_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix3_2_cholmod", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix6_3_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix6_3_cholmod", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix7_3_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix7_3_cholmod", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 7, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_var_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_var_cholmod", "Levenberg: Cholesky solver using CHOLMOD (variable blocksize)", "CHOLMOD", false, Eigen::Dynamic, Eigen::Dynamic)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix3_2_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix3_2_cholmod", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix6_3_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix6_3_cholmod", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix7_3_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix7_3_cholmod", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD", true, 7, 3)));

#ifdef ADD_SCALAR_ORDERING
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix3_2_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix3_2_cholmod_scalar", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix6_3_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix6_3_cholmod_scalar", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(gn_fix7_3_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("gn_fix7_3_cholmod_scalar", "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 7, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix3_2_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix3_2_cholmod_scalar", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 3, 2)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix6_3_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix6_3_cholmod_scalar", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 6, 3)));
  G2O_REGISTER_OPTIMIZATION_ALGORITHM(lm_fix7_3_cholmod_scalar, new CholmodSolverCreator(OptimizationAlgorithmProperty("lm_fix7_3_cholmod_scalar", "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar ordering)", "CHOLMOD", true, 7, 3)));
#endif

  G2O_REGISTER_OPTIMIZATION_ALGORITHM(dl_var_cholmod, new CholmodSolverCreator(OptimizationAlgorithmProperty("dl_var_cholmod", "Dogleg: Cholesky solver using CHOLMOD (variable blocksize)", "CHOLMOD", false, Eigen::Dynamic, Eigen::Dynamic)));
}
