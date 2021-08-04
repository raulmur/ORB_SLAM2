/* 
////////////////////////////////////////////////////////////////////////////////////
// 
//  Prototypes and definitions for the Levenberg - Marquardt minimization algorithm
//  Copyright (C) 2004  Manolis Lourakis (lourakis at ics forth gr)
//  Institute of Computer Science, Foundation for Research & Technology - Hellas
//  Heraklion, Crete, Greece.
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
////////////////////////////////////////////////////////////////////////////////////
*/

#ifndef _LEVMAR_H_
#define _LEVMAR_H_

/************************************* Start of configuration options *************************************/
/* Note that when compiling with CMake, this configuration section is automatically generated
 * based on the user's input, see levmar.h.in
 */

/* specifies whether to use LAPACK or not. Using LAPACK is strongly recommended */
#define HAVE_LAPACK

/* specifies whether the PLASMA parallel library for multicore CPUs is available */
/* #undef HAVE_PLASMA */
                      
/* to avoid the overhead of repeated mallocs(), routines in Axb.c can be instructed to
 * retain working memory between calls. Such a choice, however, renders these routines
 * non-reentrant and is not safe in a shared memory multiprocessing environment.
 * Bellow, an attempt is made to issue a warning if this option is turned on and OpenMP
 * is being used (note that this will work only if omp.h is included before levmar.h)
 */
#define LINSOLVERS_RETAIN_MEMORY
#if (defined(_OPENMP))
# ifdef LINSOLVERS_RETAIN_MEMORY
#  ifdef _MSC_VER
#  pragma message("LINSOLVERS_RETAIN_MEMORY is not safe in a multithreaded environment and should be turned off!")
#  else
#  warning LINSOLVERS_RETAIN_MEMORY is not safe in a multithreaded environment and should be turned off!
#  endif /* _MSC_VER */
# endif /* LINSOLVERS_RETAIN_MEMORY */
#endif /* _OPENMP */

/* specifies whether double precision routines will be compiled or not */
#define LM_DBL_PREC
/* specifies whether single precision routines will be compiled or not */
#define LM_SNGL_PREC

/****************** End of configuration options, no changes necessary beyond this point ******************/


#ifdef __cplusplus
extern "C" {
#endif

/* work arrays size for ?levmar_der and ?levmar_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_DER_WORKSZ(npar, nmeas) (2*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))
#define LM_DIF_WORKSZ(npar, nmeas) (4*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))

/* work arrays size for ?levmar_bc_der and ?levmar_bc_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BC_DER_WORKSZ(npar, nmeas) (2*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))
#define LM_BC_DIF_WORKSZ(npar, nmeas) LM_BC_DER_WORKSZ((npar), (nmeas)) /* LEVMAR_BC_DIF currently implemented using LEVMAR_BC_DER()! */

/* work arrays size for ?levmar_lec_der and ?levmar_lec_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_LEC_DER_WORKSZ(npar, nmeas, nconstr) LM_DER_WORKSZ((npar)-(nconstr), (nmeas))
#define LM_LEC_DIF_WORKSZ(npar, nmeas, nconstr) LM_DIF_WORKSZ((npar)-(nconstr), (nmeas))

/* work arrays size for ?levmar_blec_der and ?levmar_blec_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BLEC_DER_WORKSZ(npar, nmeas, nconstr) LM_LEC_DER_WORKSZ((npar), (nmeas)+(npar), (nconstr))
#define LM_BLEC_DIF_WORKSZ(npar, nmeas, nconstr) LM_LEC_DIF_WORKSZ((npar), (nmeas)+(npar), (nconstr))

/* work arrays size for ?levmar_bleic_der and ?levmar_bleic_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BLEIC_DER_WORKSZ(npar, nmeas, nconstr1, nconstr2) LM_BLEC_DER_WORKSZ((npar)+(nconstr2), (nmeas)+(nconstr2), (nconstr1)+(nconstr2))
#define LM_BLEIC_DIF_WORKSZ(npar, nmeas, nconstr1, nconstr2) LM_BLEC_DIF_WORKSZ((npar)+(nconstr2), (nmeas)+(nconstr2), (nconstr1)+(nconstr2))

#define LM_OPTS_SZ    	 5 /* max(4, 5) */
#define LM_INFO_SZ    	 10
#define LM_ERROR         -1
#define LM_INIT_MU    	 1E-03
#define LM_STOP_THRESH	 1E-17
#define LM_DIFF_DELTA    1E-06
#define LM_VERSION       "2.6 (November 2011)"

#ifdef LM_DBL_PREC
/* double precision LM, with & without Jacobian */
/* unconstrained minimization */
extern int dlevmar_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, int itmax, double *opts,
      double *info, double *work, double *covar, void *adata);

extern int dlevmar_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, int itmax, double *opts,
      double *info, double *work, double *covar, void *adata);

/* box-constrained minimization */
extern int dlevmar_bc_der(
       void (*func)(double *p, double *hx, int m, int n, void *adata),
       void (*jacf)(double *p, double *j, int m, int n, void *adata),  
       double *p, double *x, int m, int n, double *lb, double *ub, double *dscl,
       int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_bc_dif(
       void (*func)(double *p, double *hx, int m, int n, void *adata),
       double *p, double *x, int m, int n, double *lb, double *ub, double *dscl,
       int itmax, double *opts, double *info, double *work, double *covar, void *adata);

#ifdef HAVE_LAPACK
/* linear equation constrained minimization */
extern int dlevmar_lec_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *A, double *b, int k,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_lec_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *A, double *b, int k,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

/* box & linear equation constrained minimization */
extern int dlevmar_blec_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, double *A, double *b, int k, double *wghts,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_blec_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, double *A, double *b, int k, double *wghts,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

/* box, linear equations & inequalities constrained minimization */
extern int dlevmar_bleic_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub,
      double *A, double *b, int k1, double *C, double *d, int k2,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

extern int dlevmar_bleic_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, 
      double *A, double *b, int k1, double *C, double *d, int k2,
      int itmax, double *opts, double *info, double *work, double *covar, void *adata);

/* box & linear inequality constraints */
extern int dlevmar_blic_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, double *C, double *d, int k2,
      int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_blic_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *lb, double *ub, double *C, double *d, int k2,
      int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

/* linear equation & inequality constraints */
extern int dlevmar_leic_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *A, double *b, int k1, double *C, double *d, int k2,
      int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_leic_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *A, double *b, int k1, double *C, double *d, int k2,
      int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

/* linear inequality constraints */
extern int dlevmar_lic_der(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      void (*jacf)(double *p, double *j, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *C, double *d, int k2,
      int itmax, double opts[4], double info[LM_INFO_SZ], double *work, double *covar, void *adata);

extern int dlevmar_lic_dif(
      void (*func)(double *p, double *hx, int m, int n, void *adata),
      double *p, double *x, int m, int n, double *C, double *d, int k2,
      int itmax, double opts[5], double info[LM_INFO_SZ], double *work, double *covar, void *adata);
#endif /* HAVE_LAPACK */

#endif /* LM_DBL_PREC */


#ifdef LM_SNGL_PREC
/* single precision LM, with & without Jacobian */
/* unconstrained minimization */
extern int slevmar_der(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      void (*jacf)(float *p, float *j, int m, int n, void *adata),
      float *p, float *x, int m, int n, int itmax, float *opts,
      float *info, float *work, float *covar, void *adata);

extern int slevmar_dif(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      float *p, float *x, int m, int n, int itmax, float *opts,
      float *info, float *work, float *covar, void *adata);

/* box-constrained minimization */
extern int slevmar_bc_der(
       void (*func)(float *p, float *hx, int m, int n, void *adata),
       void (*jacf)(float *p, float *j, int m, int n, void *adata),  
       float *p, float *x, int m, int n, float *lb, float *ub, float *dscl,
       int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_bc_dif(
       void (*func)(float *p, float *hx, int m, int n, void *adata),
       float *p, float *x, int m, int n, float *lb, float *ub, float *dscl,
       int itmax, float *opts, float *info, float *work, float *covar, void *adata);

#ifdef HAVE_LAPACK
/* linear equation constrained minimization */
extern int slevmar_lec_der(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      void (*jacf)(float *p, float *j, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *A, float *b, int k,
      int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_lec_dif(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *A, float *b, int k,
      int itmax, float *opts, float *info, float *work, float *covar, void *adata);

/* box & linear equation constrained minimization */
extern int slevmar_blec_der(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      void (*jacf)(float *p, float *j, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *lb, float *ub, float *A, float *b, int k, float *wghts,
      int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_blec_dif(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *lb, float *ub, float *A, float *b, int k, float *wghts,
      int itmax, float *opts, float *info, float *work, float *covar, void *adata);

/* box, linear equations & inequalities constrained minimization */
extern int slevmar_bleic_der(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      void (*jacf)(float *p, float *j, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *lb, float *ub,
      float *A, float *b, int k1, float *C, float *d, int k2,
      int itmax, float *opts, float *info, float *work, float *covar, void *adata);

extern int slevmar_bleic_dif(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *lb, float *ub,
      float *A, float *b, int k1, float *C, float *d, int k2,
      int itmax, float *opts, float *info, float *work, float *covar, void *adata);

/* box & linear inequality constraints */
extern int slevmar_blic_der(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      void (*jacf)(float *p, float *j, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *lb, float *ub, float *C, float *d, int k2,
      int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_blic_dif(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *lb, float *ub, float *C, float *d, int k2,
      int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

/* linear equality & inequality constraints */
extern int slevmar_leic_der(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      void (*jacf)(float *p, float *j, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *A, float *b, int k1, float *C, float *d, int k2,
      int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_leic_dif(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *A, float *b, int k1, float *C, float *d, int k2,
      int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

/* linear inequality constraints */
extern int slevmar_lic_der(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      void (*jacf)(float *p, float *j, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *C, float *d, int k2,
      int itmax, float opts[4], float info[LM_INFO_SZ], float *work, float *covar, void *adata);

extern int slevmar_lic_dif(
      void (*func)(float *p, float *hx, int m, int n, void *adata),
      float *p, float *x, int m, int n, float *C, float *d, int k2,
      int itmax, float opts[5], float info[LM_INFO_SZ], float *work, float *covar, void *adata);
#endif /* HAVE_LAPACK */

#endif /* LM_SNGL_PREC */

/* linear system solvers */
#ifdef HAVE_LAPACK

#ifdef LM_DBL_PREC
extern int dAx_eq_b_QR(double *A, double *B, double *x, int m);
extern int dAx_eq_b_QRLS(double *A, double *B, double *x, int m, int n);
extern int dAx_eq_b_Chol(double *A, double *B, double *x, int m);
extern int dAx_eq_b_LU(double *A, double *B, double *x, int m);
extern int dAx_eq_b_SVD(double *A, double *B, double *x, int m);
extern int dAx_eq_b_BK(double *A, double *B, double *x, int m);
#endif /* LM_DBL_PREC */

#ifdef LM_SNGL_PREC
extern int sAx_eq_b_QR(float *A, float *B, float *x, int m);
extern int sAx_eq_b_QRLS(float *A, float *B, float *x, int m, int n);
extern int sAx_eq_b_Chol(float *A, float *B, float *x, int m);
extern int sAx_eq_b_LU(float *A, float *B, float *x, int m);
extern int sAx_eq_b_SVD(float *A, float *B, float *x, int m);
extern int sAx_eq_b_BK(float *A, float *B, float *x, int m);
#endif /* LM_SNGL_PREC */

#else /* no LAPACK */

#ifdef LM_DBL_PREC
extern int dAx_eq_b_LU_noLapack(double *A, double *B, double *x, int n);
#endif /* LM_DBL_PREC */

#ifdef LM_SNGL_PREC
extern int sAx_eq_b_LU_noLapack(float *A, float *B, float *x, int n);
#endif /* LM_SNGL_PREC */

#endif /* HAVE_LAPACK */

#ifdef HAVE_PLASMA
#ifdef LM_DBL_PREC
extern int dAx_eq_b_PLASMA_Chol(double *A, double *B, double *x, int m);
#endif
#ifdef LM_SNGL_PREC
extern int sAx_eq_b_PLASMA_Chol(float *A, float *B, float *x, int m);
#endif
extern void levmar_PLASMA_setnbcores(int cores);
#endif /* HAVE_PLASMA */

/* Jacobian verification, double & single precision */
#ifdef LM_DBL_PREC
extern void dlevmar_chkjac(
    void (*func)(double *p, double *hx, int m, int n, void *adata),
    void (*jacf)(double *p, double *j, int m, int n, void *adata),
    double *p, int m, int n, void *adata, double *err);
#endif /* LM_DBL_PREC */

#ifdef LM_SNGL_PREC
extern void slevmar_chkjac(
    void (*func)(float *p, float *hx, int m, int n, void *adata),
    void (*jacf)(float *p, float *j, int m, int n, void *adata),
    float *p, int m, int n, void *adata, float *err);
#endif /* LM_SNGL_PREC */

/* miscellaneous: standard deviation, coefficient of determination (R2),
 *                Pearson's correlation coefficient for best-fit parameters
 */
#ifdef LM_DBL_PREC
extern double dlevmar_stddev( double *covar, int m, int i);
extern double dlevmar_corcoef(double *covar, int m, int i, int j);
extern double dlevmar_R2(void (*func)(double *p, double *hx, int m, int n, void *adata), double *p, double *x, int m, int n, void *adata);

#endif /* LM_DBL_PREC */

#ifdef LM_SNGL_PREC
extern float slevmar_stddev( float *covar, int m, int i);
extern float slevmar_corcoef(float *covar, int m, int i, int j);
extern float slevmar_R2(void (*func)(float *p, float *hx, int m, int n, void *adata), float *p, float *x, int m, int n, void *adata);

extern void slevmar_locscale(
        void (*func)(float *p, float *hx, int m, int n, void *adata),
        float *p, float *x, int m, int n, void *adata,
        int howto, float locscl[2], float **residptr);

extern int slevmar_outlid(float *r, int n, float thresh, float ls[2], char *outlmap);

#endif /* LM_SNGL_PREC */

#ifdef __cplusplus
}
#endif

#endif /* _LEVMAR_H_ */
