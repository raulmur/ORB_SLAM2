      SUBROUTINE DROTM(N,DX,INCX,DY,INCY,DPARAM)
*     .. Scalar Arguments ..
      INTEGER INCX,INCY,N
*     ..
*     .. Array Arguments ..
      DOUBLE PRECISION DPARAM(5),DX(*),DY(*)
*     ..
*
*  Purpose
*  =======
*
*     APPLY THE MODIFIED GIVENS TRANSFORMATION, H, TO THE 2 BY N MATRIX
*
*     (DX**T) , WHERE **T INDICATES TRANSPOSE. THE ELEMENTS OF DX ARE IN
*     (DY**T)
*
*     DX(LX+I*INCX), I = 0 TO N-1, WHERE LX = 1 IF INCX .GE. 0, ELSE
*     LX = (-INCX)*N, AND SIMILARLY FOR SY USING LY AND INCY.
*     WITH DPARAM(1)=DFLAG, H HAS ONE OF THE FOLLOWING FORMS..
*
*     DFLAG=-1.D0     DFLAG=0.D0        DFLAG=1.D0     DFLAG=-2.D0
*
*       (DH11  DH12)    (1.D0  DH12)    (DH11  1.D0)    (1.D0  0.D0)
*     H=(          )    (          )    (          )    (          )
*       (DH21  DH22),   (DH21  1.D0),   (-1.D0 DH22),   (0.D0  1.D0).
*     SEE DROTMG FOR A DESCRIPTION OF DATA STORAGE IN DPARAM.
*
*  Arguments
*  =========
*
*  N      (input) INTEGER
*         number of elements in input vector(s)
*
*  DX     (input/output) DOUBLE PRECISION array, dimension N
*         double precision vector with N elements
*
*  INCX   (input) INTEGER
*         storage spacing between elements of DX
*
*  DY     (input/output) DOUBLE PRECISION array, dimension N
*         double precision vector with N elements
*
*  INCY   (input) INTEGER
*         storage spacing between elements of DY
*
*  DPARAM (input/output)  DOUBLE PRECISION array, dimension 5 
*     DPARAM(1)=DFLAG
*     DPARAM(2)=DH11
*     DPARAM(3)=DH21
*     DPARAM(4)=DH12
*     DPARAM(5)=DH22
*
*  =====================================================================
*
*     .. Local Scalars ..
      DOUBLE PRECISION DFLAG,DH11,DH12,DH21,DH22,TWO,W,Z,ZERO
      INTEGER I,KX,KY,NSTEPS
*     ..
*     .. Data statements ..
      DATA ZERO,TWO/0.D0,2.D0/
*     ..
*
      DFLAG = DPARAM(1)
      IF (N.LE.0 .OR. (DFLAG+TWO.EQ.ZERO)) GO TO 140
      IF (.NOT. (INCX.EQ.INCY.AND.INCX.GT.0)) GO TO 70
*
      NSTEPS = N*INCX
      IF (DFLAG) 50,10,30
   10 CONTINUE
      DH12 = DPARAM(4)
      DH21 = DPARAM(3)
      DO 20 I = 1,NSTEPS,INCX
          W = DX(I)
          Z = DY(I)
          DX(I) = W + Z*DH12
          DY(I) = W*DH21 + Z
   20 CONTINUE
      GO TO 140
   30 CONTINUE
      DH11 = DPARAM(2)
      DH22 = DPARAM(5)
      DO 40 I = 1,NSTEPS,INCX
          W = DX(I)
          Z = DY(I)
          DX(I) = W*DH11 + Z
          DY(I) = -W + DH22*Z
   40 CONTINUE
      GO TO 140
   50 CONTINUE
      DH11 = DPARAM(2)
      DH12 = DPARAM(4)
      DH21 = DPARAM(3)
      DH22 = DPARAM(5)
      DO 60 I = 1,NSTEPS,INCX
          W = DX(I)
          Z = DY(I)
          DX(I) = W*DH11 + Z*DH12
          DY(I) = W*DH21 + Z*DH22
   60 CONTINUE
      GO TO 140
   70 CONTINUE
      KX = 1
      KY = 1
      IF (INCX.LT.0) KX = 1 + (1-N)*INCX
      IF (INCY.LT.0) KY = 1 + (1-N)*INCY
*
      IF (DFLAG) 120,80,100
   80 CONTINUE
      DH12 = DPARAM(4)
      DH21 = DPARAM(3)
      DO 90 I = 1,N
          W = DX(KX)
          Z = DY(KY)
          DX(KX) = W + Z*DH12
          DY(KY) = W*DH21 + Z
          KX = KX + INCX
          KY = KY + INCY
   90 CONTINUE
      GO TO 140
  100 CONTINUE
      DH11 = DPARAM(2)
      DH22 = DPARAM(5)
      DO 110 I = 1,N
          W = DX(KX)
          Z = DY(KY)
          DX(KX) = W*DH11 + Z
          DY(KY) = -W + DH22*Z
          KX = KX + INCX
          KY = KY + INCY
  110 CONTINUE
      GO TO 140
  120 CONTINUE
      DH11 = DPARAM(2)
      DH12 = DPARAM(4)
      DH21 = DPARAM(3)
      DH22 = DPARAM(5)
      DO 130 I = 1,N
          W = DX(KX)
          Z = DY(KY)
          DX(KX) = W*DH11 + Z*DH12
          DY(KY) = W*DH21 + Z*DH22
          KX = KX + INCX
          KY = KY + INCY
  130 CONTINUE
  140 CONTINUE
      RETURN
      END
