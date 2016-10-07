      SUBROUTINE SROTM(N,SX,INCX,SY,INCY,SPARAM)
*     .. Scalar Arguments ..
      INTEGER INCX,INCY,N
*     ..
*     .. Array Arguments ..
      REAL SPARAM(5),SX(*),SY(*)
*     ..
*
*  Purpose
*  =======
*
*     APPLY THE MODIFIED GIVENS TRANSFORMATION, H, TO THE 2 BY N MATRIX
*
*     (SX**T) , WHERE **T INDICATES TRANSPOSE. THE ELEMENTS OF SX ARE IN
*     (DX**T)
*
*     SX(LX+I*INCX), I = 0 TO N-1, WHERE LX = 1 IF INCX .GE. 0, ELSE
*     LX = (-INCX)*N, AND SIMILARLY FOR SY USING USING LY AND INCY.
*     WITH SPARAM(1)=SFLAG, H HAS ONE OF THE FOLLOWING FORMS..
*
*     SFLAG=-1.E0     SFLAG=0.E0        SFLAG=1.E0     SFLAG=-2.E0
*
*       (SH11  SH12)    (1.E0  SH12)    (SH11  1.E0)    (1.E0  0.E0)
*     H=(          )    (          )    (          )    (          )
*       (SH21  SH22),   (SH21  1.E0),   (-1.E0 SH22),   (0.E0  1.E0).
*     SEE  SROTMG FOR A DESCRIPTION OF DATA STORAGE IN SPARAM.
*
*
*  Arguments
*  =========
*
*  N      (input) INTEGER
*         number of elements in input vector(s)
*
*  SX     (input/output) REAL array, dimension N
*         double precision vector with N elements
*
*  INCX   (input) INTEGER
*         storage spacing between elements of SX
*
*  SY     (input/output) REAL array, dimension N
*         double precision vector with N elements
*
*  INCY   (input) INTEGER
*         storage spacing between elements of SY
*
*  SPARAM (input/output)  REAL array, dimension 5
*     SPARAM(1)=SFLAG
*     SPARAM(2)=SH11
*     SPARAM(3)=SH21
*     SPARAM(4)=SH12
*     SPARAM(5)=SH22
*
*  =====================================================================
*
*     .. Local Scalars ..
      REAL SFLAG,SH11,SH12,SH21,SH22,TWO,W,Z,ZERO
      INTEGER I,KX,KY,NSTEPS
*     ..
*     .. Data statements ..
      DATA ZERO,TWO/0.E0,2.E0/
*     ..
*
      SFLAG = SPARAM(1)
      IF (N.LE.0 .OR. (SFLAG+TWO.EQ.ZERO)) GO TO 140
      IF (.NOT. (INCX.EQ.INCY.AND.INCX.GT.0)) GO TO 70
*
      NSTEPS = N*INCX
      IF (SFLAG) 50,10,30
   10 CONTINUE
      SH12 = SPARAM(4)
      SH21 = SPARAM(3)
      DO 20 I = 1,NSTEPS,INCX
          W = SX(I)
          Z = SY(I)
          SX(I) = W + Z*SH12
          SY(I) = W*SH21 + Z
   20 CONTINUE
      GO TO 140
   30 CONTINUE
      SH11 = SPARAM(2)
      SH22 = SPARAM(5)
      DO 40 I = 1,NSTEPS,INCX
          W = SX(I)
          Z = SY(I)
          SX(I) = W*SH11 + Z
          SY(I) = -W + SH22*Z
   40 CONTINUE
      GO TO 140
   50 CONTINUE
      SH11 = SPARAM(2)
      SH12 = SPARAM(4)
      SH21 = SPARAM(3)
      SH22 = SPARAM(5)
      DO 60 I = 1,NSTEPS,INCX
          W = SX(I)
          Z = SY(I)
          SX(I) = W*SH11 + Z*SH12
          SY(I) = W*SH21 + Z*SH22
   60 CONTINUE
      GO TO 140
   70 CONTINUE
      KX = 1
      KY = 1
      IF (INCX.LT.0) KX = 1 + (1-N)*INCX
      IF (INCY.LT.0) KY = 1 + (1-N)*INCY
*
      IF (SFLAG) 120,80,100
   80 CONTINUE
      SH12 = SPARAM(4)
      SH21 = SPARAM(3)
      DO 90 I = 1,N
          W = SX(KX)
          Z = SY(KY)
          SX(KX) = W + Z*SH12
          SY(KY) = W*SH21 + Z
          KX = KX + INCX
          KY = KY + INCY
   90 CONTINUE
      GO TO 140
  100 CONTINUE
      SH11 = SPARAM(2)
      SH22 = SPARAM(5)
      DO 110 I = 1,N
          W = SX(KX)
          Z = SY(KY)
          SX(KX) = W*SH11 + Z
          SY(KY) = -W + SH22*Z
          KX = KX + INCX
          KY = KY + INCY
  110 CONTINUE
      GO TO 140
  120 CONTINUE
      SH11 = SPARAM(2)
      SH12 = SPARAM(4)
      SH21 = SPARAM(3)
      SH22 = SPARAM(5)
      DO 130 I = 1,N
          W = SX(KX)
          Z = SY(KY)
          SX(KX) = W*SH11 + Z*SH12
          SY(KY) = W*SH21 + Z*SH22
          KX = KX + INCX
          KY = KY + INCY
  130 CONTINUE
  140 CONTINUE
      RETURN
      END
