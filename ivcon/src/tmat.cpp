#include "tmat.h"
#include <math.h>
#include <string.h>

const float PI=3.141592653589793238462643f;

const float DEG_TO_RAD=( PI / 180.0f );
const float RAD_TO_DEG=( 180.0f / PI );

void tmat_init ( float a[4][4] )

/*********************************************************************/

/*
Purpose:

TMAT_INIT initializes the geometric transformation matrix.

Definition:

The geometric transformation matrix can be thought of as a 4 by 4
matrix "A" having components:

r11 r12 r13 t1
r21 r22 r23 t2
r31 r32 r33 t3
0   0   0  1

This matrix encodes the rotations, scalings and translations that
are applied to graphical objects.

A point P = (x,y,z) is rewritten in "homogeneous coordinates" as 
PH = (x,y,z,1).  Then to apply the transformations encoded in A to 
the point P, we simply compute A * PH.

Individual transformations, such as a scaling, can be represented
by simple versions of the transformation matrix.  If the matrix
A represents the current set of transformations, and we wish to 
apply a new transformation B, { the original points are
transformed twice:  B * ( A * PH ).  The new transformation B can
be combined with the original one A, to give a single matrix C that
encodes both transformations: C = B * A.

Modified:

19 October 1998

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the geometric transformation matrix.
*/
{
	int i;
	int j;
	
	for ( i = 0; i < 4; ++i ) {
		for ( j = 0; j < 4; ++j ) {
			if ( i == j ) {
				a[i][j] = 1.0;
			}
			else {
				a[i][j] = 0.0;
			}
		}
	}
	return;
}
/*********************************************************************/

void tmat_mxm ( float a[4][4], float b[4][4], float c[4][4] )

/*********************************************************************/

/*
Purpose:

TMAT_MXM multiplies two geometric transformation matrices.

Note:

The product is accumulated in a temporary array, and { assigned
to the result.  Therefore, it is legal for any two, or all three,
of the arguments to share memory.

Modified:

19 October 1998

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the first geometric transformation matrix.

Input, float B[4][4], the second geometric transformation matrix.

Output, float C[4][4], the product A * B.
*/
{
	float d[4][4];
	int i;
	int j;
	int k;
	
	for ( i = 0; i < 4; ++i ) {
		for ( k = 0; k < 4; ++k ) {
			d[i][k] = 0.0;
			for ( j = 0; j < 4; ++j ) {
				d[i][k] = d[i][k] + a[i][j] * b[j][k];
			}
		}
	}
	
	for ( i = 0; i < 4; ++i ) {
		for ( j = 0; j < 4; ++j ) {
			c[i][j] = d[i][j];
		}
	}
	return;
}
/*********************************************************************/

void tmat_mxp ( float a[4][4], float x[4], float y[4] )

/*********************************************************************/

/*
Purpose:

TMAT_MXP multiplies a geometric transformation matrix times a point.

Modified:

19 October 1998

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the geometric transformation matrix.

Input, float X[4], the point to be multiplied.  The fourth component
of X is implicitly assigned the value of 1.

Output, float Y[4], the result of A*X.  The product is accumulated in 
a temporary vector, and { assigned to the result.  Therefore, it 
is legal for X and Y to share memory.
*/
{
	int i;
	int j;
	float z[4];
	
	for ( i = 0; i < 3; ++i ) {
		z[i] = a[i][3];
		for ( j = 0; j < 3; ++j ) {
			z[i] = z[i] + a[i][j] * x[j];
		}
	}
	
	for ( i = 0; i < 3; ++i ) {
		y[i] = z[i];
	}
	return;
}
/*********************************************************************/

void tmat_mxp2 ( float a[4][4], float x[][3], float y[][3], int n )

/*********************************************************************/

/*
Purpose:

TMAT_MXP2 multiplies a geometric transformation matrix times N points.

Modified:

20 October 1998

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the geometric transformation matrix.

Input, float X[N][3], the points to be multiplied.  

Output, float Y[N][3], the transformed points.  Each product is 
accumulated in a temporary vector, and { assigned to the
result.  Therefore, it is legal for X and Y to share memory.

*/
{
	int i;
	int j;
	int k;
	float z[4];
	
	for ( k = 0; k < n; ++k ) {
		
		for ( i = 0; i < 3; ++i ) {
			z[i] = a[i][3];
			for ( j = 0; j < 3; ++j ) {
				z[i] = z[i] + a[i][j] * x[k][j];
			}
		}
		
		for ( i = 0; i < 3; ++i ) {
			y[k][i] = z[i];
		}
		
	}
	return;
}
/*********************************************************************/

void tmat_mxv ( float a[4][4], float x[4], float y[4] )

/*********************************************************************/

/*
Purpose:

TMAT_MXV multiplies a geometric transformation matrix times a vector.

Modified:

12 August 1999

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the geometric transformation matrix.

Input, float X[3], the vector to be multiplied.  The fourth component
of X is implicitly assigned the value of 1.

Output, float Y[3], the result of A*X.  The product is accumulated in 
a temporary vector, and assigned to the result.  Therefore, it 
is legal for X and Y to share memory.
*/
{
	int i;
	int j;
	float z[4];
	
	for ( i = 0; i < 3; ++i ) {
		z[i] = 0.0;
		for ( j = 0; j < 3; ++j ) {
			z[i] = z[i] + a[i][j] * x[j];
		}
		z[i] = z[i] + a[i][3];
	}
	
	for ( i = 0; i < 3; ++i ) {
		y[i] = z[i];
	}
	return;
}
/*********************************************************************/

bool tmat_rot_axis ( float a[4][4], float b[4][4], float angle, 
					char axis )
					
/*********************************************************************/

/*
Purpose:

TMAT_ROT_AXIS applies an axis rotation to the geometric transformation matrix.

Modified:

19 April 1999

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the current geometric transformation matrix.

Output, float B[4][4], the modified geometric transformation matrix.
A and B may share the same memory.

Input, float ANGLE, the angle, in degrees, of the rotation.

Input, character AXIS, is 'X', 'Y' or 'Z', specifying the coordinate
axis about which the rotation occurs.
*/
{
	float c[4][4];
	float d[4][4];
	int i;
	int j;
	float theta;
	
	theta = angle * DEG_TO_RAD;
	
	tmat_init ( c );
	
	if ( axis == 'X' || axis == 'x' ) {
		c[1][1] =   cos ( theta );
		c[1][2] = - sin ( theta );
		c[2][1] =   sin ( theta );
		c[2][2] =   cos ( theta );
	}
	else if ( axis == 'Y' || axis == 'y' ) {
		c[0][0] =   cos ( theta );
		c[0][2] =   sin ( theta );
		c[2][0] = - sin ( theta );
		c[2][2] =   cos ( theta );
	}
	else if ( axis == 'Z' || axis == 'z' ) {
		c[0][0] =   cos ( theta );
		c[0][1] = - sin ( theta );
		c[1][0] =   sin ( theta );
		c[1][1] =   cos ( theta );
	}
	else {
//		printf ( "\n" );
//		printf ( "TMAT_ROT_AXIS - Fatal error!\n" );
//		printf ( "  Illegal rotation axis: %c.\n", axis );
//		printf ( "  Legal choices are 'X', 'Y', or 'Z'.\n" );
		return false;
	}
	
	tmat_mxm ( c, a, d );
	
	for ( i = 0; i < 4; ++i ) {
		for ( j = 0; j < 4; ++j ) {
			b[i][j] = d[i][j];
		}
	}
	return true;
}
/*********************************************************************/

void tmat_rot_vector ( float a[4][4], float b[4][4], float angle, 
					  float v1, float v2, float v3 )
					  
/*********************************************************************/

/*
Purpose:

TMAT_ROT_VECTOR applies a rotation about a vector to the geometric transformation matrix.

Modified:

27 July 1999

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the current geometric transformation matrix.

Output, float B[4][4], the modified geometric transformation matrix.
A and B may share the same memory.

Input, float ANGLE, the angle, in degrees, of the rotation.

Input, float V1, V2, V3, the X, Y and Z coordinates of a (nonzero)
point defining a vector from the origin.  The rotation will occur
about this axis.
*/
{
	float c[4][4];
	float ca;
	float d[4][4];
	int i;
	int j;
	float sa;
	float theta;
	
	if ( v1 * v1 + v2 * v2 + v3 * v3 == 0.0 ) {
		return;
	}
	
	theta = angle * DEG_TO_RAD;
	
	tmat_init ( c );
	
	ca = cos ( theta );
	sa = sin ( theta );
	
	c[0][0] =                v1 * v1 + ca * ( 1.0 - v1 * v1 );
	c[0][1] = ( 1.0 - ca ) * v1 * v2 - sa * v3;
	c[0][2] = ( 1.0 - ca ) * v1 * v3 + sa * v2;
	
	c[1][0] = ( 1.0 - ca ) * v2 * v1 + sa * v3;
	c[1][1] =                v2 * v2 + ca * ( 1.0 - v2 * v2 );
	c[1][2] = ( 1.0 - ca ) * v2 * v3 - sa * v1;
	
	c[2][0] = ( 1.0 - ca ) * v3 * v1 - sa * v2;
	c[2][1] = ( 1.0 - ca ) * v3 * v2 + sa * v1;
	c[2][2] =                v3 * v3 + ca * ( 1.0 - v3 * v3 );
	
	tmat_mxm ( c, a, d );
	
	for ( i = 0; i < 4; ++i ) {
		for ( j = 0; j < 4; ++j ) {
			b[i][j] = d[i][j];
		}
	}
	return;
}
/*********************************************************************/

void tmat_scale ( float a[4][4], float b[4][4], float sx, float sy, 
				 float sz )
				 
/*********************************************************************/

/*
Purpose:

TMAT_SCALE applies a scaling to the geometric transformation matrix.

Modified:

19 October 1998

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the current geometric transformation matrix.

Output, float B[4][4], the modified geometric transformation matrix.
A and B may share the same memory.

Input, float SX, SY, SZ, the scalings to be applied to the X, Y and
Z coordinates.
*/
{
	float c[4][4];
	float d[4][4];
	int i;
	int j;
	
	tmat_init ( c );
	
	c[0][0] = sx;
	c[1][1] = sy;
	c[2][2] = sz;
	
	tmat_mxm ( c, a, d );
	
	for ( i = 0; i < 4; ++i ) {
		for ( j = 0; j < 4; ++j ) {
			b[i][j] = d[i][j];
		}
	}
	return;
}
/*********************************************************************/

bool tmat_shear ( float a[4][4], float b[4][4], char *axis, float s )

/*********************************************************************/

/*
Purpose:

TMAT_SHEAR applies a shear to the geometric transformation matrix.

Modified:

19 October 1998

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the current geometric transformation matrix.

Output, float B[4][4], the modified geometric transformation matrix.
A and B may share the same memory.

Input, character*3 AXIS, is 'XY', 'XZ', 'YX', 'YZ', 'ZX' or 'ZY',
specifying the shear equation:

XY:  x' = x + s * y;
XZ:  x' = x + s * z;
YX:  y' = y + s * x;
YZ:  y' = y + s * z;
ZX:  z' = z + s * x;
ZY:  z' = z + s * y.

Input, float S, the shear coefficient.
*/
{
	float c[4][4];
	float d[4][4];
	int i;
	int j;
	
	tmat_init ( c );
	
	if ( strcmp ( axis, "XY" ) == 0 || strcmp ( axis, "xy" ) == 0 ) {
		c[0][1] = s;
	}
	else if ( strcmp ( axis, "XZ" ) == 0 || strcmp ( axis, "xz" ) == 0 ) {
		c[0][2] = s;
	}
	else if ( strcmp ( axis, "YX" ) == 0 || strcmp ( axis, "yx" ) == 0 ) {
		c[1][0] = s;
	}
	else if ( strcmp ( axis, "YZ" ) == 0 || strcmp ( axis, "yz" ) == 0 ) {
		c[1][2] = s;
	}
	else if ( strcmp ( axis, "ZX" ) == 0 || strcmp ( axis, "zx" ) == 0 ) {
		c[2][0] = s;
	}
	else if ( strcmp ( axis, "ZY" ) == 0 || strcmp ( axis, "zy" ) == 0 ) {
		c[2][1] = s;
	}
	else {
//		printf ( "\n" );
//		printf ( "TMAT_SHEAR - Fatal error!\n" );
//		printf ( "  Illegal shear axis: %s.\n", axis );
//		printf ( "  Legal choices are XY, XZ, YX, YZ, ZX, or ZY.\n" );
		return false;
	}
	
	tmat_mxm ( c, a, d );
	
	for ( i = 0; i < 4; ++i ) {
		for ( j = 0; j < 4; ++j ) {
			b[i][j] = d[i][j];
		}
	}
	return true;
}
/*********************************************************************/

void tmat_trans ( float a[4][4], float b[4][4], float x, float y, float z )
				 
/*********************************************************************/

/*
Purpose:

TMAT_TRANS applies a translation to the geometric transformation matrix.

Modified:

19 October 1998

Author:

John Burkardt

Reference:

Foley, van Dam, Feiner, Hughes,
Computer Graphics, Principles and Practice,
Addison Wesley, Second Edition, 1990.

Parameters:

Input, float A[4][4], the current geometric transformation matrix.

Output, float B[4][4], the modified transformation matrix.
A and B may share the same memory.

Input, float X, Y, Z, the translation.  This may be thought of as the
point that the origin moves to under the translation.
*/
{
	int i;
	int j;
	
	for ( i = 0; i < 4; ++i ) {
		for ( j = 0; j < 4; ++j ) {
			b[i][j] = a[i][j];
		}
	}
	b[0][3] = b[0][3] + x;
	b[1][3] = b[1][3] + y;
	b[2][3] = b[2][3] + z;
	
	return;
}
/******************************************************************************/
