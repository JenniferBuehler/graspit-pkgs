#include "ivconv.h"

bool IVCONV::tria_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

TRIA_READ reads an ASCII triangle file.

Example:

12                    <-- Number of triangles

(x,y,z) and (nx,ny,nz) of normal vector at:

0.0 0.0 0.0 0.3 0.3 0.3   node 1 of triangle 1.
1.0 0.0 0.0 0.3 0.1 0.3   node 2 of triangle 1,
0.0 1.0 0.0 0.3 0.1 0.3   node 3 of triangle 1,
1.0 0.5 0.0 0.3 0.1 0.3   node 1 of triangle 2,
...
0.0 0.5 0.5 0.3 0.1 0.3   node 3 of triangle 12.

Modified:

22 June 1999

Author:

John Burkardt
*/
{
	float cvec[3];
	int   icor3;
	int   iface;
	int   iface_hi;
	int   iface_lo;
	int   ivert;
	int   face_num2;
	float r1;
	float r2;
	float r3;
	float r4;
	float r5;
	float r6;
	/*
	Get the number of triangles.
	*/
	char input[LINE_MAX_LEN];
	fgets ( input, LINE_MAX_LEN, filein );
	++text_num;
	sscanf ( input, "%d", &face_num2 );
	/*
	For each triangle:
	*/
	iface_lo = face_num;
	iface_hi = face_num + face_num2;
	
	for ( iface = iface_lo; iface < iface_hi; iface++ ) {
		
		// if ( iface < FACE_MAX ) 
		{
			face_order[iface] = 3;
			face_material[iface] = 0;
		}
		/*
		For each face:
		*/
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			
			fgets ( input, LINE_MAX_LEN, filein );
			++text_num;
			sscanf ( input, "%e %e %e %e %e %e", &r1, &r2, &r3, &r4, &r5, &r6 ); 
			
			cvec[0] = r1;
			cvec[1] = r2;
			cvec[2] = r3;
			
			if ( cor3_num < 1000 ) {
				icor3 = rcol_find ( cor3, cor3_num, cvec );
			}
			else {
				icor3 = -1;
			}
			
			if ( icor3 == -1 ) {
				icor3 = cor3_num;
//				if ( cor3_num < COR3_MAX ) 
				{
					cor3[cor3_num]= cvec;
				}
				cor3_num = cor3_num + 1;
			}
			else {
				dup_num = dup_num + 1;
			}
			
			// if ( iface < FACE_MAX ) 
			{
				
				face[ivert][iface] = icor3;
				vertex_material[ivert][iface] = 0;
				vertex_normal[ivert][iface][0] = r4;
				vertex_normal[ivert][iface][1] = r5;
				vertex_normal[ivert][iface][2] = r6;
			}
			
		}
	}
	face_num = face_num + face_num2;
	
	return true;
}
/**********************************************************************/

bool IVCONV::tria_write ( FILE *fileout )

/**********************************************************************/

/*
Purpose:

TRIA_WRITE writes the graphics data to an ASCII "triangle" file.

Discussion:

This is just a private format that Greg Hood requested from me.

Example:

12                    <-- Number of triangles

(x,y,z) and (nx,ny,nz) of normal vector at:

0.0 0.0 0.0 0.3 0.3 0.3   node 1 of triangle 1.
1.0 0.0 0.0 0.3 0.1 0.3   node 2 of triangle 1,
0.0 1.0 0.0 0.3 0.1 0.3   node 3 of triangle 1,
1.0 0.5 0.0 0.3 0.1 0.3   node 1 of triangle 2,
...
0.0 0.5 0.5 0.3 0.1 0.3   node 3 of triangle 12.

Modified:

10 June 1999

Author:

John Burkardt
*/
{
	int face2[3];
	int icor3;
	int iface;
	int jlo;
	int k;
	int face_num2;
	int text_num;
	float nx;
	float ny;
	float nz;
	float x;
	float y;
	float z;
	
	text_num = 0;
	/*
	Determine the number of triangular faces.
	*/
	face_num2 = 0;
	for ( iface = 0; iface < face_num; iface++ ) {
		for ( jlo = 0; jlo < face_order[iface] - 2; jlo ++ ) {
			face_num2 = face_num2 + 1;
		}
	}
	
	fprintf ( fileout,  "%d\n", face_num2 );
	++text_num;
	/*
	Do the next face.
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
	/*
	Break the face up into triangles, anchored at node 1.
		*/
		for ( jlo = 0; jlo < face_order[iface] - 2; jlo ++ ) {
			
			face2[0] = face[    0][iface];
			face2[1] = face[jlo+1][iface];
			face2[2] = face[jlo+2][iface];
			
			for ( k = 0; k < 3; ++k ) {
				
				icor3 = face2[k];
				
				x = cor3[icor3][0];
				y = cor3[icor3][1];
				z = cor3[icor3][2];
				
				nx = cor3_normal[icor3][0];
				ny = cor3_normal[icor3][1];
				nz = cor3_normal[icor3][2];
				
				fprintf ( fileout,  "%f %f %f %f %f %f\n", x, y, z, nx, ny, nz );
				
				++text_num;
				
			}
			
		}
		
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "TRIA_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
/******************************************************************************/

bool IVCONV::trib_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

TRIB_READ reads a binary triangle file.

Example:

4 byte int = number of triangles

For each triangular face:

3 4-byte floats = coordinates of first node;
3 4-byte floats = components of normal vector at first node;
3 4-byte floats = coordinates of second node;
3 4-byte floats = components of normal vector at second node;
3 4-byte floats = coordinates of third node;
3 4-byte floats = components of normal vector at third node.

Modified:

22 June 1999

Author:

John Burkardt
*/
{
	float cvec[3];
	int icor3;
	int i;
	int iface;
	int iface_hi;
	int iface_lo;
	int ivert;
	int face_num2;
	/* 
	Read the number of triangles in the file.
	*/
	face_num2 = long_int_read ( filein );
	bytes_num = bytes_num + 4;
	/*
	For each (triangular) face,
    read the coordinates and normal vectors of three vertices,
	*/
	iface_lo = face_num;
	iface_hi = face_num + face_num2;
	
	for ( iface = iface_lo; iface < iface_hi; iface++ ) {
		
		// if ( iface < FACE_MAX ) 
		{
			face_order[iface] = 3;
			face_material[iface] = 0;
		}
		
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			
			for ( i = 0; i < 3; ++i ) {
				cvec[i] = float_read ( filein );
				bytes_num = bytes_num + 4;
			}
			
			if ( cor3_num < 1000 ) {
				icor3 = rcol_find ( cor3, cor3_num, cvec );
			}
			else {
				icor3 = -1;
			}
			
			if ( icor3 == -1 ) {
				icor3 = cor3_num;
//				if ( cor3_num < COR3_MAX ) 
				{
					cor3[cor3_num] = cvec;
				}
				cor3_num = cor3_num + 1;
			}
			else {
				dup_num = dup_num + 1;
			}
			
			//if ( iface < FACE_MAX ) 
			{
				
				face[ivert][iface] = icor3;
				vertex_material[ivert][iface] = 0;
				
				for ( i = 0; i < 3; ++i ) {
					vertex_normal[ivert][iface][i] = float_read ( filein );
					bytes_num = bytes_num + 4;
				}
				
			}
			
		}
	}
	
	face_num = face_num + face_num2;
	
	return true;
}
/**********************************************************************/

bool IVCONV::trib_write ( FILE *fileout )

/**********************************************************************/

/*
Purpose:

TRIB_WRITE writes the graphics data to a binary "triangle" file.

Discussion:

This is just a private format that Greg Hood requested from me.

Example:

12   Number of triangles
0.0  x at node 1, triangle 1,
0.0  y at node 1, triangle 1,
0.0  z at node 1, triangle 1,
0.3  nx at node 1, triangle 1,
0.3  ny at node 1, triangle 1,
0.3  nz at node 1, triangle 1.
1.0  x at node 2, triangle 1,
...
0.7  nz at node 3, triangle 1.
1.2  x at node 1, triangle 2,
...
0.3  nz at node 3, triangle 2.
9.3  x at node 1, triangle 3,
...
0.3  nz at node 3, triangle 12.

Modified:

16 June 1999

Author:

John Burkardt
*/
{
	int face2[3];
	int icor3;
	int iface;
	int jlo;
	int k;
	int face_num2;
	float nx;
	float ny;
	float nz;
	float x;
	float y;
	float z;
	
	bytes_num = 0;
	/*
	Determine the number of triangular faces.
	*/
	face_num2 = 0;
	for ( iface = 0; iface < face_num; iface++ ) {
		for ( jlo = 0; jlo < face_order[iface] - 2; jlo ++ ) {
			face_num2 = face_num2 + 1;
		}
	}
	
	bytes_num = bytes_num + long_int_write ( fileout, face_num2 );
	/*
	Do the next face.
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
	/*
	Break the face up into triangles, anchored at node 1.
		*/
		for ( jlo = 0; jlo < face_order[iface] - 2; jlo ++ ) {
			
			face2[0] = face[    0][iface];
			face2[1] = face[jlo+1][iface];
			face2[2] = face[jlo+2][iface];
			
			for ( k = 0; k < 3; ++k ) {
				
				icor3 = face2[k];
				
				x = cor3[icor3][0];
				y = cor3[icor3][1];
				z = cor3[icor3][2];
				
				nx = cor3_normal[icor3][0];
				ny = cor3_normal[icor3][1];
				nz = cor3_normal[icor3][2];
				
				bytes_num = bytes_num + float_write ( fileout, x );
				bytes_num = bytes_num + float_write ( fileout, y );
				bytes_num = bytes_num + float_write ( fileout, z );
				bytes_num = bytes_num + float_write ( fileout, nx );
				bytes_num = bytes_num + float_write ( fileout, ny );
				bytes_num = bytes_num + float_write ( fileout, nz );
				
			}
			
		}
		
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "TRIB_WRITE - Wrote %d bytes.\n", bytes_num );
	
	return true;
}
/******************************************************************************/

