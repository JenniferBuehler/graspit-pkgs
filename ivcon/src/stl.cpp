#include "ivconv.h"

/******************************************************************************/

bool IVCONV::stla_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

STLA_READ reads an ASCII STL (stereolithography) file.

Examples:

solid MYSOLID
facet normal 0.4 0.4 0.2
outerloop
vertex  1.0 2.1 3.2
vertex  2.1 3.7 4.5
vertex  3.1 4.5 6.7
endloop
endfacet
...
facet normal 0.2 0.2 0.4
outerloop
vertex  2.0 2.3 3.4
vertex  3.1 3.2 6.5
vertex  4.1 5.5 9.0
endloop
endfacet
endsolid MYSOLID

Modified:

20 October 1998

Author:

  John Burkardt
  */
{
	int   count;
	int   i;
	int   icor3;
	int   ivert;
	char *next;
	float r1;
	float r2;
	float r3;
	float r4;
	float temp[3];
	char  token[LINE_MAX_LEN];
	int   width;
	/*
	Read the next line of the file into INPUT. 
	*/
	char input[LINE_MAX_LEN];

	while ( fgets ( input, LINE_MAX_LEN, filein ) != NULL ) {
		
		++text_num;
		/*
		Advance to the first nonspace character in INPUT. 
		*/
		for ( next = input; *next != '\0' && isspace(*next); next++ ) {
		}
		/*
		Skip blank lines and comments. 
		*/
		if ( *next == '\0' || *next == '#' || *next == '!' || *next == '$' ) {
			continue;
		}
		/*
		Extract the first word in this line. 
		*/
		sscanf ( next, "%s%n", token, &width );
		/*
		Set NEXT to point to just after this token. 
		*/
		next = next + width;
		/*
		FACET
		*/
		if ( leqi ( token, "facet" )  ) {
		/* 
		Get the XYZ coordinates of the normal vector to the face. 
			*/
			sscanf ( next, "%*s %e %e %e", &r1, &r2, &r3 );  
			
			// if ( face_num < FACE_MAX ) 
			{
				face_normal[face_num] = vec3(r1,r2,r3);
			}
			
			fgets ( input, LINE_MAX_LEN, filein );
			++text_num;
			
			ivert = 0;
			
			for ( ;; ) {
				
				fgets ( input, LINE_MAX_LEN, filein );
				++text_num;
				
				count = sscanf ( input, "%*s %e %e %e", &r1, &r2, &r3 );
				
				if ( count != 3 ) {
					break;
				}
				
				temp[0] = r1;
				temp[1] = r2;
				temp[2] = r3;
				
				if ( cor3_num < 1000 ) {
					icor3 = rcol_find ( cor3, cor3_num, temp );
				}
				else {
					icor3 = -1;
				}
				
				if ( icor3 == -1 ) {
					
					icor3 = cor3_num;
					
//					if ( cor3_num < COR3_MAX ) 
					{
						cor3[cor3_num] = temp;
					}
					cor3_num = cor3_num + 1;
				}
				else {
					dup_num = dup_num + 1;
				}
				
				// if ( ivert < ORDER_MAX && face_num < FACE_MAX ) 
				{
					face[ivert][face_num] = icor3;
					vertex_material[ivert][face_num] = 0;
					for ( i = 0; i < 3; ++i ) {
						vertex_normal[ivert][face_num][i] = face_normal[face_num][i];
					}
				}
				
				ivert = ivert + 1;
			} 
			
			fgets ( input, LINE_MAX_LEN, filein );
			++text_num;
			
			// if ( face_num < FACE_MAX ) 
			{
				face_order[face_num] = ivert;
			} 
			
			++face_num;
			
		}
		/*
		COLOR 
		*/
		
		else if ( leqi ( token, "color" )  ) {
			sscanf ( next, "%*s %f %f %f %f", &r1, &r2, &r3, &r4 );
		}
		/* 
		SOLID 
		*/
		else if ( leqi ( token, "solid" )  ) {
			object_num = object_num + 1;
		}
		/* 
		ENDSOLID 
		*/
		else if ( leqi ( token, "endsolid" )  ) {
		}
		/* 
		Unexpected or unrecognized. 
		*/
		else {
			printf ( "\n" );
			printf ( "STLA_READ - Fatal error!\n" );
			printf ( "  Unrecognized first word on line.\n" );
			return false;
		}
		
  }
  return true;
}
/******************************************************************************/

bool IVCONV::stla_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

  STLA_WRITE writes an ASCII STL (stereolithography) file.
  
	Examples:
	
	  solid MYSOLID
      facet normal 0.4 0.4 0.2
	  outerloop
	  vertex  1.0 2.1 3.2
	  vertex  2.1 3.7 4.5
	  vertex  3.1 4.5 6.7
	  endloop
      endfacet
      ...
      facet normal 0.2 0.2 0.4
	  outerloop
	  vertex  2.0 2.3 3.4
	  vertex  3.1 3.2 6.5
	  vertex  4.1 5.5 9.0
	  endloop
      endfacet
	  endsolid
	  
		Discussion:
		
		  The polygons in an STL file should only be triangular.  This routine 
		  will try to automatically decompose higher-order polygonal faces into 
		  suitable triangles, without actually modifying the internal graphics 
		  data.
		  
			Modified:
			
			  01 September 1998
			  
				Author:
				
				  John Burkardt
				  */
{
	int icor3;
	int iface;
	int jvert;
	int face_num2;
	int text_num;
	/*
	Initialize.
	*/
	text_num = 0;
	face_num2 = 0;
	
	fprintf ( fileout, "solid MYSOLID created by IVCON, original data in %s\n", 
		filein_name );
	
	++text_num;
	
	for ( iface = 0; iface < face_num; iface++ ) {
		
		for ( jvert = 2; jvert < face_order[iface]; jvert++ ) {
			
			face_num2 = face_num2 + 1;
			
			fprintf ( fileout, "  facet normal %f %f %f\n", 
				face_normal[iface][0], face_normal[iface][1], face_normal[iface][2] );
			
			fprintf ( fileout, "    outer loop\n" );
			
			icor3 = face[0][iface];
			fprintf ( fileout, "      vertex %f %f %f\n", 
				cor3[icor3][0], cor3[icor3][1], cor3[icor3][2] );
			
			icor3 = face[jvert-1][iface];
			fprintf ( fileout, "      vertex %f %f %f\n", 
				cor3[icor3][0], cor3[icor3][1], cor3[icor3][2] );
			
			icor3 = face[jvert][iface];
			fprintf ( fileout, "      vertex %f %f %f\n", 
				cor3[icor3][0], cor3[icor3][1], cor3[icor3][2] );
			
			fprintf ( fileout, "    endloop\n" );
			fprintf ( fileout, "  endfacet\n" );
			text_num = text_num + 7;
		}
	}
	
	fprintf ( fileout, "endsolid MYSOLID\n" );
	++text_num;
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "STLA_WRITE - Wrote %d text lines.\n", text_num );
	
	if ( face_num != face_num2 ) {
		printf ( "  Number of faces in original data was %d.\n", face_num );
		printf ( "  Number of triangular faces in decomposed data is %d.\n",
			face_num2 );
	}
	
	return true;
}
/******************************************************************************/

bool IVCONV::stlb_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

STLB_READ reads a binary STL (stereolithography) file.

Example:

80 byte string = header containing nothing in particular

4 byte int = number of faces

For each face:

3 4-byte floats = components of normal vector to face;
3 4-byte floats = coordinates of first node;
3 4-byte floats = coordinates of second node;
3 4-byte floats = coordinates of third and final node;
2-byte int = attribute, whose value is 0.

Modified:

24 May 1999

Author:

John Burkardt
*/
{
	short int attribute = 0;
	char c;
	float cvec[3];
	int icor3;
	int i;
	int iface;
	int ivert;
	/* 
	80 byte Header.
	*/
	for ( i = 0; i < 80; ++i ) {
		c = char_read ( filein );
		if ( debug ) {
			printf ( "%d\n", c );
		}
		bytes_num = bytes_num + 1;
	}
	/*
	Number of faces.
	*/
	face_num = long_int_read ( filein );
	bytes_num = bytes_num + 4;
	/*
	For each (triangular) face,
    components of normal vector,
    coordinates of three vertices,
    2 byte "attribute".
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
		
		face_order[iface] = 3;
		face_material[iface] = 0;
		
		for ( i = 0; i < 3; ++i ) {
			face_normal[iface][i] = float_read ( filein );
			bytes_num = bytes_num + 4;
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
//					cor3[cor3_num][1] = cvec[1];
//					cor3[cor3_num][2] = cvec[2];
				}
				cor3_num = cor3_num + 1;
			}
			else {
				dup_num = dup_num + 1;
			}
			
			face[ivert][iface] = icor3;
			
		}
		attribute = short_int_read ( filein );
		if ( debug ) {
			printf ( "ATTRIBUTE = %d\n", attribute );
		}
		bytes_num = bytes_num + 2;
	}
	
	return true;
}
/******************************************************************************/

bool IVCONV::stlb_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

STLB_WRITE writes a binary STL (stereolithography) file.

Example:

80 byte string = header containing nothing in particular

4 byte int = number of faces

For each face:

3 4-byte floats = components of normal vector to face;
3 4-byte floats = coordinates of first node;
3 4-byte floats = coordinates of second node;
3 4-byte floats = coordinates of third and final node;
2-byte int = attribute, whose value is 0.

Discussion:

The polygons in an STL file should only be triangular.  This routine 
will try to automatically decompose higher-order polygonal faces into 
suitable triangles, without actually modifying the internal graphics 
data.

Modified:

24 May 1999

Author:

John Burkardt
*/
{
	short int attribute = 0;
	char c;
	int i;
	int icor3;
	int iface;
	int jvert;
	int face_num2;
	/* 
	80 byte Header.
	*/
	for ( i = 0; i < 80; ++i ) {
		c = ' ';
		bytes_num = bytes_num + char_write ( fileout, c );
	}
	/*
	Number of faces.
	*/
	face_num2 = 0;
	for ( iface = 0; iface < face_num; iface++ ) {
		face_num2 = face_num2 + face_order[iface] - 2;
	}
	
	bytes_num = bytes_num + long_int_write ( fileout, face_num2 );
	/*
	For each (triangular) face,
    components of normal vector,
    coordinates of three vertices,
    2 byte "attribute".
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
		
		for ( jvert = 2; jvert < face_order[iface]; jvert++ ) {
			
			for ( i = 0; i < 3; ++i ) {
				bytes_num = bytes_num + float_write ( fileout, face_normal[iface][i] );
			}
			
			icor3 = face[0][iface];
			for ( i = 0; i < 3; ++i ) {
				bytes_num = bytes_num + float_write ( fileout, cor3[icor3][i] );
			}
			
			icor3 = face[jvert-1][iface];
			for ( i = 0; i < 3; ++i ) {
				bytes_num = bytes_num + float_write ( fileout, cor3[icor3][i] );
			}
			
			icor3 = face[jvert][iface];
			for ( i = 0; i < 3; ++i ) {
				bytes_num = bytes_num + float_write ( fileout, cor3[icor3][i] );
			}
			
			bytes_num = bytes_num + short_int_write ( fileout, attribute );
			
		}
		
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "STLB_WRITE - Wrote %d bytes.\n", bytes_num );
	
	if ( face_num != face_num2 ) {
		printf ( "  Number of faces in original data was %d.\n", face_num );
		printf ( "  Number of triangular faces in decomposed data is %d.\n",
			face_num2 );
	}
	
	return true;
}