#include "ivconv.h"


#ifdef __linux__
#include <string.h>
#endif

bool IVCONV::dxf_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  DXF_READ reads an AutoCAD DXF file.
  
	Examples:
	
      0
	  SECTION
      2
	  HEADER
	  999
	  diamond.dxf created by IVREAD.
	  999
	  Original data in diamond.obj.
      0
	  ENDSEC
      0
	  SECTION
      2
	  TABLES
      0
	  ENDSEC
      0
	  SECTION
      2
	  BLOCKS
      0
	  ENDSEC
      0
	  SECTION
      2
	  ENTITIES
      0
	  LINE
      8
	  0
	  10
      0.00  (X coordinate of beginning of line.)
	  20
      0.00  (Y coordinate of beginning of line.)
	  30
      0.00  (Z coordinate of beginning of line.)
	  11
      1.32  (X coordinate of end of line.)
	  21
      1.73  (Y coordinate of end of line.)
	  31
      2.25  (Z coordinate of end of line.)
      0
	  3DFACE
      8
	  Cube
	  10
	  -0.50  (X coordinate of vertex 1)
	  20
	  0.50  (Y coordinate of vertex 1)   
	  30
      1.0  (Z coordinate of vertex 1)  
	  11
	  0.50  (X coordinate of vertex 2)  
	  21
	  0.50  (Y coordinate of vertex 2)
	  31
      1.0  (Z coordinate of vertex 2)
	  12
	  0.50  (X coordinate of vertex 3) 
	  22
	  0.50  (Y coordinate of vertex 3)
	  32
	  0.00  (Z coordinate of vertex 3)
      0
	  ENDSEC
      0
	  EOF
	  
		Modified:
		
		  23 May 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int   code;
	int   count;
	float cvec[3];
	int   icor3;
	char  input1[LINE_MAX_LEN];
	char  input2[LINE_MAX_LEN];
	int   ivert;
	float rval;
	int   width;
	int   linemode;
	int   cpos;
	
	linemode = 0;
	ivert = 0;
	/* 
	Read the next two lines of the file into INPUT1 and INPUT2. 
	*/
	
	for ( ;; ) {
		
	/* 
	INPUT1 should contain a single integer, which tells what INPUT2
	will contain.
		*/
		if ( fgets ( input1, LINE_MAX_LEN, filein ) == NULL ) {
			break;
		}
		
		++text_num;
		
		count = sscanf ( input1, "%d%n", &code, &width );
		if ( count <= 0 ) {
			break;
		}
		/*
		Read the second line, and interpret it according to the code.
		*/
		if ( fgets ( input2, LINE_MAX_LEN, filein ) == NULL ) {
			break;
		}
		
		++text_num;
		
		if ( code == 0 ) {
			
			if ( ivert > 0 ) {
				/* finish off the face */
				face_order[face_num] = ivert;
				++face_num;
				ivert = 0;
			}
			
			if ( strncmp( input2, "LINE", 4 ) == 0 ) {
				linemode = 1;
			}
			else if ( strncmp( input2, "3DFACE", 6 ) == 0 ) {
				linemode = 0;
				ivert = 0;
			}
		}
		else {
			
			for (cpos = 0; input1[cpos] == ' '; cpos++) 
			{};
			
			if ( input1[cpos] == '1' || input1[cpos] == '2' || input1[cpos] == '3' ) {
				
				count = sscanf ( input2, "%e%n", &rval, &width );
				
				switch ( input1[cpos] )
				{
				case '1':
					if ( line_num > 0 ) {
						if ( linemode ) {
							line_dex[line_num] = - 1;
							line_material[line_num] = - 1;
							++line_num;
						}
					}
					cvec[0] = rval;
					break;
					
				case '2':
					cvec[1] = rval;
					break;
					
				case '3':
					cvec[2] = rval;
					
					if ( cor3_num < 1000 ) {
						icor3 = rcol_find ( cor3, cor3_num, cvec );
					} 
					else {
						icor3 = -1;
					}
					
					if ( icor3 == -1 ) {
						icor3 = cor3_num;
//						if ( cor3_num < COR3_MAX ) 
						{
							cor3[cor3_num][0] = cvec[0];
							cor3[cor3_num][1] = cvec[1];
							cor3[cor3_num][2] = cvec[2];
						}
						++cor3_num;
					}
					else {
						++dup_num ;
					}
					
					if ( linemode ) {
						line_dex[line_num] = icor3;
						line_material[line_num] = 0;
						++line_num;
					}
					else {
						face[ivert][face_num] = icor3;
						++ivert;
					}
					break;
					
				default:
					break;
				}
			}
		}
  }
  
  if ( line_num > 0 ) {
	  if ( linemode ) {
		  line_dex[line_num] = - 1;
		  line_material[line_num] = - 1;
		  ++line_num;
	  }
  }
  return true;
}
/******************************************************************************/

bool IVCONV::dxf_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

  DXF_WRITE writes graphics information to an AutoCAD DXF file.
  
	Examples:
	
      0
	  SECTION
      2
	  HEADER
	  999
	  diamond.dxf created by IVREAD.
	  999
	  Original data in diamond.obj.
      0
	  ENDSEC
      0
	  SECTION
      2
	  TABLES
      0
	  ENDSEC
      0
	  SECTION
      2
	  BLOCKS
      0
	  ENDSEC
      0
	  SECTION
      2
	  ENTITIES
      0
	  LINE
      8
	  0
	  10
      0.00  (X coordinate of beginning of line.)
	  20
      0.00  (Y coordinate of beginning of line.)
	  30
      0.00  (Z coordinate of beginning of line.)
	  11
      1.32  (X coordinate of end of line.)
	  21
      1.73  (Y coordinate of end of line.)
	  31
      2.25  (Z coordinate of end of line.)
      0
	  3DFACE
      8
	  Cube
	  10
	  -0.50  (X coordinate of vertex 1)
	  20
	  0.50  (Y coordinate of vertex 1)   
	  30
      1.0  (Z coordinate of vertex 1)  
	  11
	  0.50  (X coordinate of vertex 2)  
	  21
	  0.50  (Y coordinate of vertex 2)
	  31
      1.0  (Z coordinate of vertex 2)
	  12
	  0.50  (X coordinate of vertex 3) 
	  22
	  0.50  (Y coordinate of vertex 3)
	  32
	  0.00  (Z coordinate of vertex 3)
      0
	  ENDSEC
      0
	  EOF
	  
		Modified:
		
		  16 May 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int   icor3;
	int   iline;
	int   iface;
	int   ivert;
	int   jcor3;
	int   newline;
	int   text_num;
	
	/* 
	Initialize. 
	*/
	text_num = 0;
	
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "SECTION\n" );
	fprintf ( fileout, "  2\n" );
	fprintf ( fileout, "HEADER\n" );
	fprintf ( fileout, "999\n" );
	fprintf ( fileout, "%s created by IVCON.\n", fileout_name );
	fprintf ( fileout, "999\n" );
	fprintf ( fileout, "Original data in %s.\n", filein_name );
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "ENDSEC\n" );
	text_num+=10;
	
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "SECTION\n" );
	fprintf ( fileout, "  2\n" );
	fprintf ( fileout, "TABLES\n" );
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "ENDSEC\n" );
	text_num += 6;
	
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "SECTION\n" );
	fprintf ( fileout, "  2\n" );
	fprintf ( fileout, "BLOCKS\n" );
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "ENDSEC\n" );
	text_num += 6;
	
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "SECTION\n" );
	fprintf ( fileout, "  2\n" );
	fprintf ( fileout, "ENTITIES\n" );
	text_num += 4;
	/*
	Handle lines.
	*/
	jcor3 = 0;
	newline = true;
	
	for ( iline = 0; iline < line_num; iline++ ) {
		
		icor3 = line_dex[iline];
		
		if ( icor3 == -1 ) {
			
			newline = true;
		}
		else {
			
			if (!newline) {
				
				fprintf ( fileout, "  0\n" );
				fprintf ( fileout, "LINE\n" );
				fprintf ( fileout, "  8\n" );
				fprintf ( fileout, "  0\n" );
				fprintf ( fileout, " 10\n" );
				fprintf ( fileout, "%f\n", cor3[jcor3][0] );
				fprintf ( fileout, " 20\n" );
				fprintf ( fileout, "%f\n", cor3[jcor3][1] );
				fprintf ( fileout, " 30\n" );
				fprintf ( fileout, "%f\n", cor3[jcor3][2] );
				fprintf ( fileout, " 11\n" );
				fprintf ( fileout, "%f\n", cor3[icor3][0] );
				fprintf ( fileout, " 21\n" );
				fprintf ( fileout, "%f\n", cor3[icor3][1] );
				fprintf ( fileout, " 31\n" );
				fprintf ( fileout, "%f\n", cor3[icor3][2] );
				
				text_num+=16;
				
			}
			
			jcor3 = icor3;
			newline = false;
			
		}
	}
	/*
	Handle faces.
	(If FACE_ORDER is greater than 10, you're sure to have problems here)
	*/
	for ( iface = 0; iface < face_num; iface++ ) {
		
		fprintf ( fileout, "  0\n" );
		fprintf ( fileout, "3DFACE\n" );
		fprintf ( fileout, "  8\n" );
		fprintf ( fileout, "  Cube\n" );
		text_num = text_num + 4;
		
		for ( ivert = 0; ivert < face_order[iface]; ivert++ ) {
			
			icor3 = face[ivert][iface];
			
			fprintf ( fileout, "1%d\n", ivert );
			fprintf ( fileout, "%f\n", cor3[icor3][0] );
			fprintf ( fileout, "2%d\n", ivert );
			fprintf ( fileout, "%f\n", cor3[icor3][1] );
			fprintf ( fileout, "3%d\n", ivert );
			fprintf ( fileout, "%f\n", cor3[icor3][2] );
			
			text_num = text_num + 6;
		}
	}
	
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "ENDSEC\n" );
	fprintf ( fileout, "  0\n" );
	fprintf ( fileout, "EOF\n" );
	text_num = text_num + 4;
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "DXF_WRITE - Wrote %d text lines.\n", text_num );
	
	return true;
}
