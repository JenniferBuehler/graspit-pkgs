#include "ivconv.h"

bool IVCONV::vla_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

  VLA_READ reads a VLA file.
  
	Examples:
	
	  set comment cube.vla created by IVREAD
	  set comment Original data in cube.iv.
	  set comment
	  set intensity EXPLICIT
	  set parametric NON_PARAMETRIC
	  set filecontent LINES
	  set filetype NEW
	  set depthcue 0
	  set defaultdraw stellar
	  set coordsys RIGHT
	  set author IVREAD
	  set site Buhl Planetarium
	  set library_id UNKNOWN
	  P   8.59816       5.55317      -3.05561       1.00000
	  L   8.59816       2.49756      0.000000E+00   1.00000
	  L   8.59816       2.49756      -3.05561       1.00000
	  L   8.59816       5.55317      -3.05561       1.00000
	  P   8.59816       5.55317      0.000000E+00   1.00000
	  ...etc...
	  L   2.48695       2.49756      -3.05561       1.00000
	  
		Modified:
		
		  23 May 1999
		  
			Author:
			
			  John Burkardt
			  */
{
	int   icor3;
	int   dup_num;
	char *next;
	int   text_num;
	float r1;
	float r2;
	float r3;
	float temp[3];
	char  token[LINE_MAX_LEN];
	int   width;
	/*
	Initialize. 
	*/
	dup_num = 0;
	text_num = 0;
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
		if ( *next == '\0' || *next == ';' ) {
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
		SET (ignore) 
		*/
		if ( leqi ( token, "set" )  ) {
		}
		/* 
		P (begin a line)
		L (continue a line) 
		*/
		else if ( leqi ( token, "P" )  || leqi ( token, "L")  ) {
			
			if ( leqi ( token, "P" )  ) {
				if ( line_num > 0 ) {
					if ( line_num < LINES_MAX ) {
						line_dex[line_num] = -1;
						line_material[line_num] = -1;
						++line_num;
					}
				}
			}
			
			sscanf ( next, "%e %e %e", &r1, &r2, &r3 );
			
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
				
				//				if ( cor3_num < COR3_MAX ) 
				{
					cor3[cor3_num] = temp;
				}
				cor3_num = cor3_num + 1;
			}
			else {
				dup_num = dup_num + 1;
			}
			
			if ( line_num < LINES_MAX ) {
				line_dex[line_num] = icor3;
				line_material[line_num] = 0;
				++line_num;
			}
		}
		/* 
		Unexpected or unrecognized. 
		*/
		else {
			printf ( "\n" );
			printf ( "VLA_READ - Fatal error!\n" );
			printf ( "  Unrecognized first word on line.\n" );
			return false;
		}
		
	}
	
	if ( line_num > 0 ) {
		if ( line_num < LINES_MAX ) {
			line_dex[line_num] = -1;
			line_material[line_num] = -1;
			++line_num;
		}
	}
	
	return true;
}
/******************************************************************************/

bool IVCONV::vla_write ( FILE *fileout )

/******************************************************************************/

/*
Purpose:

VLA_WRITE writes internal graphics information to a VLA file.

Discussion:

Comments begin with a semicolon in column 1.
The X, Y, Z coordinates of points begin with a "P" to
denote the beginning of a line, and "L" to denote the
continuation of a line.  The fourth entry is intensity, which 
should be between 0.0 and 1.0.

Examples:

set comment cube.vla created by IVREAD
set comment Original data in cube.iv.
set comment
set intensity EXPLICIT
set parametric NON_PARAMETRIC
set filecontent LINES
set filetype NEW
set depthcue 0
set defaultdraw stellar
set coordsys RIGHT
set author IVREAD
set site Buhl Planetarium
set library_id UNKNOWN
P   8.59816       5.55317      -3.05561       1.00000
L   8.59816       2.49756      0.000000E+00   1.00000
L   8.59816       2.49756      -3.05561       1.00000
L   8.59816       5.55317      -3.05561       1.00000
P   8.59816       5.55317      0.000000E+00   1.00000
...etc...
L   2.48695       2.49756      -3.05561       1.00000

Modified:

22 May 1999

Author:

John Burkardt
*/
{
	char  c;
	int   iline;
	float intense = 1.0;
	int   k;
	int   text_num;
	/* 
	Initialize. 
	*/
	text_num = 0;
	
	fprintf ( fileout, "set comment %s created by IVCON.\n", fileout_name );
	fprintf ( fileout, "set comment Original data in %s.\n", filein_name );
	fprintf ( fileout, "set comment\n" );
	fprintf ( fileout, "set intensity EXPLICIT\n" );
	fprintf ( fileout, "set parametric NON_PARAMETRIC\n" );
	fprintf ( fileout, "set filecontent LINES\n" );
	fprintf ( fileout, "set filetype NEW\n" );
	fprintf ( fileout, "set depthcue 0\n" );
	fprintf ( fileout, "set defaultdraw stellar\n" );
	fprintf ( fileout, "set coordsys RIGHT\n" );
	fprintf ( fileout, "set author IVCON\n" );
	fprintf ( fileout, "set site Buhl Planetarium\n" );
	fprintf ( fileout, "set library_id UNKNOWN\n" );
	
	text_num+=13;
	
	c = 'P';
	
	for ( iline = 0; iline < line_num; iline++ ) {
		
		k = line_dex[iline];
		
		if ( k == -1 ) {
			
			c = 'P';
		}
		else {
			
			fprintf ( fileout, "%c %f %f %f %f\n", 
				c, cor3[k][0], cor3[k][1], cor3[k][2], intense );
			
			++text_num;
			
			c = 'L';
		}
	}
	/*
	Report.
	*/
	printf ( "\n" );
	printf ( "VLA_WRITE - Wrote %d text lines.\n", text_num );
	
	
	return true;
}

/**********************************************************************/

