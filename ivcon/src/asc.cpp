#include "ivconv.h"

bool IVCONV::asc_read ( FILE *filein )

/*****************************************************************************/
/*
Purpose:

ASC_READ reads points stored in ASCII

8.59816       5.55317      -3.05561,
8.59816       2.49756      0.000000E+00,
...etc...
2.48695       2.49756      -3.05561,
*/
{
	for ( ;; ) {
		char input[LINE_MAX_LEN];
		if ( fgets ( input, LINE_MAX_LEN, filein ) == NULL ) break;
		
		++text_num;
		sscanf ( input, "%e %e %e", &cor3[cor3_num][0], &cor3[cor3_num][1], &cor3[cor3_num][2] );
		++cor3_num;
	}
  
  return true;
}

bool IVCONV::asc_write ( FILE *fileout )

{
	for (long j = 0; j < cor3_num; ++j ) {
		fprintf ( fileout, "%f %f %f\n", cor3[j][0], cor3[j][1], cor3[j][2] );
		++text_num;
	}
	return true;
}