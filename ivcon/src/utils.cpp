#include "utils.h"
#include "ctype.h"
#include "string.h"


/******************************************************************************/

char *file_ext ( char *file_name )

/******************************************************************************/

/*
Purpose:

FILE_EXT picks out the extension in a file name.

Modified:

21 July 1998

Author:

John Burkardt
*/
{
	int i;
	
	i = char_index_last ( file_name, '.' );
	
	if ( i == -1 ) {
		return NULL;
	}
	else {
		return file_name + i + 1;
	}
}

float float_reverse_bytes ( float x )
{
	char c;
	union {
		float yfloat;
		char ychar[4];
	} y;
	
	y.yfloat = x;
	
	c = y.ychar[0];
	y.ychar[0] = y.ychar[3];
	y.ychar[3] = c;
	
	c = y.ychar[1];
	y.ychar[1] = y.ychar[2];
	y.ychar[2] = c;
	
	return ( y.yfloat );
}



int rcol_find ( const array<vec3>& a, int n, vec3 r )

/******************************************************************************/

/*
Purpose:

RCOL_FIND finds if a vector occurs in a table.

Comment:

Explicitly forcing the second dimension to be COR3_MAX is a kludge.
I have to figure out how to do this as pointer references.

Also, since the array is not sorted, this routine should not be carelessly
called repeatedly for really big values of N, because you'll waste a
lot of time.

Modified:

27 April 1999

Author:

John Burkardt
*/
{
	int icol = -1;
	
	for ( int j = 0; j < n; ++j ) {
		int i=0;
		for (i = 0; i < 3; ++i ) 
			if ( a[j][i] != r[i] ) break;
		if (i==3) 
			return j;
	}
	
	return icol;
}

int char_index_last ( char* string, char c )

/******************************************************************************/

/*
Purpose:

CHAR_INDEX_LAST reports the last occurrence of a character in a string.

Author:

John Burkardt
*/
{
	int i;
	int j;
	int nchar;
	
	j = -1;
	
	nchar = strlen ( string );
	
	for ( i = 0; i < nchar; ++i ) {
		if ( string[i] == c ) {
			j = i;
		}
	}
	
	return j;
	
}
/******************************************************************************/

bool char_pad ( int *char_index, int *null_index, char *string, 
			  int STRING_MAX )
			  
/******************************************************************************/

/*
Purpose:

CHAR_PAD "pads" a character in a string with a blank on either side.

Modified:

16 October 1998

Author:

John Burkardt

Parameters:

Input/output, int *CHAR_INDEX, the position of the character to be padded.
On output, this is increased by 1.

Input/output, int *NULL_INDEX, the position of the terminating NULL in 
the string.  On output, this is increased by 2.

Input/output, char STRING[STRING_MAX], the string to be manipulated.

Input, int STRING_MAX, the maximum number of characters that can be stored
in the string.

Output, int CHAR_PAD, is SUCCESS if the operation worked, and ERROR otherwise.
*/
{
	int i;
	
	if ( *char_index < 0 || 
		*char_index >= *null_index || 
		*char_index > STRING_MAX-1 ) {
		return false;
	}
	
	if ( (*null_index) + 2 > STRING_MAX-1 ) {
		return false;
	}
	
	for ( i = *null_index + 2; i > *char_index + 2; i-- ) {
		string[i] = string[i-2];
	}
	string[*char_index+2] = ' ';
	string[*char_index+1] = string[*char_index];
	string[*char_index] = ' ';
	
	*char_index = *char_index + 1;
	*null_index = *null_index + 2;
	
	return true;
}
/******************************************************************************/

char char_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

CHAR_READ reads one character from a binary file.

Modified:

24 May 1999

Author:

John Burkardt
*/
{
	char c;
	
	c = ( char ) fgetc ( filein );
	
	return c;
}
/******************************************************************************/

int char_write ( FILE *fileout, char c )

/******************************************************************************/

/*
Purpose:

CHAR_WRITE writes one character to a binary file.

Modified:

24 May 1999

Author:

John Burkardt
*/
{
	fputc ( c, fileout );
	
	return 1;
}
/******************************************************************************/

float float_read ( FILE *filein )
{
	float rval;
	float temp;
	
	fread ( &temp, sizeof ( float ), 1, filein );
	
	if ( byte_swap  ) {
		rval = float_reverse_bytes ( temp );
	}
	else {
		rval = temp;
	}
	
	return rval;
}

int float_write ( FILE *fileout, float float_val )

/******************************************************************************/

/*
Purpose:

FLOAT_WRITE writes 1 float to a binary file.

Modified:

23 September 1998
*/
{
	int nbyte = sizeof ( float );
	float temp;
	
	if ( byte_swap  ) {
		temp = float_reverse_bytes ( float_val );
	}
	else {
		temp = float_val;
	}
	
	fwrite ( &temp, nbyte, 1, fileout );
	
	return nbyte;
}
/******************************************************************************/

bool leqi ( const char* string1, const char* string2 )

/******************************************************************************/

/*
Purpose:

LEQI compares two strings for equality, disregarding case.

Modified:

15 September 1998

Author:

John Burkardt
*/
{
	int i;
	int nchar;
	int nchar1;
	int nchar2;
	
	nchar1 = strlen ( string1 );
	nchar2 = strlen ( string2 );
	
	if ( nchar1 < nchar2 ) {
		nchar = nchar1;
	}
	else {
		nchar = nchar2;
	}
	/*
	The strings are not equal if they differ over their common length.
	*/
	for ( i = 0; i < nchar; ++i ) {
		
		if ( toupper ( string1[i] ) != toupper ( string2[i] ) ) {
			return false;
		}
	}
	/*
	The strings are not equal if the longer one includes nonblanks
	in the tail.
	*/
	if ( nchar1 > nchar ) {
		for ( i = nchar; i < nchar1; ++i ) {
			if ( string1[i] != ' ' ) {
				return false;
			}
		} 
	}
	else if ( nchar2 > nchar ) {
		for ( i = nchar; i < nchar2; ++i ) {
			if ( string2[i] != ' ' ) {
				return false;
			}
		} 
	}
	return true;
}
/******************************************************************************/

long long_int_read ( FILE *filein )

/******************************************************************************/

/*
Purpose:

LONG_INT_READ reads a long from a binary file.

Modified:

24 May 1999

Author:

John Burkardt
*/
{
	union {
		long yint;
		char ychar[4];
	} y;
	
	if ( byte_swap  ) {
		y.ychar[3] = fgetc ( filein );
		y.ychar[2] = fgetc ( filein );
		y.ychar[1] = fgetc ( filein );
		y.ychar[0] = fgetc ( filein );
	}
	else {
		y.ychar[0] = fgetc ( filein );
		y.ychar[1] = fgetc ( filein );
		y.ychar[2] = fgetc ( filein );
		y.ychar[3] = fgetc ( filein );
	}
	
	return y.yint;
}
/******************************************************************************/

int long_int_write ( FILE *fileout, long int_val )

/******************************************************************************/

/*
Purpose:

LONG_INT_WRITE writes a long to a binary file.

Modified:

14 October 1998

Author:

John Burkardt
*/
{
	union {
		long yint;
		char ychar[4];
	} y;
	
	y.yint = int_val;
	
	if ( byte_swap  ) {
		fputc ( y.ychar[3], fileout );
		fputc ( y.ychar[2], fileout );
		fputc ( y.ychar[1], fileout );
		fputc ( y.ychar[0], fileout );
	}
	else {
		fputc ( y.ychar[0], fileout );
		fputc ( y.ychar[1], fileout );
		fputc ( y.ychar[2], fileout );
		fputc ( y.ychar[3], fileout );
	}
	
	return 4;
}

float rgb_to_hue ( float r, float g, float b )

/**********************************************************************/

/*
Purpose:

RGB_TO_HUE converts (R,G,B) colors to a hue value between 0 and 1.

Discussion:

The hue computed here should be the same as the H value computed
for HLS and HSV, except that it ranges from 0 to 1 instead of
0 to 360.

A monochromatic color ( white, black, or a shade of gray) does not
have a hue.  This routine will return a special value of H = -1
for such cases.

Examples:

Color    R    G    B     H

red  1.0  0.0  0.0   0.00
yellow   1.0  1.0  0.0   0.16
green    0.0  1.0  0.0   0.33
cyan     0.0  1.0  1.0   0.50
blue     0.0  0.0  1.0   0.67
magenta  1.0  0.0  1.0   0.83

black    0.0  0.0  0.0  -1.00
gray     0.5  0.5  0.5  -1.00
white    1.0  1.0  1.0  -1.00

Modified:

22 May 1999

Author:

John Burkardt

Parameters:

Input, float R, G, B, the red, green and blue values of the color.
These values should be between 0 and 1.

Output, float RGB_TO_HUE, the corresponding hue of the color, or -1.0 if
the color is monochromatic.
*/
{
	float h;
	float rgbmax;
	float rgbmin;
	/*
	Make sure the colors are between 0 and 1.
	*/
	if ( r < 0.0 ) {
		r = 0.0;
	}
	else if ( r > 1.0 ) {
		r = 1.0;
	}
	
	if ( g < 0.0 ) {
		g = 0.0;
	}
	else if ( g > 1.0 ) {
		g = 1.0;
	}
	
	if ( b < 0.0 ) {
		b = 0.0;
	}
	else if ( b > 1.0 ) {
		b = 1.0;
	}
	/*
	Compute the minimum and maximum of R, G and B.
	*/
	rgbmax = r;
	if ( g > rgbmax ) {
		rgbmax = g;
	}
	if ( b > rgbmax ) {
		rgbmax = b;
	}
	
	rgbmin = r;
	if ( g < rgbmin ) {
		rgbmin = g;
	}
	if ( b < rgbmin ) {
		rgbmin = b;
	}
	/*
	If RGBMAX = RGBMIN, { the color has no hue.
	*/
	if ( rgbmax == rgbmin ) {
		h = - 1.0;
	}
	/*
	Otherwise, we need to determine the dominant color.
	*/
	else {
		
		if ( r == rgbmax ) {
			h = ( g - b ) / ( rgbmax - rgbmin );
		}
		else if ( g == rgbmax ) {
			h = 2.0 + ( b - r ) / ( rgbmax - rgbmin );
		}
		else if ( b == rgbmax ) {
			h = 4.0 + ( r - g ) / ( rgbmax - rgbmin );
		}
		
		h = h / 6.0;
		/*
		Make sure H lies between 0 and 1.0.
		*/
		if ( h < 0.0 ) {
			h = h + 1.0;
		}
		else if ( h > 1.0 ) {
			h = h - 1.0;
		}
		
	}
	
	return h;
}
/******************************************************************************/

short int short_int_read ( FILE *filein )

/******************************************************************************/
/*
Purpose:

SHORT_INT_READ reads a short int from a binary file.

Modified:

14 October 1998

Author:

John Burkardt
*/
{
	unsigned char  c1;
	unsigned char  c2;
	short int      ival;
	
	c1 = fgetc ( filein );
	c2 = fgetc ( filein );
	
	ival = c1 | ( c2 << 8 );
	
	return ival;
}
/******************************************************************************/

int short_int_write ( FILE *fileout, short int short_int_val )

/******************************************************************************/

/*
Purpose:

SHORT_INT_WRITE writes a short int to a binary file.

Modified:

14 October 1998

Author:

  John Burkardt
  */
{
	union {
		short int yint;
		char ychar[2];
	} y;
	
	y.yint = short_int_val;
	
	if ( byte_swap  ) {
		fputc ( y.ychar[1], fileout );
		fputc ( y.ychar[0], fileout );
	}
	else {
		fputc ( y.ychar[0], fileout );
		fputc ( y.ychar[1], fileout );
	}
	
	return 2;
}

int ivec_max ( int n, const array<int>& a )

/******************************************************************************/

/*
Purpose:

IVEC_MAX returns the maximum element in an integer array.

Modified:

09 October 1998

Author:

John Burkardt
*/
{
	int  imax=0;
	
	for (int i=0; i<n; ++i)
		if (a[i]>imax)
			imax=a[i];
	return imax;
}

