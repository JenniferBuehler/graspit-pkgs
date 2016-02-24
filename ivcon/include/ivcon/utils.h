#ifndef _ivcon_utils
#define _ivcon_utils

#include <stdio.h>
#include <vector>
#include <valarray>

template <typename T>
T max(const T& a, const T& b) {return a>b?a:b;}

class vec2:public std::valarray<float>
{
typedef std::valarray<float> _parent;
public:
	vec2():_parent(0.0,2) {};	
	vec2(const float* buf):_parent(buf,2) {};
	vec2(const float x, const float y):_parent(2) 
		{ operator[](0)=x; operator[](1)=y;}			
};

class vec3:public std::valarray<float>
{
typedef std::valarray<float> _parent;
public:
	vec3():_parent(0.0,3) {};	
	vec3(const float* buf):_parent(buf,3) {};
	vec3(const float x, const float y, const float z):_parent(3) 
		{ operator[](0)=x; operator[](1)=y; operator[](2)=z;}			
};


class vec4:public std::valarray<float>
{
typedef std::valarray<float> _parent;
public:
	vec4():_parent(0.0,4) {};	
	vec4(const float* buf):_parent(buf,4) {};
	vec4(const float x, const float y, const float z, const float t):_parent(4) 
		{ operator[](0)=x; operator[](1)=y; operator[](2)=z; operator[](3)=t;}			
};

template <typename T>	// a vector with automatic growth
class array:public std::vector<T>
{
typedef std::vector<T> _parent;
typedef typename std::vector<T>::size_type size_type;
public:
	array(const size_type size=0):_parent(size) {};
	T& operator[](size_type i)
	{
		if (i>=this->size()) 
		{
			if (i>=this->capacity())
				this->reserve(i*2);
			this->resize(i+1);
		}
		return _parent::operator[](i);
	}
	const T& operator[](size_type i) const
	{
		return _parent::operator[](i);
	}
};

extern bool byte_swap;

char *file_ext ( char *file_name );
float float_read ( FILE *filein );
float float_reverse_bytes ( float x );
int float_write ( FILE *fileout, float float_val );
long long_int_read ( FILE *filein );
int long_int_write ( FILE *fileout, long int_val );
float rgb_to_hue ( float r, float g, float b );
short int short_int_read ( FILE *filein );
int short_int_write ( FILE *fileout, short int int_val );

int rcol_find ( const array<vec3>& a, int n, vec3 ) ;

int char_index_last ( char* string, char c );
bool char_pad ( int *char_index, int *null_index, char *string, int STRING_MAX );
char char_read ( FILE *filein );
int char_write ( FILE *fileout, char c );

float float_read ( FILE *filein );

bool leqi ( const char* string1, const char* string2 );

int ivec_max ( int n, const array<int>& a );

#endif
