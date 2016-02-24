#ifndef _ivcon_tmat
#define _ivcon_tmap

void tmat_init ( float a[4][4] );
void tmat_mxm ( float a[4][4], float b[4][4], float c[4][4] );
void tmat_mxp ( float a[4][4], float x[4], float y[4] );
void tmat_mxp2 ( float a[4][4], float x[][3], float y[][3], int n );
void tmat_mxv ( float a[4][4], float x[4], float y[4] );
bool tmat_rot_axis ( float a[4][4], float b[4][4], float angle,  char axis );
void tmat_rot_vector ( float a[4][4], float b[4][4], float angle, float v1, float v2, float v3 );
void tmat_scale ( float a[4][4], float b[4][4], float sx, float sy, float sz );
bool tmat_shear ( float a[4][4], float b[4][4], char *axis, float s );
void tmat_trans ( float a[4][4], float b[4][4], float x, float y, float z );

#endif