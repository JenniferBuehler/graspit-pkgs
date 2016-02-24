// this file copied from the 3D Studio Max API

/* 3D object file defines */

#ifndef _OFILE_H_DEFINED

/* Error types for file loads */

#define INVALID_FILE 1
#define PARTIAL_READ 2
#define EXCEEDS_SETUP 3

#define MMAGIC 0x3D3D	/* Mesh magic */
#define SMAGIC 0x2D2D	/* Shaper magic */
#define LMAGIC 0x2D3D	/* Lofter magic */
#define MLIBMAGIC 0x3DAA	/* Material library magic */
#define MATMAGIC 0x3DFF	/* Material magic */

#define MESH_VERSION 0x3D3E  /* version chunk */

#define COLOR_F 0x0010
#define COLOR_24 0x0011
#define LIN_COLOR_24 0x0012
#define LIN_COLOR_F 0x0013
#define INT_PERCENTAGE 0x0030
#define FLOAT_PERCENTAGE 0x0031

#define MASTER_SCALE 0x0100

#define BIT_MAP 0x1100
#define USE_BIT_MAP 0x1101
#define SOLID_BGND 0x1200
#define USE_SOLID_BGND 0x1201
#define V_GRADIENT 0x1300
#define USE_V_GRADIENT 0x1301

#define LO_SHADOW_BIAS 0x1400
#define HI_SHADOW_BIAS 0x1410
#define SHADOW_MAP_SIZE 0x1420
#define SHADOW_SAMPLES 0x1430
#define SHADOW_RANGE 0x1440
#define SHADOW_FILTER 0x1450
#define RAY_BIAS 0x1460
#define RAY_SHADOWS 0x1470

#define O_CONSTS 0x1500
 
#define AMBIENT_LIGHT 0x2100

#define FOG 0x2200
#define USE_FOG 0x2201
#define FOG_BGND 0x2210
#define DISTANCE_CUE 0x2300
#define USE_DISTANCE_CUE 0x2301
#define LAYER_FOG 0x2302
#define USE_LAYER_FOG 0x2303
#define DCUE_BGND 0x2310

#define DEFAULT_VIEW 0x3000
#define VIEW_TOP 0x3010
#define VIEW_BOTTOM 0x3020
#define VIEW_LEFT 0x3030
#define VIEW_RIGHT 0x3040
#define VIEW_FRONT 0x3050
#define VIEW_BACK 0x3060
#define VIEW_USER 0x3070
#define VIEW_CAMERA 0x3080
#define VIEW_WINDOW 0x3090

#define NAMED_OBJECT 0x4000
#define OBJ_HIDDEN 0x4010
#define OBJ_VIS_LOFTER 0x4011
#define OBJ_DOESNT_CAST 0x4012
#define OBJ_MATTE 0x4013
#define OBJ_FAST 0x4014
#define OBJ_PROCEDURAL 0x4015
#define OBJ_FROZEN 0x4016
#define OBJ_DONT_RCVSHADOW  0x4017

#define N_TRI_OBJECT 0x4100

#define POINT_ARRAY 0x4110
#define POINT_FLAG_ARRAY 0x4111
#define FACE_ARRAY 0x4120
#define MSH_MAT_GROUP 0x4130
#define OLD_MAT_GROUP 0x4131
#define TEX_VERTS 0x4140
#define SMOOTH_GROUP 0x4150
#define MESH_MATRIX 0x4160
#define MESH_COLOR 0x4165
#define MESH_TEXTURE_INFO 0x4170
#define PROC_NAME 0x4181
#define PROC_DATA 0x4182
#define MSH_BOXMAP 0x4190

#define N_D_L_OLD 0x4400

#define N_CAM_OLD 0x4500

#define N_DIRECT_LIGHT 0x4600
#define DL_SPOTLIGHT 0x4610
#define DL_OFF 0x4620
#define DL_ATTENUATE 0x4625
#define DL_RAYSHAD 0x4627
#define DL_SHADOWED 0x4630
#define DL_LOCAL_SHADOW 0x4640
#define DL_LOCAL_SHADOW2 0x4641
#define DL_SEE_CONE 0x4650
#define DL_SPOT_RECTANGULAR 0x4651
#define DL_SPOT_OVERSHOOT 0x4652
#define DL_SPOT_PROJECTOR 0x4653
#define DL_EXCLUDE 0x4654
#define DL_RANGE 0x4655	/* Old style chunk -- don't write */
#define DL_SPOT_ROLL 0x4656
#define DL_SPOT_ASPECT 0x4657
#define DL_RAY_BIAS 0x4658
#define DL_INNER_RANGE 0x4659
#define DL_OUTER_RANGE 0x465A
#define DL_MULTIPLIER 0x465B

#define N_AMBIENT_LIGHT 0x4680

#define N_CAMERA 0x4700
#define CAM_SEE_CONE 0x4710
#define CAM_RANGES 0x4720

#define HIERARCHY 0x4F00
#define PARENT_OBJECT 0x4F10
#define PIVOT_OBJECT 0x4F20
#define PIVOT_LIMITS 0x4F30
#define PIVOT_ORDER 0x4F40
#define XLATE_RANGE 0x4F50

#define POLY_2D 0x5000

/* Flags in shaper file that tell whether polys make up an ok shape */

#define SHAPE_OK 0x5010
#define SHAPE_NOT_OK 0x5011

#define SHAPE_HOOK 0x5020

#define PATH_3D 0x6000
#define PATH_MATRIX 0x6005
#define SHAPE_2D 0x6010
#define M_SCALE 0x6020
#define M_TWIST 0x6030
#define M_TEETER 0x6040
#define M_FIT 0x6050
#define M_BEVEL 0x6060
#define XZ_CURVE 0x6070
#define YZ_CURVE 0x6080
#define INTERPCT 0x6090
#define DEFORM_LIMIT 0x60A0

/* Flags for Modeler options */

#define USE_CONTOUR 0x6100
#define USE_TWEEN 0x6110
#define USE_SCALE 0x6120
#define USE_TWIST 0x6130
#define USE_TEETER 0x6140
#define USE_FIT 0x6150
#define USE_BEVEL 0x6160

/* Viewport description chunks */

#define VIEWPORT_LAYOUT_OLD 0x7000
#define VIEWPORT_DATA_OLD 0x7010
#define VIEWPORT_LAYOUT 0x7001
#define VIEWPORT_DATA 0x7011
#define VIEWPORT_DATA_3 0x7012	/* Version 3.0 -- not backward-compatible */
#define VIEWPORT_SIZE 0x7020
#define NETWORK_VIEW 0x7030

/* Material file chunk IDs */

#define	MAT_ENTRY		0xAFFF
#define MAT_NAME		0xA000
#define MAT_AMBIENT		0xA010
#define MAT_DIFFUSE		0xA020
#define MAT_SPECULAR		0xA030
#define MAT_SHININESS		0xA040
#define MAT_SHIN2PCT		0xA041
#define MAT_SHIN3PCT		0xA042
#define MAT_TRANSPARENCY	0xA050
#define MAT_XPFALL	0xA052
#define MAT_REFBLUR	0xA053

#define MAT_SELF_ILLUM		0xA080
#define MAT_TWO_SIDE		0xA081
#define MAT_DECAL		0xA082
#define MAT_ADDITIVE		0xA083
#define MAT_SELF_ILPCT	0xA084
#define MAT_WIRE			0xA085
#define MAT_SUPERSMP		0xA086
#define MAT_WIRESIZE		0xA087
#define MAT_FACEMAP		0xA088
#define MAT_XPFALLIN		0xA08A
#define MAT_PHONGSOFT	0xA08C
#define MAT_WIREABS  	0xA08E

#define MAT_SHADING		0xA100

#define MAT_TEXMAP		0xA200
#define MAT_OPACMAP		0xA210
#define MAT_BUMPMAP		0xA230
#define MAT_SPECMAP		0xA204
#define MAT_REFLMAP		0xA220
#define MAT_USE_XPFALL	0xA240
#define MAT_USE_REFBLUR	0xA250
#define MAT_BUMP_PERCENT 0xA252  /* new scaled up bump value */

#define MAT_MAPNAME		0xA300
#define MAT_ACUBIC		0xA310

#define MAT_SXP_TEXT_DATA 0xA320
#define MAT_SXP_TEXT2_DATA 0xA321
#define MAT_SXP_OPAC_DATA 0xA322
#define MAT_SXP_BUMP_DATA 0xA324
#define MAT_SXP_SPEC_DATA 0xA325
#define MAT_SXP_SHIN_DATA 0xA326
#define MAT_SXP_SELFI_DATA 0xA328
#define MAT_SXP_TEXT_MASKDATA 0xA32A
#define MAT_SXP_TEXT2_MASKDATA 0xA32C
#define MAT_SXP_OPAC_MASKDATA	0xA32E
#define MAT_SXP_BUMP_MASKDATA	0xA330
#define MAT_SXP_SPEC_MASKDATA	0xA332
#define MAT_SXP_SHIN_MASKDATA	0xA334
#define MAT_SXP_SELFI_MASKDATA 0xA336
#define MAT_SXP_REFL_MASKDATA	 0xA338
#define MAT_TEX2MAP 	0xA33A
#define MAT_SHINMAP 	0xA33C
#define MAT_SELFIMAP 0xA33D
#define MAT_TEXMASK 0xA33E
#define MAT_TEX2MASK 0xA340
#define MAT_OPACMASK 0xA342
#define MAT_BUMPMASK 0xA344
#define MAT_SHINMASK 0xA346
#define MAT_SPECMASK 0xA348
#define MAT_SELFIMASK 0xA34A
#define MAT_REFLMASK 0xA34C
#define MAT_MAP_TILINGOLD 0xA350
#define MAT_MAP_TILING 0xA351
#define MAT_MAP_TEXBLUR_OLD 0xA352
#define MAT_MAP_TEXBLUR 0xA353
#define MAT_MAP_USCALE 0xA354
#define MAT_MAP_VSCALE 0xA356
#define MAT_MAP_UOFFSET 0xA358
#define MAT_MAP_VOFFSET 0xA35A
#define MAT_MAP_ANG 0xA35C
#define MAT_MAP_COL1 0xA360
#define MAT_MAP_COL2 0xA362
#define MAT_MAP_RCOL 0xA364 
#define MAT_MAP_GCOL 0xA366 
#define MAT_MAP_BCOL 0xA368 

/* NOTE Numbers 0x8000 through 0x8FFF are reserved for external
   applications to include in APP_DATA chunks */
#define APP_DATA 0x8000

/* Dummy chunk ID */

#define DUMMY 0xFFFF



#endif

#define _OFILE_H_DEFINED

