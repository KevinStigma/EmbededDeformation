#ifndef _MY_MACRO_
#define _MY_MACRO_

#ifndef SAFE_DELETE
#define SAFE_DELETE(ptr)
#endif
//{{AFX_CODEJOCK_PRIVATE
#undef SAFE_DELETE
#define SAFE_DELETE(ptr) \
	if (ptr) { delete ptr; ptr = NULL; }

#define SAFE_VDELETE(ptr) \
	if (ptr) { delete []ptr; ptr = NULL; }

#define CLAMP(x, min, max) ((x)<(min) ? (min) :((x)>(max) ? (max) : (x)))

#ifndef M_PI
#define M_PI 3.1415926535f
#endif

#define SWAP(a,b) {a=a+b;b=a-b;a=a-b;}


#define DEGREE2RADIAN(degree) (degree*M_PI/180.f)
#define RADIAN2DEGREE(radian) (radian*180.f/M_PI)

#define M_EPSILON  0.00000001f

#undef M_2_PI
#define M_2_PI 0.63661977236758134307607071493546	//		2/PI

#undef M_PI_2
#define M_PI_2 1.57079632679489661923				//		PI/2	

#define PHOTO_SIZE 512
#define SHORT_LINE_DIS2_THRESHOLD 1
#define CONS_CIRCLE 0.5522847498
#define CONS_ELLIPSE 0.2761423749154
#define SHORT_PATH_LEN2_THRESHOLD 0.01
#define SHORT_POLYLINE_LEN2_THRESHOLD 25
#define SIFT_FEATURE_DIAMETER 10.0
#endif