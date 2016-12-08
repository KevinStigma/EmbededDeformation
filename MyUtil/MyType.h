#ifndef MY_TYPE
#define MY_TYPE

#include "Wm4Vector3.h"
#include "Wm4Vector4.h"
#include "Wm4Matrix4.h"
#include "Wm4Matrix3.h"
#include "Wm4Matrix2.h"
#include "Wm4Quaternion.h"
#include "Wm4Vector2.h"
#include "Wm4BSplineCurveFit.h"
#include "Wm4Segment2.h"
#include "Wm4Line2.h"
#include "Wm4Line3.h"
#include "Wm4Triangle3.h"
#include "Wm4IntrLine2Line2.h"
#include "Wm4IntrSegment3Plane3.h"
#include "Wm4IntrPlane3Plane3.h"
#include "Wm4IntrSegment2Segment2.h"
#include "Wm4IntrBox3Box3.h"
#include "Wm4Circle3.h"
#include "Wm4GMatrix.h"
#include "Wm4IntrTriangle3Triangle3.h"
#include "Wm4TriangulateEC.h"

typedef Wm4::Vector3d VEC3D;
typedef Wm4::Vector3f VEC3F;
typedef Wm4::Vector4d VEC4D;
typedef Wm4::Vector4f VEC4F;
typedef Wm4::Vector4<int> VEC4I;
typedef Wm4::Matrix4f MAT4F;
typedef Wm4::Matrix4d MAT4D;
typedef Wm4::Matrix2d MAT2D;
typedef Wm4::Quaternionf QUATF;
typedef Wm4::Quaterniond QUATD;
typedef Wm4::Matrix3f MAT3F;
typedef Wm4::Matrix3d MAT3D;
typedef Wm4::Matrix2d MAT2D;
typedef Wm4::Vector2f VEC2F;
typedef Wm4::Vector2d VEC2D;
typedef Wm4::Vector2<int> VEC2I;
typedef Wm4::BSplineCurveFitf BSPLINECURVEFITF;
typedef Wm4::Line3f LINE3F;
typedef Wm4::Line3d LINE3D;
typedef Wm4::Triangle3f TRIANGLE3F;
typedef Wm4::Quaternionf QUATF;
typedef Wm4::Line2<float> LINE2F;
typedef Wm4::Line2<double> LINE2D;
typedef Wm4::Segment2<float> SEG2F;
typedef Wm4::Segment2<double> SEG2D;
typedef Wm4::Plane3<float> PLANE3F;
typedef Wm4::Segment3<float> SEG3F;
typedef Wm4::Segment3<double> SEG3D;
typedef Wm4::IntrLine2Line2<float> INTR_LINE2LINE2_F;
typedef Wm4::IntrLine2Line2<double> INTR_LINE2LINE2_D;
typedef Wm4::IntrSegment3Plane3<float> INTR_SEG3LINE3_F; 
typedef Wm4::IntrPlane3Plane3<float> INTR_PLANE3PLANE3_F;
typedef Wm4::IntrSegment2Segment2<float> INTR_SEG2SEG2_F;
typedef Wm4::IntrSegment2Segment2<double> INTR_SEG2SEG2_D;
typedef Wm4::IntrBox3Box3<float> INTR_BOX3BOX3_F;
typedef Wm4::Box3<float> BOX3_F;
typedef Wm4::Box3<double> BOX3_D;
typedef Wm4::Circle3<double> CIRCLE3_D;
typedef Wm4::GMatrix<double> GMATD;
typedef Wm4::GVector<double> GVECD;
typedef Wm4::IntrTriangle3Triangle3d INTRO_TRI3TRI3_D;
typedef Wm4::Triangle3d TRIANGLE3D;
typedef Wm4::TriangulateEC<float> TESS_EC_F;

typedef struct tag_CUBE_CORNERS
{
	VEC3D _pts[8];
}CUBE_CORNERS;


#endif