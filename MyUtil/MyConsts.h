#ifndef _MY_CONST_
#define _MY_CONST_

#include <string>
#include "MyMacro.h"
#include <Eigen\Dense>

const std::string BASIC_DRAWING_NAME = "basic_drawing";
const std::string DRAWING_NAME = "drawing";
const std::string SUBPAGE_NAME = "subpage";

const std::string PART_DRAWING_NAME = "part_drawing";
const std::string SCREW_DRAWING_NAME = "screw_drawing";
const std::string HIGHLIGHT_DRAWING_NAME = "highlight_drawing";
const std::string NUMBER_DRAWING_NAME = "number_drawing";
const std::string ARROW_DRAWING_NAME = "arrow_drawing";

const std::string PROXY_NAME = "proxy";
const std::string CUBE_PROXY_NAME = "cube_proxy";
const std::string EXTRUDE_PROXY_NAME = "extrude_proxy";
const std::string UNKNOWN_NAME = "unknown_obj";
const std::string X_NAME = "X";
const std::string Y_NAME = "Y";
const std::string Z_NAME = "Z";
const std::string CAM_NAME = "Camera";
const std::string PROXY_CAM_NAME = "ProxyCamera";

const std::string PROXY_RELATION_NAME = "relation";
const std::string AXIS_PARALLEL_NAME = "axis_parallel";
const std::string SAME_SIZE_NAME = "same_size";
const std::string HOLE_COUPLE_NAME = "hole_couple";
const std::string FACE_TOUCH = "face_touch";
const std::string JOINT_ON_FACE = "joint_on_face";
const std::string TWO_CONNECT_BY_ONE_NAME = "two_connect_by_one";
const std::string COPLANAR_NAME="coplanar";

const int VISIBLE_BIT =  0x1;
const float c_DEGREE_2_RADIAN = (float)M_PI / 180.f;
const float c_RADIAN_2_DEGREE = 180.f / (float)M_PI;

enum DRAWING_LABEL {UNKNOWN, PART, ARROW, SCREW, HIGHLIGHT, NUMBER};
enum PROXY_CREATE_MODE {CUBOID, CYLINDER};

const int LABEL_COLOR_TABLE_LEN = 6;
const double LABEL_COLOR_TABLE[][3] = {
	0 / 255.0f, 0 / 255.0f, 0 / 255.0f,
	255 / 255.0f, 0 / 255.0f, 0 / 255.0f,
	0 / 255.0f, 255 / 255.0f, 0 / 255.0f,
	0 / 255.0f, 0 / 255.0f, 255 / 255.0f,
	168 / 255.0f, 218 / 255.0f, 16 / 255.0f,
	255 / 255.0f, 0 / 255.0f, 255 / 255.0f
};

const Eigen::Vector3f CUBE_P8[8] = {
	Eigen::Vector3f(1, -1, 0),
	Eigen::Vector3f(1, 1, 0),
	Eigen::Vector3f(-1, 1, 0),
	Eigen::Vector3f(-1, -1, 0),
	Eigen::Vector3f(1, -1, 1),
	Eigen::Vector3f(1, 1, 1),
	Eigen::Vector3f(-1, 1, 1),
	Eigen::Vector3f(-1, -1, 1)
};

const int HEX_INDICES[6] = {4,0,1,2,6,7};

const float CUBE_P6[6][3] = {
	{1,-1,1},
	{1,-1,0},
	{1, 1,0},
	{-1,1,0},
	{-1,1,1},
	{-1,-1,1}
};

#define PROXY_VAR_NUM 9
enum PROXY_VAR {DELTA_CX, DELTA_CY, DELTA_CZ, EULER_X, EULER_Y, EULER_Z, 
	SX, SY, SZ};

#endif
