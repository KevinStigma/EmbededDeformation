#ifndef _MAT_UTIL_H
#define _MAT_UTIL_H

#include "MyType.h"

namespace MyUtil
{
	MAT3D mat4d_2_mat3d(const MAT4D &mat4f);
	MAT4D rot_only(const MAT4D &mat4f);
	MAT4D mat3d_2_mat4d(const MAT3D &mat3f);
	MAT3D mat2d_2_mat3d(const MAT2D &mat2d);
	void mat4d_2_array2(const MAT4D &mat4f, double ret[4][4]);
	MAT4D inv_rigidTransMat(const MAT4D &trans);
	MAT4D formTransMat(const MAT3D &a, const VEC3D &rot_center);
	void mat4d_2_openglMat(const MAT4D &trans, double gl_mat[16]);

	void homo_normalize(VEC3D &pt);
	void homo_normalize(VEC4D &pt);
}

#endif