#include "stdafx.h"
#include "MatUtil.h"

namespace MyUtil
{

	MAT3F mat4f_2_mat3f(const MAT4F &mat4f)
	{
		MAT3F ret;
		ret(0,0) = mat4f(0,0);	ret(0,1) = mat4f(0,1); ret(0,2) = mat4f(0,2);
		ret(1,0) = mat4f(1,0);	ret(1,1) = mat4f(1,1); ret(1,2) = mat4f(1,2);
		ret(2,0) = mat4f(2,0);	ret(2,1) = mat4f(2,1); ret(2,2) = mat4f(2,2);
		return ret;
	}

	MAT4F rot_only(const MAT4F &mat4f)
	{
		MAT4F ret = mat4f;
		ret(0,3) = ret(1,3) = ret(2,3) = 0.f;
		ret(3,0) = ret(3,1) = ret(3,2) = 0.f;
		ret(3,3) = 1.f;
		return ret;
	}

	MAT4F build_mat4f(const MAT3F &mat3f)
	{
		MAT4F ret = MAT4F::IDENTITY;
		ret(0,0) = mat3f(0,0);	ret(0,1) = mat3f(0,1); ret(0,2) = mat3f(0,2);
		ret(1,0) = mat3f(1,0);	ret(1,1) = mat3f(1,1); ret(1,2) = mat3f(1,2);
		ret(2,0) = mat3f(2,0);	ret(2,1) = mat3f(2,1); ret(2,2) = mat3f(2,2);
		return ret;
	}

	MAT3D mat4d_2_mat3d(const MAT4D &mat4f)
	{
		MAT3D ret;
		ret(0,0) = mat4f(0,0);	ret(0,1) = mat4f(0,1); ret(0,2) = mat4f(0,2);
		ret(1,0) = mat4f(1,0);	ret(1,1) = mat4f(1,1); ret(1,2) = mat4f(1,2);
		ret(2,0) = mat4f(2,0);	ret(2,1) = mat4f(2,1); ret(2,2) = mat4f(2,2);
		return ret;
	}

	MAT4D mat3d_2_mat4d(const MAT3D &mat3d)
	{
		MAT4D ret = MAT4D::IDENTITY;
		ret(0,0) = mat3d(0,0);	ret(0,1) = mat3d(0,1); ret(0,2) = mat3d(0,2);
		ret(1,0) = mat3d(1,0);	ret(1,1) = mat3d(1,1); ret(1,2) = mat3d(1,2);
		ret(2,0) = mat3d(2,0);	ret(2,1) = mat3d(2,1); ret(2,2) = mat3d(2,2);
		return ret;
	}

	MAT3D mat2d_2_mat3d(const MAT2D &mat2d)
	{
		MAT3D ret = MAT3D::IDENTITY;
		ret(0,0) = mat2d(0,0);	ret(0,1) = mat2d(0,1); 
		ret(1,0) = mat2d(1,0);	ret(1,1) = mat2d(1,1); 
		return ret;
	}

	//a(x-c)+c
	MAT4D formTransMat(const MAT3D &a, const VEC3D &rot_center)
	{
		MAT4D ret = MAT4D::IDENTITY;
		ret(0,0) = a(0,0);	ret(0,1) = a(0,1); ret(0,2) = a(0,2);
		ret(1,0) = a(1,0);	ret(1,1) = a(1,1); ret(1,2) = a(1,2);
		ret(2,0) = a(2,0);	ret(2,1) = a(2,1); ret(2,2) = a(2,2);
		VEC3D t = rot_center - a*rot_center;
		ret(0,3) = t[0];	ret(1,3) = t[1];	ret(2,3) = t[2];
		return ret;
	}


	MAT4D rot_only(const MAT4D &mat4f)
	{
		MAT4D ret = mat4f;
		ret(0,3) = ret(1,3) = ret(2,3) = 0.f;
		ret(3,0) = ret(3,1) = ret(3,2) = 0.f;
		ret(3,3) = 1.f;
		return ret;
	}

	MAT4D build_mat4f(const MAT3D &mat3f)
	{
		MAT4D ret = MAT4D::IDENTITY;
		ret(0,0) = mat3f(0,0);	ret(0,1) = mat3f(0,1); ret(0,2) = mat3f(0,2);
		ret(1,0) = mat3f(1,0);	ret(1,1) = mat3f(1,1); ret(1,2) = mat3f(1,2);
		ret(2,0) = mat3f(2,0);	ret(2,1) = mat3f(2,1); ret(2,2) = mat3f(2,2);
		return ret;
	}

	void homo_normalize(VEC3D &pt)
	{
		if (fabs(pt[2])>1e-8f)
		{
			double inv2 = 1.0/pt[2];
			pt[0] *= inv2;
			pt[1] *= inv2;
			pt[2] = 1.0;
		}
	}

	void homo_normalize(VEC4D &pt)
	{
		if (fabs(pt[3])>1e-8f)
		{
			double inv3 = 1.0/pt[3];
			pt[0] *= inv3;
			pt[1] *= inv3;
			pt[2] *= inv3;
			pt[3] = 1.0;
		}
	}
}