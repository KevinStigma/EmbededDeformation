#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <queue>
#include <set>
#include <Eigen/geometry>
#include <Eigen/dense>
#include <eigen\Core>
#include <eigen\Svd>
#include <eigen\Sparse>

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

typedef Vector3d Vec3;
typedef Vector4d Vec4;
typedef Matrix3d Mat3;
typedef Matrix4d Mat4;

typedef Eigen::Matrix<double,-1,1> Vector;
typedef Eigen::SparseMatrix<double, Eigen::ColMajor> SpMat;
enum KEYSTATUS{KEY_NONE,KEY_SELECT_POINT,KEY_DELETE_POINT,KEY_SEL_NODE,KEY_SELECT_BOX,KEY_DRAG_BOX,KEY_SEL_BOXPAIR};

struct Vec2
{
	double x,y;
	Vec2 (double a=0,double b=0):x(a),y(b){}
	double norm()
	{
		return sqrt(x*x+y*y);
	}
	void normalize()
	{
		double length=norm();
		x/=length;
		y/=length;

	}
	Vec2 normalized()
	{
		double length=norm();
		return Vec2(x/length,y/length);
	}
	bool operator ==(const Vec2& a)
	{
		bool is;
		((x==a.x)&&(y==a.y))?is=true:is=false;
		return is;
	}
	Vec2 operator +(const Vec2&a)
	{
		return Vec2(x+a.x,y+a.y);
	}
	Vec2 operator -(const Vec2&a)
	{
		return Vec2(x-a.x,y-a.y);
	}
	Vec2 operator *=(const double& v)
	{
		return Vec2(x*v,y*v);
	}
	Vec2 operator *(double a)
	{
		return Vec2(x*a,y*a);
	}
	Vec2 operator /(double a)
	{
		return Vec2(x/a,y/a);
	}
	Vec2 operator-()
	{
		return Vec2(-x,-y);
	}
	double dot(const Vec2&a)
	{
		return x*a.x+y*a.y;
	}
	double cross(const Vec2&a)
	{
		return x*a.y-a.x*y;
	}

	Vec2 rotate(double angle) //radians,clockwise 
	{
		angle=-angle;
		double sinphi=sin(angle);
		double cosphi=cos(angle);

		return Vec2(cosphi*x-sinphi*y,sinphi*x+cosphi*y);
	}


	static Vec2 getPerpendicular(Vec2 v)
	{
		double cos90=0;
		double sin90=1;
		return Vec2(cos90*v.x-sin90*v.y,sin90*v.x+cos90*v.y);
	}
};

struct INT3
{
	size_t x;
	size_t y;
	size_t z;
	INT3(size_t x1=0,size_t y1=0,size_t z1=0):x(x1),y(y1),z(z1){}
};

struct Light
{
	float ambient[4]; 	
	float diffuse[4];		
	float specular[4];	
	float position[4];
};

struct Material
{
	float Ambient[4];
	float Diffuse[4];
	float Specular[4]; // w = SpecPower
	float Reflect[4];
	float Emission[4];
};

struct BoundingBox
{
	Vec3 bot_pos;
	Vec3 top_pos;
	Vec3 init_pos;
	int proxy_id;
	int bp_id;
	double XL,YL,ZL;
	std::vector<int> vertex_list;
	BoundingBox(Vec3 b_pos=Vec3(0,0,0),Vec3 t_pos=Vec3(0,0,0)):bot_pos(b_pos),top_pos(t_pos)
	{
		XL=top_pos(0)-bot_pos(0);
		YL=top_pos(1)-bot_pos(1);
		ZL=top_pos(2)-bot_pos(2);
		init_pos=(top_pos+bot_pos)/2;
		proxy_id=-1;
		bp_id=-1;
	}
};

#define VecList(T)  std::vector<std::vector<T>>
#define VecSet(T)  std::vector<std::set<T>>

#define SAFE_DELETE(ptr) \
	if (ptr) { delete ptr; ptr = NULL; }

#endif