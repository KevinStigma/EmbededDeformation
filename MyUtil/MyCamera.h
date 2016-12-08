#ifndef _MY_CAMERA_
#define _MY_CAMERA_

#include "MyType.h"
#include "MyUtil.h"
#include "MyPara.h"

class CMyCamera
{
public:
	CMyCamera();
	CMyCamera(const CMyCamera &rhs);
	CMyCamera &operator=(const CMyCamera &rhs);
	void computeParameters();
	void GetGLMatrices(double glprojmatrix[], 
		double glmodelviewmatrix[], 
		const int glvpmatrix[],double znear, double zfar) const;
	void fromOpenGL(double znear,double fovy,const int glvp[], const double glmv[]);
	void updateRTFromOpenGL(const double glmv[]);
	VEC2D project(const VEC3D &spacept) const;
	VEC3D project_homo(const VEC3D &spacept) const;
	VEC3D ComputePointGround3dPos(const VEC2D &imgpos) const;
	VEC3D Compute3DCoordinate(const VEC2D &impos, const VEC2D &groundrefpos) const;
	bool calcCam(const VEC3F vanishPts[3],const std::vector<std::vector<SEG2F>> &vvSegCluster,
				const int vp[4], CAM_CALIBERATE_MODE mode);
	VEC4D intersect(const VEC4D &coord, const VEC4D& abcd) const;
	double computeDepth(const VEC4D& point) const ;
	VEC4D unproject(const VEC3D& icoord) const;
	VEC3D worldpt_2_eyept(const VEC3D &world_pt) const;
	double getF() const;
	void getCameraK(double k[9]);
public:
	virtual bool save(FILE *fp) const;
	virtual bool load(FILE *fp);
protected:
	//old, optimize camera based on the minimization of the variance of square_f separately from xy, yz and xz vanishing points
	//bool optimizeCam(const VEC3F vanishPts[3],const std::vector<std::vector<SEG2F>> &vvSegCluster, const int vp[4]);
	VEC4D CMyCamera::getOpticalCenter(void) const;
	bool calcCamIntrinsics(const VEC3F vanishPts[3],const int vp[4]);
	bool calcCamExtrinsics(const VEC3F vanishPts[3],const std::vector<std::vector<SEG2F>> &vvSegCluster,
		const int vp[4]);
public:
	MAT3D m_cameraK;
	MAT3D m_cameraR;
	VEC3D m_cameraT; 
	int m_width;
	int m_height;
public:
	MAT3D m_KR;
	VEC3D m_KT;
	MAT3D m_homographymatrix;	//to ground
	VEC3D m_xvanishingpoint;
	VEC3D m_yvanishingpoint;
	VEC3D m_zvanishingpoint;
	VEC3D m_origin;
	double m_xprojectionscale;
	double m_yprojectionscale;
	double m_zprojectionscale;
	//optical center
	VEC4D m_center;
	//optical axis
	VEC4D m_oaxis;

	//bool m_yInverse;
};

float getCamFVar(const VEC3F vanishPts[3],const int glvp[4]);
double calcF(const VEC3F &vanishPt1, const VEC3F &vanishPt2, float w, float h);

#endif