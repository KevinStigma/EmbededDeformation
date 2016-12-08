#include "stdafx.h"
#include "MyCamera.h"
#include "MyUtil.h"
#include "MyMacro.h"
#include "MatUtil.h"
#define XY_ONLY 0

CMyCamera::CMyCamera()
{
	m_width = m_height = 0;
	//m_yInverse = false;
}

void CMyCamera::computeParameters()
{
	m_KR= m_cameraK * m_cameraR;
	m_KT = m_cameraK * m_cameraT;
	
	VEC2D origin_img = project(VEC3D::ZERO);
	m_origin = VEC3D(origin_img[0],origin_img[1],1);

	VEC3D KR_1(m_KR(0, 0), m_KR(1, 0), m_KR(2, 0));
	VEC3D KR_2(m_KR(0, 1), m_KR(1, 1), m_KR(2, 1));
	m_homographymatrix = MAT3D(KR_1, KR_2, m_KT,true);
	m_homographymatrix = m_homographymatrix.Inverse();

	VEC3D xdir_proj = m_KR * VEC3D(1,0,0);
	MyUtil::homo_normalize(xdir_proj);
	m_xvanishingpoint[0] = xdir_proj[0];	m_xvanishingpoint[1] = xdir_proj[1];	m_xvanishingpoint[2] = 1.0;

	VEC3D ydir_proj = m_KR * VEC3D(0,1,0);
	MyUtil::homo_normalize(ydir_proj);
	m_yvanishingpoint[0] = ydir_proj[0];	m_yvanishingpoint[1] = ydir_proj[1];	m_yvanishingpoint[2] = 1.0;

	VEC3D zdir_proj = m_KR * VEC3D(0,0,1);
	MyUtil::homo_normalize(zdir_proj);
	m_zvanishingpoint[0] = zdir_proj[0];	m_zvanishingpoint[1] = zdir_proj[1];	m_zvanishingpoint[2] = 1.0;

	MAT3D X(m_xvanishingpoint, m_yvanishingpoint, m_zvanishingpoint,true);
	MAT3D I = X.Inverse() * m_KR;
	double alpha = m_KT.Dot(m_origin)/m_origin.SquaredLength();
	m_xprojectionscale = I(0, 0) / alpha;
	m_yprojectionscale = I(1, 1) / alpha;
	m_zprojectionscale = I(2, 2) / alpha;

	//optical center
	m_center = getOpticalCenter();
	//optical axis
	m_oaxis[0] = m_KR(2,0);	m_oaxis[1] = m_KR(2,1);	m_oaxis[2] = m_KR(2,2); m_oaxis[3] = 0;
	const double ftmp = m_oaxis.Length();
	if (fabs(ftmp)<1e-10f)
	{
		m_oaxis = VEC4D::ZERO;
	}
	m_oaxis[3] = m_KT[2];;
	m_oaxis /= ftmp;
}

VEC3D CMyCamera::ComputePointGround3dPos(const VEC2D &imgpos) const
{
	VEC3D point = m_homographymatrix * VEC3D(imgpos[0],imgpos[1], 1);
	MyUtil::homo_normalize(point);
	point[2]= 0;
	return point;
}

VEC3D CMyCamera::Compute3DCoordinate(const VEC2D &impos, const VEC2D &groundrefpos) const
{
	VEC3D vanishingline = m_xvanishingpoint.Cross(m_yvanishingpoint);
	VEC3D pointT(impos[0], impos[1], 1.0);
	VEC3D pointB(groundrefpos[0],groundrefpos[1], 1.0);
	VEC3D pointQ(groundrefpos[0],groundrefpos[1], 1.0);
	VEC3D pointV = m_zvanishingpoint;
	VEC3D pointS = m_homographymatrix * pointQ;
	MyUtil::homo_normalize(pointS);
	VEC3D BT = pointB.Cross(pointT);
	VEC3D VT = pointV.Cross(pointT);
	int sign = BT.Dot(VT) < 0 ? -1 : 1;
	pointS[2] = -sign * m_origin.Dot(vanishingline) * abs(BT.Length()) /
		(m_zprojectionscale * pointB.Dot(vanishingline) * abs(VT.Length()));

	return pointS;
}

//void CMyCamera::GetGLMatrices(double glprojmatrix[], 
//	double glmodelviewmatrix[], 
//	const int glvpmatrix[], double znear, double zfar)
//{
//	double mat[3][4];
//	for (int i = 0; i < 3; ++i)
//	{
//		for (int j = 0; j < 3; ++j)
//		{
//			mat[i][j] = m_KR(i,j);
//		}
//	}
//	mat[0][3] = m_KT[0];
//	mat[1][3] = m_KT[1];
//	mat[2][3] = m_KT[2];
//
//	MAT3D LHC = MAT3D::ZERO;
//	LHC(0, 0) = LHC(1, 1) = LHC(2, 2) = -1;
//	LHC(0, 0) = 1;
//
//	double icpara[3][4], trans[3][4];
//	if (MyUtil::arParamDecompMat(mat, icpara, trans) < 0)
//	{
//		printf("Fatal error: proj decompose failed!\n");
//		exit(0);
//	}
//	MAT3D R;
//	for (int i = 0; i < 3; i++)
//	{
//		for (int j = 0; j < 3; j++)
//		{
//			R(i, j) = trans[i][j];
//		}
//	}
//	MAT3D LHCR = LHC * R;
//	MAT4D modelViewMatrix = MAT4D::IDENTITY;
//	for (int i = 0; i < 3; i++)
//	{
//		for (int j = 0; j < 3; j++)
//		{
//			modelViewMatrix(i, j) = LHCR(i,j);
//		}
//	}
//	modelViewMatrix(0, 3) = trans[0][3];
//	modelViewMatrix(1, 3) = trans[1][3];
//	modelViewMatrix(2, 3) = trans[2][3];
//	modelViewMatrix(1, 3) = modelViewMatrix(1, 3) * (-1);
//	modelViewMatrix(2, 3) = modelViewMatrix(2, 3) * (-1);
//	modelViewMatrix(3, 3) = 1.0;
//	memcpy(glmodelviewmatrix,(const double*)(modelViewMatrix.Transpose()),sizeof(double)*16);
//
//	double w = glvpmatrix[2];
//	double h = glvpmatrix[3];
//	MAT4D H_inv = MAT4D::IDENTITY;
//	H_inv(0, 0) = 2.0 / w;
//	H_inv(0, 2) = -1;
//	H_inv(1, 1) = -2.0 / h;
//	H_inv(1, 2) = 1.0;
//	H_inv(3, 2) = 1.0;
//	MAT3D K = MAT3D::ZERO;
//	for (int i = 0; i < 3; i++)
//	{
//		for (int j = 0; j < 3; j++)
//		{
//			K(i, j) = icpara[i][j] / icpara[2][2];
//		}
//	}
//	MAT3D y = K * LHC;
//	MAT4D y_ = MAT4D::IDENTITY;
//	for (int i = 0; i < 3; i++)
//	{
//		for (int j = 0; j < 3; j++)
//		{
//			y_(i, j) = y(i,j);
//		}
//	}
//	MAT4D result = H_inv * (y_);
//	double C_ = -(zfar + znear) / (zfar - znear);
//	double D_ = -(2 * zfar * znear) / (zfar - znear);
//	result(2, 2) = C_;
//	result(2, 3) = D_;
//	memcpy(glprojmatrix,(const double*)(result.Transpose()),sizeof(double)*16);
//}


//void CMyCamera::GetGLMatrices(double glprojmatrix[], 
//	double glmodelviewmatrix[], 
//	const int glvpmatrix[], double znear, double zfar)
//{
//	MAT3D LHC = MAT3D::ZERO;
//	LHC(0, 0) = LHC(1, 1) = LHC(2, 2) = -1;
//	LHC(0, 0) = 1;
//	MAT3D LHCR = LHC * m_cameraR;
//	MAT4D modelViewMatrix = MAT4D::IDENTITY;
//	for (int i = 0; i < 3; i++)
//	{
//		for (int j = 0; j < 3; j++)
//		{
//			modelViewMatrix(i, j) = LHCR(i,j);
//		}
//	}
//	modelViewMatrix(0, 3) = m_cameraT[0];
//	modelViewMatrix(1, 3) = m_cameraT[1];
//	modelViewMatrix(2, 3) = m_cameraT[2];
//	modelViewMatrix(1, 3) = modelViewMatrix(1, 3) * (-1);
//	modelViewMatrix(2, 3) = modelViewMatrix(2, 3) * (-1);
//	modelViewMatrix(3, 3) = 1.0;
//	memcpy(glmodelviewmatrix,(const double*)(modelViewMatrix.Transpose()),sizeof(double)*16);
//
//	double w = glvpmatrix[2];
//	double h = glvpmatrix[3];
//	double left = glvpmatrix[0];
//	double bottom = glvpmatrix[1];
//	double right = left+w;
//	double top = bottom+h;
//	MAT4D ortho = MAT4D::IDENTITY;
//	ortho(0,0) = 2.0/w;
//	ortho(0,3) = -(left+right)/(right-left);
//	ortho(1,1) = 2.0/h;
//	ortho(1,3) = -(bottom+top)/(top-bottom);
//	ortho(2,2) = -2.0/(zfar-znear);
//	ortho(2,3) = -(zfar+znear)/(zfar-znear);
//	MAT4D newK = MAT4D::IDENTITY;
//	newK(0,0) = m_cameraK(0,0); newK(0,1) = m_cameraK(0,1); newK(0,2) = -m_cameraK(0,2);
//	newK(1,1) = -m_cameraK(1,1); //y is also inversed
//	newK(1,2) = -m_cameraK(1,2); 
//	newK(2,2) = znear+zfar; newK(2,3) = zfar*znear; 
//	newK(3,2) = -1; newK(3,3) = 0;
//	MAT4D proj = ortho * newK;
//	memcpy(glprojmatrix,(const double*)(proj.Transpose()),sizeof(double)*16);
//}


void CMyCamera::GetGLMatrices(double glprojmatrix[], 
	double glmodelviewmatrix[], 
	const int glvpmatrix[], double znear, double zfar) const
{
	MAT4D modelViewMatrix = MAT4D::IDENTITY;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			modelViewMatrix(i, j) = m_cameraR(i,j);
		}
	}
	modelViewMatrix(0, 3) = m_cameraT[0];
	modelViewMatrix(1, 3) = m_cameraT[1];
	modelViewMatrix(2, 3) = m_cameraT[2];
	memcpy(glmodelviewmatrix,(const double*)(modelViewMatrix.Transpose()),sizeof(double)*16);

	double w = glvpmatrix[2];
	double h = glvpmatrix[3];
	double left = glvpmatrix[0];
	double bottom = glvpmatrix[1];
	double right = left+w;
	double top = bottom+h;
	MAT4D ortho = MAT4D::IDENTITY;
	ortho(0,0) = 2.0/w;
	ortho(0,3) = -(left+right)/(right-left);
	ortho(1,1) = 2.0/h;
	ortho(1,3) = -(bottom+top)/(top-bottom);
	ortho(2,2) = -2.0/(zfar-znear);
	ortho(2,3) = -(zfar+znear)/(zfar-znear);
	MAT4D newK = MAT4D::IDENTITY;
	newK(0,0) = m_cameraK(0,0); newK(0,1) = m_cameraK(0,1); newK(0,2) = m_cameraK(0,2);
	newK(1,1) = m_cameraK(1,1); 
	newK(1,2) = m_cameraK(1,2); 
	newK(2,2) = znear+zfar; newK(2,3) = zfar*znear; 
	newK(3,2) = m_cameraK(2,2); newK(3,3) = 0;
	MAT4D proj = ortho * newK;
	memcpy(glprojmatrix,(const double*)(proj.Transpose()),sizeof(double)*16);
}

VEC2D CMyCamera::project(const VEC3D &spacept) const
{
	VEC3D proj = m_KR * spacept + m_KT;
	if (fabs(proj[2])>1e-6f)
	{
		return VEC2D(proj[0]/proj[2],proj[1]/proj[2]);
	}else
	{
		return VEC2D(proj[0],proj[1]);
	}
}


VEC3D CMyCamera::project_homo(const VEC3D &spacept) const
{
	return m_KR * spacept + m_KT;
}


void CMyCamera::updateRTFromOpenGL(const double glmv[])
{
	MAT4D mvmat(glmv,false);
	m_cameraR = MyUtil::mat4d_2_mat3d(mvmat);

	m_cameraT[0] = mvmat(0,3);
	m_cameraT[1] = mvmat(1,3);
	m_cameraT[2] = mvmat(2,3);
	computeParameters();
}

void CMyCamera::fromOpenGL(double znear,double fovy,const int glvp[], const double glmv[])
{
	//assume the near z plane is the photo plane
	int w_pixel = glvp[2];
	int h_pixel = glvp[3];
	double aspect = (double)w_pixel/(double)h_pixel;
	double fl = znear;
	double h = fl * tan(fovy*0.5*M_PI/180.0) * 2.0f;
	double pixel_divided_by_real = (float)h_pixel / h;
	double fl_pixel = fl * pixel_divided_by_real;
	
	m_cameraK = MAT3D::IDENTITY;
	m_cameraK(0,0) = m_cameraK(1,1) = fl_pixel;
	m_cameraK(0,2) = -(glvp[0] + glvp[2]*0.5);
	m_cameraK(1,2) = -(glvp[1] + glvp[3]*0.5);
	m_cameraK(2,2) = -1.f;

	MAT4D mvmat(glmv,false);
	m_cameraR = MyUtil::mat4d_2_mat3d(mvmat);

	m_cameraT[0] = mvmat(0,3);
	m_cameraT[1] = mvmat(1,3);
	m_cameraT[2] = mvmat(2,3);

	m_height = glvp[3];
	m_width = glvp[2];
	
	computeParameters();
}


bool CMyCamera::calcCam(const VEC3F vanishPts[3],const std::vector<std::vector<SEG2F>> &vvSegCluster,
	const int glvp[4], CAM_CALIBERATE_MODE mode)
{
	if (mode == K_RT)	//fully caliberate
	{
		//calc Intrinsics
		if (!calcCamIntrinsics(vanishPts,glvp))
		{
			return false;
		}
	}

	if (!calcCamExtrinsics(vanishPts,vvSegCluster,glvp))
	{
		return false;
	}
	computeParameters();
	return true;
}

//bool CMyCamera::calcCam(const VEC3F vanishPts[3],const std::vector<std::vector<SEG2F>> &vvSegCluster,
//					const int glvp[4], CAM_CALIBERATE_MODE mode)
//{
//	if (vvSegCluster.size()!=3 || !vvSegCluster[2].size() || !vvSegCluster[1].size()
//		|| !vvSegCluster[0].size())
//	{
//		printf("error: the seg clusters do not have 3 clusters!\n");
//		return false;
//	}
//
//	//here we suppose the cam_K[0,2] = w/2, camK[1,2] = h/2, and the cam is following opengl style
//	m_cameraK = MAT3D::IDENTITY;
//	m_cameraK(0,2) = -(glvp[0] + glvp[2]*0.5);
//	m_cameraK(1,2) = -(glvp[1] + glvp[3]*0.5);
//	m_cameraK(2,2) = -1.f;
//
//	m_width = glvp[2];
//	m_height = glvp[3];
//
//	//1. Intrinsic, here we suppose only f is unknown, see the report Camera caliberation from cube sketch
//	double u = -m_cameraK(0,2);	//they are w/2 and h/2
//	double v = -m_cameraK(1,2);
//	//vanishing points, turned from window coordinates to opengl cooridnates
//	double x0 = vanishPts[0][0] / vanishPts[0][2];	double y0 = vanishPts[0][1] / vanishPts[0][2];
//	double x1 = vanishPts[1][0] / vanishPts[1][2];	double y1 = vanishPts[1][1] / vanishPts[1][2];
//	double x2 = vanishPts[2][0] / vanishPts[2][2];	double y2 = vanishPts[2][1] / vanishPts[2][2];
//	double a01 = u*x1+v*y1+u*x0+v*y0-x0*x1-y0*y1-u*u-v*v;
//	double a02 = u*x2+v*y2+u*x0+v*y0-x0*x2-y0*y2-u*u-v*v;
//	double a12 = u*x1+v*y1+u*x2+v*y2-x2*x1-y2*y1-u*u-v*v;
//
//
//	//as the errors of the user drawn sketch, a01, a02 or a12 might be very wrong, even will be negative
//	int isXYOK = (a01>0?1:0);
//	int isXZOK = (a02>0?1:0);
//	int isYZOK = (a12>0?1:0);
//	//also if the value is too different from the average one,
//	if (!isXYOK && !isXZOK && !isYZOK)
//	{
//		printf("failed to calculate camera K !\n");
//		return false;
//	}
//
//#if 1
//	double f = sqrt((a01*isXYOK+a02*isXZOK+a12*isYZOK)/(isXYOK+isXZOK+isYZOK));
//#else
//	double f;
//	if (isXYOK)
//	{
//		f = sqrt(a01);
//	}else if (isXZOK)
//	{
//		f = sqrt(a02);
//	}else
//	{
//		f = sqrt(a12);
//	}
//#endif
//	m_cameraK(0,0) = f;	m_cameraK(1,1) = f;
//
//	//2. Extrinsic
//	//calculate R first
//	MAT3D invK = MAT3D::IDENTITY;
//	invK(2,2) = 1.0 / m_cameraK(2,2);
//	invK(0,0) = 1/m_cameraK(0,0); invK(0,2) = -invK(2,2)*m_cameraK(0,2)/m_cameraK(0,0);
//	invK(1,1) = 1/m_cameraK(1,1); invK(1,2) = -invK(2,2)*m_cameraK(1,2)/m_cameraK(1,1);
//
//	VEC3D oriRx, oriRy, oriRz;
//	oriRx = invK * VEC3D(vanishPts[0][0]/vanishPts[0][2],vanishPts[0][1]/vanishPts[0][2],1.0);
//	oriRy = invK * VEC3D(vanishPts[1][0]/vanishPts[1][2],vanishPts[1][1]/vanishPts[1][2],1.0);
//	oriRz = invK * VEC3D(vanishPts[2][0]/vanishPts[2][2],vanishPts[2][1]/vanishPts[2][2],1.0);
//	VEC3D Rz, Rx, Ry;
//	if (isXYOK)
//	{
//		Rx = oriRx;
//		Rx.Normalize();
//		Ry = oriRy;
//		Ry = Ry - Rx * (Ry.Dot(Rx));
//		Ry.Normalize();
//		Rz = Rx.Cross(Ry);
//		Rz.Normalize();
//		if(Rz.Dot(oriRz)<0)	//because we do not whether vanishPt[0] is for x or for y;
//		{
//			Rz = -Rz;
//			Rx = -Rx;
//		}
//	}else if (isXZOK)
//	{
//		//invert y, as the vainishing pixel is in window coordinate
//		Rz = oriRz;
//		Rz.Normalize();
//		Rx = oriRx;
//		Rx = Rx - Rz * (Rx.Dot(Rz));
//		Rx.Normalize();
//		Ry = Rz.Cross(Rx);
//		Ry.Normalize();
//	}else if (isYZOK)
//	{
//		Rz = oriRz;
//		Rz.Normalize();
//		Ry = oriRy;
//		Ry = Ry - Rz * (Ry.Dot(Rz));
//		Ry.Normalize();
//		Rx = Ry.Cross(Rz);
//		Rx.Normalize();
//	}
//	
//	m_cameraR = MAT3D(Rx,Ry,Rz,true);
//
//
//#if 1	//Here we don't want the camera to turn the ground upside down
//	VEC3D new_up_dir = m_cameraR*VEC3D(0,0,1);
//	if (new_up_dir.Dot(VEC3D(0,-1,0))<0)	//in local camera coordinate, y is up dir
//	{
//		return false;
//	}
//#endif
//
//	//MAT3D ttt = m_cameraR * m_cameraR.Transpose();
//
//	//calculate T
//	//We suppose that the down point of one segment in segCluster[2] is on the xy plane with coordinate(0,0,0)
//	VEC2F neg = vvSegCluster[2][0].GetNegEnd();
//	VEC2F pos = vvSegCluster[2][0].GetPosEnd();
//	VEC2D ground_pixel;
//	if (neg[1]>pos[1]) {ground_pixel[0] = pos[0];ground_pixel[1] = pos[1];}
//	else {ground_pixel[0] = neg[0];ground_pixel[1] = neg[1];}
//
//	//KR*[0,0,0]~ + KT = [ground_pixel,1]
//	m_cameraT[2] = 1.0/m_cameraK(2,2);
//	m_cameraT[0] = (ground_pixel[0] - m_cameraK(0,2)*m_cameraT[2])/m_cameraK(0,0);
//	m_cameraT[1] = (ground_pixel[1] - m_cameraK(1,2)*m_cameraT[2])/m_cameraK(1,1);
//
//	computeParameters();
//	return true;
//}

CMyCamera::CMyCamera(const CMyCamera &rhs)
{
	m_cameraK = rhs.m_cameraK;
	m_cameraR = rhs.m_cameraR;
	m_cameraT = rhs.m_cameraT; 
	m_width = rhs.m_width;
	m_height = rhs.m_height;
	m_KR = rhs.m_KR;
	m_KT = rhs.m_KT;
	m_homographymatrix = rhs.m_homographymatrix;	
	m_xvanishingpoint = rhs.m_xvanishingpoint;
	m_yvanishingpoint = rhs.m_yvanishingpoint;
	m_zvanishingpoint = rhs.m_zvanishingpoint;
	m_origin = rhs.m_origin;
	m_xprojectionscale = rhs.m_xprojectionscale;
	m_yprojectionscale = rhs.m_yprojectionscale;
	m_zprojectionscale = rhs.m_zprojectionscale;
	m_center = rhs.m_center;
	m_oaxis = rhs.m_oaxis;
}
CMyCamera &CMyCamera::operator=(const CMyCamera &rhs)
{
	m_cameraK = rhs.m_cameraK;
	m_cameraR = rhs.m_cameraR;
	m_cameraT = rhs.m_cameraT; 
	m_width = rhs.m_width;
	m_height = rhs.m_height;
	m_KR = rhs.m_KR;
	m_KT = rhs.m_KT;
	m_homographymatrix = rhs.m_homographymatrix;	//to ground
	m_xvanishingpoint = rhs.m_xvanishingpoint;
	m_yvanishingpoint = rhs.m_yvanishingpoint;
	m_zvanishingpoint = rhs.m_zvanishingpoint;
	m_origin = rhs.m_origin;
	m_xprojectionscale = rhs.m_xprojectionscale;
	m_yprojectionscale = rhs.m_yprojectionscale;
	m_zprojectionscale = rhs.m_zprojectionscale;
	m_center = rhs.m_center;
	m_oaxis = rhs.m_oaxis;
	return *this;
}


//NOTE: this must be done after all parameters are computed
VEC4D CMyCamera::getOpticalCenter(void) const 
{
	// orthographic case
	VEC4D ans;
	if (m_KR(2,0) == 0.0 && m_KR(2,1) == 0.0 &&
		m_KR(2,2) == 0.0) 
	{
		VEC3D vtmp[2];
		for (int i = 0; i < 2; ++i)
			for (int y = 0; y < 3; ++y)
			{
				vtmp[i][y] = m_KR(i,y);
			}

		VEC3D vtmp2 = vtmp[0].Cross(vtmp[1]);
		vtmp2.Normalize();
		for (int y = 0; y < 3; ++y)
		{
			ans[y] = vtmp2[y];
		}
		ans[3] = 0.0;
	}
	else 
	{
		MAT3D A;
		VEC3D b;
		for (int y = 0; y < 3; ++y) 
		{
			for (int x = 0; x < 3; ++x)
			{
				A[y][x] = m_KR(y,x);
			}
			b[y] = - m_KT[y];
		}
		MAT3D iA = A.Inverse();
		b = iA * b;
		for (int y = 0; y < 3; ++y)
		{
			ans[y] = b[y];
		}
		ans[3] = 1.0;
	}
	return ans;
}

VEC4D CMyCamera::intersect(const VEC4D &coord, const VEC4D& abcd) const 
{
	VEC4D ray = m_center - coord;

	const double A = coord.Dot(abcd);
	const double B = ray.Dot(abcd);

	if (fabs(B)<1e-8)
		return VEC4D(0.0f, 0.0f, 0.0f, -1.0f);
	else
		return coord - A / B * ray;
}

double CMyCamera::computeDepth(const VEC4D& point) const 
{
	return m_oaxis.Dot(point);
}

VEC4D CMyCamera::unproject(const VEC3D& icoord) const 
{
	const MAT3D &A = m_KR;
	VEC3D b(icoord[0], icoord[1], icoord[2]);
	for (int y = 0; y < 3; ++y) 
	{
		b[y] -= m_KT[y];    
	}
	MAT3D IA = A.Inverse();
	VEC3D x = IA * b;
	return VEC4D(x[0],x[1],x[2],1.0f);
}

//
//bool CMyCamera::optimizeCam(const VEC3F detected_vp[3],const std::vector<std::vector<SEG2F>> &vvSegCluster, const int viewport[4])
//{
//	if (vvSegCluster.size()!=3 || vvSegCluster[2].size()<3 || vvSegCluster[1].size()<3
//		|| vvSegCluster[0].size()<3)
//	{
//		printf("error: the seg clusters do not have 3 clusters or cluster has smaller than 2 segments!\n");
//		return false;
//	}
//	//random sampling 2 segs and perturbe them to generate the vainishing lines
//	VEC3F vanishingPts[3] , best_vanishingPts[3];
//	best_vanishingPts[0] = detected_vp[0];
//	best_vanishingPts[1] = detected_vp[1];
//	best_vanishingPts[2] = detected_vp[2];
//	const int ITERNUM = 10000;
//	int iter = 0;
//	srand ( (unsigned int)time(NULL) );
//	LINE2F vl[3][2];
//	float max_delta_angle = M_PI/36.f;	//the delta angle will be -2.5 ~ 2.5
//	float inv_RAND_MAX = 1.f / (float)RAND_MAX;
//	VEC2F intrPt;
//	float f_var, f;
//	float min_f_var = getCamFVar(best_vanishingPts,viewport);
//	while(1)
//	{
//		//step1. gather vanishing lines
//		gatherLines(vvSegCluster[0],vl[0]);
//		gatherLines(vvSegCluster[1],vl[1]);
//		gatherLines(vvSegCluster[2],vl[2]);
//		//step2. perturb these lines to calc intersection points, they are vanishing points
//		bool bFail = false;
//		for (int i=0;i<3;i++)
//		{
//			for (int j=0;j<2;j++)
//			{
//				vl[i][j] = MyUtil::perturbLine(vl[i][j],((float)rand()*inv_RAND_MAX-0.5f)*max_delta_angle);
//				INTR_LINE2LINE2_F intr_line(vl[i][0],vl[i][1]);
//				if (intr_line.Find() && intr_line.GetQuantity()==1)
//				{
//					intrPt = intr_line.GetPoint();
//					vanishingPts[i][0] = intrPt[0];	vanishingPts[i][1] = intrPt[1];	vanishingPts[i][2] = 1.f;
//				}else
//				{
//					bFail = true;
//					break;
//				}
//			}
//			if (bFail)
//			{
//				break;
//			}
//		}
//		if (bFail)
//		{
//			continue;
//		}
//#if XY_ONLY	//we only cares whether xy are orthogonal
//		if ( (f = calcF(vanishingPts[0],vanishingPts[1],(float)viewport[2],(float)viewport[3]))>0)
//		{
//			best_vanishingPts[0] = vanishingPts[0];
//			best_vanishingPts[1] = vanishingPts[1];
//			break;
//		}
//#else	//minimize the f variance based x y z vanishing points
//		//Step 3. calc the f variance
//		f_var = getCamFVar(vanishingPts,viewport);
//		if (f_var<min_f_var)
//		{
//			min_f_var = f_var;
//			best_vanishingPts[0] = vanishingPts[0];
//			best_vanishingPts[1] = vanishingPts[1];
//			best_vanishingPts[2] = vanishingPts[2];
//		}
//#endif
//		iter++;
//		if(iter>=ITERNUM)
//		{
//			break;
//		}
//	}
//	//if the best vanishing points are got, calc the camera
//#if XY_ONLY
//	if (f>0)
//	{
//		calcCam(best_vanishingPts,vvSegCluster,viewport);
//	}else
//	{
//		return false;
//	}
//#else
//	if (min_f_var < FLT_MAX)
//	{
//		calcCam(best_vanishingPts,vvSegCluster,viewport);
//	}else
//	{
//		return false;
//	}
//#endif
//
//	return true;
//}

double calcF(const VEC3F &vanishPt1, const VEC3F &vanishPt2, float w, float h)
{
	double u = w*0.5f;
	double v = h*0.5f;
	//vanishing points, turned from window coordinates to opengl cooridnates
	double x0 = vanishPt1[0] / vanishPt1[2];	double y0 = vanishPt1[1] / vanishPt1[2];
	double x1 = vanishPt2[0] / vanishPt2[2];	double y1 = vanishPt2[1] / vanishPt2[2];
	return u*x1+v*y1+u*x0+v*y0-x0*x1-y0*y1-u*u-v*v;
}

//for different pairs of vanishing points,the K might failed. Or even if they won't fail,
//the variance of different f will be large. So the we choose the minimal var ones
float getCamFVar(const VEC3F vanishPts[3],const int glvp[4])
{
	float w = (float)glvp[2];
	float h = (float)glvp[3];

	//1. Intrinsic, here we suppose only f is unknown, see the report Camera caliberation from cube sketch
	double u = w*0.5f;
	double v = h*0.5f;
	//vanishing points, turned from window coordinates to opengl cooridnates
	double x0 = vanishPts[0][0] / vanishPts[0][2];	double y0 = vanishPts[0][1] / vanishPts[0][2];
	double x1 = vanishPts[1][0] / vanishPts[1][2];	double y1 = vanishPts[1][1] / vanishPts[1][2];
	double x2 = vanishPts[2][0] / vanishPts[2][2];	double y2 = vanishPts[2][1] / vanishPts[2][2];
	double a01 = u*x1+v*y1+u*x0+v*y0-x0*x1-y0*y1-u*u-v*v;
	double a02 = u*x2+v*y2+u*x0+v*y0-x0*x2-y0*y2-u*u-v*v;
	double a12 = u*x1+v*y1+u*x2+v*y2-x2*x1-y2*y1-u*u-v*v;

	if (a01<0 || a02<0 || a12<0)	//failed, return very big
	{
		return FLT_MAX;
	}

	float e = (float)(a01 + a02 + a12) / 3.f;
	return (float)((a01-e)*(a01-e) + (a02-e)*(a02-e) + (a12-e)*(a12-e));
}

bool CMyCamera::calcCamIntrinsics(const VEC3F vanishPts[3],const int glvp[4])
{	
	//here we suppose the cam_K[0,2] = w/2, camK[1,2] = h/2, and the cam is following opengl style
	m_cameraK = MAT3D::IDENTITY;
	m_cameraK(0,2) = -(glvp[0] + glvp[2]*0.5);
	m_cameraK(1,2) = -(glvp[1] + glvp[3]*0.5);
	m_cameraK(2,2) = -1.f;

	m_width = glvp[2];
	m_height = glvp[3];

	//1. Intrinsic, here we suppose only f is unknown, see the report Camera caliberation from cube sketch
	double u = -m_cameraK(0,2);	//they are w/2 and h/2
	double v = -m_cameraK(1,2);
	//vanishing points, turned from window coordinates to opengl cooridnates
	double x0 = vanishPts[0][0] / vanishPts[0][2];	double y0 = vanishPts[0][1] / vanishPts[0][2];
	double x1 = vanishPts[1][0] / vanishPts[1][2];	double y1 = vanishPts[1][1] / vanishPts[1][2];
	double x2 = vanishPts[2][0] / vanishPts[2][2];	double y2 = vanishPts[2][1] / vanishPts[2][2];
	double a01 = u*x1+v*y1+u*x0+v*y0-x0*x1-y0*y1-u*u-v*v;
	double a02 = u*x2+v*y2+u*x0+v*y0-x0*x2-y0*y2-u*u-v*v;
	double a12 = u*x1+v*y1+u*x2+v*y2-x2*x1-y2*y1-u*u-v*v;


	//as the errors of the user drawn sketch, a01, a02 or a12 might be very wrong, even will be negative
	int isXYOK = (a01>0?1:0);
	int isXZOK = (a02>0?1:0);
	int isYZOK = (a12>0?1:0);
	//also if the value is too different from the average one,
	if (!isXYOK && !isXZOK && !isYZOK)
	{
		printf("failed to calculate camera K !\n");
		return false;
	}

#if 1
	double f = sqrt((a01*isXYOK+a02*isXZOK+a12*isYZOK)/(isXYOK+isXZOK+isYZOK));
#else
	double f;
	if (isXYOK)
	{
		f = sqrt(a01);
	}else if (isXZOK)
	{
		f = sqrt(a02);
	}else
	{
		f = sqrt(a12);
	}
#endif
	m_cameraK(0,0) = f;	m_cameraK(1,1) = f;
	return true;
}



bool CMyCamera::calcCamExtrinsics(const VEC3F vanishPts[3],const std::vector<std::vector<SEG2F>> &vvSegCluster,
						const int glvp[4])
{
	if (vvSegCluster.size()!=3 || !vvSegCluster[2].size() || !vvSegCluster[1].size()
		|| !vvSegCluster[0].size())
	{
		printf("error: the seg clusters do not have 3 clusters!\n");
		return false;
	}
	//1. Intrinsic, here we suppose only f is unknown, see the report Camera caliberation from cube sketch
	double u = -m_cameraK(0,2);	//they are w/2 and h/2
	double v = -m_cameraK(1,2);
	//vanishing points, turned from window coordinates to opengl cooridnates
	double x0 = vanishPts[0][0] / vanishPts[0][2];	double y0 = vanishPts[0][1] / vanishPts[0][2];
	double x1 = vanishPts[1][0] / vanishPts[1][2];	double y1 = vanishPts[1][1] / vanishPts[1][2];
	double x2 = vanishPts[2][0] / vanishPts[2][2];	double y2 = vanishPts[2][1] / vanishPts[2][2];
	double a01 = u*x1+v*y1+u*x0+v*y0-x0*x1-y0*y1-u*u-v*v;
	double a02 = u*x2+v*y2+u*x0+v*y0-x0*x2-y0*y2-u*u-v*v;
	double a12 = u*x1+v*y1+u*x2+v*y2-x2*x1-y2*y1-u*u-v*v;


	//as the errors of the user drawn sketch, a01, a02 or a12 might be very wrong, even will be negative
	int isXYOK = (a01>0?1:0);
	int isXZOK = (a02>0?1:0);
	int isYZOK = (a12>0?1:0);
	//also if the value is too different from the average one,
	if (!isXYOK && !isXZOK && !isYZOK)
	{
		printf("failed to calculate camera K !\n");
		return false;
	}

	//2. Extrinsic
	//calculate R first
	MAT3D invK = MAT3D::IDENTITY;
	invK(2,2) = 1.0 / m_cameraK(2,2);
	invK(0,0) = 1/m_cameraK(0,0); invK(0,2) = -invK(2,2)*m_cameraK(0,2)/m_cameraK(0,0);
	invK(1,1) = 1/m_cameraK(1,1); invK(1,2) = -invK(2,2)*m_cameraK(1,2)/m_cameraK(1,1);

	VEC3D oriRx, oriRy, oriRz;
	oriRx = invK * VEC3D(vanishPts[0][0]/vanishPts[0][2],vanishPts[0][1]/vanishPts[0][2],1.0);
	oriRy = invK * VEC3D(vanishPts[1][0]/vanishPts[1][2],vanishPts[1][1]/vanishPts[1][2],1.0);
	oriRz = invK * VEC3D(vanishPts[2][0]/vanishPts[2][2],vanishPts[2][1]/vanishPts[2][2],1.0);
	VEC3D Rz, Rx, Ry;
	if (isXYOK)
	{
		Rx = oriRx;
		Rx.Normalize();
		Ry = oriRy;
		Ry = Ry - Rx * (Ry.Dot(Rx));
		Ry.Normalize();
		Rz = Rx.Cross(Ry);
		Rz.Normalize();
		if(Rz.Dot(oriRz)<0)	//because we do not whether vanishPt[0] is for x or for y;
		{
			Rz = -Rz;
			Rx = -Rx;
		}
	}else if (isXZOK)
	{
		//invert y, as the vainishing pixel is in window coordinate
		Rz = oriRz;
		Rz.Normalize();
		Rx = oriRx;
		Rx = Rx - Rz * (Rx.Dot(Rz));
		Rx.Normalize();
		Ry = Rz.Cross(Rx);
		Ry.Normalize();
	}else if (isYZOK)
	{
		Rz = oriRz;
		Rz.Normalize();
		Ry = oriRy;
		Ry = Ry - Rz * (Ry.Dot(Rz));
		Ry.Normalize();
		Rx = Ry.Cross(Rz);
		Rx.Normalize();
	}

	m_cameraR = MAT3D(Rx,Ry,Rz,true);


#if 1	//Here we don't want the camera to turn the ground upside down
	VEC3D new_up_dir = m_cameraR*VEC3D(0,0,1);
	if (new_up_dir.Dot(VEC3D(0,-1,0))<0)	//in local camera coordinate, y is up dir
	{
		return false;
	}
#endif

	//MAT3D ttt = m_cameraR * m_cameraR.Transpose();

	//calculate T
	//We suppose that the down point of one segment in segCluster[2] is on the xy plane with coordinate(0,0,0)
	VEC2F neg = vvSegCluster[2][0].GetNegEnd();
	VEC2F pos = vvSegCluster[2][0].GetPosEnd();
	VEC2D ground_pixel;
	if (neg[1]>pos[1]) {ground_pixel[0] = pos[0];ground_pixel[1] = pos[1];}
	else {ground_pixel[0] = neg[0];ground_pixel[1] = neg[1];}

	//KR*[0,0,0]~ + KT = [ground_pixel,1]
	m_cameraT[2] = 1.0/m_cameraK(2,2);
	m_cameraT[0] = (ground_pixel[0] - m_cameraK(0,2)*m_cameraT[2])/m_cameraK(0,0);
	m_cameraT[1] = (ground_pixel[1] - m_cameraK(1,2)*m_cameraT[2])/m_cameraK(1,1);
	return true;
}

VEC3D CMyCamera::worldpt_2_eyept(const VEC3D &world_pt) const
{
	return m_cameraR * world_pt + m_cameraT;
}

bool CMyCamera::save(FILE *fp) const
{
	fwrite((const double*)m_cameraK,sizeof(double),9,fp);
	fwrite((const double*)m_cameraR,sizeof(double),9,fp);
	fwrite((const double*)m_cameraT,sizeof(double),3,fp);
	fwrite(&m_width,sizeof(int),1,fp);
	fwrite(&m_height,sizeof(int),1,fp);
	return true;
}

bool CMyCamera::load(FILE *fp)
{
	fread((double*)m_cameraK,sizeof(double),9,fp);
	fread((double*)m_cameraR,sizeof(double),9,fp);
	fread((double*)m_cameraT,sizeof(double),3,fp);
	fread(&m_width,sizeof(int),1,fp);
	fread(&m_height,sizeof(int),1,fp);
	computeParameters();
	return true;
}

double CMyCamera::getF() const
{
	return m_cameraK(0,0);
}

void CMyCamera::getCameraK(double k[9]){
	memcpy(k, (double*)m_cameraK, sizeof(double)*9);
	return;
}