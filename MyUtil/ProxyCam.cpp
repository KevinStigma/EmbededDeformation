#include "stdafx.h"
#include "ProxyCam.h"
#include "../Sketch/Sketch.h"
#include "../MyUtil/MyUtil.h"
#include "../MyUtil/MyConsts.h"


std::string CProxyCamera::CProxyCam_baseType = CAM_NAME;
std::string CProxyCamera::CProxyCam_subType = PROXY_CAM_NAME;

CProxyCamera::CProxyCamera()
{

}

CProxyCamera::CProxyCamera(const CProxyCamera &rhs) : CMyCamera(rhs)
{

}

CProxyCamera& CProxyCamera::operator=(const CProxyCamera &rhs)
{
	CMyCamera::operator=(rhs);
	return *this;
}


//bool CProxyCamera::caliberate(const CSketches *pSketch, const int vp[4])
//{
//	if (pSketch->getType()  != CUBE_SKETCH)
//	{
//		printf("can not caliberate from sketch of type %s\n",getSketchTypeName(pSketch->getType() ));
//		return false;
//	}
//#if 1
//	if(!optimizeCam(pSketch->getVanishPts(),*pSketch->getVanishSegClusterPtr(),vp))
//	{
//		printf("caliberate camera failed!\n");
//		return false;
//	}
//#else
//	if(!calcCam(pSketch->getVanishPts(),*pSketch->getVanishSegClusterPtr(),vp))
//	{
//		printf("caliberate camera failed!\n");
//		return false;
//	}
//#endif
//	return true;
//}


bool CProxyCamera::generateCam_RandSample(const std::vector<std::vector<SEG2F>> &vvSegCluster,const int viewport[4], const PROXY_FIT_PARA &fit_para)
{
	if (vvSegCluster.size()!=3 || vvSegCluster[2].size()<2 || vvSegCluster[1].size()<2
		|| vvSegCluster[0].size()<2)
	{
		printf("error to generate cam: the seg clusters do not have 3 clusters or cluster has smaller than 2 segments!\n");
		return false;
	}
	//random sampling 2 segs and perturbe them to generate the vainishing lines
	LINE2F vl[3][2];
	float max_delta_angle = fit_para._sampling_angle;	//the delta angle will be -2.5 ~ 2.5
	float inv_RAND_MAX = 1.f / (float)RAND_MAX;
	VEC2F intrPt;
	VEC3F vanishingPts[3];

	//step1. gather vanishing lines
	MyUtil::gatherLines(vvSegCluster[0],vl[0]);
	MyUtil::gatherLines(vvSegCluster[1],vl[1]);
	MyUtil::gatherLines(vvSegCluster[2],vl[2]);
	//step2. perturb these lines to calc intersection points, they are vanishing points
	for (int i=0;i<3;i++)
	{
		for (int j=0;j<2;j++)
		{
			vl[i][j] = MyUtil::perturbLine(vl[i][j],((float)rand()*inv_RAND_MAX-0.5f)*max_delta_angle);
			//INTR_LINE2LINE2_F intr_line(vl[i][0],vl[i][1]);
			//if (intr_line.Find() && intr_line.GetQuantity()==1)
			//{
			//	intrPt = intr_line.GetPoint();
			//	vanishingPts[i][0] = intrPt[0];	vanishingPts[i][1] = intrPt[1];	vanishingPts[i][2] = 1.f;
			//}else
			//{
			//	return false;
			//}
		}
		INTR_LINE2LINE2_F intr_line(vl[i][0],vl[i][1]);
		if (intr_line.Find() && intr_line.GetQuantity()==1)
		{
			intrPt = intr_line.GetPoint();
			vanishingPts[i][0] = intrPt[0];	vanishingPts[i][1] = intrPt[1];	vanishingPts[i][2] = 1.f;
		}else
		{
			return false;
		}
	}
	//Step 3. calc the f variance to make sure 3 f2 are all > 0
	float f_var = getCamFVar(vanishingPts,viewport);
	bool flag = true;
	if (f_var < FLT_MAX)
	{
		flag = calcCam(vanishingPts,vvSegCluster,viewport,fit_para._cam_calib_mode);
	}else
	{
		return false;
	}
	return flag;
}

bool CProxyCamera::save_forYouyi(FILE *fp) const
{
	return CMyCamera::save(fp);
}

bool CProxyCamera::save(FILE *fp) const
{
	//int str_len = (int)m_objName.length();
	//fwrite(&str_len,sizeof(int),1,fp);
	//fwrite(m_objName.c_str(),sizeof(char),str_len,fp);
	MyUtil::save_str(m_objName,fp);
	return CMyCamera::save(fp);
}

bool CProxyCamera::load(FILE *fp)
{
	//int str_len;
	//char buf[128];
	//fread(&str_len,sizeof(int),1,fp);
	//fread(buf,sizeof(char),str_len,fp);
	//buf[str_len]='\0';
	//m_objName = buf;
	MyUtil::load_str(m_objName,fp);
	bool flag = CMyCamera::load(fp);
	m_init_K = m_cameraK;
	return flag;
}


void CProxyCamera::reset()
{
	m_cameraK = m_init_K;
	computeParameters();
}

void CProxyCamera::setF(double f)
{
	m_cameraK(0,0) = m_cameraK(1,1) = f;
	computeParameters();
}

double CProxyCamera::getInitF() const
{
	return m_init_K(0,0);
}