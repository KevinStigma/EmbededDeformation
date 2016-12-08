#ifndef _PROXY_CAMERA_
#define _PROXY_CAMERA_

#include "../MyUtil/MyCamera.h"
#include "../MyUtil/MyPara.h"
#include "../Sketch/NamedTypeObj.h"

class CSketches;
class CProxyPrim;

class CProxyCamera : public CMyCamera , public CNamedTypeObj
{
protected:
	static std::string CProxyCam_baseType;
	static std::string CProxyCam_subType;
public:
	virtual const std::string &getBaseType() const { return CProxyCam_baseType; }
	virtual const std::string &getSubType() const {return CProxyCam_subType; }
	virtual bool save(FILE *fp) const;
	virtual bool load(FILE *fp);
	void reset();
	void setF(double f);
	double getInitF() const;
	//new added, for youyi
	bool save_forYouyi(FILE *fp) const;
public:
	CProxyCamera();
	CProxyCamera(const CProxyCamera &cam);
	CProxyCamera &operator=(const CProxyCamera &rhs);
	//bool caliberate(const CSketches *pSketch,const int vp[4]);
	bool generateCam_RandSample(const std::vector<std::vector<SEG2F>> &vvClusterSegs,const int vp[4], const PROXY_FIT_PARA &fit_para);
public:
	MAT3D m_init_K;	//for reset
};

#endif