#include "stdafx.h"
#include "MyMaterial.h"
#include "GL/GL.h"


CMaterial::CMaterial(void)
{
	default_set();
}

CMaterial::~CMaterial()
{

}

void CMaterial::default_set()
{
	ambient_[0] = 0.2f; ambient_[1] = 0.2f; 
	ambient_[2] = 0.2f; ambient_[3] = 1.0f; 

	diffuse_[0] = 0.5f; diffuse_[1] = 0.5f;
	diffuse_[2] = 0.5f; diffuse_[3] = 1.0f;

	specular_[0] = 0.5f; specular_[1] = 0.5f;
	specular_[2] = 0.5f; specular_[3] = 0.5f;

	memset(emission_, 0, sizeof(float)*4);

	shininess_ = 10.0f;
}

CMaterial::CMaterial(const CMaterial &rhs)
{
	memcpy(ambient_, rhs.ambient_, sizeof(float)*4);
	memcpy(diffuse_, rhs.diffuse_, sizeof(float)*4);
	memcpy(specular_, rhs.specular_, sizeof(float)*4);
	memcpy(emission_, rhs.emission_, sizeof(float)*4);

	shininess_ = rhs.shininess_;
}

CMaterial &CMaterial::operator=(const CMaterial &rhs)
{
	memcpy(ambient_, rhs.ambient_, sizeof(float)*4);
	memcpy(diffuse_, rhs.diffuse_, sizeof(float)*4);
	memcpy(specular_, rhs.specular_, sizeof(float)*4);
	memcpy(emission_, rhs.emission_, sizeof(float)*4);

	shininess_ = rhs.shininess_;
	return *this;
}

void CMaterial::setColor(float r, float g, float b)
{
	ambient_[0] = r; ambient_[1] = g; 
	ambient_[2] = b; ambient_[3] = 1.0f; 

	diffuse_[0] = r; diffuse_[1] = g;
	diffuse_[2] = b; diffuse_[3] = 1.0f;

	specular_[0] = r; specular_[1] = g;
	specular_[2] = b; specular_[3] = 0.5f;
}

void CMaterial::applyGLMaterial()
{
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient_);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse_);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,specular_);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS,shininess_);
}
