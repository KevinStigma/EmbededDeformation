#ifndef _MY_MATERIAL_
#define _MY_MATERIAL_



class CMaterial
{
public:
	CMaterial(void);
	~CMaterial();
	CMaterial(const CMaterial &rhs);
	CMaterial &operator=(const CMaterial &rhs);
public:
	void default_set();
	void setColor(float r, float g, float b);
	void applyGLMaterial();
public:
	float ambient_[4], diffuse_[4], specular_[4], emission_[4], shininess_;
};

#endif