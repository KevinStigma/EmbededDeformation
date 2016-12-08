#include "stdafx.h"
#include "StrUtil.h"
#include <limits>
#include <fstream>

namespace MyUtil
{

	bool save_str(const std::string &str, FILE *fp)
	{
		int str_len = (int)str.length();
		fwrite(&str_len,sizeof(int),1,fp);
		fwrite(str.c_str(),sizeof(char),str_len,fp);
		return true;
	}

	bool load_str(std::string &str, FILE *fp)
	{
		int str_len;
		char buf[128];
		fread(&str_len,sizeof(int),1,fp);
		fread(buf,sizeof(char),str_len,fp);
		buf[str_len]='\0';
		str =std::string(buf);
		return true;
	}

}