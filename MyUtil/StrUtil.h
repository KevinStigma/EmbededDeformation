#ifndef _STR_UTILS_H
#define _STR_UTILS_H

#include <string>

namespace MyUtil
{
	bool save_str(const std::string &str, FILE *fp);
	bool load_str(std::string &str, FILE *fp);
}


#endif