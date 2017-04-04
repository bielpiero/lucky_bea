#ifndef FL_UTILS_H
#define FL_UTILS_H

#include <iostream>
#include <algorithm>
#include <string>
#include <cstring>
#include <vector>

#define IF_STR "if"
#define THEN_STR "then"
#define WITH_STR "with"

namespace fuzzy{

	class FLUtils{
	public:
		static std::string trim(const std::string& str);
		static std::vector<std::string> split(char* buffer, const char* delimiter);
	};
}

#endif