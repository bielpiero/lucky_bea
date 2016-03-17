#ifndef RN_UTILS_H
#define RN_UTILS_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <cstring>
#include <ctime>

class

void printLn(char* _format, ...){

	char buffer[256];
	va_list arguments;
	va_start(arguments, _format);

	
	vsprintf(buffer, _format, arguments);
	printf(buffer);
	va_end(arguments);
}

#endif