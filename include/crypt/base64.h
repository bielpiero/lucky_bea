#ifndef BASE64_H
#define BASE64_H

#include <iostream>
#include <string>
#include <stdexcept>

class Base64{
public:
	static unsigned char* decode(std::string input);
	static std::string encode(unsigned char* input, int length);
private:
	static const std::string codes;
};

#endif