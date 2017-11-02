#ifndef JSON_H
#define JSON_H

#include "RNUtils.h"

class JSONObject{
private:
	std::string value;
	std::pair<std::string, JSONObject*>* tuple;
public:
	JSONObject(){}
	~JSONObject(){}
	
};

#endif