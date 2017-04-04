#include "FLUtils.h"
namespace fuzzy{
	std::string FLUtils::trim(const std::string& str){
		std::string trimmed;
		bool oneSpaceOnly = false;
		for (int i = 0; i < str.length(); i++){
			if(str.at(i) != ' '){
				trimmed.push_back(str.at(i));
				oneSpaceOnly = false;
			} else if(!oneSpaceOnly){
				trimmed.push_back(' ');
			}
		}
		return trimmed;
	}

	std::vector<std::string> FLUtils::split(char* buffer, const char* delimiter){
	std::vector<std::string> result;
	char* buffer_in = new char[strlen(buffer) + 1];
	memcpy(buffer_in, buffer, strlen(buffer) + 1);
	char* current;

	current = std::strtok(buffer_in, delimiter);

	while(current != NULL){
		result.push_back(std::string(current));
		current = std::strtok(NULL, delimiter);
	}
	delete current;
	delete[] buffer_in;
	return result;
}
}