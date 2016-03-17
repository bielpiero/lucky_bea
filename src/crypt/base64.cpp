#include "crypt/base64.h"

const std::string Base64::codes="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/=";

unsigned char* Base64::decode(std::string input){
	if(input.length() % 4 != 0){
		throw std::invalid_argument("Error!!! Invalid base64 input");
	}
	unsigned char* result = new unsigned char[((input.length() * 3) / 4) - (input.find("=") > 0 ? (input.length() - input.find("=")) : 0)];

	char* inChars = (char*)input.c_str();
	int j = 0;
	int b[4];

	for(int i = 0; i < input.length(); i += 4){
		b[0] = codes.find(inChars[i]);
        b[1] = codes.find(inChars[i + 1]);
        b[2] = codes.find(inChars[i + 2]);
        b[3] = codes.find(inChars[i + 3]);
        result[j++] = (unsigned char)((b[0] << 2) | (b[1] >> 4));
        if(b[2] < 64){
        	result[j++] = (unsigned char)((b[1] << 4) | (b[2] >> 2));
        	if(b[3] < 64){
        		result[j++] = (unsigned char)((b[2] << 6) | b[3]);
        	}
        }
	}

	return result;
}

std::string Base64::encode(unsigned char* input, int length){
	std::string coded;
	int num;

	for(int i = 0; i < length; i += 3){
		num = (input[i] & 0xFC) >> 2;
		coded.push_back(codes.at(num));

		num = (input[i] & 0x03) << 4;
		if(i + 1 < length){
			num |= (input[i + 1] & 0xF0) >> 4;
			coded.push_back(codes.at(num));

			num = (input[i + 1] & 0x0F) << 2;
			if(i + 2 < length){
				num |= (input[i + 2] & 0xC0) >> 6;
				coded.push_back(codes.at(num));

				num = (input[i + 2] & 0x3F);
				coded.push_back(codes.at(num));
			} else {
				coded.push_back(codes.at(num));
				coded.append("=");	
			}
		} else {
			coded.push_back(codes.at(num));
			coded.append("==");
		}
	}

	return coded;
}

