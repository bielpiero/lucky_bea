#ifndef SHA512_H
#define SHA512_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class SHA512{
private:
	static const unsigned long long int hash_k512[80];
	unsigned long long int hash_h512[8];
public:
	SHA512(){}
	~SHA512(){}

	std::string execute(std::string input);
private:
	void init();
	void transform(unsigned long long int* data);
	void preProcessing(unsigned char* data, int length, int nBlocks, unsigned long long int* output);
	static unsigned long long int pack64(unsigned char* str);
	static const unsigned int SHA512_BIT_BLOCK_SIZE = (128);
	static const unsigned int SHA512_BLOCK_SIZE = (16);
	static const unsigned int DIGEST_BIT_SIZE = (64);
	static const unsigned int DIGEST_SIZE = (8);
	static unsigned long long int rightRotate(unsigned long long int value, int n);
	static unsigned long long int leftRotate(unsigned long long int value, int n);

};

#endif