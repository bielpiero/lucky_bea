#ifndef SHA_H
#define SHA_H

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SHA_1 0
#define SHA_512 1

class SHA{
private:
	int type;
	static const unsigned long long int hash_k512[80];
	unsigned long long int hash_h512[8];

	unsigned long int hash_h1[5];
public:
	SHA(int type = 0){ this->type = type; }
	~SHA(){}

	std::string execute(std::string input);
private:
	void init32();
	void init64();
	void transform32(unsigned long int* data);
	void transform64(unsigned long long int* data);
	void preProcessing32(unsigned char* data, int length, int nBlocks, unsigned long int* output);
	void preProcessing64(unsigned char* data, int length, int nBlocks, unsigned long long int* output);

	static unsigned long long int pack64(unsigned char* str);
	static const unsigned int SHA512_BIT_BLOCK_SIZE = (128);
	static const unsigned int SHA512_BLOCK_SIZE = (16);
	static const unsigned int DIGEST512_BIT_SIZE = (64);
	static const unsigned int DIGEST512_SIZE = (8);
	static const unsigned int MAX_LEHGTH_SHA512_SIZE = (8);

	static unsigned long int pack32(unsigned char* str);
	static const unsigned int SHA1_BIT_BLOCK_SIZE = (64);
	static const unsigned int SHA1_BLOCK_SIZE = (16);
	static const unsigned int DIGEST1_BIT_SIZE = (32);
	static const unsigned int DIGEST1_SIZE = (5);
	static const unsigned int MAX_LEHGTH_SHA1_SIZE = (4);

	static unsigned long int rightRotate32(unsigned long int value, int n);
	static unsigned long long int rightRotate64(unsigned long long int value, int n);
	static unsigned long int leftRotate32(unsigned long int value, int n);
	static unsigned long long int leftRotate64(unsigned long long int value, int n);

};

#endif