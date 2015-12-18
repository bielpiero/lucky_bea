#include "sha.h"

const unsigned long long int SHA::hash_k512[80] = {
	(0x428a2f98d728ae22), (0x7137449123ef65cd), (0xb5c0fbcfec4d3b2f), (0xe9b5dba58189dbbc), (0x3956c25bf348b538), 
    (0x59f111f1b605d019), (0x923f82a4af194f9b), (0xab1c5ed5da6d8118), (0xd807aa98a3030242), (0x12835b0145706fbe), 
    (0x243185be4ee4b28c), (0x550c7dc3d5ffb4e2), (0x72be5d74f27b896f), (0x80deb1fe3b1696b1), (0x9bdc06a725c71235), 
    (0xc19bf174cf692694), (0xe49b69c19ef14ad2), (0xefbe4786384f25e3), (0x0fc19dc68b8cd5b5), (0x240ca1cc77ac9c65), 
    (0x2de92c6f592b0275), (0x4a7484aa6ea6e483), (0x5cb0a9dcbd41fbd4), (0x76f988da831153b5), (0x983e5152ee66dfab), 
    (0xa831c66d2db43210), (0xb00327c898fb213f), (0xbf597fc7beef0ee4), (0xc6e00bf33da88fc2), (0xd5a79147930aa725), 
    (0x06ca6351e003826f), (0x142929670a0e6e70), (0x27b70a8546d22ffc), (0x2e1b21385c26c926), (0x4d2c6dfc5ac42aed), 
    (0x53380d139d95b3df), (0x650a73548baf63de), (0x766a0abb3c77b2a8), (0x81c2c92e47edaee6), (0x92722c851482353b), 
    (0xa2bfe8a14cf10364), (0xa81a664bbc423001), (0xc24b8b70d0f89791), (0xc76c51a30654be30), (0xd192e819d6ef5218), 
    (0xd69906245565a910), (0xf40e35855771202a), (0x106aa07032bbd1b8), (0x19a4c116b8d2d0c8), (0x1e376c085141ab53), 
    (0x2748774cdf8eeb99), (0x34b0bcb5e19b48a8), (0x391c0cb3c5c95a63), (0x4ed8aa4ae3418acb), (0x5b9cca4f7763e373), 
    (0x682e6ff3d6b2b8a3), (0x748f82ee5defb2fc), (0x78a5636f43172f60), (0x84c87814a1f0ab72), (0x8cc702081a6439ec), 
    (0x90befffa23631e28), (0xa4506cebde82bde9), (0xbef9a3f7b2c67915), (0xc67178f2e372532b), (0xca273eceea26619c), 
    (0xd186b8c721c0c207), (0xeada7dd6cde0eb1e), (0xf57d4f7fee6ed178), (0x06f067aa72176fba), (0x0a637dc5a2c898a6), 
    (0x113f9804bef90dae), (0x1b710b35131c471b), (0x28db77f523047d84), (0x32caab7b40c72493), (0x3c9ebe0a15c9bebc), 
    (0x431d67c49c100d4c), (0x4cc5d4becb3e42b6), (0x597f299cfc657e2a), (0x5fcb6fab3ad6faec), (0x6c44198c4a475817)
};


void SHA::preProcessing64(unsigned char* data, int length, int nBlocks, unsigned long long int* output){
	int index = 0;
	bool addedOne = false;

	unsigned long long int fullBlock[nBlocks * SHA512_BLOCK_SIZE];
	memset(&fullBlock, 0, nBlocks * SHA512_BIT_BLOCK_SIZE);

	for(int i = 0; i < length; i += MAX_LEHGTH_SHA512_SIZE){

		unsigned int remLength = (length - i) < MAX_LEHGTH_SHA512_SIZE ? (length - i) : MAX_LEHGTH_SHA512_SIZE;

		unsigned char* currentstr = new unsigned char[MAX_LEHGTH_SHA512_SIZE];
		memset(currentstr, 0, MAX_LEHGTH_SHA512_SIZE);
		memcpy(currentstr, data + i, remLength);
		if(remLength < MAX_LEHGTH_SHA512_SIZE){
			currentstr[remLength] = SHA512_BIT_BLOCK_SIZE;
			addedOne = true;
		}
		fullBlock[index] = pack64(currentstr);
		index++;
	}

	if(not addedOne){
		unsigned char* currentstr = new unsigned char[MAX_LEHGTH_SHA512_SIZE];
		memset(currentstr, 0, MAX_LEHGTH_SHA512_SIZE);
		currentstr[0] = SHA512_BIT_BLOCK_SIZE;
		fullBlock[index] = pack64(currentstr);
	}	

	memcpy(output, &fullBlock, nBlocks * SHA512_BIT_BLOCK_SIZE);
}

void SHA::preProcessing32(unsigned char* data, int length, int nBlocks, unsigned long int* output){
	int index = 0;
	bool addedOne = false;

	unsigned long int fullBlock[nBlocks * SHA1_BLOCK_SIZE];
	memset(&fullBlock, 0, nBlocks * SHA1_BIT_BLOCK_SIZE);

	for(int i = 0; i < length; i += MAX_LEHGTH_SHA1_SIZE){

		unsigned int remLength = (length - i) < MAX_LEHGTH_SHA1_SIZE ? (length - i) : MAX_LEHGTH_SHA1_SIZE;

		unsigned char* currentstr = new unsigned char[MAX_LEHGTH_SHA1_SIZE];
		memset(currentstr, 0, MAX_LEHGTH_SHA1_SIZE);
		memcpy(currentstr, data + i, remLength);
		if(remLength < MAX_LEHGTH_SHA1_SIZE){
			currentstr[remLength] = SHA512_BIT_BLOCK_SIZE;
			addedOne = true;
		}
		fullBlock[index] = pack32(currentstr);
		index++;
	}

	if(not addedOne){
		unsigned char* currentstr = new unsigned char[MAX_LEHGTH_SHA1_SIZE];
		memset(currentstr, 0, MAX_LEHGTH_SHA1_SIZE);
		currentstr[0] = SHA512_BIT_BLOCK_SIZE;
		fullBlock[index] = pack32(currentstr);
	}	

	memcpy(output, &fullBlock, nBlocks * SHA1_BIT_BLOCK_SIZE);
}

void SHA::transform64(unsigned long long int* data){
	unsigned long long int* w = new unsigned long long int[80];
	unsigned long long int* wv = new unsigned long long int[8];
	unsigned long long int s0, s1, S0, S1, ch, temp1, maj, temp2;

	for(unsigned int i = 0; i < SHA512_BLOCK_SIZE; i++){
		w[i] = data[i];
	}

	for(int i = 16; i < 80; i++){
		s0 = (rightRotate64(w[i-15], 1) ^ rightRotate64(w[i-15], 8) ^ (w[i-15] >> 7));
		s1 = (rightRotate64(w[i-2], 19) ^ rightRotate64(w[i-2], 61) ^ (w[i-2] >> 6));
		w[i] = w[i-16] + s0 + w[i-7] + s1;
	}

	for(unsigned int i = 0; i < DIGEST512_SIZE; i++){
		wv[i] = hash_h512[i];
	}	

	for(int i = 0; i < 80; i++){
		S0 = (rightRotate64(wv[0], 28) ^ rightRotate64(wv[0], 34) ^ rightRotate64(wv[0], 39));
		S1 = (rightRotate64(wv[4], 14) ^ rightRotate64(wv[4], 18) ^ rightRotate64(wv[4], 41));
		ch = (wv[4] & wv[5]) ^ ((~wv[4]) & wv[6]);
		temp1 = wv[7] + S1 + ch + hash_k512[i] + w[i];
		maj = (wv[0] & wv[1]) ^ (wv[0] & wv[2]) ^ (wv[1] & wv[2]);
		temp2 = S0 + maj;

		wv[7] = wv[6];
		wv[6] = wv[5];
		wv[5] = wv[4];
		wv[4] = wv[3] + temp1;
		wv[3] = wv[2];
		wv[2] = wv[1];
		wv[1] = wv[0];
		wv[0] = temp1 + temp2;
	}

	for(unsigned int i = 0; i < DIGEST512_SIZE; i++){
		hash_h512[i] = hash_h512[i] + wv[i];
	}	
}

void SHA::transform32(unsigned long int* data){
	unsigned long int* w = new unsigned long int[80];
	unsigned long int* wv = new unsigned long int[5];
	unsigned long int f, k, temp;

	for(unsigned int i = 0; i < SHA1_BLOCK_SIZE; i++){
		w[i] = data[i];
	}

	for(int i = 16; i < 80; i++){
		w[i] = leftRotate32((w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16]), 1);
	}

	for(unsigned int i = 0; i < DIGEST1_SIZE; i++){
		wv[i] = hash_h1[i];
	}

	for(int i = 0; i < 80; i++){
		if(i >= 0 and i <= 19){
			f = (wv[1] & wv[2]) ^ ((~wv[1]) & wv[3]);
			k = 0x5A827999;

		} else if(i >= 20 and i <= 39){
			f = wv[1] ^ wv[2] ^ wv[3];
			k = 0x6ED9EBA1;			

		} else if(i >= 40 and i <= 59){
			f = (wv[1] & wv[2]) ^ (wv[1] & wv[3]) ^ (wv[2] & wv[3]);
			k = 0x8F1BBCDC;

		} else if(i >= 60 and i <= 79){
			f = wv[1] ^ wv[2] ^ wv[3];
			k = 0xCA62C1D6;

		}
		temp = leftRotate32(wv[0], 5) + f + wv[4] + k + w[i];
		wv[4] = wv[3];
		wv[3] = wv[2];
		wv[2] = leftRotate32(wv[1], 30);
		wv[1] = wv[0];
		wv[0] = temp;
	}

	for(unsigned int i = 0; i < DIGEST1_SIZE; i++){
		hash_h1[i] = hash_h1[i] + wv[i];
	}	

}

std::string SHA::execute(std::string input){	
	char* buffer;
	
	if(type == SHA_512){
		int length = input.length() * 8;
		int n_blocks = length/896 + 1;
		init64();

		unsigned long long int* block = new unsigned long long int[n_blocks * SHA512_BLOCK_SIZE];
		memset(block, 0, n_blocks * SHA512_BIT_BLOCK_SIZE);

		preProcessing64((unsigned char*)input.c_str(), input.length(), n_blocks, block);
		block[n_blocks * SHA512_BLOCK_SIZE - 1] = length;

		for(int i = 0; i < n_blocks; i++){
			unsigned long long int* data = new unsigned long long int[SHA512_BLOCK_SIZE];
			memset(data, 0, SHA512_BIT_BLOCK_SIZE);
			memcpy(data, block + (i * SHA512_BLOCK_SIZE), SHA512_BIT_BLOCK_SIZE);
			transform64(data);
		}

		buffer = new char[DIGEST512_SIZE * DIGEST512_BIT_SIZE];
		for(unsigned int i = 0; i < DIGEST512_SIZE; i++){
			sprintf(buffer + (i*SHA512_BLOCK_SIZE), "%016llx", hash_h512[i]);
		}
		delete[] block;
	} else if(type == SHA_1){
		init32();
		int length = input.length() * 8;
		int n_blocks = length/448 + 1;

		unsigned long int* block = new unsigned long int[n_blocks * SHA1_BLOCK_SIZE];
		memset(block, 0, n_blocks * SHA1_BIT_BLOCK_SIZE);

		preProcessing32((unsigned char*)input.c_str(), input.length(), n_blocks, block);
		block[n_blocks * SHA1_BLOCK_SIZE - 1] = length;

		for(int i = 0; i < n_blocks; i++){
			unsigned long int* data = new unsigned long int[SHA1_BLOCK_SIZE];
			memset(data, 0, SHA1_BIT_BLOCK_SIZE);
			memcpy(data, block + (i * SHA1_BLOCK_SIZE), SHA1_BIT_BLOCK_SIZE);
			transform32(data);
		}

		buffer = new char[DIGEST1_SIZE * DIGEST1_BIT_SIZE];
		for(unsigned int i = 0, value = 128; i < DIGEST1_SIZE; i++, value -= 32){
			sprintf(buffer + (i*(SHA1_BLOCK_SIZE/2)), "%08lx", (hash_h1[i] << value));
		}

	}
	
	return std::string(buffer);

}

void SHA::init64(){
	
	hash_h512[0] = (0x6a09e667f3bcc908);
	hash_h512[1] = (0xbb67ae8584caa73b);
	hash_h512[2] = (0x3c6ef372fe94f82b);
	hash_h512[3] = (0xa54ff53a5f1d36f1);
	hash_h512[4] = (0x510e527fade682d1);
	hash_h512[5] = (0x9b05688c2b3e6c1f);
	hash_h512[6] = (0x1f83d9abfb41bd6b);
	hash_h512[7] = (0x5be0cd19137e2179);

	
}

void SHA::init32(){
	hash_h1[0] = 0x67452301;
	hash_h1[1] = 0xEFCDAB89;
	hash_h1[2] = 0x98BADCFE;
	hash_h1[3] = 0x10325476;
	hash_h1[4] = 0xC3D2E1F0;
}

unsigned long int SHA::rightRotate32(unsigned long int value, int n){
	return ((value >> n) | (value << ((32) - n)));
}

unsigned long int SHA::leftRotate32(unsigned long int value, int n){
	return ((value << n) | (value >> ((32) - n)));
}

unsigned long long int SHA::rightRotate64(unsigned long long int value, int n){
	return ((value >> n) | (value << ((sizeof(value) << 3) - n)));
}

unsigned long long int SHA::leftRotate64(unsigned long long int value, int n){
	return ((value << n) | (value >> ((sizeof(value) << 3) - n)));
}

unsigned long long int SHA::pack64(unsigned char* str){
	unsigned long long int value;
	value = ((unsigned long long int)(str[7])) | ((unsigned long long int)(str[6]) << 8)
	 | ((unsigned long long int)(str[5]) << 16) | ((unsigned long long int)(str[4]) << 24)
	  | ((unsigned long long int)(str[3]) << 32) | ((unsigned long long int)(str[2]) << 40)
	   | ((unsigned long long int)(str[1]) << 48) | ((unsigned long long int)(str[0]) << 56);
	return value;
}

unsigned long int SHA::pack32(unsigned char* str){
	unsigned long long int value;
	value = ((unsigned long long int)(str[3])) | ((unsigned long long int)(str[2]) << 8)
	 | ((unsigned long long int)(str[1]) << 16) | ((unsigned long long int)(str[0]) << 24);
	return value;
}