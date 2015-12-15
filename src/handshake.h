#ifndef HANDSHAKE_H
#define HANDSHAKE_H

#include <iostream>
#include <string>

enum wsFrameType { // errors starting from 0xF0
    WS_EMPTY_FRAME = 0xF0,
    WS_ERROR_FRAME = 0xF1,
    WS_INCOMPLETE_FRAME = 0xF2,
    WS_TEXT_FRAME = 0x01,
    WS_BINARY_FRAME = 0x02,
    WS_PING_FRAME = 0x09,
    WS_PONG_FRAME = 0x0A,
    WS_OPENING_FRAME = 0xF3,
    WS_CLOSING_FRAME = 0x08
};

class Handshake {
public:
	Handshake(){ 
		host = "";
		origin = "";
		key = "";
		resource = "";
		protocol = "";
		frameType = WS_EMPTY_FRAME;
	}
	virtual ~Handshake(){}

	std::string getHost(){ return host; }
	void setHost(std::string host) { this->host = host; }

	std::string getOrigin(){ return origin; }
	void setOrigin(std::string origin) { this->origin = origin; }

	std::string getKey(){ return key; }
	void setKey(std::string key) { this->key = key; }

	std::string getResource(){ return resource; }
	void setResource(std::string resource) { this->resource = resource; }

	std::string getProtocol(){ return protocol; }
	void setProtocol(std::string protocol) { this->protocol = protocol; }
private:
    std::string host;
    std::string origin;
    std::string key;
    std::string resource;
    std::string protocol;
    wsFrameType frameType;
};

#endif