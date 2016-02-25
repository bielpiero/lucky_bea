// SocketNode.cpp: implementation of the CSocketNode class.
//
//////////////////////////////////////////////////////////////////////
#include "SocketNode2.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/timeb.h>
#include <iostream>

const std::string CSocketNode::connectionField = "Connection:";
const std::string CSocketNode::upgrade = "upgrade";
const std::string CSocketNode::upgrade2 = "Upgrade";
const std::string CSocketNode::upgradeField = "Upgrade:";
const std::string CSocketNode::websocket = "websocket";
const std::string CSocketNode::webSocketStr = "WebSocket";
const std::string CSocketNode::hostField = "Host:";
const std::string CSocketNode::originField = "Origin:";
const std::string CSocketNode::keyField = "Sec-WebSocket-Key:";
const std::string CSocketNode::protocolField = "Sec-WebSocket-Protocol:";
const std::string CSocketNode::versionField = "Sec-WebSocket-Version:";
const std::string CSocketNode::acceptField = "Sec-WebSocket-Accept:";
const std::string CSocketNode::extensionsField = "Sec-WebSocket-Extensions:";
const std::string CSocketNode::version = "13";
const std::string CSocketNode::secret = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
const std::string CSocketNode::switchingProtocolField = "HTTP/1.1 101 Switching Protocols";
const std::string CSocketNode::applicationDataField = "Application data";
	
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CSocketNode::CSocketNode()
{
	tokenOwner = -1;
	lastTokenOwner = -1;
	for(int i = 0; i < MAX_CLIENTS; i++){
		socket_conn[i] = TCPSocketClient(INVALID_SOCKET);
	}

	thread_status = 0;
	//pingTimerThreadStatus = 0;
	//pthread_create(&pingThread, NULL, pingTimerThread, (void *)(this));
}

CSocketNode::~CSocketNode(){
	pingTimerThreadStatus = 0;
	pthread_cancel(pingThread);
}

int CSocketNode::init(const char *address,int port, int t){
	type=t;
	this->ip_address = new char[strlen(address)];
	strcpy(this->ip_address, address);
	
	for(int i = 0; i < MAX_CLIENTS; i++){
		socket_conn[i] = TCPSocketClient(INVALID_SOCKET);
	}

// Configuracion del socket del Servidor	
	socket_server_address.sin_family = AF_INET;
	socket_server_address.sin_addr.s_addr = inet_addr(address);
	if(strlen(address)==0)
		socket_server_address.sin_addr.s_addr = INADDR_ANY;
	socket_server_address.sin_port = htons(port);
//SERVER
	if(type==SOCKET_SERVER){
		socket_server = socket(AF_INET, SOCK_STREAM, 0);

		if (socket_server==INVALID_SOCKET){
			//Error(socketIndex, "Server: Error Socket");
			return -1;
		}

		int optval = 1;
		setsockopt(socket_server, SOL_SOCKET, SO_REUSEADDR, (char*)&optval, sizeof(optval));

		unsigned int len = sizeof(socket_server_address);
		
		if (bind(socket_server,(struct sockaddr *) &socket_server_address, len) < 0){
			perror("Hay un peo en el bind");
			return -1;
		}
		// Damos como mximo 5 puertos de conexin.
		if (listen(socket_server, 5) < 0){
			perror("Hay un peo en el listen");
			return -1;
		}
			
		return 0;
	}
//CLIENT
	if(type==SOCKET_CLIENT){
		return 0;
	}
	return -1;
}
void CSocketNode::handleConnection(void){

	if(type==SOCKET_SERVER){


		unsigned int len = sizeof(socket_address);

		fd_set readfds;
	
		FD_ZERO(&readfds);
		FD_SET(socket_server, &readfds);
		int maxSD = socket_server;

		for(int i = 0; i < MAX_CLIENTS; i++){
			int sd = socket_conn[i].getSocket();
			if(sd > INVALID_SOCKET){
				FD_SET(sd, &readfds);
			}

			if(sd > maxSD){
				maxSD = sd;
			}
		}

		timeval timeout;
		timeout.tv_sec=0;
		timeout.tv_usec=10;//10000;
		int ret=select(socket_server + 1, &readfds, NULL, NULL, &timeout);
		if(ret==1){
			if(FD_ISSET(socket_server, &readfds)){
				int newIncommingConnection = accept(socket_server,(struct sockaddr *)&socket_address, &len);
				if(newIncommingConnection != INVALID_SOCKET){
					bool stored = false;
					for(int i = 0; i < MAX_CLIENTS and not stored; i++)	{
						if(socket_conn[i].getSocket() == INVALID_SOCKET){
							stored = true;

							socket_conn[i].setSocket(newIncommingConnection);
							socket_conn[i].setIsAWebSocket(false);
							socket_conn[i].setCheckedIfIsAWebSocket(false);
							socket_conn[i].setHandshakeDone(false);

							int optval;
							setsockopt(socket_conn[i].getSocket(), SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
							int buff_size = BUFFER_SIZE;
							setsockopt(socket_conn[i].getSocket(), SOL_SOCKET, SO_SNDBUF, &buff_size, sizeof(buff_size));
							setsockopt(socket_conn[i].getSocket(), SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size));
							std::cout << "Salta en el handleConnection" << std::endl;
							onConnection(i);
						}
					}
				}
			}
		}
	}

	if(type == SOCKET_CLIENT){
		if(socket_conn[CLIENT_DEFAULT_INDEX].getSocket()==INVALID_SOCKET){
			//Aux socket to avoid sock_conn!=INVALID without connecting
			int aux_sock = 0;
			aux_sock=socket(AF_INET, SOCK_STREAM, 0);
			if (aux_sock == INVALID_SOCKET){
				return;
			}
		

			int len= sizeof(socket_server_address);
			if(connect(aux_sock,(const struct sockaddr *) &socket_server_address,len)!=0){
				shutdown(aux_sock,SD_BOTH);
				closesocket(aux_sock);
				return;
			}
		
			socket_conn[CLIENT_DEFAULT_INDEX].setSocket(aux_sock);
			int buff_size = BUFFER_SIZE;
			setsockopt(socket_conn[CLIENT_DEFAULT_INDEX].getSocket(), SOL_SOCKET, SO_SNDBUF, &buff_size, sizeof(buff_size));
			setsockopt(socket_conn[CLIENT_DEFAULT_INDEX].getSocket(), SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size));
		
			onConnection(CLIENT_DEFAULT_INDEX);
		}
	}

}
void CSocketNode::error(int socketIndex, const char* cad){
	//printf("%s\n", cad);
	perror(cad);
	if(socket_conn[socketIndex].getSocket()!=INVALID_SOCKET)
	{	
		shutdown(socket_conn[socketIndex].getSocket(), SD_BOTH);
		closesocket(socket_conn[socketIndex].getSocket());
		socket_conn[socketIndex].setSocket(INVALID_SOCKET);
		std::cout << "Salta en el error" << std::endl;
		onConnection(socketIndex);
	}
}

int CSocketNode::sendMsg(int socketIndex, const char opr, const char* cad, unsigned long long int length){
	if (socket_conn[socketIndex].getSocket() == INVALID_SOCKET)
		return -1;
	int pos = 0;
	int result = 0;
	memset(Buffer_out, 0, BUFFER_SIZE);

	if(!socket_conn[socketIndex].isWebSocket()){
		//Build header
		Buffer_out[pos] = 57; //12345 % 256;
		Buffer_out[pos + 1] = 48; //12345 / 256;
		
		length++;
		if((length / 256) >= 256){
			int divi = length / 256;
			Buffer_out[pos + 2] = length % 256;
			Buffer_out[pos + 3] = divi % 256;
			Buffer_out[pos + 4] = divi / 256;
		} else {
			Buffer_out[pos + 2] = length % 256;
			Buffer_out[pos + 3] = 0;
			Buffer_out[pos + 4] = length / 256;
		}
		Buffer_out[pos + 5] = opr;
		pos = 5;
		memcpy(Buffer_out + pos + 1, cad, length);
		
	} else {
		length++;
		Buffer_out[pos++] = 0x81;
		Buffer_out[pos++] = length <= 125 ? length : (length <= 65535 ? 126 : 127);
		if(Buffer_out[pos-1] == 126){
			Buffer_out[pos++] = (length >> 8) & 0xff;
			Buffer_out[pos++] = (length & 0xff);
		} else if(Buffer_out[pos-1] == 127){ 
			Buffer_out[pos++] = (length >> 56) & 0xff;
			Buffer_out[pos++] = (length >> 48) & 0xff;
			Buffer_out[pos++] = (length >> 40) & 0xff;
			Buffer_out[pos++] = (length >> 32) & 0xff;
			Buffer_out[pos++] = (length >> 24) & 0xff;
			Buffer_out[pos++] = (length >> 16) & 0xff;
			Buffer_out[pos++] = (length >> 8) & 0xff;
			Buffer_out[pos++] = (length & 0xff);
		}
		Buffer_out[pos++] = opr;
		
		for(int i = 0; i < length - 1; i++){
			Buffer_out[pos++] = cad[i];	
		}
		length = 0;

	}
	result = sendBytes(socketIndex, Buffer_out, length + pos);	
	return result;
}

int CSocketNode::wsSendPingMsg(int socketIndex){
	int pos = 0;
	memset(Buffer_out, 0, BUFFER_SIZE);
	if(socket_conn[socketIndex].isWebSocket()){
		Buffer_out[pos++] = 0x89;
		Buffer_out[pos++] = applicationDataField.length();
		for(int i = 0; i < applicationDataField.length(); i++){
			Buffer_out[pos++] = applicationDataField.at(i);	
		}
		sendBytes(socketIndex, Buffer_out, pos);
	}
}

int CSocketNode::sendBytes(int socketIndex, char *cad, unsigned long long int length){
	//Send it
	int err = write(socket_conn[socketIndex].getSocket(), cad, length);
	if (err == SOCKET_ERROR ){
		error(socketIndex, "SendBytes Error");
		return -1;
	}
	return 0;
}

int CSocketNode::receiveBytes(int socketIndex, char *cad, unsigned long long int& length, int timeout){

	if (socket_conn[socketIndex].getSocket() == INVALID_SOCKET)
		return -1;
	int sd = socket_conn[socketIndex].getSocket();
	fd_set readfds; FD_ZERO(&readfds); FD_SET(sd, &readfds);
	timeval tout; tout.tv_sec = 0; tout.tv_usec = 10000;

	if(select(sd + 1, &readfds, NULL, NULL, &tout) == 1){
		int r = recv(sd, cad, length, 0);

		if(r == 0 || r == SOCKET_ERROR){
			error(socketIndex, "Receive bytes Error");
			return -1;
		} else {
			length = r;
			return 0;
		}
	}
	return -1;
}
int CSocketNode::receiveMsg(int socketIndex, char* cad, unsigned long long int& size, int timeout){
	int result = 0;
	int ret;
	unsigned long long int nChars;
	unsigned long long int len;
	bool firstReceiveDone = false;
	char* header = new char[5];
	wsState state = WS_STATE_OPENING;
	wsFrameType frameType = WS_INCOMPLETE_FRAME;
	Handshake hs;

	
	if(!socket_conn[socketIndex].checkedIfIsAWebSocket()){
		nChars = 3;
		if(receiveBytes(socketIndex, header, nChars, timeout) == 0){
			socket_conn[socketIndex].setCheckedIfIsAWebSocket(true);
			if(memcmp(header, "GET", nChars) == 0){
				printf("WebSocket detected...\n");
				socket_conn[socketIndex].setIsAWebSocket(true);
				len = BUFFER_SIZE;
				result = receiveBytes(socketIndex, Buffer_in, len, timeout);
				Buffer_in[len] = 0;
				memcpy(cad, "GET", 3);
				memcpy(cad + 3, Buffer_in, size<len ? size : len);

				size = len + 3;
				wsParseHandshake(cad, size, &hs);
				memset(&Buffer_out, 0, BUFFER_SIZE);
				len = 0;
				wsGetHandshakeAnswer(&hs, Buffer_out, len);
				if(sendBytes(socketIndex, Buffer_out, len) == 0){
					printf("Handshake done...\n");
				}

			} else if(!socket_conn[socketIndex].isWebSocket()){
				nChars = 2;
				if(receiveBytes(socketIndex, header + 3, nChars, timeout) == 0){
					firstReceiveDone = true;

					if(header[3] != 0){
						len = header[2] + (header[3] + header[4] * 256) * 256;
					} else {
						len = header[2] + header[4] * 256;
					}
					
					if (header[0] != 57 ||	// 12345 % 256
						header[1] != 48 ||	// 12345 / 256
						len <= 0){
						error(socketIndex, "Header error");
						result = -2; //header error
					}
					result = receiveBytes(socketIndex, Buffer_in, len, timeout);
					Buffer_in[len] = 0;
					memcpy(cad, Buffer_in, size < len ? size : len);

					if (size < len)
						result = -3;//short buffer

					size = len;
				} else {
					result = -1;
				}
			}
		} else {
			result = -1;
		}
	}

	if(socket_conn[socketIndex].checkedIfIsAWebSocket() and !socket_conn[socketIndex].isWebSocket() and !firstReceiveDone){
		nChars = 5;
		if(receiveBytes(socketIndex, header, nChars, timeout) == 0){
			if(header[3] != 0){
				len = header[2] + (header[3] + header[4] * 256) * 256;
			} else {
				len = header[2] + header[4] * 256;
			}
			
			if (header[0] != 57 ||	// 12345 % 256
				header[1] != 48 ||	// 12345 / 256
				len <= 0){
				error(socketIndex, "Header error");
				result = -2; //header error
			}
			memset(&Buffer_in, 0, BUFFER_SIZE);
			memset(cad, 0, BUFFER_SIZE);
			result = receiveBytes(socketIndex, Buffer_in, len, timeout);
			Buffer_in[len] = 0;
			memcpy(cad, Buffer_in, size<len ? size : len);

			if (size<len)
				result = -3;//short buffer

			size = len;

		} else {
			result = -1;
		}

	} else if(socket_conn[socketIndex].checkedIfIsAWebSocket() and socket_conn[socketIndex].isWebSocket()){
		len = BUFFER_SIZE;
		memset(&Buffer_in, 0, BUFFER_SIZE);
		if((result = receiveBytes(socketIndex, Buffer_in, len, timeout)) == 0){
			
			char bufferIn[BUFFER_SIZE];
			memset(bufferIn, 0, BUFFER_SIZE);
			memset(cad, 0, BUFFER_SIZE);
			unsigned long long int sizeOutput = 0;
			wsParseInputFrame((unsigned char*)Buffer_in, len, bufferIn, sizeOutput);
			bufferIn[sizeOutput] = 0;
			memcpy(cad, bufferIn, sizeOutput);
			size = sizeOutput;
			
		} else{
			result = -1;
		}
	}

	return result;
}

int CSocketNode::isConnected(int socketIndex){
	if(socket_conn[socketIndex].getSocket()==INVALID_SOCKET)
		return 0;
	else
		return 1;
}

bool CSocketNode::isWebSocket(int socketIndex){
	return socket_conn[socketIndex].isWebSocket();
}

void* CSocketNode::launchThread(void* p){
	CSocketNode* self=(CSocketNode*) p;

	self->thread_status=1;
	while(self->thread_status==1) {

		self->handleConnection();
		if(self->type == SOCKET_SERVER){
			for(int i = 0; i < MAX_CLIENTS; i++){
				if(self->socket_conn[i].getSocket() != INVALID_SOCKET){
					char msg[BUFFER_SIZE];
					unsigned long long int l=BUFFER_SIZE;
					if(self->receiveMsg(i, msg, l, 0) == 0){
						if(l > 0){
							self->onMsg(i, msg,l);
						}
					}
				}
			}
		} else {
			if(self->socket_conn[CLIENT_DEFAULT_INDEX].getSocket() != INVALID_SOCKET){
				char msg[BUFFER_SIZE];
				unsigned long long int l=BUFFER_SIZE;
				if(self->receiveMsg(CLIENT_DEFAULT_INDEX, msg, l, 0) == 0){
					self->onMsg(CLIENT_DEFAULT_INDEX, msg, l);
				}
			}

		}
		Sleep(10);
	}
	self->thread_status=0;
	return NULL;
}

void CSocketNode::startThread(){
	pthread_t t1;
	pthread_create(&t1,NULL,launchThread, (void *) (this)  );
}


int CSocketNode::closeConnection(){
	if(thread_status==1){
		//wait until thread terminates
		thread_status=2;
		while(thread_status!=0)Sleep(100);
	}
	for(int i = 0; i < MAX_CLIENTS; i++){
		if(socket_conn[i].getSocket() != INVALID_SOCKET){
			shutdown(socket_conn[i].getSocket(), SD_BOTH);
			closesocket(socket_conn[i].getSocket());
			socket_conn[i].setSocket(INVALID_SOCKET);
		}
	}
	
	closesocket(socket_server);
	return 1;
}

char* CSocketNode::getClientIPAddress(int socketIndex){
	char* ip_address = new char[15];
	socklen_t len;
	struct sockaddr_in addr;


	len = sizeof(addr);
	getpeername(socket_conn[socketIndex].getSocket(), (struct sockaddr*)&addr, &len);
	sprintf(ip_address, "%d.%d.%d.%d", int(addr.sin_addr.s_addr&0xFF), int((addr.sin_addr.s_addr&0xFF00)>>8), int ((addr.sin_addr.s_addr&0xFF0000)>>16), int((addr.sin_addr.s_addr&0xFF000000)>>24));

	return ip_address;
}

int CSocketNode::getClientPort(int socketIndex){

	socklen_t len;
	struct sockaddr_in addr;

	len = sizeof(addr);
	getpeername(socket_conn[socketIndex].getSocket(), (struct sockaddr*)&addr, &len);
	
	return ntohs(addr.sin_port);

}

int CSocketNode::getServerPort(){
	socklen_t len;
	struct sockaddr_in addr;

	len = sizeof(addr);
	if (getsockname(socket_server, (struct sockaddr *) &addr, &len) < 0){
    		return -1;
	}

	return ntohs(addr.sin_port);

}

int CSocketNode::getTokenOwner(){
	return tokenOwner;
}

void CSocketNode::setTokenOwner(int token){
	setLastTokenOwner(this->tokenOwner);
	this->tokenOwner = token;
}

int CSocketNode::getLastTokenOwner(){
	return lastTokenOwner;
}

void CSocketNode::setLastTokenOwner(int token){
	this->lastTokenOwner = token;
}

wsFrameType CSocketNode::wsParseHandshake(char* buffer, unsigned long long int size, Handshake* hs){
	std::vector<std::string> strings = split(buffer, WS_EOL);

	for(int i = 0; i < strings.size(); i++){
		std::vector<std::string> spCurrentString = split((char*)strings[i].c_str(), " ");
		if(spCurrentString[0] == hostField){
			hs->setHost(spCurrentString[1]);
		} else if(spCurrentString[0] == originField){
			hs->setOrigin(spCurrentString[1]);
		} else if(spCurrentString[0] == keyField){
			hs->setKey(spCurrentString[1]);
		} else if(spCurrentString[0] == protocolField){
			hs->setProtocol(spCurrentString[1]);
		}
	}

	std::cout << "Handshake Parsed... OK" << std::endl;

	return WS_OPENING_FRAME;
}

void CSocketNode::wsGetHandshakeAnswer(Handshake* hs, char* outFrame, unsigned long long int& outLength){
	std::string answer;

	answer = switchingProtocolField + WS_EOL;
	answer += upgradeField + " " + websocket + WS_EOL;
	answer += connectionField + " " + upgrade2 + WS_EOL;

	if(hs->getKey().length() > 0){
		std::string acceptKey;
		//std::cout << hs->getKey() << std::endl;
		acceptKey = hs->getKey() + secret;

		// perform sha
		SHA* sha = new SHA;
		acceptKey = sha->execute(acceptKey);

		//std::cout << "Hash code: " << acceptKey << std::endl;
		unsigned char* hashCode = new unsigned char[20];
		char* resultHashCode = (char*)acceptKey.c_str();
		int index = 0;
		for(int i = 0; i < acceptKey.length(); i+=2){
			char* hex = new char[3];
			memcpy(hex, resultHashCode + i, 2);
			hex[2] = 0;
			hashCode[index] = (unsigned char)strtol(hex, NULL, 16);
			index++;
		}

		acceptKey = Base64::encode(hashCode, 20);

		answer += acceptField + " " + acceptKey + WS_EOL;
	}
	if(hs->getProtocol().length() > 0){
		answer += protocolField + " " + hs->getProtocol() + WS_EOL;
	}
	answer += WS_EOL;
	outLength = answer.length();
	memcpy(outFrame, answer.c_str(), answer.length());
}

wsFrameType CSocketNode::wsParseInputFrame(unsigned char* bufferIn, unsigned long long int sizeIn, char* bufferOut, unsigned long long int& sizeOut){
	wsFrameType result = WS_CLOSING_FRAME;
	unsigned char opCode = bufferIn[0] & 0x0F;
	if(opCode == WS_TEXT_FRAME){
		unsigned long long int payload = 0;
		int pos = 2;
		int lengthField = bufferIn[1] & (~0x80);
		if(lengthField <= 125){
			payload = lengthField;
		} else if(lengthField == 126){
			payload = (bufferIn[2] << 8) + bufferIn[3];
			pos += 2;
		} else if (lengthField == 127){
			payload += (((unsigned long long int)bufferIn[2]) << 56);
			payload += (((unsigned long long int)bufferIn[3]) << 48);
			payload += (((unsigned long long int)bufferIn[4]) << 40);
			payload += (((unsigned long long int)bufferIn[5]) << 32);
			payload += (((unsigned long long int)bufferIn[6]) << 24);
			payload += (((unsigned long long int)bufferIn[7]) << 16);
			payload += (((unsigned long long int)bufferIn[8]) << 8);
			payload += bufferIn[9];
			pos += 8;
		}

		if(sizeIn < (payload + pos)){
			result = WS_INCOMPLETE_FRAME;
		} else {
			unsigned char msg_masked = (bufferIn[1] >> 7) & 0x01;
			if(msg_masked){
				unsigned int mask = *((unsigned int*)(bufferIn + pos));
				pos += 4;

				unsigned char* current = bufferIn+pos;
				for(unsigned long long int i = 0; i < payload; i++){
					current[i] = current[i] ^ ((unsigned char*)(&mask))[i%4];
				}
			}

			memcpy(bufferOut, bufferIn+pos, payload);
			bufferOut[payload] = 0;
			sizeOut = payload;
		}
	} else if(opCode == WS_PING_FRAME){
		//printf("Llego un frame de PING....\n");
	} else if(opCode == WS_PONG_FRAME){
		//printf("Llego un frame de PONG....\n");
	}
}

std::vector<std::string> CSocketNode::split(char* buffer, const char* delimiter){
	std::vector<std::string> result;

	char* current;

	current = std::strtok(buffer, delimiter);

	while(current != NULL){
		result.push_back(std::string(current));
		current = std::strtok(NULL, delimiter);
	}
	return result;

}

void* CSocketNode::pingTimerThread(void* p){
	CSocketNode* self=(CSocketNode*)p;

	self->pingTimerThreadStatus = 1;
	while(self->pingTimerThreadStatus == 1){
		for(int i = 0; i < MAX_CLIENTS; i++){
			if(self->isConnected(i) and self->isWebSocket(i)){
				self->wsSendPingMsg(i);
				Sleep(10000);
			}
		}
	}

}