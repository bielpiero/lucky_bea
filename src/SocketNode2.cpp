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
const std::string CSocketNode::version = "13";
const std::string CSocketNode::secret = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
const std::string CSocketNode::switchingProtocolField = "HTTP/1.1 101 Switching";
	
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CSocketNode::CSocketNode()
{
	socket_conn=INVALID_SOCKET;
	thread_status=0;
	webSocket = false;
	checkedWebSocket = false;
	handshakeDone = false;
}

CSocketNode::~CSocketNode()
{
	//#ifdef SOCKET_NODE_WINDOWS
	//	WSACleanup();
	//#endif
}

int CSocketNode::Init(const char *address,int port, int t){
	type=t;
	this->ip_address = new char[strlen(address)];
	strcpy(this->ip_address, address);
	
	socket_conn=INVALID_SOCKET;
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
			Error("Server: Error Socket");
			return -1;
		}
		int len = sizeof(socket_server_address);
		if (bind(socket_server,(struct sockaddr *) &socket_server_address,len) < 0){
			return -1;
		}
		// Damos como mximo 5 puertos de conexin.
		if (listen(socket_server, 5) < 0){
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
void CSocketNode::HandleConnection(void){
	webSocket = false;
	checkedWebSocket = false;
	handshakeDone = false;
	if(type==SOCKET_SERVER){

		if(socket_conn==INVALID_SOCKET){
			unsigned int len = sizeof(socket_address);

			fd_set readfds;
		
			FD_ZERO(&readfds);
			FD_SET(socket_server,&readfds);
			timeval timeout;
			timeout.tv_sec=0;
			timeout.tv_usec=10;//10000;
			int ret=select(socket_server + 1, &readfds, NULL, NULL, &timeout);
			if(ret==1){
				socket_conn = accept(socket_server,(struct sockaddr *)&socket_address, &len);
				
				//socket_conn = accept(socket_server, 0, 0);
				if(socket_conn==INVALID_SOCKET){
					return;
				}	
				int optval;
				setsockopt(socket_conn, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
				int buff_size = BUFFER_SIZE;
				setsockopt(socket_conn, SOL_SOCKET, SO_SNDBUF, &buff_size, sizeof(buff_size));
				setsockopt(socket_conn, SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size));
				OnConnection();		
			}
			
		}
	}

	if(type==SOCKET_CLIENT ){
		if(socket_conn==INVALID_SOCKET){
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
		
			socket_conn=aux_sock;
			int buff_size = BUFFER_SIZE;
			setsockopt(socket_conn, SOL_SOCKET, SO_SNDBUF, &buff_size, sizeof(buff_size));
			setsockopt(socket_conn, SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size));
		
			OnConnection();
		}
	}	
}
void CSocketNode::Error(const char* cad){
	printf("%s\n", cad);
	if(socket_conn!=INVALID_SOCKET)
	{	
		shutdown(socket_conn,SD_BOTH);
		closesocket(socket_conn);
		socket_conn=INVALID_SOCKET;
		OnConnection();
	}
}

int CSocketNode::SendMsg(const char opr, const char* cad,int length){
	if (socket_conn == INVALID_SOCKET)
		return -1;
	//Build header
	Buffer_out[1] = 48; //12345 / 256;
	Buffer_out[0] = 57; //12345 % 256;
	length++;
	if((length / 256) >= 256){
		int divi = length / 256;
		Buffer_out[2] = length % 256;
		Buffer_out[3] = divi % 256;
		Buffer_out[4] = divi / 256;
	} else {
		Buffer_out[2] = length % 256;
		Buffer_out[3] = 0;
		Buffer_out[4] = length / 256;
	}
	Buffer_out[5] = opr;
	memcpy(Buffer_out + 6, cad, length);
	
	//Send it
	int err = send(socket_conn, Buffer_out, length + 5, 0);
	//	delete[] cad_aux;
	if (err == SOCKET_ERROR){
		Error("SendMsg Error");
		return -1;
	}

	return 0;
}

int CSocketNode::SendBytes(char *cad, int length){
	//Send it
	int err = send(socket_conn, cad, length,0);
	if (err == SOCKET_ERROR ){
		Error("SendBytes Error");
		return -1;
	}
	return 0;
}

int CSocketNode::ReceiveBytes(char *cad, int *length,int timeout){
	if (socket_conn == INVALID_SOCKET)
		return -1;
	fd_set readfds; FD_ZERO(&readfds); FD_SET(socket_conn, &readfds);
	timeval tout; tout.tv_sec = 0; tout.tv_usec = 10000;
	int ret = select(socket_conn + 1, &readfds, NULL, NULL, &tout);
	if (1 == ret){
		int r = recv(socket_conn, cad, *length, 0);
		
		if (r == SOCKET_ERROR || r == 0){
			Error("Receive bytes Error");
			return -1;

		} else {
			*length = r;
			return 0;
		}
	}
	return -1;
}
int CSocketNode::ReceiveMsg(char* cad, int* size, int timeout){
	int result = 0;
	int ret;
	int nChars;
	int len;
	char* header = new char[5];
	char webHeader[3];
	char partHeader[2];
	wsState state = WS_STATE_OPENING;
	wsFrameType frameType = WS_INCOMPLETE_FRAME;
	Handshake hs;

	nChars = 3;

	if(ReceiveBytes(webHeader, &nChars, timeout) == 0){
		checkedWebSocket = true;
		if(memcmp(webHeader, "GET", sizeof(webHeader)) == 0){
			printf("WebSocket detected...\n");
			webSocket = true;
			len = BUFFER_SIZE;
			result = ReceiveBytes(Buffer_in, &len, timeout);
			Buffer_in[len] = 0;
			memcpy(cad, "GET", 3);
			cad += 3;
			memcpy(cad, Buffer_in, *size<len ? *size : len);
			cad -= 3;

			*size = len + 3;
			printf("------------>Received:%s\n------------>Size: %d\n", cad, *size);

			wsParseHandshake(cad, *size, &hs);
			

		} else {
			nChars = 2;
			ReceiveBytes(partHeader, &nChars, timeout);
			memcpy(header, webHeader, sizeof(webHeader));
			header += sizeof(webHeader);
			memcpy(header, partHeader, sizeof(partHeader));
			header -= sizeof(webHeader);

			if(header[3] != 0){
				len = header[2] + (header[3] + header[4] * 256) * 256;
			} else {
				len = header[2] + header[4] * 256;
			}
			
			if (header[0] != 57 ||	// 12345 % 256
				header[1] != 48 ||	// 12345 / 256
				len <= 0){
				Error("Header error");
				result = -2; //header error
			}
			result = ReceiveBytes(Buffer_in, &len, timeout);
			Buffer_in[len] = 0;
			memcpy(cad, Buffer_in, *size<len ? *size : len);

			if (*size<len)
				result = -3;//short buffer

			*size = len;
		}
	} else {
		result = -1; //no header
	}

	return result;
}

int CSocketNode::IsConnected(){
	if(socket_conn==INVALID_SOCKET)
		return 0;
	else
		return 1;
}

bool CSocketNode::isWebSocket(){
	return webSocket;
}

void* CSocketNode::LaunchThread(void* p){
	CSocketNode* node=(CSocketNode*) p;

	node->thread_status=1;
	while(node->thread_status==1) {
		if(!node->IsConnected()){
			node->HandleConnection();
		} else {	
			char msg[BUFFER_SIZE];
			int l=BUFFER_SIZE;
			if(0==node->ReceiveMsg(msg, &l, 0)){
				node->OnMsg(msg,l);
			}
		}
		Sleep(10);
	}
	node->thread_status=0;
	return NULL;
}

void CSocketNode::StartThread(){
	pthread_t t1;
	pthread_create(&t1,NULL,LaunchThread, (void *) (this)  );
}


int CSocketNode::Close(){
	if(thread_status==1){
		//wait until thread terminates
		thread_status=2;
		while(thread_status!=0)Sleep(100);
	}
		
	shutdown(socket_conn,SD_BOTH);
	closesocket(socket_conn);
	socket_conn=INVALID_SOCKET;
	closesocket(socket_server);
	return 1;
}

char* CSocketNode::getClientIPAddress(){
	char* ip_address = new char[15];
	socklen_t len;
	struct sockaddr_in addr;


	len = sizeof(addr);
	getpeername(socket_conn, (struct sockaddr*)&addr, &len);
	sprintf(ip_address, "%d.%d.%d.%d", int(addr.sin_addr.s_addr&0xFF), int((addr.sin_addr.s_addr&0xFF00)>>8), int ((addr.sin_addr.s_addr&0xFF0000)>>16), int((addr.sin_addr.s_addr&0xFF000000)>>24));

	return ip_address;
}

int CSocketNode::getClientPort(){

	socklen_t len;
	struct sockaddr_in addr;

	len = sizeof(addr);
	getpeername(socket_conn, (struct sockaddr*)&addr, &len);
	
	return addr.sin_port;

}

wsFrameType CSocketNode::wsParseHandshake(char* buffer, int size, Handshake* hs){
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

void CSocketNode::wsGetHandshakeAnswer(Handshake* hs, char* outFrame, int& outLength){
	std::string answer;
	unsigned char digest[20];

	answer = switchingProtocolField + WS_EOL;
	answer += upgradeField + " " + webSocketStr + WS_EOL;
	answer += connectionField + " " + upgrade2 + WS_EOL;

	if(hs->getKey().length() > 0){
		std::string acceptKey;
		acceptKey = hs->getKey() + secret;

		// perform sha

		//Little endian to big endian
		for(int i = 0; i < 20; i += 4){
			unsigned char character;
			character = digest[i];
			digest[i] = digest[i + 3];
			digest[i + 3] = character;

			character = digest[i + 1];
			digest[i + 1] = digest[i + 2];
			digest[i + 2] = character;
		}

		answer += acceptField + " " + acceptKey + WS_EOL;
	}
	if(hs->getProtocol().length() > 0){
		answer += protocolField + " " + hs->getProtocol() + WS_EOL;
	}
	answer += WS_EOL;
	outLength = answer.length();
	memcpy(outFrame, answer.c_str(), answer.length());
}

wsFrameType wsParseInputFrame(char* bufferIn, int sizeIn, char* bufferOut, int& sizeOut){

}

std::vector<std::string> CSocketNode::split(char* buffer, const char* delimiter){
	std::vector<std::string> result;

	char* current;
	current = strtok(buffer, delimiter);
	while(current != NULL){
		result.push_back(std::string(current));
		current = strtok(NULL, delimiter);
	}
	return result;

}