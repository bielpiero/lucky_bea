#include "RNSocketClient.h"

RNSocketClient::RNSocketClient(){
    this->ipAddress = new char[15];
    socketConn = new TCPSocketClient();
    thread_status = 0;
}

RNSocketClient::~RNSocketClient(){
	delete socketConn;
}

void RNSocketClient::init(const char* address, int portTCP, int portUDP){
    this->portTCP = portTCP;
    this->portUDP = portUDP;
    strcpy(this->ipAddress, address);
	socket_server_address.sin_family = AF_INET;
	socket_server_address.sin_addr.s_addr = inet_addr(address);
	if(strlen(address)==0){
		socket_server_address.sin_addr.s_addr = INADDR_ANY;
	}
	socket_server_address.sin_port = htons(portTCP);
}

void RNSocketClient::startThread(){
	pthread_create(&socketsThread, NULL, launchThread, (void *) (this)  );
}

void RNSocketClient::handleConnection(){
	if(socketConn->getSocket() == RN_NONE){
		//Aux socket to avoid sock_conn!=INVALID without connecting
		int aux_sock = 0;
		aux_sock = socket(AF_INET, SOCK_STREAM, 0);
        		if (aux_sock == RN_NONE){
			return;
		}
		

		int len = sizeof(socket_server_address);
		if(connect(aux_sock, (const struct sockaddr *)&socket_server_address, len) != 0){
			shutdown(aux_sock, SHUT_RD);
			close(aux_sock);
			return;
		}
        
		socketConn->setSocket(aux_sock);
		int optval = 1;
		setsockopt(socketConn->getSocket(), SOL_SOCKET, SO_REUSEADDR, (char*)&optval, sizeof(optval));
		int buff_size = RN_BUFFER_SIZE;
		setsockopt(socketConn->getSocket(), SOL_SOCKET, SO_SNDBUF, &buff_size, sizeof(buff_size));
		setsockopt(socketConn->getSocket(), SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size));
		
		onConnection();
        if(this->portUDP > RN_NONE){
            socketConn->setUDPClient(new UDPClient(this->ipAddress, this->portUDP));
        }

	}
}

int RNSocketClient::isConnected(){
	if(socketConn->getSocket() == RN_NONE)
		return 0;
	else
		return 1;
}

int RNSocketClient::sendBytes(unsigned char *cad, unsigned long long int length){
	//Send it
	unsigned long int err = write(socketConn->getSocket(), cad, length);
	if (err == RN_NONE){
		error("SendBytes Error");
		return -1;
	}
	return 0;
}

int RNSocketClient::sendMsg(const char opr, unsigned char *cad, unsigned long long length){
    if (socketConn->getSocket() == INVALID_SOCKET)
        return -1;
    int pos = 0;
    int result = 0;
    memset(Buffer_out, 0, BUFFER_SIZE);
    
    if(!socketConn->isWebSocket()){
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
    result = sendBytes(Buffer_out, length + pos);
    return result;
}

void RNSocketClient::error(const char* cad){
	perror(cad);
	if(socketConn->getSocket() != RN_NONE)
	{	
		shutdown(socketConn->getSocket(), SHUT_RD);
		close(socketConn->getSocket());
		socketConn->setSocket(RN_NONE);
		std::cout << "Salta en el error" << std::endl;
		onConnection();
	}
}

int RNSocketClient::receiveBytes(unsigned char* cad, unsigned long long int& length, int timeout){
	if (socketConn->getSocket() == RN_NONE)
		return -1;
	int sd = socketConn->getSocket();
	fd_set readfds; FD_ZERO(&readfds); FD_SET(sd, &readfds);
	timeval tout; tout.tv_sec = 0; tout.tv_usec = 10000;
	int rselect;
	if((rselect = select(sd + 1, &readfds, NULL, NULL, &tout)) == 1){
		unsigned long int r = recv(sd, cad, length, 0);
		if(r == 0 || r == RN_NONE){
			error("Receive bytes error");
			return -1;
		} else {

			length = r;
			return 0;
		}
	} 
	return -1;
}

void* RNSocketClient::launchThread(void* p){
	RNSocketClient* self = (RNSocketClient*)p;

	self->thread_status = 1;
	while(RNUtils::ok() and self->thread_status == 1) {
		if(self->isConnected() == 0){
			self->handleConnection();
		} else {
			unsigned char* msg = new unsigned char[1024];
			unsigned long long int l = 1024;
			if(self->receiveBytes(msg, l, 0) == 0){			
				self->onMsg((char*)msg, l);
			}
		}	
		RNUtils::sleep(10);
	}
	self->thread_status = 0;
	return NULL;
}

int RNSocketClient::closeConnection(){

	if(thread_status == 1){
		thread_status = 2;
		while(thread_status != 0) RNUtils::sleep(10);
	}
	pthread_cancel(socketsThread);
	RNUtils::printLn("Finished Client Connection Thread...");

	if(socketConn->getSocket() != RN_NONE){
		shutdown(socketConn->getSocket(), SHUT_RD);
		close(socketConn->getSocket());
		socketConn->setSocket(RN_NONE);
	}

	return 1;
}
