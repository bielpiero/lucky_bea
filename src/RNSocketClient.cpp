#include "RNSocketClient.h"

RNSocketClient::RNSocketClient(){
		socketConn = new TCPSocketClient();
		thread_status = 0;
}

RNSocketClient::~RNSocketClient(){
	delete socketConn;
}

void RNSocketClient::init(const char* address, int port){
	socket_server_address.sin_family = AF_INET;
	socket_server_address.sin_addr.s_addr = inet_addr(address);
	if(strlen(address)==0){
		socket_server_address.sin_addr.s_addr = INADDR_ANY;
	}
	socket_server_address.sin_port = htons(port);
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
	int err = write(socketConn->getSocket(), cad, length);
	if (err == RN_NONE){
		error("SendBytes Error");
		return -1;
	}
	return 0;
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
		int r = recv(sd, cad, length, 0);
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