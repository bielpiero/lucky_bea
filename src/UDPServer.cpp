#include "UDPServer.h"

UDPServer::UDPServer(int port){
	init(port);
}

void UDPServer::init(int port){
	thread_status=0;
	socket_server = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket_server == INVALID_SOCKET){
		error("Server: Error Socket UDP");
		return;
	}
	int optval = 1;
  	setsockopt(socket_server, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));

	socket_local_address.sin_family = AF_INET;
	socket_local_address.sin_addr.s_addr = INADDR_ANY;
	socket_local_address.sin_port = htons(port);
	int err = bind(socket_server, (struct sockaddr*)&socket_local_address, sizeof(socket_local_address));
	if(err < 0){
		error("Binding UDP ERROR");
		return;
	}
	pthread_mutex_init(&clientsMutex, NULL);
}

UDPServer::~UDPServer(){
	pthread_mutex_destroy(&clientsMutex);
}

int UDPServer::sendData(const unsigned char* buffer, unsigned int length){
	pthread_mutex_lock(&clientsMutex);
	for(int i = 0; i < clients.size(); i++){
		struct sockaddr_in address = clients.at(i);
		long err = sendto(socket_server, buffer, length, 0, (struct sockaddr*)&address, sizeof(address));
		if(err < 0){
			perror("sendTo-UDP");
			//error("Sending Data Error");
			return -1;
		}
	}
	pthread_mutex_unlock(&clientsMutex);
	return 0;
}

int UDPServer::receiveData(unsigned char* buffer, unsigned int& size){
	int result = 0;
	struct sockaddr_in incommingAddress;
	unsigned int sourceSize = sizeof(incommingAddress);
	long err = recvfrom(socket_server, buffer, size, 0, (struct sockaddr *) &incommingAddress, &sourceSize);
	if(err < 0){
		error("recvFrom-UDP");
		result = -1;
	}
	if(memcmp(buffer, "HELLO", size) == 0){
		
		processHello(incommingAddress);
		result = 1;							//Hello...
		
	} else if(memcmp(buffer, "BYBYE", size) == 0){
		processByBye(incommingAddress);		//ByBye
		result = 2;
	}
	size = (int)err;
	return result;
}

void UDPServer::processHello(struct sockaddr_in client){
	char* ipAddress = new char[15];
	unsigned int port = 0;
	pthread_mutex_lock(&clientsMutex);
	getClientIPAndPort(client, ipAddress, port);
	RNUtils::printLn("Incomming Connection ip: %s, port: %u", ipAddress, port);
	clients.push_back(client);

	pthread_mutex_unlock(&clientsMutex);
}

void UDPServer::processByBye(struct sockaddr_in client){
	bool found = false;
	int index = RN_NONE;
	pthread_mutex_lock(&clientsMutex);
	for (int i = 0; i < clients.size() and not found; i++){
		char* ipAddressConnected = new char[15];
		char* ipAddressToDisconnect = new char[15];
		unsigned int portConnected, portToDisconnect;
		getClientIPAndPort(client, ipAddressToDisconnect, portToDisconnect);
		getClientIPAndPort(clients.at(i), ipAddressConnected, portConnected);

		if((strcmp(ipAddressToDisconnect, ipAddressConnected) == 0) and (portToDisconnect == portConnected)){
			found = true;
			index = i;
			RNUtils::printLn("Disconnection from ip: %s, port: %u", ipAddressToDisconnect, portToDisconnect);
		}
	}
	
	if(index != RN_NONE){
		clients.erase(clients.begin() + index);
	}

	pthread_mutex_unlock(&clientsMutex);

}

void UDPServer::startThread(){
	pthread_t t1;
	pthread_create(&t1, NULL, launchThread, (void *)(this));
}

void* UDPServer::launchThread(void* p){
	UDPServer* self = (UDPServer*)p;
	self->thread_status=1;
	while(RNUtils::ok() and self->thread_status==1){
		unsigned char* msg = new unsigned char[BUFFER_SIZE];
		unsigned int l = BUFFER_SIZE;
		
		if(self->receiveData(msg, l) == 0){
			self->OnMessageReceivedWithData(msg, l);
		}
		RNUtils::sleep(10);
		delete [] msg;
	}
	self->thread_status=0;
	return NULL;
}

void UDPServer::closeConnection(){
	if(thread_status == 1){
		thread_status = 2;
		while (thread_status != 0){
			RNUtils::sleep(100);
		}
	}	
	closesocket(socket_server);
}

void UDPServer::error(const char* err){
	//std::cout << std::string(err) << std::endl;
	perror(err);
	if (socket_server != INVALID_SOCKET){
		closeConnection();
	}
}

void UDPServer::getClientIPAndPort(struct sockaddr_in client, char* ip, unsigned int& port){
	sprintf(ip, "%d.%d.%d.%d", int(client.sin_addr.s_addr&0xFF), int((client.sin_addr.s_addr&0xFF00)>>8), int((client.sin_addr.s_addr&0xFF0000)>>16), int((client.sin_addr.s_addr&0xFF000000)>>24));
	port = ntohs(client.sin_port);
}
