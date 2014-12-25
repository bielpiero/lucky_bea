#include "UDPClient.h"
#include <iostream>

UDPClient::UDPClient(int port){
	init("", port);
}

UDPClient::UDPClient(const char* address, int port){
	init(address, port);
}

void UDPClient::init(const char* address, int port){
	thread_status=0;
	socket_conn = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket_conn == INVALID_SOCKET)
	{
		error("Server: Error Socket");
		return;
	}
	if(strlen(address) == 0){
		socket_local_address.sin_family = AF_INET;
		socket_local_address.sin_addr.s_addr = INADDR_ANY;
		socket_local_address.sin_port = htons(port);
		int err = bind(socket_conn, (struct sockaddr*)&socket_local_address, sizeof(socket_local_address));
		if(err < 0){
			error("Binding ERROR");
			return;
		}
	}
	if(strlen(address) != 0){
		socket_dest_address.sin_family = AF_INET;
		socket_dest_address.sin_addr.s_addr = inet_addr(address);
		socket_dest_address.sin_port = htons(port);
		
	}
}

UDPClient::~UDPClient()
{
}

int UDPClient::sendData(unsigned const char* buffer,int length){
	long err = sendto(socket_conn, buffer, length, 0, (struct sockaddr*)&socket_dest_address, sizeof(socket_dest_address));
	if(err < 0){
		perror("send");
		//error("Sending Data Error");
		return -1;
	}
	return 0;
}

int UDPClient::receiveData(unsigned char* buffer, int size){
	struct sockaddr_in source;
	unsigned int sourceSize = sizeof(source);
	long err = recvfrom(socket_conn, buffer, size, 0, (struct sockaddr*)&source, &sourceSize);
	if(err < 0){
		error("Receiving Data Error");
		return -1;
	}
	return 0;
}	

void UDPClient::startThread(){
	pthread_t t1;
	pthread_create(&t1, NULL, launchThread, (void *)(this));
}

void* UDPClient::launchThread(void* p)
{
	UDPClient* node=(UDPClient*)p;

	node->thread_status=1;
	while(node->thread_status==1)
	{
		unsigned char* msg = new unsigned char[BUFFER_SIZE];
		int l=BUFFER_SIZE;
		if(0==node->receiveData(msg,l))
		{
			node->OnMessageReceivedWithData(msg,l);
		}
		Sleep(10);
	}
	node->thread_status=0;
	return NULL;
}

void UDPClient::OnMessageReceivedWithData(unsigned char* data, int length){

}

void UDPClient::closeConnection(){
	if(thread_status == 1){
		thread_status = 2;
		while (thread_status != 0){
			Sleep(100);
		}
	}	
	closesocket(socket_conn);
}

void UDPClient::error(const char* err){
	//std::cout << std::string(err) << std::endl;
	perror(err);
	if (socket_conn != INVALID_SOCKET){
		closeConnection();
	}
}

char* UDPClient::getClientIPAddress()
{
	char* ip_address = new char[15];
	socklen_t len;
	struct sockaddr_in addr;


	len = sizeof(addr);
	getpeername(socket_conn, (struct sockaddr*)&addr, &len);
	sprintf(ip_address, "%d.%d.%d.%d", int(addr.sin_addr.s_addr&0xFF), int((addr.sin_addr.s_addr&0xFF00)>>8), int		 ((addr.sin_addr.s_addr&0xFF0000)>>16), int((addr.sin_addr.s_addr&0xFF000000)>>24));

	return ip_address;
}
