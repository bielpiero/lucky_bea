#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h> 
#include "RNUtils.h"

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SD_BOTH SHUT_RDWR
#define closesocket(x) close(x)

#define BUFFER_SIZE 1048576

class UDPServer{
private:
	unsigned char*	bufferIn;	  // Maximun buiffer to send
	unsigned char*	bufferOut;

	int 	socket_server;
	int 	thread_status;//0 not started, 1 started, 2 waiting to finish
	struct sockaddr_in socket_local_address;
	std::vector<struct sockaddr_in> clients;
	pthread_mutex_t clientsMutex;
public:
	char* ip_address;

	UDPServer(int port);

	virtual ~UDPServer();	
	void closeConnection();
	int sendData(const unsigned char* cad, unsigned int length);
	int receiveData(unsigned char* cad, unsigned int &size);		
	
	void getClientIPAndPort(struct sockaddr_in client, char* ip, unsigned int& port);
	void startThread();//launch a 10 ms thread loop over the following actions
private:
	void init(int port);
	void error(const char* cad = "");
	void processHello(struct sockaddr_in client);
	void processByBye(struct sockaddr_in client);
	static void* launchThread(void* p);
protected:
	//virtual void OnConnection();//callback for client and server
	virtual void OnMessageReceivedWithData(unsigned char* cad, int length);//callback for client and server	
	
};
#endif
