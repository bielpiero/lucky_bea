#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

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

class UDPClient
{
private:
	unsigned char*	bufferIn;	  // Maximun buiffer to send
	unsigned char*	bufferOut;

	int 	socket_conn;
	int 	thread_status;//0 not started, 1 started, 2 waiting to finish
	struct sockaddr_in socket_local_address, socket_dest_address;
public:
	char* ip_address;
	UDPClient(const char* address, int port);

	virtual ~UDPClient();	
	void closeConnection();
	int sendData(const unsigned char* cad, unsigned int length);
	int receiveData(unsigned char* cad, unsigned int *size);		
	
	char* getClientIPAddress();
	void startThread();//launch a 10 ms thread loop over the following actions
private:
	void init(const char* address, int port);
	void error(const char* cad="");
	static void* launchThread(void* p);
protected:
	//virtual void OnConnection();//callback for client and server
	virtual void OnMessageReceivedWithData(unsigned char* cad, int length);//callback for client and server	
	
};
#endif
