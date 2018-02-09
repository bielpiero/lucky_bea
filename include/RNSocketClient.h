#ifndef RN_SOCKET_CLIENT_H
#define RN_SOCKET_CLIENT_H

#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include "RNUtils.h"
#include "TCPSocketClient.h"

#define RN_BUFFER_SIZE 1048576

class RNSocketClient {
public:
	RNSocketClient();
	virtual ~RNSocketClient();

	void init(const char* address, int portTCP, int portUDP = RN_NONE);

	void startThread();
	void handleConnection();
	int isConnected();
	int sendMsg(const char opr, unsigned char *cad, unsigned long long length);
	int sendBytes(unsigned char *cad, unsigned long long int length);
	int receiveBytes(unsigned char* cad, unsigned long long int& length, int timeout);
	int closeConnection();
private:
	unsigned char Buffer_in[RN_BUFFER_SIZE];	  // Maximun buiffer to send
	unsigned char Buffer_out[RN_BUFFER_SIZE];

	int thread_status;
    int portTCP;
    int portUDP;
	TCPSocketClient* socketConn;
	pthread_t socketsThread;
    char* ipAddress;
	struct sockaddr_in socket_address, socket_server_address;
protected:
	virtual void onConnection() = 0;//callback for client and server
	virtual void onMsg(char* cad, unsigned long long int length) = 0;//callback for client and server
private:
	static void* launchThread(void* p);
	void error(const char* cad = "");
};

#endif
