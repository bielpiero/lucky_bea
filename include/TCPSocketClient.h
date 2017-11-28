#ifndef TCP_SOCKET_CLIENT_H
#define TCP_SOCKET_CLIENT_H

#include "UDPClient.h"

class TCPSocketClient{
public:
	TCPSocketClient(UDPClient* clientUdp = NULL, int socket = INVALID_SOCKET, bool webSocket = false) {
		this->clientUdp = clientUdp;
		this->socket = socket; 
		this->webSocket = webSocket;
		checkedWebSocket = false;
		handshakeDone = false;

	}
	virtual ~TCPSocketClient(){
		if(this->clientUdp != NULL){
			this->clientUdp->closeConnection();
		}
	}

	int getSocket() { return socket; }
	void setSocket(int socket) { this->socket = socket; }

	UDPClient* getUDPClient() { return clientUdp; }
	void setUDPClient(UDPClient* clientUdp) { this->clientUdp = clientUdp; }

	bool checkedIfIsAWebSocket() { return checkedWebSocket; } 
	void setCheckedIfIsAWebSocket(bool checkedWebSocket) { this->checkedWebSocket = checkedWebSocket; }

	bool isWebSocket() { return webSocket; }
	void setIsAWebSocket(bool webSocket) { this->webSocket = webSocket; }

	bool isHandshakeDone() { return handshakeDone; }
	void setHandshakeDone(bool handshakeDone) { this->handshakeDone = handshakeDone; }



private:
	UDPClient* clientUdp;
	int socket;
	bool webSocket;
	bool checkedWebSocket;
	bool handshakeDone;
};

#endif