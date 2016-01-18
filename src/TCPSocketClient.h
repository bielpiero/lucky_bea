#ifndef TCP_SOCKET_CLIENT_H
#define TCP_SOCKET_CLIENT_H

class TCPSocketClient{
public:
	TCPSocketClient(int socket = -1, bool webSocket = false) { 
		this->socket = socket; 
		this->webSocket = webSocket;
		checkedWebSocket = false;
		handshakeDone = false;

	}
	virtual ~TCPSocketClient(){}

	int getSocket() { return socket; }
	void setSocket(int socket) { this->socket = socket; }

	bool checkedIfIsAWebSocket() { return checkedWebSocket; } 
	void setCheckedIfIsAWebSocket(bool checkedWebSocket) { this->checkedWebSocket = checkedWebSocket; }

	bool isWebSocket() { return webSocket; }
	void setIsAWebSocket(bool webSocket) { this->webSocket = webSocket; }

	bool isHandshakeDone() { return handshakeDone; }
	void setHandshakeDone(bool handshakeDone) { this->handshakeDone = handshakeDone; }

private:
	int socket;
	bool webSocket;
	bool checkedWebSocket;
	bool handshakeDone;
};

#endif