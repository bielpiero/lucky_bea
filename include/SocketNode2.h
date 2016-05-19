// SocketNode.h: interface for the CSocketNode class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_SOCKETNODE_H__895095F5_AD65_4FF5_973F_0DD16E9BC2FE__INCLUDED_)
#define AFX_SOCKETNODE_H__895095F5_AD65_4FF5_973F_0DD16E9BC2FE__INCLUDED_

#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#include "RNUtils.h"

#include "handshake.h"
#include "crypt/base64.h"
#include "crypt/sha.h"
#include "TCPSocketClient.h"


#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SD_BOTH SHUT_RDWR
#define closesocket(x) close(x)

#define MAX_CLIENTS 10
#define CLIENT_DEFAULT_INDEX 0

#define SOCKET_SERVER 0
#define SOCKET_CLIENT 1

#define BUFFER_SIZE 1048576

#define SOCKET_UDP 0
#define SOCKET_TCP 1

#define FRAME_MAX_HEADER_SIZE 16

#define WS_EOL "\r\n"
#define WS_EOMSG "\r\n\r\n"
    
enum wsState {
    WS_STATE_OPENING,
    WS_STATE_NORMAL,
    WS_STATE_CLOSING
};

class CSocketNode 
{

public:
	char*   ip_address;
	unsigned char	Buffer_in[BUFFER_SIZE];	  // Maximun buiffer to send
	unsigned char	Buffer_out[BUFFER_SIZE];
	CSocketNode();
	virtual ~CSocketNode();	
	int init(const char* address, int port,int type);
	int closeConnection();
	int isConnected(int socketIndex);
	int sendMsg(int socketIndex, const char opr, const char* cad, unsigned long long int length);
	int wsSendPingMsg(int socketIndex);
	int sendBytes(int socketIndex, unsigned char* cad, unsigned long long int length);
	int receiveMsg(int socketIndex, char* cad, unsigned long long int& size, int timeout=200);		
	int receiveBytes(int socketIndex, unsigned char* cad, unsigned long long int& length, int timeout);
	int getServerPort();

	bool isWebSocket(int socketIndex);

	int getTokenOwner();
	void setTokenOwner(int token);
	int getLastTokenOwner();

	char* getClientIPAddress(int socketIndex);
	int getClientPort(int socketIndex);

	void startThread();//launch a 10 ms thread loop over the following actions
	void handleConnection();//manages all connection and reconnection
protected:
	virtual void onConnection(int socketIndex) = 0;//callback for client and server
	virtual void onMsg(int socketIndex, char* cad, unsigned long long int length) = 0;//callback for client and server
protected:
	void error(int socketIndex, const char* cad="");
	
private:
	int type;//server or client
	int thread_status;//0 not started, 1 started, 2 waiting to finish
	int pingTimerThreadStatus;
	
	pthread_t pingThread;

	int tokenOwner;
	int lastTokenOwner;

	TCPSocketClient socket_conn[MAX_CLIENTS];
	int socket_server;
	pthread_t socketsThread;

	struct sockaddr_in socket_address, socket_server_address;
	
	void setLastTokenOwner(int token);
	//WebSocket Implementation
	wsFrameType wsParseHandshake(char* buffer, unsigned long long int size, Handshake* hs);
	wsFrameType wsParseInputFrame(unsigned char* bufferIn, unsigned long long int sizeIn, char* bufferOut, unsigned long long int& sizeOut);

	void wsGetHandshakeAnswer(Handshake* hs, char* outFrame, unsigned long long int &outLength);
	
	static const std::string connectionField;
	static const std::string upgrade;
	static const std::string upgrade2;
	static const std::string upgradeField;
	static const std::string websocket;
	static const std::string webSocketStr;
	static const std::string hostField;
	static const std::string originField;
	static const std::string keyField;
	static const std::string protocolField;
	static const std::string versionField;
	static const std::string acceptField;
	static const std::string version;
	static const std::string secret;
	static const std::string switchingProtocolField;
	static const std::string extensionsField;
	static const std::string applicationDataField;

	static void* launchThread(void* p);
	static void* pingTimerThread(void* p);
};

#endif // !defined(AFX_SOCKETNODE_H__895095F5_AD65_4FF5_973F_0DD16E9BC2FE__INCLUDED_)
