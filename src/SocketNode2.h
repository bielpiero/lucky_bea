// SocketNode.h: interface for the CSocketNode class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_SOCKETNODE_H__895095F5_AD65_4FF5_973F_0DD16E9BC2FE__INCLUDED_)
#define AFX_SOCKETNODE_H__895095F5_AD65_4FF5_973F_0DD16E9BC2FE__INCLUDED_

#include <sys/stat.h>
#include <sys/time.h>   
#include <sys/types.h>   
#include <unistd.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <errno.h>	
#include <string.h>

#include "handshake.h"
#include "crypt/base64.h"
#include "crypt/sha.h"
#include <vector>

#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define SD_BOTH SHUT_RDWR
#define closesocket(x) close(x)
#define Sleep(x) usleep(x*1000)


#define SOCKET_SERVER 0
#define SOCKET_CLIENT 1

#define BUFFER_SIZE 1048576

#define SOCKET_UDP 0
#define SOCKET_TCP 1

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
	char* ip_address;
	char	Buffer_in[BUFFER_SIZE];	  // Maximun buiffer to send
	char	Buffer_out[BUFFER_SIZE];
	CSocketNode();
	virtual ~CSocketNode();	
	int Init(const char* address, int port,int type);
	int Close();
	int IsConnected();
	int SendMsg(const char opr, const char* cad,int length);
	int SendBytes(char*cad, int length);
	int ReceiveMsg(char* cad, int* size, int timeout=200);		
	int ReceiveBytes(char* cad,int* length, int timeout);
	int getClientPort();

	bool isWebSocket();

	char* getClientIPAddress();
	
	void StartThread();//launch a 10 ms thread loop over the following actions
	void HandleConnection();//manages all connection and reconnection
protected:
	virtual void OnConnection() = 0;//callback for client and server
	virtual void OnMsg(char* cad, int length) = 0;//callback for client and server
protected:
	void Error(const char* cad="");
private:
	int type;//server or client
	int thread_status;//0 not started, 1 started, 2 waiting to finish
	
	int socket_conn;
	int socket_server;

	bool webSocket;
	bool checkedWebSocket;
	bool handshakeDone;

	struct sockaddr_in socket_address, socket_server_address;
	
	//WebSocket Implementation
	wsFrameType wsParseHandshake(char* buffer, int size, Handshake* hs);
	wsFrameType wsParseInputFrame(char* bufferIn, int sizeIn, char* bufferOut, int& sizeOut);
	void wsGetHandshakeAnswer(Handshake* hs, char* outFrame, int &outLength);
	std::vector<std::string> split(char* buffer, const char* delimiter);

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

	static void* LaunchThread(void* p);
};

#endif // !defined(AFX_SOCKETNODE_H__895095F5_AD65_4FF5_973F_0DD16E9BC2FE__INCLUDED_)
