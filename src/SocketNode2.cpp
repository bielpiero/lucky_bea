// SocketNode.cpp: implementation of the CSocketNode class.
//
//////////////////////////////////////////////////////////////////////
#include "SocketNode2.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/timeb.h>
	
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CSocketNode::CSocketNode()
{
	socket_conn=INVALID_SOCKET;
	thread_status=0;
	//WSADATA wsaData;
   
	//WSAStartup(MAKEWORD(2, 2),&wsaData);
}

CSocketNode::~CSocketNode()
{
	//#ifdef SOCKET_NODE_WINDOWS
	//	WSACleanup();
	//#endif
}

int CSocketNode::Init(const char *address,int port, int t)
{
	type=t;
	socket_conn=INVALID_SOCKET;
// Configuracion del socket del Servidor	
	socket_server_address.sin_family = AF_INET;
	socket_server_address.sin_addr.s_addr = inet_addr(address);
	if(strlen(address)==0)
		socket_server_address.sin_addr.s_addr = INADDR_ANY;
	socket_server_address.sin_port = htons(port);
//SERVER
	if(type==SOCKET_SERVER)
	{
		socket_server = socket(AF_INET, SOCK_STREAM, 0);
		if (socket_server==INVALID_SOCKET)
		{
			Error("Server: Error Socket");
			return -1;
		}
		int len = sizeof(socket_server_address);
		if (bind(socket_server,(struct sockaddr *) &socket_server_address,len) < 0)
		{
			return -1;
		}
		// Damos como mximo 5 puertos de conexin.
		if (listen(socket_server,5) < 0)
		{
			return -1;
		}	
		return 0;
	}
//CLIENT
	if(type==SOCKET_CLIENT)
	{
		return 0;
	}
	return -1;
}
void CSocketNode::HandleConnection(void)
{
	if(type==SOCKET_SERVER)
	{
		//if disconnected, Try connection
		if(socket_conn==INVALID_SOCKET)
		{
//			#ifdef SOCKET_NODE_WINDOWS
//			int len = sizeof(socket_address);
//			#else
			unsigned int len = sizeof(socket_address);
//			#endif
			fd_set readfds;
			
			FD_ZERO(&readfds);
			FD_SET(socket_server,&readfds);
			timeval timeout;
			timeout.tv_sec=0;
			timeout.tv_usec=10;//10000;
			int ret=select (socket_server+1,&readfds,NULL,NULL,&timeout );
			if(ret==1)
			{
				socket_conn = accept(socket_server,(struct sockaddr *)&socket_address, &len);
				if(socket_conn==INVALID_SOCKET)
				{
					return;
				}	
                                int optval;
                                setsockopt(socket_conn, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);
				OnConnection();		
			}
		}
	}

	if(type==SOCKET_CLIENT )
	{
		if(socket_conn==INVALID_SOCKET)
		{
			//Aux socket to avoid sock_conn!=INVALID without connecting
			int aux_sock=socket(AF_INET, SOCK_STREAM,0);
			if (aux_sock == INVALID_SOCKET) 
			{
				return;
			}
	
			int len= sizeof(socket_server_address);
			if(connect(aux_sock,(const struct sockaddr *) &socket_server_address,len)!=0) 
			{
				shutdown(aux_sock,SD_BOTH);
				closesocket(aux_sock);
				return;
			}
			socket_conn=aux_sock;
			OnConnection();
		}
	}	
}
void CSocketNode::Error(const char* cad)
{
	if(socket_conn!=INVALID_SOCKET)
	{	
		shutdown(socket_conn,SD_BOTH);
		closesocket(socket_conn);
		socket_conn=INVALID_SOCKET;
		OnConnection();
	}
}

int CSocketNode::SendMsg(const char opr, const char* cad,int length)
{
	if (socket_conn == INVALID_SOCKET)
		return -1;
	//Build header
	Buffer_out[1] = 48; //12345 / 256;
	Buffer_out[0] = 57; //12345 % 256;
	length++;
	Buffer_out[3] = length / 256;
	Buffer_out[2] = length % 256;
	Buffer_out[4] = opr;
	memcpy(Buffer_out + 5, cad, length);
	//Send it

	int err = send(socket_conn, Buffer_out, length + 4, 0);

	//	delete[] cad_aux;
	if (err == SOCKET_ERROR)
	{
		Error("SendMsg Error");
		return -1;
	}
	return 0;
}

int CSocketNode::SendBytes(char *cad, int length)
{
	//Send it
	int err=send(socket_conn, cad, length,0);

	if (err == SOCKET_ERROR )
	{
		Error("SendBytes Error");
		return -1;
	}
	return 0;
}

int CSocketNode::ReceiveBytes(char *cad, int *length,int timeout)
{
	if (socket_conn == INVALID_SOCKET)
		return -1;

	fd_set readfds; FD_ZERO(&readfds); FD_SET(socket_conn, &readfds);
	timeval tout; tout.tv_sec = 0; tout.tv_usec = 10000;
	int ret = select(socket_conn + 1, &readfds, NULL, NULL, &tout);
	if (1 == ret)
	{
		int r = recv(socket_conn, cad, *length, 0);
		if (r == SOCKET_ERROR || r == 0)
		{
			Error("Receive bytes Error");
			return -1;

		}
		else
		{
			*length = r;
			return 0;
		}
	}
	return -1;
}
int CSocketNode::ReceiveMsg(char* cad, int* size, int timeout) 
{
	int ret;
	int nChars;
	int len;
	char header[4];
	nChars = 4;
        
	if (0 != ReceiveBytes(header, &nChars, timeout))
		return -1;//no header
        
	len = header[2] + header[3] * 256;
	if (header[0] != 57 ||	// 12345 % 256
		header[1] != 48 ||	// 12345 / 256
		len <= 0)
	{
		Error("Header error");
		return -2;//header error
	}
	ret = ReceiveBytes(Buffer_in, &len, timeout);
	Buffer_in[len] = 0;
	memcpy(cad, Buffer_in, *size<len ? *size : len);

	if (*size<len)
		return -3;//short buffer

	*size = len;
	return ret;
}

void CSocketNode::OnMsg(char* cad,int length)
{// Virtual function
    
}

void CSocketNode::OnConnection()
{// Virtual function
}

int CSocketNode::IsConnected()
{
	if(socket_conn==INVALID_SOCKET)
		return 0;
	else
		return 1;
}


//#ifdef SOCKET_NODE_WINDOWS
//void LaunchThread(void* p)	
//#else
void* LaunchThread(void* p)
//#endif
{
	
	CSocketNode* node=(CSocketNode*) p;

	node->thread_status=1;
	while(node->thread_status==1)
	{
		if(!node->IsConnected())
		{
			
			node->HandleConnection();
		}
		else
		{	
			char msg[4096];
			int l=4096;
			if(0==node->ReceiveMsg(msg,&l,0))
			{
				//printf("mensaje recibido\n");
				node->OnMsg(msg,l);
				//printf("mensaje procesado\n");
			}
		}
		Sleep(10);
	}
	node->thread_status=0;
//	#if	!defined SOCKET_NODE_WINDOWS
	return NULL;
//	#endif
}

void CSocketNode::StartThread()
{
	pthread_t t1;
	pthread_create(&t1,NULL,LaunchThread, (void *) (this)  );
}


int CSocketNode::Close()
{
	if(thread_status==1)
	{
		//wait until thread terminates
		thread_status=2;
		while(thread_status!=0)Sleep(100);
	}
		
	shutdown(socket_conn,SD_BOTH);
	closesocket(socket_conn);
	socket_conn=INVALID_SOCKET;
	closesocket(socket_server);
	return 1;
}
