//#include <sys/socket.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <netinet/in.h>
#include "wave.h"
#define MAX_CLIENTS 32

#define PORT 8888 //8008 field, 8888 for my own test
//#include "MultiClientSocket.h"

int master_socket;
int client_socket[MAX_CLIENTS];
struct sockaddr_in address;

int initSocket();
int acceptConnection();
int sendToClients(char *);
int closeSockets();
