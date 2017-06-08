//#include <sys/socket.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <netinet/in.h>
#include "wave.h"
#define MAX_CLIENTS 32

#define PORT 8888 //8008 field, 8888 for my own test

// TCP server 1 for Smartphone Sep.2016 Demo
int master_socket;
int client_socket[MAX_CLIENTS];
struct sockaddr_in address;

int initSocket();
int acceptConnection();
int sendToClients(char *);
int closeSockets();

//// TCP server 2 for PC monitoring and extended app
//#define PORT2_PC 8123 //8123 for debug information sending to PC
//int master_socket2;
//int client_socket2[MAX_CLIENTS];
//struct sockaddr_in address2;
