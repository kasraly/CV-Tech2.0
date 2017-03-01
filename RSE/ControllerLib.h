#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/udp.h>    //Provides declarations for udp header
//#include <netinet/tcp.h>    //Provides declarations for tcp header
#include <netinet/ip.h>     //Provides declarations for ip header
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdint.h>

#include "SPAT.h"

#define SPaT_READ_INTERVAL 1 //seconds

//struct sockaddr_in controller_addr;
int closeController(void);
int readSPaT(SPAT_t *spat, double currentTime);
void parseControllerSPaTBroadcast(unsigned char* buffer, SPAT_t *spat);
int initController(char *controllerIP, uint16_t controllerSnmpPort);
int signalPreempt(unsigned char phases);

void print_udp_packet(unsigned char *Buffer , int Size);
void PrintData (unsigned char* data , int Size);
void print_ip_header(unsigned char* Buffer, int Size);
