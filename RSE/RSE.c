#define _GNU_SOURCE

#include "SPaTLib.h"
#include "gpsc_probe.h"
#include "wave.h"
#include "CVTechMessages_7.1.h"
#include <stdio.h>
#include <stdlib.h>
//#include <sys/socket.h>

#define BUFFER_SIZE 1024

#define MIN_INTERVAL 0.01 //seconds
#define DSRC_BROADCAST_INTERVAL 0.1 //seconds
#define TMC_BROADCAST_INTERVAL 0 //seconds

#define CONFIG_FILE "/var/RSE_Config.txt"

uint16_t serverPort = 8008;

struct sockaddr_in server_addr;
int server_socket_fd;


#undef DEMO_MODE

char gpsAddr[] = "127.0.0.1";

/*
#define MAX_OIDS 8
struct oid
{
    char Name[32];
    oid Oid[MAX_OID_LEN];
    int OidLen;
} oids[MAX_OIDS];*/

static uint64_t packets;
static uint64_t drops = 0;
static int pid;
int notxpkts = 0;
int IPdelay = 1000;

static WMEApplicationRequest entryTx;
static WMEApplicationRequest entryRx;
static WMETARequest tareq;
static WSMRequest wsmreq;

struct Message dsrcm;
struct Message dsrcmRx;
uint16_t dsrcmLen = sizeof(struct Message);

GPSData gpsData;
int gpsSockFd;

int txWSMPPkts(int); /* Function to Transmit the WSMP packets */
/* Signal Handling Functions */
void sig_int(void);
void sig_term(void);
void initSocket();
void initDsrc();
void initDsrcMessage(struct Message*);
void closeAll(void);
void parseTmcMessage(struct Message*, struct Message*);
int readConfig(void);

int main()
{
    printf("Start \n");

    int rx_ret = 0;
//    WSMIndication rxpkt;

    initDsrcMessage(&dsrcm); // Initialize the DSRC message to zeros

    readConfig();

    initController();

    initDsrc(); // initialize the DSRC channels and invoke the drivers for sending and recieving

    //initSocket(); // initialize socket for communication with TMC and recieving the controller SPaT broadcast.

    printf("Initialized socket\n");

    gpsSockFd = gpsc_connect(gpsAddr);

    printf("created GPS socket\n");


    /* Call the Transmit function to Tx WSMP packets,
    there is an infinite loop in this function and program will not pass the next line (unless the loop paramters are changed)*/
    int ret = 0, count = 0;
    struct timeval currentTimeTV;
    double previousTime, currentTime;
    /* catch control-c and kill signal*/
    signal(SIGINT,(void *)sig_int);
    signal(SIGTERM,(void *)sig_term);

    //get the device time to calculate the transmit intreval
    gettimeofday(&currentTimeTV, NULL);
    currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
    previousTime = floor(currentTime/DSRC_BROADCAST_INTERVAL) * DSRC_BROADCAST_INTERVAL;

    //double preemptTime = previousTime;
    //unsigned char preemptPhase = 0;

    while (1)
    {
        static int tmcIntervalCounter = 0;
        static int dsrcIntervalCounter = 0;
        static int spatReadCounter = 0;

        // check whether there is new packet recieved from TMC
/*        struct timeval socketCheckTimout = {0,1};
        fd_set server_rfds;
        FD_ZERO(&server_rfds);
        FD_SET(server_socket_fd, &server_rfds);
        retval = select(server_socket_fd + 1, &server_rfds, NULL, NULL, &socketCheckTimout);
        if (retval > 0) // if there is packet available from TMC, read the socket and process the packet
        {
            printf("TMC message \n");
            socklen_t server_addr_length = sizeof(server_addr);
            struct Message buffer;
            recvfrom(server_socket_fd, &buffer, sizeof(buffer),0,(struct sockaddr*)&server_addr, &server_addr_length);
            parseTmcMessage(&buffer, &dsrcm);
            // send some response to TMC
            sendto(server_socket_fd, &dsrcm, sizeof(dsrcm),0, (struct sockaddr*)&server_addr, server_addr_length);
        }*/


        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;

/*        if ((currentTime - preemptTime) >= 60)
        {
            preemptTime = currentTime;
            if (preemptPhase)
                preemptPhase = 0;
            else
                preemptPhase = 0x88;
        }*/

        if ((currentTime - previousTime) >= MIN_INTERVAL) // check if enough time is passed to broadcast new message
        {
            previousTime = previousTime + MIN_INTERVAL;

            tmcIntervalCounter++;
            dsrcIntervalCounter++;
            spatReadCounter++;

            if (spatReadCounter >= (int)(SPaT_READ_INTERVAL/MIN_INTERVAL))
            {
                //printf("\nsystem Time: %.1f, Calling Preempt Function\n",currentTime);
                //signalPreempt(preemptPhase);

                printf("\nsystem Time: %f, Time to Check Controller status\n",currentTime);
                spatReadCounter = 0;
                if (dsrcm.DemoPhase > 0)
                {
                    printf("Reding SPaT from Controller, DemoPhase is %d\n",dsrcm.DemoPhase);
                    readSPaT(&dsrcm, currentTime);
                }
                else
                {
                    printf("Controller not present, DemoPhase is %d\n",dsrcm.DemoPhase);
                }

            }

            if (dsrcIntervalCounter >= (int)(DSRC_BROADCAST_INTERVAL/MIN_INTERVAL))
            {
                printf("\v\nSystem Time: %f, Time to Broadcast DSRC message\n",currentTime);
                dsrcIntervalCounter = 0;
                /*
                 * Send the Request out.
                 */

                printf("\nwriting current time to DSRC message....\n");


                dsrcm.TimeStamp = currentTime;

                printf("\nReading GPS information....\n");

                char ch = '1';
                write(gpsSockFd,&ch,1);
                read(gpsSockFd,(void *)&gpsData,sizeof(gpsData));

                printf("RSE GPS Data\nTime: %.3f, GPSTime: %.1f, Lat: %.7f, Lon: %.7f\nAlt: %.1f, course: %.0f, speed, %.2f\n",
                    currentTime,
                    gpsData.actual_time,
                    gpsData.latitude,
                    gpsData.longitude,
                    gpsData.altitude,
                    gpsData.course,
                    gpsData.speed);

                if (gpsData.actual_time > 0)
                {
                    dsrcm.SenderLat = gpsData.latitude;
                    dsrcm.SenderLon = gpsData.longitude;
                    dsrcm.SenderAlt = gpsData.altitude;
                    dsrcm.SenderSpeed = gpsData.speed;
                    dsrcm.SenderCourse = gpsData.course;
                }
                dsrcm.gpsTime = gpsData.actual_time;


                /*printf("Packet raw:");
                char *buff = (char *)&dsrcm;
                int i;
                for (i=0; i<sizeof(dsrcm); i++)
                    printf("%x ",buff[i]);
                printf("\n");*/
                /*printf("packet: %f %d %d %d %d %d %d %d %d, lat %f lon %f sats:%d\n",
                    dsrcm.TimeStamp,
                    dsrcm.PhaseStatus[0], dsrcm.PhaseTiming[0],
                    dsrcm.PhaseStatus[1], dsrcm.PhaseTiming[1],
                    dsrcm.PhaseStatus[2], dsrcm.PhaseTiming[2],
                    dsrcm.PhaseStatus[3], dsrcm.PhaseTiming[3],
                    dsrcm.SenderLat, dsrcm.SenderLon,gpsData.numsats);*/


                //copy the latest DSRC message in the RSE memory to wsmreq.data for broadcast
                memcpy (wsmreq.data.contents, &dsrcm, sizeof(dsrcm));
                wsmreq.data.length = dsrcmLen;

                ret = txWSMPacket(pid, &wsmreq); //send the DSRC message
                if( ret < 0)
                {
                    drops++;
                }
                else
                {
                    packets++;
                    count++;
                }
                if((notxpkts != 0) && (count >= notxpkts))
                    break;
//                printf("SPaT: \n");
//                printf("State1 %d, Time1 %d\nState2 %d, Time2 %d\nState3 %d, Time3 %d\nState4 %d, Time4 %d\nState5 %d, Time5 %d\nState6 %d, Time6 %d\nState7 %d, Time7 %d\nState8 %d, Time8 %d\n",
//                    dsrcm.PhaseStatus[0],dsrcm.PhaseTiming[0],
//                    dsrcm.PhaseStatus[1],dsrcm.PhaseTiming[1],
//                    dsrcm.PhaseStatus[2],dsrcm.PhaseTiming[2],
//                    dsrcm.PhaseStatus[3],dsrcm.PhaseTiming[3],
//                    dsrcm.PhaseStatus[4],dsrcm.PhaseTiming[4],
//                    dsrcm.PhaseStatus[5],dsrcm.PhaseTiming[5],
//                    dsrcm.PhaseStatus[6],dsrcm.PhaseTiming[6],
//                    dsrcm.PhaseStatus[7],dsrcm.PhaseTiming[7]);

                printf("DSRC message Transmitted #%llu#      Drop #%llu#     len #%u#\n",
                    packets,
                    drops,
                    wsmreq.data.length);
            }


            if ((TMC_BROADCAST_INTERVAL) & (tmcIntervalCounter >= (int)(TMC_BROADCAST_INTERVAL/MIN_INTERVAL)))
            {
                printf("\nsystem Time: %f, Time to send Mesage to TMC\n",currentTime);
                tmcIntervalCounter = 0;
/*                if (server_addr.sin_addr.s_addr)
                {
                    dsrcm.TimeStamp = currentTime;
                    sendto(server_socket_fd, &dsrcm, sizeof(dsrcm),0, (struct sockaddr*)&server_addr, sizeof(server_addr));
                }*/
            }

        }

        if(rx_ret > 0)
        {
/*            memcpy(&dsrcmRx, rxpkt.data.contents, sizeof(dsrcmRx));
            printf("Received WSMP packet: Time:%f Lat:%f Lon%f ID:%d, Packet size :%d\n",
                dsrcmRx.TimeStamp,
                dsrcmRx.SenderLat,
                dsrcmRx.SenderLon,
                dsrcmRx.SenderID,
                rxpkt.data.length);*/

        }

        //rx_ret = rxWSMPacket(pid, &rxpkt);
        sched_yield();
        //usleep(2000);
        usleep(IPdelay);
    }
    printf("\n Transmitted =  %d dropped = %llu\n",count,drops);

    sig_term();

    return 0;
}


/* Signal handling functions */
/* Before killing/Termination your application,
 * make sure you unregister the application.
 */
void sig_int(void)
{
    closeAll();
    exit(0);
}

void sig_term(void)
{
    closeAll();
    exit(0);
}

void closeAll(void)
{
    removeProvider(pid, &entryTx);
    removeUser(pid, &entryRx);
    signal(SIGINT,SIG_DFL);
    close(server_socket_fd);
    closeController();
    gpsc_close_sock();
    printf("\n\nPackets Sent =  %llu\n",packets);
    printf("Packets Dropped = %llu\n",drops);
    printf("localtx killed by control-C\n");
}

void initSocket()
{

    // build UDP
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    server_addr.sin_port = htons(serverPort);

    // build socket
    server_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if(server_socket_fd == -1)
    {
        perror("Create Socket Failed:");
        exit(1);
    }

    // binding socket
    if(-1 == (bind(server_socket_fd,(struct sockaddr*)&server_addr,sizeof(server_addr))))
    {
        perror("Server Bind Failed:");
        exit(1);
    };

}

void initDsrc()
{

    pid = getpid();

    printf("Filling Provider Service Table entry\n");
    entryTx.psid = 10;/* Provider Service IDentifier of the process.
                      you cant register 2 applications with same psid. */
    entryTx.priority = 2;
    entryTx.channel = 172;
    entryTx.repeatrate = 50;
    entryTx.channelaccess = 0;

    printf("Building a WSM Request Packet\n");
    wsmreq.chaninfo.channel = entryTx.channel;
    wsmreq.chaninfo.rate = 3;
    wsmreq.chaninfo.txpower = 14;
    wsmreq.version = 1;
    wsmreq.security = 0;
    wsmreq.psid = 10;
    wsmreq.txpriority = 2;
    memset(&wsmreq.data, 0, sizeof( WSMData));
    wsmreq.data.length = dsrcmLen;

    printf("Builing TA request\n");
    tareq.action = TA_ADD;
    tareq.repeatrate = 100;
    tareq.channel = 172;
    tareq.channelinterval = 1;
    tareq.servicepriority = 2;

    if ( invokeWAVEDriver(0) < 0 )
    {
        printf( "Opening Failed.\n ");
        exit(-1);
    }
    else
    {
        printf("Driver invoked\n");
    }

    printf("Registering provider\n ");
    removeProvider(pid, &entryTx);
    if ( registerProvider( pid, &entryTx ) < 0 )
    {
        printf("\nRegister Provider failed\n");
        removeProvider(pid, &entryTx);
        registerProvider(pid, &entryTx);
    }
    else
    {
        printf("provider registered with PSID = %u\n",entryTx.psid );
    }
    printf("starting TA\n");
    if (transmitTA(&tareq) < 0)
    {
        printf("send TA failed\n ");
    }
    else
    {
        printf("send TA successful\n") ;
    }

/*    entryRx.psid = 10;
    entryRx.userreqtype = 2;
    entryRx.channel = 172;
    entryRx.schaccess  = 0;
    entryRx.schextaccess = 1;

    printf("Invoking WAVE driver \n");

    if (invokeWAVEDevice(WAVEDEVICE_LOCAL, 0) < 0)
    {
        printf("Open Failed. Quitting\n");
        exit(-1);
    }

    printf("Registering User %d\n", entryRx.psid);
    if ( registerUser(pid, &entryRx) < 0)
    {
        printf("Register User Failed \n");
        printf("Removing user if already present  %d\n", !removeUser(pid, &entryRx));
        printf("USER Registered %d with PSID =%u \n", registerUser(pid, &entryRx), entryRx.psid );
    }*/
}

void initDsrcMessage(struct Message* dsrcmp)
{
    memset(dsrcmp,0,sizeof(struct Message*));

    dsrcmp->DemoPhase = 0;

    // ending message
    dsrcmp->END1 = 'A'; // END1= A
    dsrcmp->END2 = 'A'; // END2= A
}

void parseTmcMessage(struct Message* buffer, struct Message* dsrcmp)
{
    int i;

    dsrcmp->DemoPhase = buffer->DemoPhase;
    dsrcmp->HCW = buffer->HCW;
    dsrcmp->CSW = buffer->CSW;
    if (dsrcmp->CSW)
        dsrcmp->CAS = buffer->CAS;
    dsrcmp->ASWarning = buffer->ASWarning;
    if (dsrcmp->ASWarning)
        dsrcmp->VSL = buffer->VSL;

    if (1)
    {
        unsigned char *byteBuffer = (unsigned char *)buffer;
        printf("TMC message:");
        for(i = 0; i<sizeof(*buffer); i++)
            printf(" %x",byteBuffer[i]);
        printf("\n");
    }
}

int readConfig(void)
{
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    FILE *configFile;

    if ((configFile = fopen(CONFIG_FILE, "r")) != NULL)
    {
        printf("file open successgful\n");
    }
    else
    {
        printf("error openning CONFIG_FILE for reading\n");
        return 1;
    }

    while ((read = getline(&line, &len, configFile)) != -1)
    {
        char *str;
        str = strtok (line," ,");
        if (strcasecmp(str,"SenderID")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.SenderID = atoi(str);
            printf("SenderID is %d\n",dsrcm.SenderID);
        }
        else if (strcasecmp(str,"SenderType")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.SenderType = atoi(str);
            printf("SenderType is %d\n",dsrcm.SenderType);
        }
        else if (strcasecmp(str,"DemoPhase")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.DemoPhase = atoi(str);
            printf("DemoPhase is %d\n",dsrcm.DemoPhase);
        }
        else if (strcasecmp(str,"Latitude")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.SenderLat = strtod(str, NULL);
            printf("Latitude is %.6f\n",dsrcm.SenderLat);
        }
        else if (strcasecmp(str,"Longitude")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.SenderLon = strtod(str, NULL);
            printf("Longitude is %.6f\n",dsrcm.SenderLon);
        }
        else if (strcasecmp(str,"HCW")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.HCW = atoi(str);
            printf("HCW is %d\n",dsrcm.HCW);
        }
        else if (strcasecmp(str,"CSW")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.CSW = atoi(str);
            printf("CSW is %d\n",dsrcm.CSW);
        }
        else if (strcasecmp(str,"CAS")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.CAS = atoi(str);
            printf("CAS is %d\n",dsrcm.CAS);
        }
        else if (strcasecmp(str,"ASWarning")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.ASWarning = atoi(str);
            printf("ASWarning is %d\n",dsrcm.ASWarning);
        }
        else if (strcasecmp(str,"VSL")==0)
        {
            str = strtok (NULL," ,");
            dsrcm.VSL = atoi(str);
            printf("VSL is %d\n",dsrcm.VSL);
        }
        else if (strcasecmp(str,"Controller_IP")==0)
        {
            str = strtok (NULL," ,");
            strcpy(controllerIP, str);
            printf("Controller_IP is %s\n",controllerIP);
        }
        else if (strcasecmp(str,"Controller_SNMP_Port")==0)
        {
            str = strtok (NULL," ,");
            controllerSnmpPort = atoi(str);
            printf("Controller_SNMP_Port is %d\n",controllerSnmpPort);
        }
        else if (strcasecmp(str,"Controller_Broadcast_Port")==0)
        {
            str = strtok (NULL," ,");
            controllerBroadcastPort = atoi(str);
            printf("Controller_Broadcast_Port is %d\n",controllerBroadcastPort);
        }
        else if (strcasecmp(str,"Server_Port")==0)
        {
            str = strtok (NULL," ,");
            serverPort = atoi(str);
            printf("Server_Port is %d\n",serverPort);
        }
    }
    free(line);
    fclose(configFile);
    return 0;
}
