#define _GNU_SOURCE

#include "gpsc_probe.h"
#include "wave.h"
#include "CVTechMessages_7.1.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <math.h>

#define CONFIG_FILE "/var/OBU_PedTC_Config.txt"

#define GPS_RECORDING "/tmp/gpsRecording.csv"
#define MIN_INTERVAL 0.01 //seconds
#define DSRC_BROADCAST_INTERVAL 0.1 //seconds

char gpsAddr[] ="127.0.0.1";

static uint64_t packets;
static uint64_t drops = 0;
static int pid;
int notxpkts = 0;
int IPdelay = 1000;

static WMEApplicationRequest entryTx;
static WMETARequest tareq;
static WSMRequest wsmreq;

struct Message dsrcm;
uint16_t dsrcmLen = sizeof(dsrcm);

GPSData gpsData;
int gpsSockFd;


int txWSMPPkts(int); /* Function to Transmit the WSMP packets */
/* Signal Handling Functions */
void sig_int(void);
void sig_term(void);
void initDsrc();
void initDsrcMessage(struct Message *);
void closeAll(void);
int readConfig(void);

int main()
{

    printf("Start/n");

    initDsrcMessage(&dsrcm); // Initialize the DSRC message to zeros
    readConfig();

    gpsSockFd = gpsc_connect(gpsAddr);

    initDsrc(); // initialize the DSRC channels and invoke the drivers for sending and recieving

    /* Call the Transmit function to Tx WSMP packets,
    there is an infinite loop in this function and program will not pass the next line (unless the loop paramters are changed)*/
    int ret = 0, count = 0;

    /* catch control-c and kill signal*/
    signal(SIGINT,(void *)sig_int);
    signal(SIGTERM,(void *)sig_term);

    gpsData.actual_time = -5000;
    while (gpsData.actual_time < 0)
    {
        char ch = '1';
        write(gpsSockFd,&ch,1);
        read(gpsSockFd,(void *)&gpsData,sizeof(gpsData));
        printf("waiting for GPS - GPSTime: %.1f\n", gpsData.actual_time);
        sleep(1);
    }

    //get the device time to calculate the transmit intreval
    double previousTime;
    struct timeval currentTimeTV;
    double currentTime;

    gettimeofday(&currentTimeTV, NULL);
    currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
    previousTime = floor(currentTime/DSRC_BROADCAST_INTERVAL)*DSRC_BROADCAST_INTERVAL;

    while (1)
    {
        static int dsrcIntervalCounter = 0;

        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;

        if ((currentTime - previousTime) >= MIN_INTERVAL) // check if enough time is passed to broadcast new message
        {
            previousTime = previousTime + MIN_INTERVAL;

            dsrcIntervalCounter++;

            if (dsrcIntervalCounter >= (int)(DSRC_BROADCAST_INTERVAL/MIN_INTERVAL))
            {
                dsrcIntervalCounter = 0;
                /*
                 * Send the Request out.
                 */
                dsrcm.TimeStamp = currentTime;

                char ch = '1';
                write(gpsSockFd,&ch,1);
                read(gpsSockFd,(void *)&gpsData,sizeof(gpsData));

                dsrcm.SenderLat = gpsData.latitude;
                dsrcm.SenderLon = gpsData.longitude;
                dsrcm.SenderAlt = gpsData.altitude;
                dsrcm.SenderSpeed = gpsData.speed;
                dsrcm.SenderCourse = gpsData.course;
                dsrcm.gpsTime = gpsData.actual_time;

                FILE *gpsRecordFile;

                if ((gpsRecordFile = fopen(GPS_RECORDING,"a")) != NULL)
                {
                    fprintf(gpsRecordFile, "%.3f,%.1f,%.8f,%.8f,%.1f,%.3f,%.3f\n",
                    currentTime,
                    gpsData.actual_time,
                    gpsData.latitude,
                    gpsData.longitude,
                    gpsData.altitude,
                    gpsData.course,
                    gpsData.speed);
                    fclose(gpsRecordFile);
                }
                else
                {
                    printf("error openning CONFIG_FILE for appending\n");
                }


                printf("Time: %.1f, GPSTime: %.1f, Lat: %.7f, Lon: %.7f, Alt: %.1f, course: %.3f, speed, %.3f\nTransmitted #%llu#      Dropped #%llu# len #%u#\n",
                    currentTime,
                    gpsData.actual_time,
                    gpsData.latitude,
                    gpsData.longitude,
                    gpsData.altitude,
                    gpsData.course,
                    gpsData.speed,
                    packets,
                    drops,
                    wsmreq.data.length);

                //copy the latest DSRC message in the RSE memory to wsmreq.data for broadcast
                memcpy (wsmreq.data.contents, &dsrcm, sizeof(dsrcm));
                memcpy (&wsmreq.data.length, &dsrcmLen, sizeof(dsrcmLen));

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

            }
        }

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
    signal(SIGINT,SIG_DFL);
    gpsc_close_sock();
    printf("\n\nPackets Sent =  %llu\n",packets);
    printf("Packets Dropped = %llu\n",drops);
    printf("localtx killed by control-C\n");
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
    memcpy(&wsmreq.data.length, &dsrcmLen, sizeof( dsrcmLen));

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

}

void initDsrcMessage(struct Message *dsrcmp)
{
    bzero(dsrcmp, sizeof(dsrcmp));

    dsrcmp->SenderID = 12; //sender id
    dsrcmp->SenderType = 1; // 0-RSE, 1-OBE, 2-Transit OBE, 3- Portable OBE
    dsrcmp->TimeStamp = 0; // unix timestamp of sent message with respect to the number of seconds from Jan 1, 1970 00:00:00, defaulted to be 8 bytes
    dsrcmp->SenderLat = 0; // latitude of sender, defaulted to be 8 bytes
    dsrcmp->SenderLon = 0; // longitude of sender, defaulted to be 8 bytes
    dsrcmp->SenderAlt = 0;
    dsrcmp->SenderSpeed = 0;
    dsrcmp->SenderCourse = 0;
    dsrcmp->gpsTime = 0;

    // ending message
    dsrcmp->END1 = 'A'; // END1= A
    dsrcmp->END2 = 'A'; // END2= A
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
    }
    free(line);
    fclose(configFile);
    return 0;
}
