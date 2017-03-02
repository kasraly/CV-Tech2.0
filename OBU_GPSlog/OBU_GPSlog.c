#define _GNU_SOURCE

#include "gpsc_probe.h"
#include "wave.h"
#include "asnwave.h"
#include "CVTechMessages_7.1.h"
#include "GPS_offline.h"

#define MIN_INTERVAL 0.1 //seconds
#define TIME_STEP 1 //seconds

#define CONFIG_FILE "/var/OBU_Config.txt"

int offline = 1;
int GPSprocess = 0; //to process GPS offline should be 1


GPSData gpsData;
int gpsSockFd;
char gpsAddr[] = "127.0.0.1";

static int pid;
static WMEApplicationRequest entryRx;
static WMEApplicationRequest entryTx;
static WMEApplicationRequest wreqTx;
static WSMRequest wsmreqTx;

static uint64_t packetsTx = 0;
static uint64_t dropsTx = 0;
int notxpkts = 0;
int countTx = 0;
int countRx = 0;

/* Callback function declarations */
void 	receiveWME_NotifIndication(WMENotificationIndication *wmeindication);
void 	receiveWRSS_Indication(WMEWRSSRequestIndication *wrssindication);
void 	receiveTsfTimerIndication(TSFTimer *timer);
int	confirmBeforeJoin(WMEApplicationIndication *); /* Callback function to join with the Tx application */

/* Function Declarations */
int buildPSTEntry();
int buildUSTEntry();
int buildWSMRequestPacket();
int buildWMEApplicationRequest();

int buildSRMPacket();
int buildSPATPacket();

void sig_int(void);
void sig_term(void);
void closeAll(void);
void initDsrc();
int readConfig(void);

int main()
{
    printf("Start \n");

    readConfig();

    int rx_ret = 0;
    WSMMessage rxmsg;
    WSMIndication rxpkt;
    rxmsg.wsmIndication = &rxpkt;


    struct timeval currentTimeTV;
    double previousTime, currentTime;

    // Initializations:
    {
        pid = getpid();

        initDsrc(); // initialize the DSRC channels and invoke the drivers for sending and recieving

        gpsSockFd = gpsc_connect(gpsAddr);
        printf("created GPS socket\n");

        /* catch control-c and kill signal*/
        signal(SIGINT,(void *)sig_int);
        signal(SIGTERM,(void *)sig_term);

    }

    if (!offline)
    {
        gpsData.actual_time = -1;
        while (gpsData.actual_time < 0)
        {
            sleep(1);
            char ch = '1';
            write(gpsSockFd,&ch,1);
            read(gpsSockFd,(void *)&gpsData,sizeof(gpsData));
            printf("waiting for GPS - GPSTime: %.1f\n", gpsData.actual_time);
        }
        sched_yield();
        sleep(5);

        //GPS recording File
        FILE *gpsRecordFile;

        if ((gpsRecordFile = fopen(GPS_RECORDING,"r")) != NULL)
        {
            printf("the %s file already exist, renaming the file to",GPS_RECORDING);
            fclose(gpsRecordFile);
            char fileName[256];
            int i = 0;
            sprintf(fileName, "%s.old%d", GPS_RECORDING, i);
            while((gpsRecordFile = fopen(fileName,"r")) != NULL)
            {
                fclose(gpsRecordFile);
                sprintf(fileName, "%s.old%d", GPS_RECORDING, ++i);
            }
            rename(GPS_RECORDING,fileName);
            printf(" %s\n",fileName);
        }

        if ((gpsRecordFile = fopen(GPS_RECORDING,"w")) != NULL)
        {
            fprintf(gpsRecordFile, "GPSorRSSI, CurrentTime,GPSTime,latitude,longitude,altitude,course,speed");
            fprintf(gpsRecordFile, ",RcvdMsgTimeStamp,RcvdMsgGPSTime,RcvdMsgLat,RcvdMsgLon,RcvdMsgAlt,RcvdMsgType,RcvdMsgID,RcvdMsgRSSi\n");
            fclose(gpsRecordFile);
        }
        else
        {
            printf("error creating %s file\n",GPS_RECORDING);
        }

    //****************** bin file

        FILE *gpsRecordBinFile;

        if ((gpsRecordBinFile = fopen(GPS_RECORDING_BIN,"r")) != NULL)
        {
            printf("the %s file already exist, renaming the file to",GPS_RECORDING_BIN);
            fclose(gpsRecordBinFile);
            char fileName[256];
            int i = 0;
            sprintf(fileName, "%s.old%d", GPS_RECORDING_BIN, i);
            while((gpsRecordBinFile = fopen(fileName,"r")) != NULL)
            {
                fclose(gpsRecordBinFile);
                sprintf(fileName, "%s.old%d", GPS_RECORDING_BIN, ++i);
            }
            rename(GPS_RECORDING_BIN,fileName);
            printf(" %s\n",fileName);
        }

        if ((gpsRecordBinFile = fopen(GPS_RECORDING_BIN,"w")) != NULL)
        {
            fclose(gpsRecordBinFile);
        }
        else
        {
            printf("error creating %s file\n",GPS_RECORDING_BIN);
        }
    }


    struct GPSRecording gpsRec;
    memset(&gpsRec, 0, sizeof(struct GPSRecording));

    gettimeofday(&currentTimeTV, NULL);
    currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
    previousTime = floor(currentTime/TIME_STEP) * TIME_STEP;


    if (GPSprocess & offline)
    {
        gpsRec.rssi = -1;
        gpsRec.SenderGPSTime = -1;
        gpsRec.SenderID = -1;
        gpsRec.SenderLat = -1;
        gpsRec.SenderLon = -1;
        gpsRec.SenderTime = -1;
        gpsRec.GPSorRSSI = 1;

        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;

        int begin = 13000; //23 Ave
        int end = 14700;  //23 Ave
        begin = 7130; //Whole route
        end = 20300; //Whole route
        int i = 0;
        while(1)
        {
            i++;
            currentTime += 0.1;
            if (read_GPS_log(&gpsData, currentTime))
            {
                closeAll();
                return 0;
            }
            memcpy(&gpsRec.gpsData, &gpsData, sizeof(gpsData));
            gpsRec.currentTime = currentTime;

            if (i >= begin)
            {
                logDatatoFile(&gpsRec);
                if (i > end)
                {
                    closeAll();
                    return 0;
                }
            }
        }
    }

    while (1) //infinite loop
    {
        static int counter = 0;
        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
        if ((currentTime - previousTime) >= MIN_INTERVAL) // check if enough time is passed to broadcast new message
        {
            previousTime = previousTime + MIN_INTERVAL;
            counter ++;
            if (counter >= (int)(TIME_STEP/MIN_INTERVAL))
            {
                counter = 0;
                // Do the control and broadcasting tasks.

                printf("\nReading GPS information....\n");

                if(offline)
                {
                    read_GPS_log(&gpsData, currentTime);
                }
                else
                {
                    char ch = '1';

                    write(gpsSockFd,&ch,1);
                    read(gpsSockFd,(void *)&gpsData,sizeof(gpsData));

                    memcpy(&gpsRec.gpsData, &gpsData, sizeof(gpsData));
                    gpsRec.currentTime = currentTime;
                    gpsRec.GPSorRSSI = 1;

                    logDatatoFile(&gpsRec);

                    gpsRec.rssi = -1;
                    gpsRec.SenderGPSTime = -1;
                    gpsRec.SenderID = -1;
                    gpsRec.SenderLat = -1;
                    gpsRec.SenderLon = -1;
                    gpsRec.SenderTime = -1;
                }


                printf("RSE GPS Data\nTime: %.3f, GPSTime: %.1f, Lat: %.7f, Lon: %.7f\nAlt: %.1f, course: %.0f, speed, %.2f\n",
                    currentTime,
                    gpsData.actual_time,
                    gpsData.latitude,
                    gpsData.longitude,
                    gpsData.altitude,
                    gpsData.course,
                    gpsData.speed);

            }
        }

        //rx_ret = rxWSMPacket(pid, &rxpkt);
        rx_ret = rxWSMMessage(pid, &rxmsg); /* Function to receive the Data from TX application */
        if (rx_ret > 0){
            printf("Received WSMP Packet txpower= %d, rateindex=%d Packet No =#%d#\n", rxpkt.chaninfo.txpower, rxpkt.chaninfo.rate, countRx++);
            struct Message *dsrcmp = (struct Message *)rxpkt.data.contents;

            if (!offline)
            {
                gpsRec.rssi = rxpkt.rssi;
                gpsRec.SenderGPSTime = dsrcmp->gpsTime;
                gpsRec.SenderID = dsrcmp->SenderID;
                gpsRec.SenderLat = dsrcmp->SenderLat;
                gpsRec.SenderLon = dsrcmp->SenderLon;
                gpsRec.SenderTime = dsrcmp->TimeStamp;

                gpsRec.currentTime = currentTime;
                gpsRec.GPSorRSSI = 2;


                logDatatoFile(&gpsRec);
            }
        }

        sched_yield();
        usleep(1000);
    }
    printf("\n Transmitted =  %d dropped = %llu\n",countTx,dropsTx);
    printf("\n Recieved =  %d\n",countRx);

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
    stopWBSS(pid, &wreqTx);
    removeProvider(pid, &entryTx);
    removeUser(pid, &entryRx);
    //closeController();
    gpsc_close_sock();
    signal(SIGINT,SIG_DFL);
    printf("\n\nPackets Sent =  %llu\n",packetsTx);
    printf("\nPackets Dropped = %llu\n",dropsTx);
    printf("localtx killed by control-C\n");
}

void receiveWME_NotifIndication(WMENotificationIndication *wmeindication)
{
}

void receiveWRSS_Indication(WMEWRSSRequestIndication *wrssindication)
{
    printf("WRSS recv channel %d",(u_int8_t)wrssindication->wrssreport.channel);
    printf("WRSS recv reportt %d",(u_int8_t)wrssindication->wrssreport.wrss);
}

void receiveTsfTimerIndication(TSFTimer *timer)
{
    printf("TSF Timer: Result=%d, Timer=%llu",(u_int8_t)timer->result,(u_int64_t)timer->timer);
}

/* Function to build the Provider Service Table Entry */
int buildPSTEntry(void)
{
    //Transmitting entry
    entryTx.psid = 10;/* Provider Service IDentifier of the process.
                      you cant register 2 applications with same psid. */
    entryTx.priority = 2;
    entryTx.channel = 172;
    entryTx.repeatrate = 50;
    entryTx.channelaccess = 0;

    return 1;
}

/* Function to build the User Service Table Entry */
int buildUSTEntry(void)
{
    //Receiveing entry
    entryRx.psid = 11;
    entryRx.userreqtype = 2;
    entryRx.channel = 172;
    entryRx.schaccess  = 1;
    entryRx.schextaccess = 1;

    return 1;
}

/* Function to build the WSM Request packet. */
int buildWSMRequestPacket()
{
    wsmreqTx.chaninfo.channel = 172;
    wsmreqTx.chaninfo.rate = 3;
    wsmreqTx.chaninfo.txpower = 15;
    wsmreqTx.version = 1;
    wsmreqTx.security = 0;
    wsmreqTx.psid = 10;
    wsmreqTx.txpriority = 2;
    memset(&wsmreqTx.data, 0, sizeof( WSMData));
    wsmreqTx.data.length = 0;

    return 1;
}

int  buildWMEApplicationRequest()
{
    wreqTx.psid = 10 ;
    printf(" WME App Req %d \n",wreqTx.psid);
    //strncpy(wreq.acm.contents, entry.acm.contents, OCTET_MAX_LENGTH);
    //printf(" WME App Req %s \n",wreq.acm.contents);
    //wreq.acm.length = entry.acm.length;
    wreqTx.repeats = 1;
    wreqTx.persistence = 1;
    wreqTx.channel = 172;
    return 1;
}

void initDsrc()
{
    printf("Filling Provider Service Table entry %d\n",buildPSTEntry());
    printf("Building a WSM Request Packet %d\n", buildWSMRequestPacket());
    printf("Building a WME Application  Request %d\n",buildWMEApplicationRequest());

    /* Function invokeWAVEDevice(int type, int blockflag)/invokeWAVEDriver(int blockflag) instructs the libwave
     * to open a connection to a wave device either on the local machine or on a remote machine.
     * Invoke the wave device before issuing any request to the wave device.
     * If you going to run your application on Local Device(RSU/OBU) you should call invokeWAVEDriver(int blockflag).
     * If you going to run your application on Remote machine(Computer/laptop,etc),
     * you should call invokeWAVEDevice(int type, int blockflag).
     * For type = WAVEDEVICE_REMOTE, before calling invokeWAVEDevice(int type, int blockflag) make a call to API Details
     * int setRemoteDeviceIP(char *ipaddr) to set the IP address of the remote wave device
     */
    if ( invokeWAVEDriver(0) < 0 ){
        printf( "Opening Failed.\n ");
        exit(-1);
    } else {
        printf("Driver invoked\n");
    }
    /* Registering the call back functions */
    registerWMENotifIndication(receiveWME_NotifIndication); /* It is called when WME Notification received by application */
    registerWRSSIndication(receiveWRSS_Indication); 	/* It is WRSS indication */
    registertsfIndication(receiveTsfTimerIndication); 	/* Used to Indicate the Timing Synchronus Function */

    /* Registering the application. You can register the application as a provider/user.
     * Provider can transmit WSA(WBSS) packets. but user cant.
     * In order to initiate communications on a SCH, an RSU or an OBU transmits WAVE Announcement action frames on the CCH
     * to advertise offered services available on that SCH such a device is the initiator of a WBSS called a provider.
     * An OBU receives the announcement on the CCH and generally establishes communications with the provider on the specified SCH,
     * such a device is called a user.
     */
    printf("Registering provider\n ");
    if ( registerProvider( pid, &entryTx ) < 0 ){
        printf("\nRegister Provider failed\n");
        removeProvider(pid, &entryTx);
        registerProvider(pid, &entryTx);
    } else {
        printf("provider registered with PSID = %u\n",entryTx.psid );
    }


    printf("Filling User Service Table entry %d\n",buildUSTEntry());

    registerLinkConfirm(confirmBeforeJoin); /* Registering the callback function */

    printf("Invoking WAVE driver \n");

    if (invokeWAVEDevice(WAVEDEVICE_LOCAL, 0) < 0)
    {
        printf("Open Failed. Quitting\n");
        exit(-1);
    }

    printf("Registering User %d\n", entryRx.psid);
    int ret = registerUser(pid, &entryRx);
    if ( ret < 0)
    {
        printf("Register User Failed \n");
        printf("Removing user if already present  %d\n", !removeUser(pid, &entryRx));
        printf("USER Registered %d with PSID =%u \n", registerUser(pid, &entryRx), entryRx.psid );
    }
    else
        printf("USER Registered %d with PSID =%u \n", ret, entryRx.psid );

}

int confirmBeforeJoin(WMEApplicationIndication *appind)
{
    printf("\nJoin\n");
    return 1; /* Return 0 for NOT Joining the WBSS */
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

        if (strcasecmp(str,"Offline")==0)
        {
            str = strtok (NULL," ,");
            offline = atoi(str);
            if (offline)
                printf("GPS is in offline mode\n");
            else
                printf("GPS is online\n");
        }
    }
    free(line);
    fclose(configFile);
    return 0;
}
