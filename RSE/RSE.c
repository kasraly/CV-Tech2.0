#define _GNU_SOURCE

#include "gpsc_probe.h"
#include "wave.h"
#include "asnwave.h"
#include "SPAT.h"
#include "MapData.h"
#include "SignalRequestMsg.h"
#include "ProbeDataManagement.h"

#include "ControllerLib.h"
#define MIN_INTERVAL 0.1 //seconds
#define TIME_STEP 1 //seconds

#define CONFIG_FILE "/var/RSE_Config.txt"

//golbale variable
u_int8_t SRM_enable_g = 0;
u_int16_t SRM_ACK_Num_g = 0;

// params
int intersectionID = 0;

char controllerIP[32] = "192.168.0.79";
uint16_t controllerSnmpPort = 161;
uint16_t controllerBroadcastPort = 6053;

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
int	    confirmBeforeJoin(WMEApplicationIndication *); /* Callback function to join with the Tx application */

/* Function Declarations */
int buildPSTEntry();
int buildUSTEntry();
int buildWSMRequestPacket();
int buildWMEApplicationRequest();

int buildSRMPacket();
int buildSPATPacket(double currentTime);

void sig_int(void);
void sig_term(void);
void closeAll(void);
void initDsrc();
int readConfig(void);

long Rsemin,Rsesec,Obemin,Obesec;

struct timeval currentTimeTV;
double previousTime, currentTime;


int processSRM(SignalRequestMsg_t *srm, unsigned char *preemptPhase, float *preemptPhaseTime);

int main()
{
    printf("Start \n");

    int rx_ret = 0;
	WSMMessage rxmsg;
    WSMIndication rxpkt;
    rxmsg.wsmIndication = &rxpkt;


//    struct timeval currentTimeTV;
//    double previousTime, currentTime;

    // Initializations:
    {
        pid = getpid();

        readConfig();

        initDsrc(); // initialize the DSRC channels and invoke the drivers for sending and recieving

        initController(controllerIP, controllerSnmpPort);

        gpsSockFd = gpsc_connect(gpsAddr);
        printf("created GPS socket\n");

        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
        previousTime = floor(currentTime/TIME_STEP) * TIME_STEP;

        /* catch control-c and kill signal*/
        signal(SIGINT,(void *)sig_int);
        signal(SIGTERM,(void *)sig_term);

    }


    while (1) //infinite loop
    {

        static unsigned char preemptPhase = 0x00;
        static float preemptPhaseTime = 0;
        static int counter = 0;
        static int spatCounter = 0;
        static int preemptCounter = 0;

        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
        if ((currentTime - previousTime) >= MIN_INTERVAL) // check if enough time is passed to broadcast new message
        {
            previousTime = previousTime + MIN_INTERVAL;

            preemptCounter++;
            if (preemptCounter >= (int)(PREEMPT_INTERVAL/MIN_INTERVAL))
            {
                preemptCounter = 0;
                if (preemptPhaseTime > 0)
                {
                    preemptPhaseTime -= SPaT_READ_INTERVAL;
                }
                else
                {
                    preemptPhaseTime = 0;
                    preemptPhase = 0;
                }

                printf("executing Preemption algorithm with requested phase 0x%02x\n",preemptPhase);
                signalPreempt(preemptPhase);
            }

            spatCounter++;
            if (spatCounter >= (int)(SPaT_READ_INTERVAL/MIN_INTERVAL))
            {
                spatCounter = 0;

                printf("SPaT routine ...\n");
                buildSPATPacket(currentTime);
                //buildMAPPacket();
                //send the DSRC message
                if( txWSMPacket(pid, &wsmreqTx) < 0)
                {
                    dropsTx++;
                }
                else
                {
                    packetsTx++;
                    countTx++;
                }

				// finished SPaT transmission
				if (SRM_enable_g == 1)
				{
					SRM_enable_g =0;
					SRM_ACK_Num_g = 0;
				}

                if((notxpkts != 0) && (countTx >= notxpkts))
                    break;

                printf("DSRC message Transmitted #%llu#      Drop #%llu#     len #%u#\n",
                    packetsTx,
                    dropsTx,
                    wsmreqTx.data.length);
            }

            counter ++;
            if (counter >= (int)(TIME_STEP/MIN_INTERVAL))
            {
                counter = 0;
                // Do the control and broadcasting tasks.

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
                //buildSRMPacket();
            }
        }

        //rx_ret = rxWSMPacket(pid, &rxpkt);
        rx_ret = rxWSMMessage(pid, &rxmsg); /* Function to receive the Data from TX application */
        if (rx_ret > 0){
            printf("Received WSMP Packet txpower= %d, rateindex=%d Packet No =#%d#\n", rxpkt.chaninfo.txpower, rxpkt.chaninfo.rate, countRx++);
            rxWSMIdentity(&rxmsg,WSMMSG_SRM); //Identify the type of received Wave Short Message.
            if (!rxmsg.decode_status) {
                SignalRequestMsg_t *srmRcv = (SignalRequestMsg_t *)rxmsg.structure;
                printf("Received Signal Request Message, Mesage count %d\n\v", (int)srmRcv->msgCnt);
                xml_print(rxmsg); /* call the parsing function to extract the contents of the received message */

                processSRM(srmRcv, &preemptPhase, &preemptPhaseTime);
                printf("Test for the preemptPhase = 0x%02x, preemptPhaseTime = %f\n", preemptPhase,preemptPhaseTime);
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

int buildSPATPacket(double currentTime)
{
    int Sstate, Stime;
    char Intersection1_name[] = "Intersection1";
    char movementstate1_name[] = "EastToWestStraight";
    char movementstate2_name[] = "EastToWestLeft";
    char movementstate3_name[] = "WestToEastStraight";
    char movementstate4_name[] = "WestToEastLeft";
    char movementstate5_name[] = "NorthToSouthStraight";
    char movementstate6_name[] = "NorthToSouthLeft";
    char movementstate7_name[] = "SouthToNorthStraight";
    char movementstate8_name[] = "SouthToNorthLeft";

    int SignalMeg[8][4];
    int i;

    Rsemin = (long) currentTime;
    Rsesec = (long) ((currentTime-Rsemin)*1000000);

//    delaymin = Rsemin - Obemin;
//    delaysec = Rsesec - Obesec;

    printf("Initializing SPaT Packet\n");
    static int spatCount = 0;
    asn_enc_rval_t rvalenc;
    SPAT_t *spat;
	IntersectionState_t *intersectionstate1;
    MovementState_t *movementstate1, *movementstate2, *movementstate3, *movementstate4, *movementstate5, *movementstate6, *movementstate7, *movementstate8;

    /* SRM related information */

    spat = (SPAT_t *) calloc(1,sizeof(SPAT_t));
    spat->msgID.buf = (uint8_t *) calloc(1, sizeof(uint8_t)); /* allocate memory for buffer which is used to store,
                                                              * what type of message it is
                                                              */
    spat->msgID.size = sizeof(uint8_t);
    spat->msgID.buf[0] = DSRCmsgID_signalPhaseAndTimingMessage;
    // spatCount++;

    intersectionstate1 = (IntersectionState_t *) calloc(1, sizeof(IntersectionState_t));
    intersectionstate1->name = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    intersectionstate1->name->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    intersectionstate1->name->size = sizeof(Intersection1_name);

    for(i=0;i<sizeof(Intersection1_name);i++)
    {
        intersectionstate1->name->buf[i] = Intersection1_name[i];
    }

    intersectionstate1->id.buf = (uint8_t *) calloc(4, sizeof(uint8_t));
    intersectionstate1->id.size = 4;
    intersectionstate1->id.buf[0] = 0x00;
    intersectionstate1->id.buf[1] = 0x00;
    intersectionstate1->id.buf[2] = 0x00;
    intersectionstate1->id.buf[3] = 0x64;

    intersectionstate1->status.buf = (uint8_t *) calloc(1, sizeof(uint8_t));
    intersectionstate1->status.size = sizeof(uint8_t);
    intersectionstate1->status.buf[0] = 0;
    intersectionstate1->timeStamp = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(intersectionstate1->timeStamp) = 0x4EBAF6B5;

    readSPaT(SignalMeg,currentTime);

//  movementstate1 defined as "EastToWestStraight"
    movementstate1 = (MovementState_t *) calloc(1, sizeof(MovementState_t));
    movementstate1->movementName = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    movementstate1->movementName->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
//    movementstate1->movementName->size = sizeof(movementstate1_name);
//    for(i=0;i<sizeof(movementstate1_name);i++)
//    {
//      movementstate1->movementName->buf[i] = movementstate1_name[i];
//    }
    movementstate1->movementName->size = 1;
    movementstate1->movementName->buf[0] = 0x01;

    movementstate1->laneCnt=(uint8_t *) calloc(1, sizeof(uint8_t));

    movementstate1->laneSet.buf = (uint8_t *) calloc(2, sizeof(uint8_t));
    movementstate1->laneSet.size = 1;
    movementstate1->laneSet.buf[0] = 0x01;

    movementstate1->pedState= (PedestrianSignalState_t *) calloc(1, sizeof(uint8_t ));

    movementstate1->currState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate1->currState[0] = SignalMeg[0][0];
    movementstate1->timeToChange = SignalMeg[0][1];
    movementstate1->yellState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate1->yellState[0] = 0x03;
    movementstate1->yellTimeToChange = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(movementstate1->yellTimeToChange) = 0x04;

//  movementstate2 defined as "EastToWestLeft"
    movementstate2 = (MovementState_t *) calloc(1, sizeof(MovementState_t));
    movementstate2->movementName = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    movementstate2->movementName->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
//    movementstate2->movementName->size = sizeof(movementstate2_name);
//    for(i=0;i<sizeof(movementstate2_name);i++)
//    {
//      movementstate2->movementName->buf[i] = movementstate2_name[i];
//    }

    movementstate2->movementName->size = 1;
    movementstate2->movementName->buf[0] = 0x02;

    movementstate2->laneCnt=(uint8_t *) calloc(1, sizeof(uint8_t));

    movementstate2->laneSet.buf = (uint8_t *) calloc(2, sizeof(uint8_t));
    movementstate2->laneSet.size = 1;
    movementstate2->laneSet.buf[0] = 0x01;

    movementstate2->pedState= (PedestrianSignalState_t *) calloc(1, sizeof(uint8_t ));

    movementstate2->currState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate2->currState[0] = SignalMeg[1][0];
    movementstate2->timeToChange = SignalMeg[1][1];
    movementstate2->yellState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate2->yellState[0] = 0x03;
    movementstate2->yellTimeToChange = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(movementstate2->yellTimeToChange) = 0x04;

//  movementstate3 defined as "WestToEastStraight"
    movementstate3 = (MovementState_t *) calloc(1, sizeof(MovementState_t));
    movementstate3->movementName = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    movementstate3->movementName->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
//    movementstate3->movementName->size = sizeof(movementstate3_name);
//    for(i=0;i<sizeof(movementstate3_name);i++)
//    {
//      movementstate3->movementName->buf[i] = movementstate3_name[i];
//    }
    movementstate3->movementName->size = 1;
    movementstate3->movementName->buf[0] = 0x03;

    movementstate3->laneCnt=(uint8_t *) calloc(1, sizeof(uint8_t));

    movementstate3->laneSet.buf = (uint8_t *) calloc(2, sizeof(uint8_t));
    movementstate3->laneSet.size = 1;
    movementstate3->laneSet.buf[0] = 0x01;

    movementstate3->pedState= (PedestrianSignalState_t *) calloc(1, sizeof(uint8_t ));

    movementstate3->currState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate3->currState[0] = SignalMeg[2][0];
    movementstate3->timeToChange = SignalMeg[2][1];
    movementstate3->yellState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate3->yellState[0] = 0x03;
    movementstate3->yellTimeToChange = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(movementstate3->yellTimeToChange) = 0x04;

//  movementstate4 defined as "WestToEastLeft"
    movementstate4 = (MovementState_t *) calloc(1, sizeof(MovementState_t));
    movementstate4->movementName = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    movementstate4->movementName->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
//    movementstate4->movementName->size = sizeof(movementstate4_name);
//    for(i=0;i<sizeof(movementstate4_name);i++)
//    {
//      movementstate4->movementName->buf[i] = movementstate4_name[i];
//    }
    movementstate4->movementName->size = 1;
    movementstate4->movementName->buf[0] = 0x04;

    movementstate4->laneCnt=(uint8_t *) calloc(1, sizeof(uint8_t));

    movementstate4->laneSet.buf = (uint8_t *) calloc(2, sizeof(uint8_t));
    movementstate4->laneSet.size = 1;
    movementstate4->laneSet.buf[0] = 0x01;

    movementstate4->pedState= (PedestrianSignalState_t *) calloc(1, sizeof(uint8_t ));

    movementstate4->currState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate4->currState[0] = SignalMeg[3][0];
    movementstate4->timeToChange = SignalMeg[3][1];
    movementstate4->yellState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate4->yellState[0] = 0x03;
    movementstate4->yellTimeToChange = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(movementstate4->yellTimeToChange) = 0x04;

//  movementstate5 defined as "NorthToSouthStraight"
    movementstate5 = (MovementState_t *) calloc(1, sizeof(MovementState_t));
    movementstate5->movementName = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    movementstate5->movementName->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
//    movementstate5->movementName->size = sizeof(movementstate5_name);
//    for(i=0;i<sizeof(movementstate5_name);i++)
//    {
//      movementstate5->movementName->buf[i] = movementstate5_name[i];
//    }
    movementstate5->movementName->size = 1;
    movementstate5->movementName->buf[0] = 0x05;

    movementstate5->laneCnt=(uint8_t *) calloc(1, sizeof(uint8_t));

    movementstate5->laneSet.buf = (uint8_t *) calloc(2, sizeof(uint8_t));
    movementstate5->laneSet.size = 1;
    movementstate5->laneSet.buf[0] = 0x01;

    movementstate5->pedState= (PedestrianSignalState_t *) calloc(1, sizeof(uint8_t ));

    movementstate5->currState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate5->currState[0] = SignalMeg[4][0];
    movementstate5->timeToChange = SignalMeg[4][1];
    //movementstate5->timeToChange = Rsemin;
    movementstate5->yellState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate5->yellState[0] = 0x03;
    movementstate5->yellTimeToChange = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(movementstate5->yellTimeToChange) = 0x04;

//  movementstate6 defined as "NorthToSouthLeft"
    movementstate6 = (MovementState_t *) calloc(1, sizeof(MovementState_t));
    movementstate6->movementName = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    movementstate6->movementName->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
//    movementstate6->movementName->size = sizeof(movementstate6_name);
//    for(i=0;i<sizeof(movementstate6_name);i++)
//    {
//      movementstate6->movementName->buf[i] = movementstate6_name[i];
//    }
    movementstate6->movementName->size = 1;
    movementstate6->movementName->buf[0] = 0x06;

    movementstate6->laneCnt=(uint8_t *) calloc(1, sizeof(uint8_t));

    movementstate6->laneSet.buf = (uint8_t *) calloc(2, sizeof(uint8_t));
    movementstate6->laneSet.size = 1;
    movementstate6->laneSet.buf[0] = 0x01;

    movementstate6->pedState= (PedestrianSignalState_t *) calloc(1, sizeof(uint8_t ));

    movementstate6->currState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate6->currState[0] = SignalMeg[5][0];
    movementstate6->timeToChange = SignalMeg[5][1];
    //movementstate6->timeToChange = Rsesec;
    movementstate6->yellState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate6->yellState[0] = 0x03;
    movementstate6->yellTimeToChange = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(movementstate6->yellTimeToChange) = 0x04;

//  movementstate7 defined as "SouthToNorthStraight"
    movementstate7 = (MovementState_t *) calloc(1, sizeof(MovementState_t));
    movementstate7->movementName = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    movementstate7->movementName->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
//    movementstate7->movementName->size = sizeof(movementstate7_name);
//    for(i=0;i<sizeof(movementstate7_name);i++)
//    {
//      movementstate7->movementName->buf[i] = movementstate7_name[i];
//    }
    movementstate7->movementName->size = 1;
    movementstate7->movementName->buf[0] = 0x07;

    movementstate7->laneCnt=(uint8_t *) calloc(1, sizeof(uint8_t));

    movementstate7->laneSet.buf = (uint8_t *) calloc(2, sizeof(uint8_t));
    movementstate7->laneSet.size = 1;
    movementstate7->laneSet.buf[0] = 0x01;

    movementstate7->pedState= (PedestrianSignalState_t *) calloc(1, sizeof(uint8_t ));

    movementstate7->currState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate7->currState[0] = SignalMeg[6][0];
    movementstate7->timeToChange = SignalMeg[6][1];

//   if(SRM_enable_g == 1)
//	{
//		movementstate7->timeToChange = SRM_ACK_Num_g;
//	}
//	else
//	{
//		movementstate7->timeToChange = 0x00;
//	}

    //movementstate7->timeToChange = Obemin;
    movementstate7->yellState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate7->yellState[0] = 0x03;
    movementstate7->yellTimeToChange = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(movementstate7->yellTimeToChange) = 0x04;


//  movementstate8 defined as "SouthToNorthLeft"
    movementstate8 = (MovementState_t *) calloc(1, sizeof(MovementState_t));
    movementstate8->movementName = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
    movementstate8->movementName->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
//    movementstate8->movementName->size = sizeof(movementstate8_name);
//    for(i=0;i<sizeof(movementstate8_name);i++)
//    {
//      movementstate8->movementName->buf[i] = movementstate8_name[i];
//    }
    movementstate8->movementName->size = 1;
    movementstate8->movementName->buf[0] = 0x08;

    movementstate8->laneCnt=(uint8_t *) calloc(1, sizeof(uint8_t));

    movementstate8->laneSet.buf = (uint8_t *) calloc(2, sizeof(uint8_t));
    movementstate8->laneSet.size = 1;
    movementstate8->laneSet.buf[0] = 0x01;

    movementstate8->pedState= (PedestrianSignalState_t *) calloc(1, sizeof(uint8_t ));

    movementstate8->currState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate8->currState[0] = SignalMeg[7][0];
    movementstate8->timeToChange = SignalMeg[7][1];
    //movementstate8->timeToChange = Obesec;
    movementstate8->yellState = (SignalLightState_t *) calloc(1, sizeof(SignalLightState_t ));
    movementstate8->yellState[0] = 0x03;
    movementstate8->yellTimeToChange = (TimeMark_t *)calloc(1, sizeof(uint32_t));
    *(movementstate8->yellTimeToChange) = 0x04;

    ASN_SEQUENCE_ADD(&intersectionstate1->states.list,movementstate1);
    ASN_SEQUENCE_ADD(&intersectionstate1->states.list,movementstate2);
    ASN_SEQUENCE_ADD(&intersectionstate1->states.list,movementstate3);
    ASN_SEQUENCE_ADD(&intersectionstate1->states.list,movementstate4);
    ASN_SEQUENCE_ADD(&intersectionstate1->states.list,movementstate5);
    ASN_SEQUENCE_ADD(&intersectionstate1->states.list,movementstate6);
    ASN_SEQUENCE_ADD(&intersectionstate1->states.list,movementstate7);
    ASN_SEQUENCE_ADD(&intersectionstate1->states.list,movementstate8);

    ASN_SEQUENCE_ADD(&spat->intersections.list, intersectionstate1);

    rvalenc = der_encode_to_buffer(&asn_DEF_SPAT, spat, &wsmreqTx.data.contents, 1000); /* Encode your SRM in to WSM Packets */
    if (rvalenc.encoded == -1) {
        fprintf(stderr, "Cannot encode %s: %s\n",
                rvalenc.failed_type->name, strerror(errno));
    } else  {
        printf("Structure successfully encoded %d\n", rvalenc.encoded);
        wsmreqTx.data.length = rvalenc.encoded;
        asn_DEF_SPAT.free_struct (&asn_DEF_SPAT, spat, 0);
    }

    return 1;
}

 int buildMAPPacket()
 {

	    asn_enc_rval_t rvalenc;
        MapData_t *map;
        static int msgcount;
        int i;

        Intersection_t *intersection;
        ApproachObject_t *Approach1;
        char Intersection1_name[] = "Intersection1";
        char Approach_name[] = "westbound";

        map = (MapData_t *) calloc(1,sizeof(MapData_t));
        map->msgID.buf = (uint8_t *) calloc(1, sizeof(uint8_t));
        map->msgID.size = sizeof(uint8_t);
        map->msgID.buf[0] = DSRCmsgID_mapData;

        map->msgCnt = msgcount%127;
        msgcount++;

        map->name =(DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
        map->name->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
        map->name->size = sizeof(Intersection1_name);
        for(i=0;i<sizeof(Intersection1_name);i++)
        map->name->buf[i] = Intersection1_name[i];

        map->layerType = (uint8_t *) calloc(1,sizeof(uint8_t));
        map->layerType->buf = (uint8_t *) calloc(1, sizeof(uint8_t));
        map->layerType->size = sizeof(uint8_t);
        map->layerType->buf[0] = LayerType_generalMapData;


        intersection = (Intersection_t *) calloc(1,sizeof(Intersection_t));
        intersection->id.buf=(IntersectionID_t *)calloc(1,sizeof(IntersectionID_t));
        intersection->id.size=sizeof(IntersectionID_t);
        intersection->id.buf[0]=0x06;

        intersection->refPoint= (Position3D_t *) calloc(1,sizeof(Position3D_t));
        intersection->refPoint->Long=0x0001;
        intersection->refPoint->lat=0x001;


        //Approach_drivingLanes *drivingLanes;
//        Approach1->approach->drivingLanes = (Approach__drivingLanes *) calloc(1,sizeof(Approach__drivingLanes));
        // implementation of ApproachObject_t
        Approach1 = (ApproachObject_t *) calloc(1,sizeof(ApproachObject_t));
        Approach1->approach->name = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
        Approach1->approach->name->buf = (DescriptiveName_t *) calloc(1,sizeof(DescriptiveName_t));
        Approach1->approach->name->size = sizeof(Approach_name);
        for(i=0;i<sizeof(Approach_name);i++)
        Approach1->approach->name->buf[i] = Approach_name[i];

        Approach1->approach->id = (ApproachNumber_t *) calloc(1,sizeof(ApproachNumber_t));
        Approach1->approach->id = 1;


        VehicleReferenceLane_t *lanes;
        lanes = (VehicleReferenceLane_t *) calloc(1,sizeof(VehicleReferenceLane_t));

        lanes->laneNumber.buf = (LaneNumber_t *) calloc(1,sizeof(LaneNumber_t));
        lanes->laneNumber.size = 1;
        lanes->laneNumber.buf[0] = 1;

        lanes->laneWidth = (LaneWidth_t *) calloc(1,sizeof(LaneWidth_t));
        lanes->laneWidth = 3;


        /***************************** implementation of node list here ***************************************/

        /******************************************************************************************************/
//        ASN_SEQUENCE_ADD(&Approach1->approach->drivingLanes.list,lanes);
        ASN_SEQUENCE_ADD(&intersection->approaches.list,Approach1);
        ASN_SEQUENCE_ADD(&map->intersections->list,intersection);

        rvalenc = der_encode_to_buffer(&asn_DEF_MapData, map, &wsmreqTx.data.contents, 1000);

        if (rvalenc.encoded == -1) {
                fprintf(stderr, "Cannot encode %s: %s\n",
                rvalenc.failed_type->name, strerror(errno));
        } else  {
                printf("Structure successfully encoded %d\n", rvalenc.encoded);
                wsmreqTx.data.length = rvalenc.encoded;
                asn_DEF_MapData.free_struct (&asn_DEF_MapData, map, 0);
        }

	    return 1;
}

 /* Main use of this function is we will encapsulate the SRM related info in to WSM request packet */
int buildSRMPacket()
{
    static int srmCount = 0;
    asn_enc_rval_t rvalenc;
    SignalRequestMsg_t *srm;

    /* SRM related information */
    srm = (SignalRequestMsg_t *) calloc(1,sizeof(SignalRequestMsg_t));
    srm->msgID.buf = (uint8_t *) calloc(1, sizeof(uint8_t)); /* allocate memory for buffer which is used to store,
                                                              * what type of message it is
                                                              */
    /* DSRCmsgID_basicSafetyMessage - BSM
     * DSRCmsgID_probeVehicleData - PVD
     * DSRCmsgID_roadSideAlert - RSA
     * DSRCmsgID_intersectionCollisionAlert
     * DSRCmsgID_mapData - MAP
     * DSRCmsgID_signalPhaseAndTimingMessage - SPAT
     * DSRCmsgID_travelerInformation - TIM
     * These are the available/supported SAE message format's
     * For each type you need choose the appropriate structure
     */
    srm->msgID.size = sizeof(uint8_t);
    srm->msgID.buf[0] = DSRCmsgID_signalRequestMessage; /* Choose what type of message you want to transfer */
    srm->msgCnt = srmCount++;

    rvalenc = der_encode_to_buffer(&asn_DEF_SignalRequestMsg, srm, &wsmreqTx.data.contents, 1000); /* Encode your SRM in to WSM Packets */
    if (rvalenc.encoded == -1) {
        fprintf(stderr, "Cannot encode %s: %s\n",
                rvalenc.failed_type->name, strerror(errno));
    } else  {
        printf("Structure successfully encoded %d\n", rvalenc.encoded);
        wsmreqTx.data.length = rvalenc.encoded;
        asn_DEF_SignalRequestMsg.free_struct (&asn_DEF_SignalRequestMsg, srm, 0);
    }

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

int processSRM(SignalRequestMsg_t *srm, unsigned char *preemptPhase, float *preemptPhaseTime)
{
    static int vehicleID = 0;

	SRM_enable_g = 1;
	SRM_ACK_Num_g = 0x4A;

    if (srm->request.id.buf[0] == intersectionID)
    {
        if (*preemptPhaseTime <= 0)
        {
            vehicleID = srm->vehicleVIN->id->buf[0];
            *preemptPhase = 0x01<<(srm->request.requestedAction->buf[0]-1);
            *preemptPhaseTime = srm->endOfService->second;


             Obemin = srm->endOfService->minute;
             Obesec = srm->endOfService->second;

             Obemin = (long) currentTime - Obemin;
             Obesec = (long) ((currentTime-Rsemin)*1000000) - Obesec;

            return 1;
        }
        else
        {
            if (vehicleID == srm->vehicleVIN->id->buf[0])
            {
                *preemptPhaseTime = srm->endOfService->second;
                *preemptPhase = 0x01<<(srm->request.requestedAction->buf[0]-1);
                return 1;
            }
        }
    }
    return 0;
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

        if (strcasecmp(str,"Controller_IP")==0)
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
        else if (strcasecmp(str,"intersectionID")==0)
        {
            str = strtok (NULL," ,");
            intersectionID = atoi(str);
            printf("intersectionID is %d\n",intersectionID);
        }

/*        else if (strcasecmp(str,"Latitude")==0)
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
        else if (strcasecmp(str,"Server_Port")==0)
        {
            str = strtok (NULL," ,");
            serverPort = atoi(str);
            printf("Server_Port is %d\n",serverPort);
        }*/
    }
    free(line);
    fclose(configFile);
    return 0;
}
