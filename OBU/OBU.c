/* change logs
170115 initial from the template
170221 SRM message's predifined values, map matching/SRM declarations
170306 filled the phase number and discussed about the preemption control in OBE
170308 1.finished the discussions about the implementation in OBE and started the implement this in OBE.
       2. Major function = loading table + searching ID +distance Calulation
       3. the map macting part is still  under constroctions.
170309 1.loading table. 2.searching copresponding Intersection Info
       3. perform preemption control strategy


*/

#define _GNU_SOURCE


#define OFFLINE
//#undef OFFLINE

#ifdef OFFLINE
    #include "GPS_offline.h"
#endif // OFFLINE

#include "gpsc_probe.h"
#include "MapMatch.h"
#include "wave.h"
#include "asnwave.h"
#include "SPAT.h"
#include "SignalRequestMsg.h"
#include "ProbeDataManagement.h"
#include "BasicSafetyMessageVerbose.h"
#include "PreemptionControlOBESide.h"
#include "SmartphoneMsgPro.h"
#include "MultiClientSocket.h"

// #include "ControllerLib.h"
#define MIN_INTERVAL 0.1 //seconds
#define TIME_STEP 1 //seconds TIMER1, sending broadcast per 1 second
#define TIMER2_MAPMATCHING 1 // map matching callback
#define TIMER3_SRMGE4PRE 1 // message genarating, per second
#define TIMER4_PHONEMSG 1 //message to phone

#define CONFIG_FILE "/var/RSE_Config.txt"


// testing Git

// BSMBlob, selected from the BasicSafetyMessageVersbose.h file
typedef struct BSMblobVerbose {
    MsgCount_t	 msgCnt;
	TemporaryID_t	 id;
	DSecond_t	 secMark;
	Latitude_t	 lat;
	Longitude_t	 Long;
	Elevation_t	 elev;
	PositionalAccuracy_t	 accuracy;
	TransmissionAndSpeed_t	 speed;
	Heading_t	 heading;
	SteeringWheelAngle_t	 angle;
	AccelerationSet4Way_t	 accelSet;
	BrakeSystemStatus_t	 brakes;
	VehicleSize_t	 size;
} BSMblobVerbose_t;
// signal request message
SignalRequestMsg_t *srm;
int linkID_g = 0;
double distanceToPoint_g;
double intersectionID_g;

unsigned char  endOfServiceSecCount = 0;

//global variables for Preemption control
int preemptionControlDurationTime = 5;
int preempDistance2IntersectionThreshold = 800;

// Smartphone message
char TCPsmartPhoneMsg[256]; // TCPSmartphoneMsag
char TCPsmartPhoneMsg_gene[256]; // TCPSmartphoneMsag
struct PhoneMsg SmartphoneMsg;
static int TCPsendSmartPhoneMsg = 1; // TCP sends to smartphone
double dist2ApprInters = 0;
int reqPhase_g = 0;


GPSData gpsData;
int gpsSockFd;
char gpsAddr[] = "127.0.0.1";

char controllerIP[32] = "192.168.0.79";
uint16_t controllerSnmpPort = 161;
uint16_t controllerBroadcastPort = 6053;

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

int buildSRMPacket(int intersectionID, int reqPhase);
int buildSPATPacket();
int processSPAT(SPAT_t *spat, int *preemptPhase);

void sig_int(void);
void sig_term(void);
void closeAll(void);
void initDsrc();
int readConfig(void);
int updateGPSCourse(GPSData *gpsData);

// 20170301
int fullMapMatching (GPSData *gpsData, int * linkIDtmp, double *distanceToPoint, double *intersectionIDtmp );
//int parsePreemptionRoute(int linkID_g, preemptionRouteColumn_t *preemptionRouteColumnVar);
//int parsePreemptionRoute(int linkID_g );


int main()
{
    printf("Start \n");

    int rx_ret = 0;
    WSMMessage rxmsg;
    WSMIndication rxpkt;
    rxmsg.wsmIndication = &rxpkt;


    struct timeval currentTimeTV;
    double previousTime, currentTime;

    // Initializations:
    {
        pid = getpid();

        readConfig();

        initMapMatch();

        initPreemption(preempDistance2IntersectionThreshold);

        initDsrc(); // initialize the DSRC channels and invoke the drivers for sending and recieving

        initSocket(); //initialize socket for communication with OBE and Smartphone.
//        initController(controllerIP, controllerSnmpPort);

        gpsSockFd = gpsc_connect(gpsAddr);
        printf("created GPS socket\n");

        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
        previousTime = floor(currentTime/TIME_STEP) * TIME_STEP;

        /* catch control-c and kill signal*/
        signal(SIGINT,(void *)sig_int);
        signal(SIGTERM,(void *)sig_term);

        // smartphone initinization


    }


    while (1) //infinite loop
    {
        static int counter = 0;
        static int counter2 = 0;
        static int counter3 = 0;
        static int counter4 = 0;

        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;

        /* function process*/
        if ((currentTime - previousTime) >= MIN_INTERVAL) // check if enough time is passed to broadcast new message
        {
            previousTime = previousTime + MIN_INTERVAL;
            counter ++;
            counter2 ++;
            counter3 ++;
            counter4 ++;

            acceptConnection();
            //regularly to accept new connection from smartphones and close the lost connections

            /*GPS reading*/
            if (counter >= (int)(TIME_STEP/MIN_INTERVAL)) // GPS reading per 1 second
            {
                counter = 0;
                // Do the control and broadcasting tasks.

                printf("\nReading GPS information....\n");

                #ifdef OFFLINE
                    read_GPS_log(&gpsData, currentTime);
                #else
                    char ch = '1';
                    write(gpsSockFd,&ch,1);
                    read(gpsSockFd,(void *)&gpsData,sizeof(gpsData));
                #endif // OFFLINE

                updateGPSCourse(&gpsData);

                printf("RSE GPS Data\nTime: %.3f, GPSTime: %.1f, Lat: %.7f, Lon: %.7f\nAlt: %.1f, course: %.0f, speed, %.2f\n",
                    currentTime,
                    gpsData.actual_time,
                    gpsData.latitude,
                    gpsData.longitude,
                    gpsData.altitude,
                    gpsData.course,
                    gpsData.speed);


                //buildSRMPacket();
                //buildSPATPacket();

//                //send the DSRC message
//                {
//                    if( txWSMPacket(pid, &wsmreqTx) < 0)
//                    {
//                        dropsTx++;
//                    }
//                    else
//                    {
//                        packetsTx++;
//                        countTx++;
//                    }
//
//
//                    if((notxpkts != 0) && (countTx >= notxpkts))
//                        break;
//
//                    printf("DSRC message Transmitted #%llu#      Drop #%llu#     len #%u#\n",
//                        packetsTx,
//                        dropsTx,
//                        wsmreqTx.data.length);
//                }
            }

            if (counter2 >= (int)(TIMER2_MAPMATCHING/MIN_INTERVAL)) // map matching callback functions
            {

                counter2 = 0;
                printf("Map matching ok\n");

                float linkStartDistance;
                //linkID_g = mapMatch(&gpsData, &linkStartDistance);

                printf("MapMatch matched gps point to link %d, distance from link start %f\n",linkID_g, linkStartDistance);

//                fullMapMatching (&gpsData, &linkID_g, &distanceToPoint_g, &intersectionID_g );
               linkID_g = 1094; // for demo purpose, the map macting is still  under constroctions.
            }

            if (counter3 >= (int)(TIMER3_SRMGE4PRE/MIN_INTERVAL)) // message generating
            {
                counter3 = 0;
//                fullMapMatching (&gpsData, &linkID_g, &distanceToPoint_g, &intersectionID_g );
                //printf(" sending ok 11 \n");
                printf("Link ID: %d\n",linkID_g);
//                parsePreemptionRoute(linkID_g);
                int srmActive;
                int intersectionID;
                int reqPhase;

                if (srmActive = preemptionStrategy(&gpsData, linkID_g,&intersectionID, &reqPhase, &dist2ApprInters))
                {
                    buildSRMPacket(intersectionID, reqPhase);
                    reqPhase_g = reqPhase;

                    printf("Sending ok \n");
                    //send the DSRC message
                    {
                        if( txWSMPacket(pid, &wsmreqTx) < 0)
                        {
                            dropsTx++;
                        }
                        else
                        {
                            packetsTx++;
                            countTx++;
                        }


                        if((notxpkts != 0) && (countTx >= notxpkts))
                            break;

                        printf("DSRC message Transmitted #%llu#      Drop #%llu#     len #%u#\n",
                            packetsTx,
                            dropsTx,
                            wsmreqTx.data.length);
                    }
                }
            }

            if (counter4 >= (int)(TIMER4_PHONEMSG/MIN_INTERVAL)) // message generating
            {
                counter4 = 0;
                if (TCPsendSmartPhoneMsg == 1){
                    // memcpy(addr1,s,strlen(s));
                    if ( strcmp(TCPsmartPhoneMsg,TCPsmartPhoneMsg_gene) != 0 ) {
                        printf( "2 Original Message: %s",  TCPsmartPhoneMsg_gene );
                        printf( "3 Chenged  Message: %s",  TCPsmartPhoneMsg );
                    }
                    else{
                        printf("3 Sent     Message: %s\n",TCPsmartPhoneMsg);
                    }

                    sendToClients(TCPsmartPhoneMsg);
                }
            }

        }

        /* receving function part*/
        {
            //rx_ret = rxWSMPacket(pid, &rxpkt);
            rx_ret = rxWSMMessage(pid, &rxmsg); /* Function to receive the Data from TX application */
            if (rx_ret > 0){
                printf("Received WSMP Packet txpower= %d, rateindex=%d Packet No =#%d#\n", rxpkt.chaninfo.txpower, rxpkt.chaninfo.rate, countRx++);

//                { // used for identifying the SRM's status
//                    rxWSMIdentity(&rxmsg,WSMMSG_SRM); //Identify the type of received Wave Short Message.
//                    if (!rxmsg.decode_status) {
//                        SignalRequestMsg_t *srmRcv = (SignalRequestMsg_t *)rxmsg.structure;
//                        printf("Received Signal Request Message, Mesage count %d\n\v", (int)srmRcv->msgCnt);
//                        //xml_print(rxmsg); /* call the parsing function to extract the contents of the received message */
//                    }
//                }

                { // used for identifying the SPAT's status
                    rxWSMIdentity(&rxmsg,WSMMSG_SPAT); //Identify the type of received Wave Short Message.
                    if (!rxmsg.decode_status) {
                        SPAT_t *spatRcv = (SPAT_t *)rxmsg.structure;
                        printf("Received Soignal Phase and Timing Message, Mesage ID %d\n\v", (int)spatRcv->msgID.buf[0]);
                        xml_print(rxmsg); /* call the parsing function to extract the contents of the received message */

                        processSPAT(spatRcv, &reqPhase_g);
                    }
                }
            }
//        SPAT_t *spatRcv; //for test
//        reqPhase_g = 5;
//        processSPAT(spatRcv, &reqPhase_g); // for test
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
    cleanMapMatch();
    closeSockets();
    closePreemption();
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

/* Transmitting : Function to build the Provider Service Table Entry */
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

/* Receiveing : Function to build the User Service Table Entry */
int buildUSTEntry(void)
{
    //Receiveing entry
    entryRx.psid = 11; //11
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

int buildSPATPacket()
{
    static int spatCount = 0;
    asn_enc_rval_t rvalenc;
    SPAT_t *spat;
    /* SRM related information */
    spat = (SPAT_t *) calloc(1,sizeof(SPAT_t));
    spat->msgID.buf = (uint8_t *) calloc(1, sizeof(uint8_t)); /* allocate memory for buffer which is used to store,
                                                              * what type of message it is
                                                              */
    spat->msgID.size = sizeof(uint8_t);
    spat->msgID.buf[0] = DSRCmsgID_signalPhaseAndTimingMessage;
    spatCount++;

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

 /* Main use of this function is we will encapsulate the SRM related info in to WSM request packet */
int buildSRMPacket(int intersectionID, int reqPhase)
{
    static int srmCount = 0;
    asn_enc_rval_t rvalenc;
//    SignalRequestMsg_t *srm;

    /* SRM related information */
    srm = (SignalRequestMsg_t *) calloc(1,sizeof(SignalRequestMsg_t));
    srm->msgID.buf = (uint8_t *) calloc(1, sizeof(uint8_t)); /* allocate memory for buffer which is used to store,
                                                              * what type of message it is
                                                              */
    /* DSRCmsgID_signalRequestMessage - SRM
     * These are the available/supported SAE message format's
     * For each type you need choose the appropriate structure
     */
    srm->msgID.size = sizeof(uint8_t);
   // *(srm->msgID.buf) = DSRCmsgID_signalReq uestMessage;
    srm->msgID.buf[0] = DSRCmsgID_signalRequestMessage; /* Choose what type of message you want to transfer */
    srm->msgCnt = srmCount++;

    // build SRM pachket for preemption control
    {
        // request element
        {
            // id
            srm->request.id.buf = (uint8_t *) calloc(1, sizeof(uint8_t));
            srm->request.id.size = sizeof(uint8_t);
            srm->request.id.buf[0] = intersectionID; // just for a demo, it is a ID
            // requestedAction
            srm->request.requestedAction = (SignalReqScheme_t *) calloc(1, sizeof(SignalReqScheme_t));
            srm->request.requestedAction->buf = (uint8_t *) calloc(1, sizeof(uint8_t));
            srm->request.requestedAction->size = sizeof(uint8_t);
            srm->request.requestedAction->buf[0] = reqPhase; // phase number

        }

        // endofService element
        {
            if (endOfServiceSecCount > 59)
            {
                endOfServiceSecCount = 1;
            }
            else
            {
                endOfServiceSecCount++;
            }
            // before we use it we directly allocate some memeory spaces
            srm->endOfService = (struct DTime *) calloc(1,sizeof(struct DTime));
//            srm->endOfService->second = endOfServiceSecCount; //way1, use '->' to point a member belongs to a struct pointer variable
            //srm->endOfService->second = 5; //way1, use '->' to point a member belongs to a struct pointer
            srm->endOfService->second = preemptionControlDurationTime;

////      srm->endOfService.second
//        struct DTime *endOfService_t; // way2, use use a pointer
//        endOfService_t = (struct DTime *) calloc(1,sizeof(struct DTime));
//        endOfService_t->second = 1;
//        srm->endOfService = endOfService_t;
          // way3, use use a struct variable
          // struct DTime endOfService_t;
          //endOfService_t.second = 1; // way2, use '.' to point a member belongs to a struct variable
//        srm->endOfService = &endOfService_t; //
        }

        // vehicleVin element
        {
            srm->vehicleVIN = (struct VehicleIdent*) calloc(1,sizeof(struct VehicleIdent));

            srm->vehicleVIN->id = (TemporaryID_t*) calloc(1,sizeof(TemporaryID_t));

            srm->vehicleVIN->id->buf = (uint8_t *) calloc(1, sizeof(uint8_t)); // do not miss any pointer without memery before using
            srm->vehicleVIN->id->size = sizeof(uint8_t);
            srm->vehicleVIN->id->buf[0] = 28; /* Choose what type of message you want to transfer */
        }

//        // vehicleData->bsm_blob mannually
        {
            //predefined variables
            int j;
            uint8_t temp_id = 29;
            uint16_t intg16, tmpintg16;
            uint32_t intg32;

            uint8_t stw = 0;
            int16_t lon_acc_accl = 0, yaw_rate, sintg16;

//            SignalRequestMsg_t  srm1;
//            srm1.msgCnt = 1;
//            SignalRequestMsg_t* srm2;
//            srm2 = (SignalRequestMsg_t*) malloc();
//            srm2->
//
            //
            srm->vehicleData.buf = (uint8_t *) calloc(1, 38*sizeof(uint8_t));
            srm->vehicleData.size = 38*sizeof(uint8_t);
            srm->vehicleData.buf[0] =srmCount%128;
            srmCount++;


            //intg32 = htobe32(*((uint32_t *)(temp_id)));
            intg32 = 12;
            memcpy(srm->vehicleData.buf+1,&intg32,4);
//            memcpy(&(srm->vehicleData.buf[1]),&intg32,4);
//
////            intg16=htobe16(sec_16);
            //intg16=htobe16(gpsData.actual_time); // original one
            intg16 = (uint16_t)(gpsData.actual_time);
            memcpy(srm->vehicleData.buf+5,&intg16,2);
//
////            j = latitude_val;
//            intg32=htobe32(gpsData.latitude); // original one
            intg32 = (uint32_t)(gpsData.latitude);
            memcpy(srm->vehicleData.buf+7,&intg32,4);
//
////            j=  longitude_val;
//            intg32=htobe32(gpsData.longitude);
            intg32 = (uint32_t)(gpsData.longitude);
            memcpy(srm->vehicleData.buf+11,&intg32,4);
//
////          elev Elevation, -x- 2 bytes
//            intg16=htobe16(gpsData.altitude);
            intg16=(uint16_t)(gpsData.altitude);
            memcpy(srm->vehicleData.buf+15,&intg16,2);
//
            j= 0xFFFFFFFF;// default value of positional_Accuracy
            memcpy(srm->vehicleData.buf+17,&j,4);
//
//            intg16=htobe16((uint16_t)(gpsData.speed));
            intg16=(uint16_t)(gpsData.speed);
            memcpy(srm->vehicleData.buf+21,&intg16,2);
//
//            intg16 = htobe16((uint16_t)(gpsData.course));
            intg16=(uint16_t)(gpsData.course);
            memcpy(srm->vehicleData.buf+23,&intg16,2);
//
            stw = 127;//default value for Steering Wheel Angle
            memcpy(srm->vehicleData.buf+25,&stw,1);
//
////            if(wsmgps.speed == GPS_INVALID_DATA)
////                lon_acc_accl = 2001;//default value for longitude accleration
            lon_acc_accl = 2001;
            sintg16 = lon_acc_accl;
//            sintg16 = htobe16(sintg16);
            sintg16 = (int16_t)(sintg16);
            memcpy(srm->vehicleData.buf+26,&sintg16,2);//longi acc
//
            tmpintg16 = 2001;//default value for latitude accleration
            sintg16 = (uint16_t)(tmpintg16);
//            intg16 = htobe16(tmpintg16);
            memcpy(srm->vehicleData.buf+28,&intg16,2);
//
            stw = -127;//default value for vertical accleration
            memcpy(srm->vehicleData.buf+30,&stw,1);
//
////            if(wsmgps.course == GPS_INVALID_DATA || heading_val == 28800)
////                yaw_rate = 32767;//default value for yaw accleration
            yaw_rate = 32767;//default value for yaw accleration
            sintg16 = yaw_rate;
           // sintg16 = htobe16(sintg16);
            sintg16 = (int16_t)(sintg16);
            memcpy(srm->vehicleData.buf+31,&sintg16,2);//yaw
//
//            tmpintg16 = 0x0800;//default value for brakeSystem Status
//            intg16 = htobe16(tmpintg16);
//            memcpy(srm->vehicleData.buf+33,&intg16,2);
//
////            memcpy(srm->vehicleData.buf+35,vsize,3);
        }

    }

    // encode part for encoding them to ASN.1 standard
    rvalenc = der_encode_to_buffer(&asn_DEF_SignalRequestMsg, srm, &wsmreqTx.data.contents, 1000); /* Encode your SRM in to WSM Packets */
    if (rvalenc.encoded == -1) {
        fprintf(stderr, "Cannot encode %s: %s\n",
                rvalenc.failed_type->name, strerror(errno));
    } else  {
        printf("Structure successfully encoded %d\n", rvalenc.encoded);
        wsmreqTx.data.length = rvalenc.encoded;
        asn_DEF_SignalRequestMsg.free_struct (&asn_DEF_SignalRequestMsg, srm, 0); // release the allocated memory
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
    printf("\nRegistering provider with PSID = %u\n ",entryTx.psid);
    if ( registerProvider( pid, &entryTx ) < 0 ){
        printf("Register Provider failed\n");
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

    printf("\nRegistering User with PSID = %d\n", entryRx.psid);
    int ret = registerUser(pid, &entryRx);
    if ( ret < 0)
    {
        printf("Register User Failed \n");
        printf("Removing user if already present  %d\n", !removeUser(pid, &entryRx));
        printf("USER Registered %d with PSID =%u \n", registerUser(pid, &entryRx), entryRx.psid );
    }
    else
        printf(" USER Registered %d with PSID =%u \n", ret, entryRx.psid );

}

int confirmBeforeJoin(WMEApplicationIndication *appind)
{
    printf("\nJoin\n");
    return 1; /* Return 0 for NOT Joining the WBSS */
}

int readConfig(void) //  used for reading things from files
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

/*        if (strcasecmp(str,"Latitude")==0)
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
        }*/
    }
    free(line);
    fclose(configFile);
    return 0;
}

int fullMapMatching (GPSData *gpsData, int * linkIDtmp, double *distanceToPoint, double *intersectionIDtmp )
{
    //printf("####21\n");
    if (( gpsData->actual_time ) > 1473708847) //5
    {
        *linkIDtmp = 0;
        *distanceToPoint = 0;
        *intersectionIDtmp = 0;
        return 0; // -1
    }

    return 1;
}

int updateGPSCourse(GPSData *gpsData)
{
    static double lastCourse = 0;
    if ((gpsData->speed < 0.2) & (gpsData->course == 0))
    {
        gpsData->course = lastCourse;
        return 1;
    }
    else
    {
        lastCourse = gpsData->course;
        return 0;
    }

}


int processSPAT(SPAT_t *spat, int *preemptPhase)
{
        int TotalIntersNum = 0;
        TotalIntersNum = spat->intersections.list.count;
        if(TotalIntersNum != NULL){
            //printf("NOT NULL\n");
            printf("Number the total intersections =  %d\n", TotalIntersNum);
            printf("We get total %d intersections\n",spat->intersections.list.count);
        }
        else {
            printf("NULL\n");
        }

        // process the info. for each intersection
        {
            int i=0,j=0;
            int intersize = 0, interID = 0;
            SignalLightState_t *statuscurrState;
            TimeMark_t statuscurrtimeToChange = 0;

            IntersectionState_t *intersectionstate;
            MovementState_t *movementstate;

            for(i=0;i<spat->intersections.list.count;i++)
            {
                printf("The intersections intery index is %d\n",i+1);
                intersectionstate = (IntersectionState_t *)spat->intersections.list.array[i];

                intersize = intersectionstate->id.size;
                interID = intersectionstate->id.buf[intersize-1];
                printf("We have got an information of intersection %d with ID = 0x%x\n",i+1,interID);
                //memcpy(&spatmsg.intsec_id,intersectionstate->id.buf, intersectionstate->id.size);
                //memcpy(&spatmsg.intsec_status,intersectionstate->status.buf, intersectionstate->status.size);

                if(intersectionstate->timeStamp != NULL){
                    //memcpy(&spatmsg.timestamp,intersectionstate->timeStamp,4);
                    //printf("We have got the timesatmp info.of intersection\n");
                }
                //spatmsg.ts_tenths = 0;
                //sspatmsg.movcount = intersectionstate->states.list.count;

                printf("We get total %d intersectionstate\n",intersectionstate->states.list.count);
                for(j=0;j<intersectionstate->states.list.count;j++)
                {
                    printf("The movementstate intery index is %d\n",j+1);
                    movementstate = (MovementState_t *)intersectionstate->states.list.array[j];

                    if(movementstate->currState != NULL){
                        //memcpy(&spatmsg.mptr[j].cur_state,movementstate->currState,4);
                        statuscurrState = (movementstate->currState) ;
                        printf("currState = %ld\n",*statuscurrState);
                    }
                    if(movementstate->timeToChange != NULL){
                        statuscurrtimeToChange = (uint16_t)movementstate->timeToChange ;
                        printf("timeToChange = %ld\n",statuscurrtimeToChange);
                    }
                    //memcpy(&spatmsg.mptr[j].mintimerem,&movementstate->timeToChange,4);
    //                spatmsg.mptr[j].mintimerem = (uint16_t)movementstate->timeToChange;
                }
            }


        }


        SmartphoneMsg.speed = gpsData.speed*3.6;
//        SmartphoneMsg.SenderID_1hopConnected = srm->request.id.buf[0]; //???
        SmartphoneMsg.SenderID_1hopConnected = 5; //???
        SmartphoneMsg.Distance_1hopConnected = dist2ApprInters;

        //SPAT
        SmartphoneMsg.PhaseStatus = 1;
        SmartphoneMsg.PhaseTiming = 12;
//        SmartphoneMsg.PhaseStatus = spat->intersections.array[0]->states.array[preemptPhase]->currState;  // just for tests, 0 has been defined all red.
//        SmartphoneMsg.PhaseTiming = spat->intersections.array[0]->states.array[preemptPhase]->timeToChange;

        //printf("We are inside the SPAT process\n");

        sprintf(TCPsmartPhoneMsg, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%d,%d,%d\n",
                SmartphoneMsg.speed,
        // SPaT
                SmartphoneMsg.PhaseStatus,
           (int)SmartphoneMsg.PhaseTiming,

       // Advisory Speed Warning
                SmartphoneMsg.ASW, // state of VSL: 0-no VSL, 1-Advisory Speed Limit ahead, 2-Driving on advisory speed segment and driving below VSL; 3-Driving on advisory speed segment and driving above VSL
                SmartphoneMsg.ASWdist11, // distance from vehicle location to advisory speed start point at t1 timestamp, in meters
                SmartphoneMsg.VSL, // VSL value in kph
        // Curve Speed Warning
                SmartphoneMsg.CSW , // state of CAS: 0-no CAS, 1-Curve Speed ahead, 2-Driving on curve speed segment and driving below CAS; 3-Driving on curve speed segment and driving above CAS
                SmartphoneMsg.CSWdist11, // distance from vehicle location to curve speed start point at t1 timestamp, in meters
                SmartphoneMsg.CAS, // curve speed value in kph
        // High Collision Location Warning
                SmartphoneMsg.HCW, // state of High collsion warning, 0-no HCL, 1-HCL ahead
                SmartphoneMsg.dist1, // distance from vehicle location to HCL location at t1 timestamp
        // Following-too-close Warning
                SmartphoneMsg.FTCW,
                SmartphoneMsg.TTC, // Time to collision, in seconds
        // Pedestrian Warning
                SmartphoneMsg.PedW,
                //dsrcm.PedWarning,// Pedestrian warning status, 0-no pedestrian warning, 1- pedestrian nearby with 10m<dist<20m, 2-pedestrian too close with dist<10m
        //once connected to a decvice, show the ID and distance
                SmartphoneMsg.SenderID_1hopConnected,
           (int)SmartphoneMsg.Distance_1hopConnected
                );

//        printf("1 TCP gene Message: %s",TCPsmartPhoneMsg);
        strncpy(TCPsmartPhoneMsg_gene,TCPsmartPhoneMsg,sizeof(TCPsmartPhoneMsg)); // copy string for future verifying
//        printf("2 TCPsmartPhoneMsg Message: %s",TCPsmartPhoneMsg_gene);
        memset(&SmartphoneMsg,0,sizeof(SmartphoneMsg)); //reset all varibales to zeros after sending

        return 0;
}
