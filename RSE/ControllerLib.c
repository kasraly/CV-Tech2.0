#include "ControllerLib.h"

#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/if_ether.h>

//#define CONTROLLER_RAW_SOCKET
#define CONTROLLER_SNMP

#define phaseControlGroupTable "1.3.6.1.4.1.1206.4.2.1.1.5.0"
#define phaseControlGroupNumber_1 "1.3.6.1.4.1.1206.4.2.1.1.5.1.1.1"
#define phaseControlGroupPhaseOmit_1 "1.3.6.1.4.1.1206.4.2.1.1.5.1.2.1"
#define phaseControlGroupPhaseOmit_2 "1.3.6.1.4.1.1206.4.2.1.1.5.1.2.2"
#define phaseControlGroupPedOmit_1 "1.3.6.1.4.1.1206.4.2.1.1.5.1.3.1"
#define phaseControlGroupPedOmit_2 "1.3.6.1.4.1.1206.4.2.1.1.5.1.3.2"
#define phaseControlGroupHold_1 "1.3.6.1.4.1.1206.4.2.1.1.5.1.4.1"
#define phaseControlGroupHold_2 "1.3.6.1.4.1.1206.4.2.1.1.5.1.4.2"
#define phaseControlGroupForceOff_1 "1.3.6.1.4.1.1206.4.2.1.1.5.1.5.1"
#define phaseControlGroupForceOff_2 "1.3.6.1.4.1.1206.4.2.1.1.5.1.5.2"
#define phaseStatusGroupPhaseOns_1 "1.3.6.1.4.1.1206.4.2.1.1.4.1.10.1"
#define phaseStatusGroupPhaseOns_2 "1.3.6.1.4.1.1206.4.2.1.1.4.1.10.2"
#define phaseStatusGroupPhaseNexts_1 "1.3.6.1.4.1.1206.4.2.1.1.4.1.11.1"
#define phaseStatusGroupPhaseNexts_2 "1.3.6.1.4.1.1206.4.2.1.1.4.1.11.2"

#define SPAT_PHASES 8


int phaseLength[SPAT_PHASES][3];
double previousTimeStamp[SPAT_PHASES];
int previousPhaseTiming[SPAT_PHASES];
int previousPhaseStatus[SPAT_PHASES];
float remainingPhaseTiming[SPAT_PHASES];
int allRedPhase[SPAT_PHASES];

unsigned char getPhase();
void holdPhase(unsigned char phase);
void forceOffPhase(unsigned char phase);
void omitPhase(unsigned char phase);

int controller_socket_fd;
struct snmp_session session, *ss;

int controllerActive = 1;

enum PreemptState {INACTIVE, MIN_GREEN, TRANSITION, ACTIVE, EXITING};
enum GetPhaseType {GET_PHASE_CURRENT, GET_PHASE_NEXT};

unsigned char getPhase(enum GetPhaseType phase)
{
    struct snmp_pdu *req, *resp;
    oid anOID[MAX_OID_LEN];
    size_t anOID_len = MAX_OID_LEN;

    //reading next phase
    req = snmp_pdu_create(SNMP_MSG_GET);

    if (phase == GET_PHASE_CURRENT)
        read_objid(phaseStatusGroupPhaseOns_1, anOID, &anOID_len);
    else
        read_objid(phaseStatusGroupPhaseNexts_1, anOID, &anOID_len);

    snmp_add_null_var(req, anOID, anOID_len);
    if (snmp_synch_response(ss, req, &resp) == STAT_TIMEOUT)
    {
        printf("SNMP timeout, controller not accessible\n");
        controllerActive = 0;
        if (resp)
        {
            snmp_free_pdu(resp);
        }
        return 0;
    }
    if (resp)
    {
        unsigned char nextPhase = resp->variables->val.integer[0];
        snmp_free_pdu(resp);
        return nextPhase;
    }
    else
    {
        printf("Error Reading the Green States\n");
        return 0;
    }
}

void holdPhase(unsigned char phase)
{

    struct snmp_pdu *req, *resp;
    oid anOID[MAX_OID_LEN];
    size_t anOID_len = MAX_OID_LEN;
    uint16_t val;
    int status;

    //Hold Phase
    val = phase;
    req = snmp_pdu_create(SNMP_MSG_SET);
    read_objid(phaseControlGroupHold_1, anOID, &anOID_len);
    snmp_pdu_add_variable(req, anOID, anOID_len, ASN_INTEGER, (const void *)&val, sizeof(val));
    status = snmp_synch_response(ss, req, &resp);
    if (status == STAT_TIMEOUT)
    {
        printf("SNMP timeout, controller not accessible\n");
        controllerActive = 0;
    }
    if (resp)
    {
        snmp_free_pdu(resp);
    }
}

void omitPhase(unsigned char phase)
{

    struct snmp_pdu *req, *resp;
    oid anOID[MAX_OID_LEN];
    size_t anOID_len = MAX_OID_LEN;
    uint16_t val;
    int status;

    //Omit Phase
    val = phase;
    req = snmp_pdu_create(SNMP_MSG_SET);
    read_objid(phaseControlGroupPhaseOmit_1, anOID, &anOID_len);
    snmp_pdu_add_variable(req, anOID, anOID_len, ASN_INTEGER, (const void *)&val, sizeof(val));
    status = snmp_synch_response(ss, req, &resp);
    if (status == STAT_TIMEOUT)
    {
        printf("SNMP timeout, controller not accessible\n");
        controllerActive = 0;
    }
    if (resp)
    {
        snmp_free_pdu(resp);
    }

    if (phase == 0)
        val = 0x00;
    else
        val = 0xff;
    req = snmp_pdu_create(SNMP_MSG_SET);
    read_objid(phaseControlGroupPhaseOmit_2, anOID, &anOID_len);
    snmp_pdu_add_variable(req, anOID, anOID_len, ASN_INTEGER, (const void *)&val, sizeof(val));
    status = snmp_synch_response(ss, req, &resp);
    if (status == STAT_TIMEOUT)
    {
        printf("SNMP timeout, controller not accessible\n");
        controllerActive = 0;
    }
    if (resp)
    {
        snmp_free_pdu(resp);
    }
}

void forceOffPhase(unsigned char phase)
{

    struct snmp_pdu *req, *resp;
    oid anOID[MAX_OID_LEN];
    size_t anOID_len = MAX_OID_LEN;
    uint16_t val;
    int status;

    //ForceOff Phase
    val = phase;
    req = snmp_pdu_create(SNMP_MSG_SET);
    read_objid(phaseControlGroupForceOff_1, anOID, &anOID_len);
    snmp_pdu_add_variable(req, anOID, anOID_len, ASN_INTEGER, (const void *)&val, sizeof(val));
    status = snmp_synch_response(ss, req, &resp);
    if (status == STAT_TIMEOUT)
    {
        printf("SNMP timeout, controller not accessible\n");
        controllerActive = 0;
    }
    if (resp)
    {
        snmp_free_pdu(resp);
    }

    if (phase == 0)
        val = 0x00;
    else
        val = 0xff;
    req = snmp_pdu_create(SNMP_MSG_SET);
    read_objid(phaseControlGroupForceOff_2, anOID, &anOID_len);
    snmp_pdu_add_variable(req, anOID, anOID_len, ASN_INTEGER, (const void *)&val, sizeof(val));
    status = snmp_synch_response(ss, req, &resp);
    if (status == STAT_TIMEOUT)
    {
        printf("SNMP timeout, controller not accessible\n");
        controllerActive = 0;
    }
    if (resp)
    {
        snmp_free_pdu(resp);
    }
}

int signalPreempt(unsigned char preemptPhase)
{
    static enum PreemptState preemptState = INACTIVE;
    static unsigned char nextPhase = 0;
    static unsigned char desiredPhase = 0;

    switch(preemptState){
        case INACTIVE:
            if (preemptPhase != 0)
            {
                desiredPhase = preemptPhase;
                nextPhase = getPhase(GET_PHASE_CURRENT);
                if (nextPhase != desiredPhase)
                {
                    omitPhase(~desiredPhase);
                    forceOffPhase(nextPhase);
                }
                preemptState = TRANSITION;
            }
            break;
        case MIN_GREEN:
            break;
        case TRANSITION:
            if (getPhase(GET_PHASE_CURRENT) == desiredPhase)
            {
                holdPhase(desiredPhase);
                omitPhase(0x00);
                preemptState = ACTIVE;
            }
            break;
        case ACTIVE:
            if (preemptPhase == 0)
            {
                if (nextPhase != desiredPhase)
                {
                    omitPhase(~nextPhase);
                }
                holdPhase(0x00);
                //forceOffPhase(desiredPhase);
                preemptState = EXITING;
            }
            break;
        case EXITING:
            if (getPhase(GET_PHASE_CURRENT) == nextPhase)
            {
                omitPhase(0x00);
                preemptState = INACTIVE;
            }
            break;
    }

    printf("\n Pre-emption: PreemptState %d, preemptPhase %x, desiredPhase %x, nextPhase %x\n",preemptState, preemptPhase, desiredPhase, nextPhase);

    if (preemptState == INACTIVE)
        return 0;
    else
        return 1;
}

int initController(char *controllerIP, uint16_t controllerSnmpPort)
{
    snmp_sess_init( &session );                   /* set up defaults */
    session.peername = malloc(24*sizeof(char));
    sprintf(session.peername, "%s:%d", controllerIP, controllerSnmpPort);

    /* set the SNMP version number */
    session.version = SNMP_VERSION_1;

    /* set the SNMPv1 community name used for authentication */
    session.community = "public";
    session.community_len = strlen(session.community);

    /*
     * Open the session
     */
    ss = snmp_open(&session);                     /* establish the session */

    if (!ss)
    {
        snmp_perror("ack");
        snmp_log(LOG_ERR, "something horrible happened!!!\n");
        exit(2);
    }

    // Activate the SPaT broadcast on the controller
    {
        // activate the controller SPaT function
        struct snmp_pdu *req, *resp;
        oid anOID[MAX_OID_LEN];
        size_t anOID_len = MAX_OID_LEN;
        uint16_t val = 6;
        int status;

        req = snmp_pdu_create(SNMP_MSG_SET);
        read_objid("1.3.6.1.4.1.1206.3.5.2.9.44.1.0", anOID, &anOID_len);
        snmp_pdu_add_variable(req, anOID, anOID_len, ASN_INTEGER, (const void *)&val, sizeof(val));
        status = snmp_synch_response(ss, req, &resp);
        if (status == STAT_TIMEOUT)
        {
            printf("SNMP timeout, controller not accessible\n");
            controllerActive = 0;
        }
        if (resp)
        {
            snmp_free_pdu(resp);
        }
    }// END - activate the controller SPaT function

    int i;
    for(i = 0; i<SPAT_PHASES; i++)
    {
        previousTimeStamp[i] = 0;
        previousPhaseTiming[i] = 0;
        previousPhaseStatus[i] = 0;
        phaseLength[i][0] = 0;
        phaseLength[i][1] = 0;
        phaseLength[i][2] = 0;

        allRedPhase[i] = 0;

    }

/*    phaseLength[1][0] = 40;
    phaseLength[1][1] = 4;
    phaseLength[1][2] = 110 - (phaseLength[1][0] + phaseLength[1][1]);
    phaseLength[2][0] = 40;
    phaseLength[2][1] = 7;
    phaseLength[1][2] = 110 - (phaseLength[2][0] + phaseLength[2][1]);
    phaseLength[3][0] = 47;
    phaseLength[3][1] = 4;
    phaseLength[1][2] = 110 - (phaseLength[3][0] + phaseLength[3][1]);
*/

    return 0;
}


int closeController()
{
#ifdef CONTROLLER_SNMP
    if (controllerActive == 1)
    {
        struct snmp_pdu *req, *resp;
        oid anOID[MAX_OID_LEN];
        size_t anOID_len = MAX_OID_LEN;
        uint16_t val = 0;

        req = snmp_pdu_create(SNMP_MSG_SET);
        read_objid("1.3.6.1.4.1.1206.3.5.2.9.44.1.0", anOID, &anOID_len);
        snmp_pdu_add_variable(req, anOID, anOID_len, ASN_INTEGER, (const void *)&val, sizeof(val));
        snmp_synch_response(ss, req, &resp);
        if (resp)
        {
            snmp_free_pdu(resp);
        }
    }
    snmp_close(ss);
#else
    close(controller_socket_fd);
#endif // CONTROLLER_SNMP
    return 0;
}

int readSPaT(SPAT_t* spat, double currentTime)
{
    int currentPhaseTiming[SPAT_PHASES];
    int currentPhaseStatus[SPAT_PHASES];

    int PhaseStatus[SPAT_PHASES];
    int PhaseTiming[SPAT_PHASES];

    struct snmp_pdu *req, *resp;
    oid anOID[MAX_OID_LEN];
    size_t anOID_len = MAX_OID_LEN;
    struct variable_list *vars;
    unsigned int greenState = 0;
    unsigned int yellowState = 0;
    unsigned int redState = 0;
    int i;

    //reading green status
    req = snmp_pdu_create(SNMP_MSG_GET);
    read_objid(".1.3.6.1.4.1.1206.4.2.1.1.4.1.4.1", anOID, &anOID_len);
    snmp_add_null_var(req, anOID, anOID_len);
    if (snmp_synch_response(ss, req, &resp) == STAT_TIMEOUT)
    {
        printf("SNMP timeout, controller not accessible\n");
        controllerActive = 0;
        if (resp)
        {
            snmp_free_pdu(resp);
        }
        return 0;
    }
    if (resp)
    {
        greenState = resp->variables->val.integer[0];
        snmp_free_pdu(resp);
    }
    else
    {
        printf("Error Reading the Green States\n");
        return 0;
    }
    //reading yellow status
    req = snmp_pdu_create(SNMP_MSG_GET);
    read_objid(".1.3.6.1.4.1.1206.4.2.1.1.4.1.3.1", anOID, &anOID_len);
    snmp_add_null_var(req, anOID, anOID_len);
    snmp_synch_response(ss, req, &resp);
    if (resp)
    {
        yellowState = resp->variables->val.integer[0];
        snmp_free_pdu(resp);
    }
    else
    {
        printf("Error Reading the Yello States\n");
        return 0;
    }
    //reading red status
    req = snmp_pdu_create(SNMP_MSG_GET);
    read_objid(".1.3.6.1.4.1.1206.4.2.1.1.4.1.2.1", anOID, &anOID_len);
    snmp_add_null_var(req, anOID, anOID_len);
    snmp_synch_response(ss, req, &resp);
    if (resp)
    {
        redState = resp->variables->val.integer[0];
        snmp_free_pdu(resp);
    }
    else
    {
        printf("Error Reading the Red States\n");
        return 0;
    }

    for(i=0; i<SPAT_PHASES; i++)
    {
        char oidStr[128];

        currentPhaseStatus[i] = ((greenState & (0x01<<i))>0)*1 + ((yellowState & (0x01<<i))>0)*2 + ((redState & (0x01<<i))>0)*3;

        //spatVehMinTimeToChange
        req = snmp_pdu_create(SNMP_MSG_GET);
        sprintf(oidStr,".1.3.6.1.4.1.1206.3.47.1.1.2.%d",i+1);
        read_objid(oidStr, anOID, &anOID_len);
        snmp_add_null_var(req, anOID, anOID_len);
        snmp_synch_response(ss, req, &resp);
        if(resp)
        {
            vars = resp->variables;
            //printf("extracted variable %d\n",vars->type);
            currentPhaseTiming[i] = vars->val.integer[0];
            //printf("read the time remaining: %f\n",time_left);
            snmp_free_pdu(resp);
            //printf("1.3.6.1.4.1.1206.3.47.1.1.2.%d spatVehMinTimeToChange for phase %d: %d\n",i+1,i+1,(int)(vars->val.integer[0]));
        }
        else
        {
            printf("Error Reading the reamining time for %d phase\n", i+1);
            return 0;
        }

        /*//spatVehMaxTimeToChange
        req = snmp_pdu_create(SNMP_MSG_GET);
        sprintf(oidStr,".1.3.6.1.4.1.1206.3.47.1.1.3.%d",i+1);
        read_objid(oidStr, anOID, &anOID_len);
        snmp_add_null_var(req, anOID, anOID_len);
        snmp_synch_response(ss, req, &resp);
        if(resp)
        {
            vars = resp->variables;
            //printf("extracted variable %d\n",vars->type);
            //printf("read the time remaining: %f\n",time_left);
            snmp_free_pdu(resp);
            printf("1.3.6.1.4.1.1206.3.47.1.1.3.%d spatVehMaxTimeToChange for phase %d: %d\n",i+1,i+1,(int)(vars->val.integer[0]));
        }
        else
        {
            printf("Error Reading the reamining time for %d phase\n", i+1);
            return 0;
        }

        //spatOvlpMinTimeToChange
        req = snmp_pdu_create(SNMP_MSG_GET);
        sprintf(oidStr,".1.3.6.1.4.1.1206.3.47.2.1.2.%d",i+1);
        read_objid(oidStr, anOID, &anOID_len);
        snmp_add_null_var(req, anOID, anOID_len);
        snmp_synch_response(ss, req, &resp);
        if(resp)
        {
            vars = resp->variables;
            //printf("extracted variable %d\n",vars->type);
            //printf("read the time remaining: %f\n",time_left);
            snmp_free_pdu(resp);
            printf("1.3.6.1.4.1.1206.3.47.2.1.2.%d spatOvlpMinTimeToChange for phase %d: %d\n",i+1,i+1,(int)(vars->val.integer[0]));
        }
        else
        {
            printf("Error Reading the reamining time for %d phase\n", i+1);
            return 0;
        }

        //spatOvlpMaxTimeToChange
        req = snmp_pdu_create(SNMP_MSG_GET);
        sprintf(oidStr,".1.3.6.1.4.1.1206.3.47.2.1.3.%d",i+1);
        read_objid(oidStr, anOID, &anOID_len);
        snmp_add_null_var(req, anOID, anOID_len);
        snmp_synch_response(ss, req, &resp);
        if(resp)
        {
            vars = resp->variables;
            //printf("extracted variable %d\n",vars->type);
            //printf("read the time remaining: %f\n",time_left);
            snmp_free_pdu(resp);
            printf("1.3.6.1.4.1.1206.3.47.2.1.3.%d spatOvlpMaxTimeToChange for phase %d: %d\n",i+1,i+1,(int)(vars->val.integer[0]));
        }
        else
        {
            printf("Error Reading the reamining time for %d phase\n", i+1);
            return 0;
        }*/
    }

    //processing SPaT
    {
        for(i=0; i<SPAT_PHASES; i++)
        {
            if (currentPhaseStatus[i] != previousPhaseStatus[i])
            {
                if (previousPhaseStatus[i] == 0)
                {
                    previousPhaseStatus[i] = currentPhaseStatus[i];
                }
                else
                {
                    // phase update based on timing
                    if (previousTimeStamp[i] != 0)
                    {
                        phaseLength[i][previousPhaseStatus[i]-1] = (int)round(currentTime - previousTimeStamp[i]);
                    }

                    if (previousPhaseStatus[i] == 2) //all red state
                    {
                        allRedPhase[i] = 1;
                    }

                    previousPhaseStatus[i] = currentPhaseStatus[i];
                    previousTimeStamp[i] = currentTime;
                    remainingPhaseTiming[i] = phaseLength[i][currentPhaseStatus[i]-1];
                }
            }
            else
            {
                remainingPhaseTiming[i] -= SPaT_READ_INTERVAL;
                remainingPhaseTiming[i] = remainingPhaseTiming[i]>0 ? remainingPhaseTiming[i] : 0;

                if (allRedPhase[i] == 1)
                {
                    if (currentPhaseTiming[i] > previousPhaseTiming[i])
                    {
                        allRedPhase[i] = 0;
                    }
                }
            }

            if (allRedPhase[i] == 1)
            {
                PhaseTiming[i] = remainingPhaseTiming[i];
            }
            else
            {
                if ((currentPhaseTiming[i] <= 30) & (currentPhaseTiming[i] != previousPhaseTiming[i]))
                {
                    PhaseTiming[i] = ceil(currentPhaseTiming[i]/10.0);
                }
                else
                {
                    if ((remainingPhaseTiming[i] < 3) & (remainingPhaseTiming[i]*10 < (currentPhaseTiming[i])))
                    {
                        PhaseTiming[i] = ceil(currentPhaseTiming[i]/10.0);
                    }
                    else
                    {
                        PhaseTiming[i] = remainingPhaseTiming[i];
                    }
                }

            }

            PhaseStatus[i] = currentPhaseStatus[i];
            previousPhaseTiming[i] = currentPhaseTiming[i];
//            printf("Current Time %.2f, Phase #%d, controller timing %d, PhaseStatus %d, remainingPhaseTiming %.1f, PhaseLength %d, allRed %d\n",
//                    currentTime, i+1, currentPhaseTiming[i], currentPhaseStatus[i], remainingPhaseTiming[i], phaseLength[i][currentPhaseStatus[i]-1], allRedPhase[i]);
        }

    }

    printf("SPaT: \n");
    for(i=0; i<SPAT_PHASES; i++)
    {
        printf("Phase %d,\tState %d,\tTime %d,\tController Time %d,\tremainingTime %d\n",
            i+1,
            PhaseStatus[i],
            PhaseTiming[i],
            currentPhaseTiming[i],
            (int)remainingPhaseTiming[i]);
    }


    return 1;
}

// old work trying based on RAW socket to cpature communication
/*int readSPaT(struct Message *dsrcmp)
{
    int retval = 0;
    struct timeval socketCheckTimout = {0,1};

    // check whether there is new packet recieved from controller
    fd_set controller_rfds;
    FD_ZERO(&controller_rfds);
    FD_SET(controller_socket_fd, &controller_rfds);
    retval = select(controller_socket_fd + 1, &controller_rfds, NULL, NULL, &socketCheckTimout);

    printf("controller_socket_fd %d, retval %d, FD_ISSET(master_socket, &readfds) %d\n", controller_socket_fd, retval, FD_ISSET(controller_socket_fd, &controller_rfds));
    if (retval >= 0) // if there is packet available from controller, read the socket and process the packet
    {
        unsigned char buffer[2048];
        int data_size;
        struct sockaddr controller_addr;
        socklen_t controller_addr_length = sizeof(controller_addr);

        printf("*************** Packet recieved ************\n");


        data_size = recvfrom(controller_socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)&controller_addr, &controller_addr_length);
        struct iphdr *iph = (struct iphdr*)buffer;
        if (iph->protocol == IPPROTO_UDP)
        {
            print_udp_packet(buffer , data_size);



            unsigned short iphdrlen;
            struct sockaddr_in source;

            struct iphdr* iph = (struct iphdr *)buffer;
            iphdrlen = iph->ihl*4;

            struct udphdr* udph = (struct udphdr*)(buffer + iphdrlen);

            memset(&source, 0, sizeof(source));
            source.sin_addr.s_addr = iph->saddr;



            if ((ntohs(udph->dest) == controllerBroadcastPort) && (iph->saddr == (u_int32_t)inet_addr(controllerIP)))
            //if (ntohs(udph->dest) == controllerBroadcastPort)
            {
                retval = 1;

                parseControllerSPaTBroadcast(buffer + iphdrlen + sizeof(struct udphdr), dsrcmp);
                printf("SPaT: \nState1 %d, Time1 %d\nState2 %d, Time2 %d\nState3 %d, Time3 %d\nState4 %d, Time4 %d\n",
                dsrcmp->PhaseStatus[0],dsrcmp->PhaseTiming[0],
                dsrcmp->PhaseStatus[1],dsrcmp->PhaseTiming[1],
                dsrcmp->PhaseStatus[2],dsrcmp->PhaseTiming[2],
                dsrcmp->PhaseStatus[3],dsrcmp->PhaseTiming[3]);
            }
        }
    }


    return retval;
}*/

/*
void parseControllerSPaTBroadcast(unsigned char *buffer, SPAT_t *spat)
{
    int i;
    uint16_t *greenState;
    uint16_t *yellowState;
    uint16_t *redState;

    greenState = (uint16_t *)&(buffer[214]);
    yellowState = (uint16_t *)&(buffer[212]);
    redState = (uint16_t *)&(buffer[210]);

    for(i=0; i<SPAT_PHASES; i++)
    {
        //dsrcmp->PhaseStatus[i] = ((*greenState & (0x01<<i))>0)*1 + ((*yellowState & (0x01<<i))>0)*2 + ((*redState & (0x01<<i))>0)*3;

        //dsrcmp->PhaseTiming[i] = (0x100*buffer[13*i+3] + buffer[13*i+4])/10.0;
        //dsrcmp->PhaseTiming[i] = *(uint16_t *)(&buffer[13*i+3])/10.0;
    }
    if (0)
    {
        printf("controller message:");
        for(i = 0; i<sizeof(*buffer); i++)
            printf(" %x",buffer[i]);
        printf("\n");
    }
}*/

void PrintData (unsigned char* data, int Size)
{
    int i, j;

    for(i=0 ; i < Size ; i++)
    {
        if( i!=0 && i%16==0) //if one line of hex printing is complete...
        {
            printf(" ");
            for(j=i-16 ; j<i ; j++)
            {
                if(data[j]>=32 && data[j]<=128)
                    printf("%c",(unsigned char)data[j]); //if its a number or alphabet

                else printf("."); //otherwise print a dot
            }
            printf("\n");
        }

        if (i%16==0)
            printf(" ");
        printf(" %02X",(unsigned int)data[i]);

        if( i==Size-1) //print the last spaces
        {
            for(j=0; j<15-i%16; j++) printf(" "); //extra spaces

            printf(" ");

            for(j=i-i%16 ; j<=i ; j++)
            {
                if(data[j]>=32 && data[j]<=128) printf("%c",(unsigned char)data[j]);
                else printf(".");
            }
            printf("\n");
        }
    }
}

void print_ip_header(unsigned char* Buffer, int Size)
{
//    unsigned short iphdrlen;
    struct sockaddr_in source,dest;

    struct iphdr *iph = (struct iphdr *)Buffer;
//    iphdrlen = iph->ihl*4;

    memset(&source, 0, sizeof(source));
    source.sin_addr.s_addr = iph->saddr;

    memset(&dest, 0, sizeof(dest));
    dest.sin_addr.s_addr = iph->daddr;

    printf("\n");
    printf("IP Header\n");
    printf("   |-IP Version    : %d\n",(unsigned int)iph->version);
    printf("   |-IP Header Length  : %d DWORDS or %d Bytes\n",(unsigned int)iph->ihl,((unsigned int)(iph->ihl))*4);
    printf("   |-Type Of Service   : %d\n",(unsigned int)iph->tos);
    printf("   |-IP Total Length   : %d  Bytes(Size of Packet)\n",ntohs(iph->tot_len));
    printf("   |-Identification    : %d\n",ntohs(iph->id));
//printf("   |-Reserved ZERO Field   : %d\n",(unsigned int)iphdr->ip_reserved_zero);
//printf("   |-Dont Fragment Field   : %d\n",(unsigned int)iphdr->ip_dont_fragment);
//printf("   |-More Fragment Field   : %d\n",(unsigned int)iphdr->ip_more_fragment);
    printf("   |-TTL  : %d\n",(unsigned int)iph->ttl);
    printf("   |-Protocol : %d\n",(unsigned int)iph->protocol);
    printf("   |-Checksum : %d\n",ntohs(iph->check));
    printf("   |-Source IP    : %s\n",inet_ntoa(source.sin_addr));
    printf("   |-Destination IP   : %s\n",inet_ntoa(dest.sin_addr));
}

void print_udp_packet(unsigned char *Buffer, int Size)
{

    unsigned short iphdrlen;

    struct iphdr *iph = (struct iphdr *)Buffer;
    iphdrlen = iph->ihl*4;

    struct udphdr *udph = (struct udphdr*)(Buffer + iphdrlen);

    printf("\n\n***********************UDP Packet*************************\n");

    print_ip_header(Buffer,Size);


    printf("\nUDP Header\n");
    printf(" |-Source Port : %d\n", ntohs(udph->source));
    printf(" |-Destination Port : %d\n", ntohs(udph->dest));
    printf(" |-UDP Length : %d\n", ntohs(udph->len));
    printf(" |-UDP Checksum : %d\n", ntohs(udph->check));

    printf("\n");
    printf("IP Header\n");
    PrintData(Buffer, iphdrlen);

    printf("UDP Header\n");
    PrintData(Buffer+iphdrlen, sizeof(struct udphdr));

    printf("Data Payload\n");

    PrintData(Buffer + iphdrlen + sizeof(struct udphdr),( Size - sizeof(struct udphdr) - iph->ihl * 4 ));

    printf("\n###########################################################");
}
