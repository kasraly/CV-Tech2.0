// Message definition structure

/*
version 1.0 :
2016-07-25

Functionality changes:
add a phase structure describing each phase;

Changes to variables(added, removed, renamed):
Added all basic variables for applications 1,2 and 4 in simple case
*/

/*
version 2.0 :
2016-07-26

Functionality changes:
Finishing application of 3,5 and 6.

Changes to variables(added, removed, renamed):
Added variables: int CAS, int HCW, int CSW
Changed description of double TimeStamp in struct DSRCM
*/

/*
version 3.0
2016-07-28

delete phase structure;
Add int phaseStatus[] and int PhaseTiming[] into struct DSRCM

*/


/*
vesion 4.0
2016-07-29

combine three sturcture into one and classify them with respect to basic message information and different application

*/


/*
version 4.1
2016-08-02

Added ending message required by jiangchen

*/

/*
version 4.2
2016-08-03

Added default cell length of double and float variables required by jiangchen

*/

/*
version 5.0-All
2016-08-07

Added infomation of OBE that sending message back to TMC

*/

/*
version 5.1
2016-08-08

Added defined phase number indicator; FTCWaring; DemoPhase;
Change variables type
Deleted DemoPhaseStatus and IntervalCouting

*/

/*
version 5.2
2016-08-13

Change potential value and descreption of SenderType and BackOBEType
Delete VehicleType

*/

/*
version 7.0
2016-09-01
Based on the version 5.2,
added some additional informaton from GPS for more feature rich Pedestrian and FTC application

*/

#define SPAT_PHASES   8

struct Message
{
    // basic information about sender and message
    int SenderID; //sender id
    unsigned char SenderType; // 0-RSE, 1-auto OBE, 2-Transit OBE, 3-Portable OBE
    double TimeStamp; // unix timestamp of sent message with respect to the number of seconds from Jan 1, 1970 00:00:00, defaulted to be 8 bytes
    double SenderLon; // longitude of sender, defaulted to be 8 bytes
    double SenderLat; // latitude of sender, defaulted to be 8 bytes

    // information for SPaT application
    unsigned char PhaseStatus[SPAT_PHASES]; // indicating active interval of current phase, 0-All red, 1-green, 2-yellow, 3-red; array number representing NEMA 8 phases number with northbound through to be phase 2
    unsigned char PhaseTiming[SPAT_PHASES]; // indicating the remaining time of current active interval (second),array number representing NEMA 8 phases number with northbound through to be phase 2
    unsigned char DemoPhase; // Indicator refering to phase number that used in demo

    // information for pedestrian warning application & following-too-close application
    double IntLon; // longitude of intersection(degree), defaulted to be 8 bytes
    double IntLat; // latitude of intersection(degree), defaulted to be 8 bytes
    unsigned char IntRange; // intersection range within which a show up of pedestrian would be considered as warning needed (meter), defaulted to be 4 bytes
    unsigned char PedWarning; // binary pedestrian warning, 0-no warning, 1-warning
    unsigned char FTCWarning; // binary following-too-close warning, 0-no warning, 1-warning

    // information for high collision locaiton warning application
    unsigned char HCW; // high collision location warning,  0-no warning, 1-warning

    // information for curve speed warning application
    unsigned char CSW; // curve speed warning,  0-no warning, 1-warning
    unsigned char CAS; // curve advisory speed (km/h)

    // information for advisory speed application
    unsigned char ASWarning; // binary advisory speed warning, 0-no warning, 1-warning
    unsigned char VSL; // advisory speed (km/h)

    // information for transmission from OBE to TMC
    unsigned char Back; // Indicating message status, 0-other message, 1-message sending back from OBE to RSE，2-message sending back from RSE to TMC
    unsigned char BackOBEid; // Device id of OBE that sending back location message to TMC
    unsigned char BackOBEType; // 1-auto OBE, 2-Transit OBE, 3-Portable OBE
    double BackOBETimeStamp; // unix timestamp of OBE location infomation, with respect to the number of seconds from Jan 1, 1970 00:00:00, defaulted to be 8 bytes
    double BackOBELon; // longitude of OBE that sending back message(degree), defaulted to be 8 bytes
    double BackOBELat; // latitude of OBE that sending back message(degree), defaulted to be 8 bytes

// 6.1 contents

    // information for SPaT application
    unsigned char SPaTexist; // indicating whether this RSE is broadcasting SPaT message, 0-no SPaT for this RSE，1-SPaT for this RSE

    // information for curve speed warning application
    unsigned char CSWexist; // indicating whether this RSE is broadcasting curve speed warning message, 0-no CSW for this RSE， 1-CSW for this RSE
    double CSWLon1; // Longitude of start point of curve speed segment
    double CSWLat1; // Latitude of start point of curve speed segment
    double CSWLon2; // Longitude of end point of curve speed segment
    double CSWLat2; // Latitude of end point of curve speed segment

    // information for advisory speed application
    unsigned char ASWexist; // indicating whether this RSE is broadcasting advisory speed warning message, 0-no ASW for this RSE， 1-ASW for this RSE
    double ASWLon1; // Longitude of start point of advisory speed segment
    double ASWLat1; // Latitude of start point of advisory speed segment
    double ASWLon2; // Longitude of end point of advisory speed segment
    double ASWLat2; // Latitude of end point of advisory speed segment

// 7.0 contents
    // Follow too close and Pedestrian Warning (added by Kasra: more info about GPS )
    double SenderAlt;
    double SenderSpeed;
    double SenderCourse;
    double gpsTime;

    // ending message
    unsigned char END1; // END1= A
    unsigned char END2; // END2= A
};


