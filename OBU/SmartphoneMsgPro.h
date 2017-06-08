// massage definition from OBE to Smartphone
// Ver 1.1,20160905 adding varaibales for ID and distance,
// Ver 1.2,20160906 adding additional varaibels for FTW+PedW,
// Ver 2.4, 20170313

struct PhoneMsg // used for struct definition in OBE
{
    int     speed;

    int     PhaseStatus;
    double  PhaseTiming;

    int     ASW;
    int     ASWdist11;
    int     VSL;

    int     CSW;
    int     CSWdist11;
    int     CAS;

    int     HCW;
    int     dist1;

    int     FTCW;
    double  TTC;

    // version 2.2
    double  headingV2V; //version2.2
    double  distV2V;    //version2.2
    double  headwayV2V; //version2.2

    int     PedW;
    double  headingV2P;//version2.2
    double  distV2P;   //version2.2
    double  timeToPed; //version2.2

    //version 2.3
    int     SenderID_1hopConnected; // for non-application area
    int     Distance_1hopConnected; // for non-applications area
};


struct PhoneMsgTCP // used for TCP sending's struct definition
{
    int     speed;

    int     PhaseStatus;
    double  PhaseTiming;

    int     ASW;
    int     ASWdist11;
    int     VSL;

    int     CSW;
    int     CSWdist11;
    int     CAS;

    int     HCW;
    int     dist1;

    int     FTCW;
    double  TTC;

    int     PedW;

    //version 2.3
    int     SenderID_1hopConnected; // for non-application area
    int     Distance_1hopConnected; // for non-applications area
};

// send to phone
//        dsrcm.speed, // current speed of vehicle, in km/h
//// SPaT
//        dsrcm.PhaseStatus[dsrcm.DemoPhase-1],  //status of the phase 0-all red, 1-green, 2-yellow, 3-red
//	(double)dsrcm.PhaseTiming[dsrcm.DemoPhase-1],  //remain time of the current phase in seconds
//// Advisory Speed Warning
//        dsrcm.ASW, // state of VSL: 0-no VSL, 1-Advisory Speed Limit ahead, 2-Driving on advisory speed segment and driving below VSL; 3-Driving on advisory speed segment and driving above VSL
//        dsrcm.ASWdist11, // distance from vehicle location to advisory speed start point at t1 timestamp, in meters
//        dsrcm.VSL, // VSL value in kph
//// Curve Speed Warning
//        dsrcm.CSW, // state of CAS: 0-no CAS, 1-Curve Speed ahead, 2-Driving on curve speed segment and driving below CAS; 3-Driving on curve speed segment and driving above CAS
//        dsrcm.CSWdist11, // distance from vehicle location to curve speed start point at t1 timestamp, in meters
//        dsrcm.CAS, // curve speed value in kph
//// High Collision Location Warning
//        dsrcm.HCW, // state of High collsion warning, 0-no HCL, 1-HCL ahead
//        dsrcm.dist1, // distance from vehicle location to HCL location at t1 timestamp
//// Following-too-close Warning
//    	dsrcm.FTCW, // follow to close status, 0-FTC is not active, 1-Following vehicle too close with 1s<TTC<2s, 2Following vehicle too close with TTC<1s
//        (float)dsrcm.TTC, // Time to collision, in seconds
//// Pedestrian Warning
//        dsrcm.PedW, // Pedestrian warning status, 0-no pedestrian warning, 1- pedestrian nearby with 10m<dist<20m, 2-pedestrian too close with dist<10m
//// once connected to a decvice, show the ID and distance
//        SmartphoneMsg.SenderID_1hopConnected,
//        (int)SmartphoneMsg.Distance_1hopConnected
