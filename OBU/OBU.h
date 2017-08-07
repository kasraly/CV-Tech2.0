#ifndef _OBUH_
#define _OBUH_

#define IN_5TH_LAB 0// 1=in lab;0=out lab


// PC debug monitoring parameter
typedef struct Debug_Info_Ext {
    int link_ID_Debug;
    int Distance_2RSE_Debug;
    int SRMStatus_Debug;
    double SRM_delay;
    double SRM_distance_diff;
    double gps_course;

} Debug_Info_Ext_t;

void Debug_Info_Ext_Gene();

#endif // _OBUH_
