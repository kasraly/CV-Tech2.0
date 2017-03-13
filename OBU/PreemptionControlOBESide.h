// typedefine for preemption control

#include "gpsc_probe.h"

// preemptionRoute table
typedef struct preemptionRouteColumn {
    int coloumnOrder;
    int linkID;
    int intersectionID;
    double intersectionLatitude;
    double intersectionLongitude;
    int phaseNum;
    int distanceThreshold;
} preemptionRouteColumn_t;

int initPreemption(int);
int closePreemption();
int preemptionStrategy(GPSData *gpsData, int linkID_g,int *intersectionID, int *reqPhase, double *dist2ApprInters);
// perform preemption Strategy according to location and intersectrion info
double distance_calc(double lat1, double lon1, double lat2, double lon2, double elev);
//double distanceCalcSimple(double lat1, double lon1, double lat2, double lon2);
