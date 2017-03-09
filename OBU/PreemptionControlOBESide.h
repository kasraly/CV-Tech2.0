// typedefine for preemption control

#include "gpsc_probe.h"

#define EARTH_RADIUS 6371000.0
#define pi 3.14159265358979323846
#define pidiv180 0.017453293
#define R1 40680631590769.0 //square of constant 6378137
#define R2 40408299984087.05552164//square of constant 6356752.3142
#define R2divR1 0.99330562

#define preemptionRouteTable "/var/preemptionRouteTable.txt"
#define preemptionRoute_Length_DEFINE 25
#define preemp_Distance2Intersection_Threshold 800
#define preemp_Durationtime_Threshold 5

// preemptionRoute table
typedef struct preemptionRouteColumn {
    int *coloumnOrder;
    int *linkID;
    int *intersectionID;
    double *intersectionLongitude;
    double *intersectionLatitude;
    int *phaseNum;
    double *distanceThreshold;
} preemptionRouteColumn_t;

int parsePreemptionRoute(int linkID_g );
int extractCorrespondingLinkInfo (int linkID_g);
int preemptionStrategy(GPSData *gpsData, int linkID_g);
double distance_calc(double lat1, double lon1, double lat2, double lon2,double elev);
//double distanceCalcSimple(double lat1, double lon1, double lat2, double lon2);
