#define PREEMPTION_ROUTE_TABLE "/var/preemptionRouteTable.txt"
#define preemptionRoute_Length_DEFINE 25

#include "PreemptionControlOBESide.h"
#include "gpsc_probe.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <stddef.h>
#include <math.h>

#define EARTH_RADIUS 6371000.0
#define pi 3.14159265358979323846
#define pidiv180 0.017453293
#define R1 40680631590769.0 //square of constant 6378137
#define R2 40408299984087.05552164//square of constant 6356752.3142
#define R2divR1 0.99330562

int preempDistance2IntersectionThreshold;

preemptionRouteColumn_t *preemptionRouteTable;
int preemptionRouteTableLength = 0;

//char broadcastSRM_enableFlag; // the enabling flag that shows whether to braodcast SRM or not
double dist2ApproachingIntersection; // disatnce between current position to approaching intersection
int preemptionControlDurationTime;

int closePreemption()
{
    free(preemptionRouteTable);
    return 0;
}
int initPreemption(int distThd)
{
    preempDistance2IntersectionThreshold = distThd;

    printf("We are going to parse the table defined the preemption route.\n");
    printf("Length = %d\n",preemptionRouteTableLength);

    // reading/loading table with predefined file
    int Row_index = 0;  // row index of that table
    FILE *preemptionRouteTableFile;

    if ((preemptionRouteTableFile = fopen(PREEMPTION_ROUTE_TABLE, "r")) == NULL)
    {
        printf("error openning %s for reading\n",PREEMPTION_ROUTE_TABLE);
        return 1;
    }
    else
    {
        printf("%s File is open successgful\n", PREEMPTION_ROUTE_TABLE);
        printf("begin to read the tool\n");
    }

    fscanf(preemptionRouteTableFile,"%*s %d\n", &preemptionRouteTableLength);
    printf("Table has %d rows\n", preemptionRouteTableLength);

    preemptionRouteTable = (preemptionRouteColumn_t *)calloc(preemptionRouteTableLength, sizeof(preemptionRouteColumn_t));

    for(Row_index = 0; Row_index < (preemptionRouteTableLength) ; Row_index++)
    {
        printf("Current row index is %d.\n", Row_index+1);

        fscanf(preemptionRouteTableFile,"%d,%d,%d,%lf,%lf,%d,%d\n",
               &(preemptionRouteTable[Row_index].coloumnOrder),
               &(preemptionRouteTable[Row_index].linkID),
               &(preemptionRouteTable[Row_index].intersectionID),
               &(preemptionRouteTable[Row_index].intersectionLatitude),
               &(preemptionRouteTable[Row_index].intersectionLongitude),
               &(preemptionRouteTable[Row_index].phaseNum),
               &(preemptionRouteTable[Row_index].distanceThreshold));

        printf("%d,%d,%d,%.6f,%.6f,%d,%d\n",
               preemptionRouteTable[Row_index].coloumnOrder,
               preemptionRouteTable[Row_index].linkID,
               preemptionRouteTable[Row_index].intersectionID,
               preemptionRouteTable[Row_index].intersectionLatitude,
               preemptionRouteTable[Row_index].intersectionLongitude,
               preemptionRouteTable[Row_index].phaseNum,
               preemptionRouteTable[Row_index].distanceThreshold);
    }

    fclose(preemptionRouteTableFile);
    return 0;
}

int preemptionStrategy(GPSData *gpsData, int linkID_g, int *intersectionID,int *reqPhase, double *dist2ApprInters)
{
    int Row_index;

    for(Row_index = 0; Row_index < preemptionRouteTableLength ; Row_index++)
    {
        printf("Searching Row%d......\n", Row_index+1);

        if (preemptionRouteTable[Row_index].linkID == linkID_g)
        {
            printf("Sucessfully finded a corresponding Row num = %d, column order %d!\n",
                   Row_index+1, preemptionRouteTable[Row_index].coloumnOrder);
            break;
        }
    }

    if (Row_index >= preemptionRouteTableLength) //if link ID not found in table exit
    {
        return 0; //return value 1 is to indicate no need for sending request
    }

    printf("InterInfo:%d,%d,%d,%.6f,%.6f,%d,%d\n",
            preemptionRouteTable[Row_index].coloumnOrder,
            preemptionRouteTable[Row_index].linkID,
            preemptionRouteTable[Row_index].intersectionID,
            preemptionRouteTable[Row_index].intersectionLongitude,
            preemptionRouteTable[Row_index].intersectionLatitude ,
            preemptionRouteTable[Row_index].phaseNum,
            preemptionRouteTable[Row_index].distanceThreshold);

    // algorithm 1 is implemennted
    // in the future the algorithm 2 will be implemented.
    // we have found one corresponsing intersection from the table

    dist2ApproachingIntersection = distance_calc(gpsData->latitude, gpsData->longitude,
                                   preemptionRouteTable[Row_index].intersectionLatitude,
                                   preemptionRouteTable[Row_index].intersectionLongitude,
                                   gpsData->altitude);

    printf("Distance = %f\n",dist2ApproachingIntersection);

    //dist2ApproachingIntersection = 200;

    if (dist2ApproachingIntersection < preempDistance2IntersectionThreshold)
    {

        *intersectionID = preemptionRouteTable[Row_index].intersectionID;
        *reqPhase = preemptionRouteTable[Row_index].phaseNum; // phase number

        printf("found an associated link and enable broadcast function\n");
        return 1; //broadcast is enabled
    }
    else
    {
        printf("found an associated link beyond the threshold, no need for signal broadcast\n");
        return 0;
    }
}

double distance_calc(double lat1, double lon1, double lat2, double lon2,double elev)
{
    double ang1,ang2,radpnt1,radpnt2,ethlat1,ethlon1,ethlat2,ethlon2,x,y,dist;

    ang1=(atan(R2divR1*tan(lat1*pidiv180)));
    ang2=(atan(R2divR1*tan(lat2*pidiv180)));
    radpnt1=(sqrt(1/(((cos(ang1))*(cos(ang1)))/R1+(((sin(ang1))*(sin(ang1)))/R2))))+elev;
    radpnt2=(sqrt(1/(((cos(ang2))*(cos(ang2)))/R1+(((sin(ang2))*(sin(ang2)))/R2))))+elev;
    ethlat1=radpnt1*cos(ang1);
    ethlat2=radpnt2*cos(ang2);
    ethlon1=radpnt1*sin(ang1);
    ethlon2=radpnt2*sin(ang2);
    x=sqrt((ethlat1-ethlat2)*(ethlat1-ethlat2)+(ethlon1-ethlon2)*(ethlon1-ethlon2));
    y=2*pi*((((ethlat1+ethlat2)/2))/360)*(lon1-lon2);
    dist=sqrt(x*x+y*y);
    return dist;
}
