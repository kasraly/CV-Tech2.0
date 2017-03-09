#include "PreemptionControlOBESide.h"
#include "gpsc_probe.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stddef.h"
#include <math.h>

extern preemptionRouteColumn_t preemptionRouteColumnVar;
extern int preemptionRoute_Length_g;
extern  *srm;

preemptionRouteColumn_t preemptionApproachingIntersectionInfo;
char broadcastSRM_enableFlag; // the enabling flag that shows whether to braodcast SRM or not
double dist2ApproachingIntersection; // disatnce between current position to approaching intersection
int preemptionControlDurationTime;

int preemptionStrategy(GPSData *gpsData, int linkID_g)
{
    int row_order = 0;
    broadcastSRM_enableFlag = 0;
    dist2ApproachingIntersection = 0;
    preemptionControlDurationTime = 0;


    row_order = parsePreemptionRoute(linkID_g);  // get information

    // algorithm 1 is implemennted
    if( row_order >= 1  )  // we have found one corresponsing intersection from the table
    {
        dist2ApproachingIntersection = distance_calc(gpsData->latitude, gpsData->longitude,
                                                     preemptionRouteColumnVar.intersectionLatitude [row_order-1],
                                                     preemptionRouteColumnVar.intersectionLongitude[row_order-1],
                                                     gpsData->altitude);
        printf("Distance = %f\n",dist2ApproachingIntersection);
//        dist2ApproachingIntersection = 900;
        if (dist2ApproachingIntersection < preemp_Distance2Intersection_Threshold)
        {
            preemptionControlDurationTime = preemp_Durationtime_Threshold;
            broadcastSRM_enableFlag = 1;
            printf("finded an associated link and enable broadcast function, broadcastSRM_enableFlag = %d\n",broadcastSRM_enableFlag);
        }
        else
        {
            broadcastSRM_enableFlag = 0;
            printf("finded an associated link beyond the threshold, broadcastSRM_enableFlag = %d\n",broadcastSRM_enableFlag);
        }
    }
    else
    {
        broadcastSRM_enableFlag = 0;
        printf("not finded an associated link, broadcastSRM_enableFlag = %d\n",broadcastSRM_enableFlag);
    }

    //printf("broadcastSRM_enableFlag = %d\n",broadcastSRM_enableFlag);

    return 0;
}

int extractCorrespondingLinkInfo (int linkID_g)
{
    int row_order = 0;

    row_order = parsePreemptionRoute(linkID_g);  // get information

    printf("A corresponding Row num = %d!\n",row_order);

    preemptionApproachingIntersectionInfo.coloumnOrder   = (int *) calloc(1, sizeof(int));
    preemptionApproachingIntersectionInfo.linkID         = (int *) calloc(1, sizeof(int));
    preemptionApproachingIntersectionInfo.intersectionID = (int *) calloc(1, sizeof(int));
    preemptionApproachingIntersectionInfo.intersectionLongitude = (double *) calloc(1, sizeof(double));
    preemptionApproachingIntersectionInfo.intersectionLatitude  = (double *) calloc(1, sizeof(double));
    preemptionApproachingIntersectionInfo.phaseNum              = (int *) calloc(1, sizeof(int));
    preemptionApproachingIntersectionInfo.distanceThreshold     = (double *) calloc(1, sizeof(double));

    if( row_order >=1  )  // we have found one corresponsing intersection from the table
    {
        preemptionApproachingIntersectionInfo.coloumnOrder[0]   = preemptionRouteColumnVar.coloumnOrder[row_order]-1;
        preemptionApproachingIntersectionInfo.linkID[0]          = preemptionRouteColumnVar.linkID[row_order-1];
        preemptionApproachingIntersectionInfo.intersectionID[0]  = preemptionRouteColumnVar.intersectionID[row_order-1];
        preemptionApproachingIntersectionInfo.intersectionLongitude[0]  = preemptionRouteColumnVar.intersectionLongitude[row_order-1];
        preemptionApproachingIntersectionInfo.intersectionLatitude[0]   = preemptionRouteColumnVar.intersectionLatitude[row_order-1];
        preemptionApproachingIntersectionInfo.phaseNum[0]              = preemptionRouteColumnVar.phaseNum[row_order-1];
        preemptionApproachingIntersectionInfo.distanceThreshold[0]      = preemptionRouteColumnVar.distanceThreshold[row_order-1];
    }
    else
    {
//        memset(&preemptionApproachingIntersectionInfo, 0 , sizeof(preemptionApproachingIntersectionInfo));
        preemptionApproachingIntersectionInfo.coloumnOrder[0]   = 0;
        preemptionApproachingIntersectionInfo.linkID[0]         = 0;
        preemptionApproachingIntersectionInfo.intersectionID[0] = 0;
        preemptionApproachingIntersectionInfo.intersectionLongitude[0] = 0;
        preemptionApproachingIntersectionInfo.intersectionLatitude[0]  = 0;
        preemptionApproachingIntersectionInfo.phaseNum[0]              = 0;
        preemptionApproachingIntersectionInfo.distanceThreshold[0]     = 0;

    }

//            printf("%d,%d,%d,%.6f,%.6f,%d,%.2f\n",
//                preemptionApproachingIntersectionInfo.coloumnOrder[0],
//                preemptionApproachingIntersectionInfo.linkID[0],
//                preemptionApproachingIntersectionInfo.intersectionID[0],
//                preemptionApproachingIntersectionInfo.intersectionLongitude[0],
//                preemptionApproachingIntersectionInfo.intersectionLatitude[0],
//                preemptionApproachingIntersectionInfo.phaseNum[0],
//                preemptionApproachingIntersectionInfo.distanceThreshold[0]
//              );
    return 0;
}

// loading and searching the table using link ID
int parsePreemptionRoute(int linkID_g)
{
    printf("We are going to parse the table defined the preemption route.\n");
    printf("Length = %d\n",preemptionRoute_Length_g);

    int corresponsdingOrderOflink = 0 ;

    // reading/loading table with predefined file
    int Row_index = 0;  // row index of that table
    FILE *preemptionRouteTableFile;

    if ((preemptionRouteTableFile = fopen(preemptionRouteTable, "r")) == NULL)
    {
        printf("error openning preemptionRouteTable for reading\n");
        return 1;
    }
    else
    {
        printf("PreemptionRouteTable File is open successgful\n");
        printf("begin to read the tool\n");
    }

    for(Row_index = 0; Row_index < (preemptionRoute_Length_g) ;Row_index++)
    {
        printf("Current row index is %d.\n", Row_index+1);

        fscanf(preemptionRouteTableFile,"%d,%d,%d,%lf,%lf,%d,%lf\n",
                &(preemptionRouteColumnVar.coloumnOrder[Row_index]),
                &(preemptionRouteColumnVar.linkID[Row_index]),
                &(preemptionRouteColumnVar.intersectionID[Row_index]),
                &(preemptionRouteColumnVar.intersectionLongitude[Row_index]),
                &(preemptionRouteColumnVar.intersectionLatitude[Row_index]),
                &(preemptionRouteColumnVar.phaseNum[Row_index]),
                &(preemptionRouteColumnVar.distanceThreshold[Row_index])
              );

        printf("%d,%d,%d,%.6f,%.6f,%d,%.2f\n",
                preemptionRouteColumnVar.coloumnOrder[Row_index],
                preemptionRouteColumnVar.linkID[Row_index],
                preemptionRouteColumnVar.intersectionID[Row_index],
                preemptionRouteColumnVar.intersectionLongitude[Row_index],
                preemptionRouteColumnVar.intersectionLatitude[Row_index],
                preemptionRouteColumnVar.phaseNum[Row_index],
                preemptionRouteColumnVar.distanceThreshold[Row_index]
              );
    }

    fclose(preemptionRouteTableFile);

    // exacting the intersection information from specific link ID

     for(Row_index = 0; Row_index < (preemptionRoute_Length_g) ;Row_index++)
     {
        printf("Searching Row%d......\n", Row_index+1);

        if (preemptionRouteColumnVar.linkID[Row_index] == linkID_g )
        {
            corresponsdingOrderOflink = preemptionRouteColumnVar.coloumnOrder[Row_index];
            printf("Sucessfully finded a corresponding Row num = %d!\n",corresponsdingOrderOflink);
            return corresponsdingOrderOflink;
        }
        else
        {
            if (Row_index == preemptionRoute_Length_g-1)
            {
                printf("do not find a corresponding link!\n");
                corresponsdingOrderOflink = -1;
                printf("failed to find out a corresponding Row, return %d!\n",corresponsdingOrderOflink);
                return corresponsdingOrderOflink;
            }
        }
     }

    return 0;
}


double distance_calc(double lat1, double lon1, double lat2, double lon2,double elev) {
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
