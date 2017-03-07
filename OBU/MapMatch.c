#define _GNU_SOURCE

#include "MapMatch.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define dot(u,v)    ((u).latDif * (v).latDif + (u).lonDif * (v).lonDif)
#define norm(v)     sqrt(dot(v,v))      // norm = length of  vector
//#define d(u,v)      norm(u-v)           // distance = norm of difference

#define EARTH_RADIUS 6371000.0

typedef struct point{
    double lat;
    double lon;
}Point;

typedef struct vector{
    double latDif;
    double lonDif;
}Vector;

struct mapNode{
    int id;
    Point P;
};

struct mapLink{
    int id;
    int startNodeIndex;
    int endNodeIndex;
    float heading;
};

struct mapNode *mapNodes;
struct mapLink *mapLinks;

int numNodes;
int numLinks;

double dist(Point*, Point*);
double dist_Point_to_Link(Point*, struct mapLink*, float*);

double heading(struct mapNode* startNode, struct mapNode* endNode)
{
    return atan2(endNode->P.lon-startNode->P.lon,endNode->P.lat-startNode->P.lat) * 180 / M_PI;
}

int initMapMatch()
{
    { //readign the config
        char *line = NULL;
        size_t len = 0;
        ssize_t read;
        FILE *configFile;

        if ((configFile = fopen(MAP_CONFIG, "r")) != NULL)
        {
            printf("%s file open successful\n", MAP_CONFIG);
        }
        else
        {
            printf("error openning %s for reading\n", MAP_CONFIG);
            return 1;
        }

        while ((read = getline(&line, &len, configFile)) != -1)
        {
            char *str;
            str = strtok (line," ,");

            if (strcasecmp(str,"Links")==0)
            {
                str = strtok (NULL," ,");
                numLinks = atoi(str);
                printf("Number of links in Map is %d\n",numLinks);
            }
            else if (strcasecmp(str,"Nodes")==0)
            {
                str = strtok (NULL," ,");
                numNodes = atoi(str);
                printf("Number of nodes in Map is %d\n",numNodes);
            }
        }
        free(line);
        fclose(configFile);
    }

    //allocate the memory for map data
    mapNodes = (struct mapNode *)calloc(numNodes, sizeof(struct mapNode));
    mapLinks = (struct mapLink *)calloc(numLinks, sizeof(struct mapLink));

    {// read the Node data
        FILE *nodeFile;
        if ((nodeFile = fopen(MAP_NODES, "r")) != NULL)
        {
            printf("%s file open successful\n", MAP_NODES);
        }
        else
        {
            printf("error openning %s for reading\n", MAP_NODES);
            return 1;
        }

        int i = 0;

        while (fscanf(nodeFile,"%d,%lf,%lf\n", &mapNodes[i].id, &mapNodes[i].P.lat, &mapNodes[i].P.lon) == 3)
        {
            printf("Node %d info: id %d, lat %.6f, lon %.6f\n", i, mapNodes[i].id, mapNodes[i].P.lat, mapNodes[i].P.lon);
            i++;
            if (i>=numNodes)
                break;
        }
        numNodes = i;
        printf("Total nodes: %d\n", numNodes);
        fclose(nodeFile);
    }

    {// read and process Link data
        FILE *linkFile;
        if ((linkFile = fopen(MAP_LINKS, "r")) != NULL)
        {
            printf("%s file open successful\n", MAP_LINKS);
        }
        else
        {
            printf("error openning %s for reading\n", MAP_LINKS);
            return 1;
        }

        int i = 0;
        int startNode, endNode;

        while (fscanf(linkFile,"%d,%d,%d\n", &mapLinks[i].id, &startNode, &endNode) == 3)
        {
            int j;
            mapLinks[i].startNodeIndex = -1;
            mapLinks[i].endNodeIndex = -1;
            for(j = 0; j < numNodes; j++)
            {
                if (mapNodes[j].id == startNode)
                {
                    mapLinks[i].startNodeIndex = j;
                }
                if (mapNodes[j].id == endNode)
                {
                    mapLinks[i].endNodeIndex = j;
                }
            }
            if (mapLinks[i].startNodeIndex == -1)
            {
                printf("Link %d, start node not found!\n", i);
                fclose(linkFile);
                return -1;
            }
            else if (mapLinks[i].endNodeIndex == -1)
            {
                printf("Link %d, end node not found!\n", i);
                fclose(linkFile);
                return -1;
            }
            else
            {
                mapLinks[i].heading = heading(&mapNodes[mapLinks[i].startNodeIndex], &mapNodes[mapLinks[i].endNodeIndex]);
                printf("Link %d info: id %d, startNode %d, endNode %d, heading %.0f\n",
                        i, mapLinks[i].id, mapNodes[mapLinks[i].startNodeIndex].id, mapNodes[mapLinks[i].endNodeIndex].id, mapLinks[i].heading);
            }

            i++;
            if (i>=numLinks)
                break;
        }
        numLinks = i;
        printf("Total links: %d", numLinks);
        fclose(linkFile);
    }
    return 0;
}

int cleanMapMatch()
{
    free(mapNodes);
    free(mapLinks);
    return 0;
}


int mapMatch(GPSData *gpsData, float *distFromStart)
{
    static int matchLinkIndex = -1;
    double distanceBest;
    int i;

    Point P;
    P.lat = gpsData->latitude;
    P.lon = gpsData->longitude;

    if (matchLinkIndex >= 0){
        distanceBest = dist_Point_to_Link(&P, &mapLinks[matchLinkIndex], distFromStart)
            - 0.0001; //about 10 meters [in degrees] for offset
    }
    else{
        distanceBest = 1000;
    }


    for(i=0; i<numLinks; i++)
    {
        if (fabs(mapLinks[i].heading - gpsData->course) < 45){
            double distance;
            float distFromStartTmp;

            distance = dist_Point_to_Link(&P, &mapLinks[i], &distFromStartTmp);
            if (distance < distanceBest){
                matchLinkIndex = i;
                distanceBest = distance;
                *distFromStart = distFromStartTmp;
            }

        }
    }
    return mapLinks[matchLinkIndex].id;
}

// dist_Point_to_Link(): get the distance of a point to a segment
//     Input:  a Point P and a Segment S (in any dimension)
//     Return: the shortest distance from P to S
double dist_Point_to_Link(Point* P, struct mapLink* S, float* distFromStart)
{
    Vector v;
    v.latDif = mapNodes[S->endNodeIndex].P.lat - mapNodes[S->startNodeIndex].P.lat;
    v.lonDif = mapNodes[S->endNodeIndex].P.lon - mapNodes[S->startNodeIndex].P.lon;
    Vector w;
    w.latDif = P->lat - mapNodes[S->startNodeIndex].P.lat;
    w.lonDif = P->lon - mapNodes[S->startNodeIndex].P.lon;

    double c1 = dot(w,v);
    if ( c1 <= 0 )
        return dist(P, &mapNodes[S->startNodeIndex].P);

    double c2 = dot(v,v);
    if ( c2 <= c1 )
        return dist(P, &mapNodes[S->endNodeIndex].P);

    double b = c1 / c2;
    Point Pb;
    Pb.lat = mapNodes[S->startNodeIndex].P.lat + b * v.latDif;
    Pb.lon = mapNodes[S->startNodeIndex].P.lat + b * v.lonDif;

    *distFromStart = dist(&mapNodes[S->startNodeIndex].P, &Pb)*M_PI/180.0*EARTH_RADIUS;
    return dist(P, &Pb);
}
//===================================================================

double dist(Point* u, Point* v)
{
    Vector w;
    w.latDif = u->lat - v->lat;
    w.lonDif = u->lon - v->lon;
    return norm(w);
}
