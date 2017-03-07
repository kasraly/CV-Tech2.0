#define _GNU_SOURCE

#include "MapMatch.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

struct mapNode{
    int id;
    double lat;
    double lon;
};

struct mapLink{
    int id;
    int startNodeIndex;
    int endNodeIndex;
    float heading;
};


struct mapNode *mapNodes;
struct mapLink *mapLinks;

int numNodes = 0;
int numLinks = 0;

double heading(struct mapNode* startNode, struct mapNode* endNode)
{
    return atan2(endNode->lon-startNode->lon,endNode->lat-startNode->lat) * 180 / M_PI;
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

        while (fscanf(nodeFile,"%d,%lf,%lf\n", &mapNodes[i].id, &mapNodes[i].lat, &mapNodes[i].lon) == 3)
        {
            printf("Node %d info: id %d, lat %.6f, lon %.6f\n", i, mapNodes[i].id, mapNodes[i].lat, mapNodes[i].lon);
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


int mapMatch(GPSData *gpsData, int *linkID, float *distFromStart)
{
    return 0;
}
