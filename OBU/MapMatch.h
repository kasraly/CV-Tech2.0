#define MAP_CONFIG "/var/map_config.csv"
#define MAP_NODES "/var/map_nodes.csv"
#define MAP_LINKS "/var/map_links.csv"

#include "gpsc_probe.h"

int initMapMatch();
int cleanMapMatch();

int mapMatch(GPSData *, int *, float *);

