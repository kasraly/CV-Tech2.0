#define MAP_CONFIG "/var/map_config.csv"
#define MAP_NODES "/var/map_nodes.csv"
#define MAP_LINKS "/var/map_links.csv"
// needing these three files otherwise there will be a bus error

#include "gpsc_probe.h"

int initMapMatch();
int cleanMapMatch();

int mapMatch(GPSData *, float *);

