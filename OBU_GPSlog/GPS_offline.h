#include "gpsc_probe.h"

#define GPS_RECORDING_BIN "/tmp/usb/GPSlog_23Ave.GPS"  //change this
#define GPS_RECORDING "/tmp/usb/GPSlog_23Ave.csv"  //change this
#define GPS_OFFLINE_BIN "/tmp/usb/GPSlog_23Ave.GPS"

struct GPSRecording {
    int GPSorRSSI;
    GPSData gpsData;
    double currentTime;
    int SenderID;
    double SenderTime;
    double SenderGPSTime;
    double SenderLat;
    double SenderLon;
    int rssi;
};

int read_GPS_log(GPSData*, double);
int logDatatoFile(struct GPSRecording *);
