#include "gpsc_probe.h"

#define GPS_RECORDING_BIN "/tmp/usb/gps_tmp.GPS"  //change this
#define GPS_RECORDING "/tmp/usb/gps_tmp.csv"  //change this
#define GPS_OFFLINE_BIN "/tmp/usb/gpsRecording.GPS"

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
