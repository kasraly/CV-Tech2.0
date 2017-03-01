#include "GPS_offline.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int read_GPS_log(GPSData* gpsDataP, double currentTime)
{
    int retval = 0;
    static long filePos = 0;
    static double timeOffset = 0;

    FILE *gpsReadFileBin;
    if ((gpsReadFileBin = fopen(GPS_OFFLINE_BIN,"r")) != NULL)
    {
        struct GPSRecording gpsRec;
        int readNextGPSLog = 1;
        fseek(gpsReadFileBin, filePos, SEEK_SET);
        while (readNextGPSLog)
        {
            if (fread((void *)&gpsRec, sizeof(struct GPSRecording), 1, gpsReadFileBin) < 1)
            {
                rewind(gpsReadFileBin);
                retval = 1;
            }
            if (gpsRec.GPSorRSSI == 1)
            {
                if (timeOffset == 0)
                {
                    timeOffset = currentTime - gpsRec.currentTime;
                }
                if ((gpsRec.currentTime + timeOffset) >= currentTime)
                {
                    readNextGPSLog = 0;
                }
            }
        }
        filePos = ftell(gpsReadFileBin);
        fclose(gpsReadFileBin);
        memcpy((void *)gpsDataP, (void *)&gpsRec.gpsData, sizeof(GPSData));

        printf("\nRead GPS binary file, currentTime: %f\n",gpsRec.currentTime);

    }
    else
    {
        printf("error openning %s for reading\n",GPS_OFFLINE_BIN);
    }

    return retval;

}

int logDatatoFile(struct GPSRecording *gpsRec)
{
    FILE *gpsRecordFileBin;

    if ((gpsRecordFileBin = fopen(GPS_RECORDING_BIN,"a")) != NULL)
    {
        if (fwrite(gpsRec, sizeof(struct GPSRecording), 1, gpsRecordFileBin) <= 0)
            printf("error appending data to %s\n",GPS_RECORDING_BIN);
        fclose(gpsRecordFileBin);
    }
    else
    {
        printf("error openning %s for appending\n",GPS_RECORDING_BIN);
    }

    FILE *gpsRecordFile;

    if ((gpsRecordFile = fopen(GPS_RECORDING,"a")) != NULL)
    {
        //OBU status
        fprintf(gpsRecordFile, "%d, %.3f,%.1f,%.8f,%.8f,%.1f,%.3f,%.3f",
            gpsRec->GPSorRSSI,
            gpsRec->currentTime,
            gpsRec->gpsData.actual_time,
            gpsRec->gpsData.latitude,
            gpsRec->gpsData.longitude,
            gpsRec->gpsData.altitude,
            gpsRec->gpsData.course,
            gpsRec->gpsData.speed);

        //recieved message
        fprintf(gpsRecordFile, ",%.3f,%.1f,%.8f,%.8f,%.1f,%d,%d,%d\n",
            gpsRec->SenderTime,
            gpsRec->SenderGPSTime,
            gpsRec->SenderLat,
            gpsRec->SenderLon,
            0.,
            0,
            gpsRec->SenderID,
            gpsRec->rssi);

        fclose(gpsRecordFile);
    }
    else
    {
        printf("error openning %s for appending\n",GPS_RECORDING);
    }

    return 1;
}
