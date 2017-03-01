#define _GNU_SOURCE

#define TIME_STEP 0.1
#define MIN_INTERVAL 0.1

#define GPS_RECORDING_BIN "/tmp/usb/gps_tmp.GPS"  //change this
#define GPS_RECORDING "/tmp/usb/gps_tmp.csv"  //change this
#define GPS_OFFLINE_BIN "/tmp/usb/gpsRecording.GPS"
#define CONFIG_FILE "/var/GPSprocess_Config.txt"

#include "gpsc_probe.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include "wave.h"

GPSData gpsData;
static int pid;

int offline = 0;

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
int readConfig(void);

int main()
{
    printf("Start \n");

    readConfig();

    struct timeval currentTimeTV;
    double previousTime, currentTime;

    // Initializations:
    {
        pid = getpid();
    }

    struct GPSRecording gpsRec;
    memset(&gpsRec, 0, sizeof(struct GPSRecording));

    gettimeofday(&currentTimeTV, NULL);
    currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
    previousTime = floor(currentTime/TIME_STEP) * TIME_STEP;

    while (1) //infinite loop
    {
        static int counter = 0;
        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
        if ((currentTime - previousTime) >= MIN_INTERVAL) // check if enough time is passed to broadcast new message
        {
            previousTime = previousTime + MIN_INTERVAL;
            counter ++;
            if (counter >= (int)(TIME_STEP/MIN_INTERVAL))
            {
                counter = 0;

                read_GPS_log(&gpsData, currentTime);

                printf("RSE GPS Data\nTime: %.3f, GPSTime: %.1f, Lat: %.7f, Lon: %.7f\nAlt: %.1f, course: %.0f, speed, %.2f\n",
                    currentTime,
                    gpsData.actual_time,
                    gpsData.latitude,
                    gpsData.longitude,
                    gpsData.altitude,
                    gpsData.course,
                    gpsData.speed);

            }
        }

        sched_yield();
        usleep(1000);
    }

    return 0;
}

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

int readConfig(void)
{
    char *line = NULL;
    size_t len = 0;
    ssize_t read;
    FILE *configFile;

    if ((configFile = fopen(CONFIG_FILE, "r")) != NULL)
    {
        printf("file open successgful\n");
    }
    else
    {
        printf("error openning CONFIG_FILE for reading\n");
        return 1;
    }

    while ((read = getline(&line, &len, configFile)) != -1)
    {
        char *str;
        str = strtok (line," ,");

        if (strcasecmp(str,"Offline")==0)
        {
            str = strtok (NULL," ,");
            offline = atoi(str);
            if (offline)
                printf("GPS is in offline mode\n");
            else
                printf("GPS is online\n");
        }
    }
    free(line);
    fclose(configFile);
    return 0;
}
