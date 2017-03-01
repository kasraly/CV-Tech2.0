#define _GNU_SOURCE

#define TIME_STEP 0.1
#define MIN_INTERVAL 0.1

#define CONFIG_FILE "/var/GPSprocess_Config.txt"

#include "gpsc_probe.h"
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include "wave.h"
#include "GPS_offline.h"

GPSData gpsData;
static int pid;

int offline = 0;

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

    {
        //GPS recording File
        FILE *gpsRecordFile;

        if ((gpsRecordFile = fopen(GPS_RECORDING,"r")) != NULL)
        {
            printf("the %s file already exist, renaming the file to",GPS_RECORDING);
            fclose(gpsRecordFile);
            char fileName[256];
            int i = 0;
            sprintf(fileName, "%s.old%d", GPS_RECORDING, i);
            while((gpsRecordFile = fopen(fileName,"r")) != NULL)
            {
                fclose(gpsRecordFile);
                sprintf(fileName, "%s.old%d", GPS_RECORDING, ++i);
            }
            rename(GPS_RECORDING,fileName);
            printf(" %s\n",fileName);
        }

        if ((gpsRecordFile = fopen(GPS_RECORDING,"w")) != NULL)
        {
            fprintf(gpsRecordFile, "GPSorRSSI, CurrentTime,GPSTime,latitude,longitude,altitude,course,speed");
            fprintf(gpsRecordFile, ",RcvdMsgTimeStamp,RcvdMsgGPSTime,RcvdMsgLat,RcvdMsgLon,RcvdMsgAlt,RcvdMsgType,RcvdMsgID,RcvdMsgRSSi\n");
            fclose(gpsRecordFile);
        }
        else
        {
            printf("error creating %s file\n",GPS_RECORDING);
        }

    //****************** bin file

        FILE *gpsRecordBinFile;

        if ((gpsRecordBinFile = fopen(GPS_RECORDING_BIN,"r")) != NULL)
        {
            printf("the %s file already exist, renaming the file to",GPS_RECORDING_BIN);
            fclose(gpsRecordBinFile);
            char fileName[256];
            int i = 0;
            sprintf(fileName, "%s.old%d", GPS_RECORDING_BIN, i);
            while((gpsRecordBinFile = fopen(fileName,"r")) != NULL)
            {
                fclose(gpsRecordBinFile);
                sprintf(fileName, "%s.old%d", GPS_RECORDING_BIN, ++i);
            }
            rename(GPS_RECORDING_BIN,fileName);
            printf(" %s\n",fileName);
        }

        if ((gpsRecordBinFile = fopen(GPS_RECORDING_BIN,"w")) != NULL)
        {
            fclose(gpsRecordBinFile);
        }
        else
        {
            printf("error creating %s file\n",GPS_RECORDING_BIN);
        }
    }

    struct GPSRecording gpsRec;
    memset(&gpsRec, 0, sizeof(struct GPSRecording));

    gpsRec.rssi = -1;
    gpsRec.SenderGPSTime = -1;
    gpsRec.SenderID = -1;
    gpsRec.SenderLat = -1;
    gpsRec.SenderLon = -1;
    gpsRec.SenderTime = -1;
    gpsRec.GPSorRSSI = 1;

    gettimeofday(&currentTimeTV, NULL);
    currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;

    int begin = 13000; //23 Ave
    int end = 14700;  //23 Ave
    begin = 7130; //Whole route
    end = 20300; //Whole route
    int i = 0;
    while(1)
    {
        i++;
        currentTime += 0.1;
        if (read_GPS_log(&gpsData, currentTime))
            return 0;
        memcpy(&gpsRec.gpsData, &gpsData, sizeof(gpsData));
        gpsRec.currentTime = currentTime;

        if (i >= begin)
        {
            logDatatoFile(&gpsRec);
            if (i > end)
                return 0;
        }
    }



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
