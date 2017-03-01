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
