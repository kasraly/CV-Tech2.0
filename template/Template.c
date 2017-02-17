#include "gpsc_probe.h"
#include "wave.h"

#define MIN_INTERVAL 0.1 //seconds
#define TIME_STEP 0.1 //seconds

GPSData gpsData;
int gpsSockFd;
char gpsAddr[] = "127.0.0.1";

static int pid;

int main()
{
    printf("Start \n");

    int rx_ret = 0;
    WSMIndication rxpkt;

    struct timeval currentTimeTV;
    double previousTime, currentTime;

    // Initializations:
    {
        pid = getpid();

        gpsSockFd = gpsc_connect(gpsAddr);
        printf("created GPS socket\n");

        gettimeofday(&currentTimeTV, NULL);
        currentTime = (double)currentTimeTV.tv_sec + (double)currentTimeTV.tv_usec/1000000;
        previousTime = floor(currentTime/TIME_STEP) * TIME_STEP;
    }


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
                counter == 0;
                // Do the control and broadcasting tasks.

                printf("\nReading GPS information....\n");

                char ch = '1';
                write(gpsSockFd,&ch,1);
                read(gpsSockFd,(void *)&gpsData,sizeof(gpsData));

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

        if(rx_ret > 0)
        {
            // process the message recieved
        }

        rx_ret = rxWSMPacket(pid, &rxpkt);
        sched_yield();
        usleep(1000);
    }
}
