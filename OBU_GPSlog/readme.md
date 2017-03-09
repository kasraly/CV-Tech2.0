# Using the Logged GPS data for offline use

1. Add the two files 'GPS_offline.c' and 'GPS_offline.h' to your project
2. call the function 'int read_GPS_log(GPSData*, double)' to recieve a gps samples. 
	- the first input is a pointer to GPSData structure where the function will return the GPS information. 
	- the second input should be the current system time in double. The fucntion will return a new GPS samples based on the elapsed time from previous function call. 
	- the function will start the from the begining if it reaches the end of logged GPS file. The function return value will be 0 on default. The return value will be 1 if the the fucntion starts from the beginning again. 
3. the target GPS log file can be changed inside the 'GPS_offline.h'. To do this change the 'GPS_OFFLINE_BIN' definition. 
	- make sure to copy the .GPS files from GPS_logs folder to the OBU and change the GPS_OFFLINE_BIN definition to the appropriate path 
