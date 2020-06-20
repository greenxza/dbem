#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include "helper.h"

#define SQLITE_DATE_FMT "%Y-%m-%d %H:%M:%S"

char *get_ict_time() {
	struct timeval tv_utc;
  struct tm *utc_tm, *ict_tm;

  time_t ict_t;
  /* ICT time is 7 hour ahead of UTC time */
  int utc_ict_delta = 7*3600;

  gettimeofday(&tv_utc, NULL);
  utc_tm = localtime(&tv_utc.tv_sec);

  ict_t = mktime(utc_tm) + utc_ict_delta;
  ict_tm = localtime(&ict_t);
	return asctime(ict_tm);
}

char *get_current_time(){
  time_t rawtime;
  struct tm * timeinfo;
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );

  //Change time format to SQLite's format
  char *buffer =  (char *) malloc(sizeof(char) * 26);
  strftime(buffer, 26, SQLITE_DATE_FMT, timeinfo);

  return buffer;
}

char *get_start_of_day(){

}

char *get_start_of_month(){
  time_t rawtimenow , rawtimestart;
  struct tm * now, * start;
  time ( &rawtimenow );
  now = localtime ( &rawtimenow );
  rawtimestart = mktime(now) - (now->tm_mday * 24 * 3600);
  start = localtime ( &rawtimestart );

  //Change time format to SQLite's format
  char *buffer =  (char *) malloc(sizeof(char) * 26);
  strftime(buffer, 26, SQLITE_DATE_FMT, start);

  return  buffer;
}

void get_time_of_day(int *hour, int *min, int *sec) {
  time_t curtime = time (NULL); // Getting current time of system
  struct tm *loctime = localtime (&curtime); // Converting current time to local time
  if(hour!=NULL) { // Hour
    *hour = loctime->tm_hour;
  }

  if(min!=NULL) { // Min
    *min = loctime->tm_min;
  }

  if(sec!=NULL) { // Sec
    *sec = loctime->tm_sec;
  }
}
