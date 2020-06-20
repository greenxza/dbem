#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sqlite3.h>
#include "db_manager.h"
#include "status_struct.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <time.h>

#define COUNT(x) (sizeof(x)/sizeof((x)[0]))

char *db_name = "dbem.db";

int init_tables();
// int insert_batt_status(const BatteryStruct **batt);
int insert_batts_status(const BatteryStruct **batts, const int count);
int update_prediction(StatusStruct **status);
int insert_batteries();
int insert_batt_types();
int insert_fault_and_conf();

static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
   	int i;
   	for(i = 0; i<argc; i++) {
      		printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   	}
   	return 0;
}

///Initializer database
int init_database(char *name) {
	db_name = name;
	init_tables();
	insert_batteries();
 	insert_batt_types();
	insert_fault_and_conf();
  return 0;
}

///Creat database and tables if not exists
int init_tables(){
   sqlite3 *db;
   char *zErrMsg = 0;
   int rc;
   char *sql;

	 //Open database
   rc = sqlite3_open(db_name, &db);

	 // Check if database is available
   if( rc ) {
      fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
      sqlite3_close(db); // Close db
      return 1;
   }

   /* Create tables statement */

   /* Create batt_type table */
   sql = "CREATE TABLE "  \
    "IF NOT EXISTS tblBattType (" \
    "id       INTEGER     PRIMARY KEY," \
    "name    TEXT     NOT NULL);" \

    /* Create battery table */
    "CREATE TABLE "  \
    "IF NOT EXISTS tblBattery (" \
    "id       INTEGER     PRIMARY KEY," \
    "name     TEXT    NOT NULL," \
    "e_charge INTEGER     NOT NULL," \
    "type     INTEGER    NOT NULL," \
    "chg_v_re INTEGER,"\
    "chg_v INTEGER," \
    "chg_i_min INTEGER," \
    "chg_i_max INTEGER," \
    "dischg_v_re INTEGER,"\
    "dischg_v INTEGER," \
    "dischg_i_min INTEGER," \
    "dischg_i_max INTEGER," \
    "FOREIGN KEY (type) REFERENCES tblBattType (id)); " \

    /* Create sys_status table */
    "CREATE TABLE "  \
    "IF NOT EXISTS tblSystemStatus (" \
    "id       INTEGER     PRIMARY KEY," \
    "date     TEXT    NOT NULL," \
    "delta_t  INTEGER DEFAULT 0 NOT NULL," \
    "temp1     REAL ," \
    "temp2     REAL ," \
    "temp3     REAL ," \
    "p_pv     INTEGER     NOT NULL," \
    "p_load   INTEGER     NOT NULL);" \

    /* Create batt_status table */
    "CREATE TABLE "  \
    "IF NOT EXISTS tblBattStatus (" \
    "id       INTEGER     PRIMARY KEY," \
    "in_en    INTEGER     NOT NULL," \
    "out_en   INTEGER     NOT NULL," \
    "i_in     REAL     NOT NULL," \
    "i_out    REAL     NOT NULL," \
    "v        REAL     NOT NULL," \
    "temp     REAL     NOT NULL," \
    "soc     REAL     NOT NULL," \
    "energy   INTEGER     NOT NULL," \
    "bank_id   INTEGER," \
    "sys_status_id   INTEGER," \
    "FOREIGN KEY (bank_id) REFERENCES tblBattery (id)" \
    "FOREIGN KEY (sys_status_id) REFERENCES tblSystemStatus (id));" \

    /* Create prediction table */
    "CREATE TABLE "  \
    "IF NOT EXISTS tblPrediction (" \
    "next_chrg       INTEGER    NOT NULL," \
    "next_dischrg    INTEGER    NOT NULL); " \

    /* Create protection limit table */
    "CREATE TABLE "  \
    "IF NOT EXISTS tblFault (" \
    "current    INTEGER    NOT NULL," \
    "temp    INTEGER    NOT NULL," \
    "over_voltage    INTEGER    NOT NULL," \
    "under_voltage   INTEGER    NOT NULL,"\
    "modbus    INTEGER    NOT NULL); " \

    /* Create current status table */
    "CREATE TABLE "  \
    "IF NOT EXISTS tblCurrentStatus (" \
    "v_bank1   REAL," \
    "v_bank2   REAL," \
    "v_bank3   REAL," \
    "i_in1   REAL," \
    "i_in2   REAL," \
    "i_in3   REAL," \
    "i_out1   REAL," \
    "i_out2   REAL," \
    "i_out3   REAL," \
    "in_en1   INTEGER," \
    "in_en2   INTEGER," \
    "in_en3   INTEGER," \
    "out_en1   INTEGER," \
    "out_en2   INTEGER," \
    "out_en3   INTEGER," \
    "temp1   REAL," \
    "temp2   REAL," \
    "temp3   REAL," \
    "soc1   REAL," \
    "soc2   REAL," \
    "soc3   REAL," \
    "heatsink1   REAL," \
    "heatsink2  REAL," \
    "inside_air   REAL," \
    "p_pv    INTEGER," \
    "p_load    INTEGER," \
    "date     TEXT    NOT NULL);" \

    /* Create custom SoC table */
    "CREATE TABLE "  \
    "IF NOT EXISTS tblCustomSetting (" \
    "date   TEXT NOT NULL," \
    "soc1   REAL NOT NULL," \
    "soc2   REAL NOT NULL," \
    "soc3   REAL NOT NULL);" \

    /* Create protection limit table */
    "CREATE TABLE "  \
    "IF NOT EXISTS tblConfig (" \
    "logfreq    INTEGER    NOT NULL );";

    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);

		// Check execution result
    if( rc != SQLITE_OK ){
      fprintf(stderr, "SQL error: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
    }

		//Close database
    sqlite3_close(db);

    return 0;
}

/// insert battery Bank 1,2,3 if not they are not exists.
int insert_batteries(){
  sqlite3 *db;
  char *err_msg = 0;
  int rc = sqlite3_open(db_name, &db);

	// Check if database is available
  if (rc != SQLITE_OK) {
      fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
      sqlite3_close(db); // Close db
      return 1;
  }

	//Prepare SQL command
  char *sql = "INSERT INTO tblBattery " \
              "SELECT 1, 'Battery Group 1', 480 , 1 , 48, 48, 51, 90, 48, 48, 0, 0 " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblBattery WHERE id = 1);" \

              "INSERT INTO tblBattery " \
              "SELECT 2, 'Battery Group 2', 360 , 2 , 46, 46, 21, 50, 46, 46, 0, 0 " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblBattery WHERE id = 2);" \

              "INSERT INTO tblBattery " \
              "SELECT 3, 'Battery Group 3', 240 , 1 , 44, 44, 1, 20, 44, 44, 0, 0 " \

              "WHERE NOT EXISTS(SELECT 1 FROM tblBattery WHERE id = 3);";

	// Execute SQL command
  rc = sqlite3_exec(db, sql, 0, 0, &err_msg);

	//Check execution result
  if (rc != SQLITE_OK ) {
      fprintf(stderr, "SQL error: %s\n", err_msg);
      sqlite3_free(err_msg);
      sqlite3_close(db);
      return 1;
  }

	//close database
  sqlite3_close(db);

  return 0;
}

//Insert limit of fault state
int insert_fault_and_conf(){
  sqlite3 *db;
  char *err_msg = 0;
  int rc = sqlite3_open(db_name, &db);

	// Check if database is available
  if (rc != SQLITE_OK) {
      fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
      sqlite3_close(db); // Close db
      return 1;
  }

  char *sql = "INSERT INTO tblFault " \
              "SELECT 0, 0, 0, 0, 0 " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblFault); " \

              "INSERT INTO tblConfig " \
              "SELECT 1 " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblConfig);";

  rc = sqlite3_exec(db, sql, 0, 0, &err_msg);

  if (rc != SQLITE_OK ) {
      fprintf(stderr, "SQL error: %s\n", err_msg);
      sqlite3_free(err_msg);
      sqlite3_close(db);
      return 1;
  }
	//Close database
  sqlite3_close(db);

  return 0;
}

static int retrieve_i_log_callback(void *data, int argc, char **argv, char **azColName) {
  // printf("Limit datasss: %f\n", *((float*)data));
  float test = atof(argv[0]);
  float *conf = (float*)data;
  *conf = test;
  // printf("Limit Log: %f, %f\n", test, *conf);
  return 0;
}

int get_limit_log(float **samplingRate){
  sqlite3 *db;
  char *zErrMsg = 0;
  int rc;
  char *sql;
  const char* data = "Callback function called";
  //Open database
  rc = sqlite3_open(db_name, &db);

// Check if database is available
  if( rc ) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db); // Close db
    return(0);
  } else {
    fprintf(stderr, "Opened database successfully\n");
  }

  /* Create SQL statement */
  sql = "SELECT logfreq FROM tblConfig;";

  /* Execute SQL statement */
  rc = sqlite3_exec(db, sql, retrieve_i_log_callback, *samplingRate, &zErrMsg);
  // printf("Sampling after querysss: %p, %p, %f\n", samplingRate, (*samplingRate), (**samplingRate));
  //Check execution result
  if( rc != SQLITE_OK ) {
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
  } else {
        fprintf(stdout, "Operation done successfully\n");
  }
  sqlite3_close(db);
	return 0;
}

//Insert battery type
int insert_batt_types(){
  sqlite3 *db;
  char *err_msg = 0;

	//Open database
	int rc = sqlite3_open(db_name, &db);

	// Check if database is available
  if (rc != SQLITE_OK) {
      fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
      sqlite3_close(db); // Close db
      return 1;
  }

  char *sql = "INSERT INTO tblBattType " \
              "SELECT 0, 'Flooded' " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblBattType WHERE id = 0);" \

              "INSERT INTO tblBattType " \
              "SELECT 1, 'Gel' " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblBattType WHERE id = 1);" \

              "INSERT INTO tblBattType " \
              "SELECT 2, 'AGM' " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblBattType WHERE id = 2);" \

              "INSERT INTO tblBattType " \
              "SELECT 3, 'Custom' " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblBattType WHERE id = 3);" \

              "INSERT INTO tblBattType " \
              "SELECT 4, 'Lithium' " \
              "WHERE NOT EXISTS(SELECT 1 FROM tblBattType WHERE id = 4);" ;

  rc = sqlite3_exec(db, sql, 0, 0, &err_msg);

	//Check execution result
  if (rc != SQLITE_OK ) {
      fprintf(stderr, "SQL error: %s\n", err_msg);
      sqlite3_free(err_msg);
      sqlite3_close(db);

      return 1;
  }
	//Close database
  sqlite3_close(db);

  return 0;
}

float getBattIin(const BatteryStruct *batt)
{
  return (batt->in_en)?batt->comb_i_in:0.0;
}

float getBattIout(const BatteryStruct *batt)
{
  return (batt->out_en)?batt->comb_i_out:0.0;
}

int saveStatusToDB(StatusStruct **status_addr){
    sqlite3 *db;
  	char *err_msg = 0;
  	int rc = sqlite3_open(db_name, &db);

    // Check if database is available
  	if (rc != SQLITE_OK) {
      		fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
          sqlite3_close(db); // Close db
      		return 1;
  	}

    const time_t curr_time = time(NULL);
    time_t delta_time = curr_time-((*status_addr)->prev_log_time);
    if(delta_time<0 || delta_time>3600) { // Range 0-3600 (avoid missing datetime setting in beaglebone)
      delta_time = 0;
    }
    (*status_addr)->prev_log_time = curr_time;

  	//use sqlite3_mprint instead of sprintf
  	char* sqlSysStatus = sqlite3_mprintf("INSERT INTO tblSystemStatus(date, delta_t, p_load,p_pv,temp1,temp2,temp3) VALUES(datetime('now', 'localtime'), %ld, %d, %d,%.2f,%.2f,%.2f); ",
              delta_time,
              (*status_addr)->p_load,
              (*status_addr)->p_pv,
              (*status_addr)->temp1,
              (*status_addr)->temp2,
              (*status_addr)->temp3);

  	rc = sqlite3_exec(db, sqlSysStatus, 0, 0, &err_msg);
    sqlite3_free(sqlSysStatus); // Free memory
  	if (rc != SQLITE_OK ) {
      		fprintf(stderr, "SQL error: saveStatusToDB : %s\n", err_msg);
      		sqlite3_free(err_msg);
      		sqlite3_close(db);
      		return 1;
  	}
		//Close database
  	sqlite3_close(db);

  	update_prediction(status_addr);

    const BatteryStruct *batts[] = {(*status_addr)->b1, (*status_addr)->b2, (*status_addr)->b3};
    insert_batts_status(batts, sizeof(batts)/sizeof(batts[0]));
    // sleep(1);
    // insert_batt_status(&((*status_addr)->b1));
    // sleep(1);
    // insert_batt_status(&((*status_addr)->b2));
    // sleep(1);
    // insert_batt_status(&((*status_addr)->b3));

  	return 0;
}

/// Save a status(V,I,temp...) of battery which read by sensor
int insert_batts_status(const BatteryStruct **batts, const int count)
{
  sqlite3 *db;
  int rc = sqlite3_open(db_name, &db);
  if (rc != SQLITE_OK) { // Check if database is available
      fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
      sqlite3_close(db); // Close db
      return 1;
  }

  float i_in, i_out;
  char sqlSysStatus[1500];
  for(int i=0; i<count; i++)
  {
    char tmp[500];
    const BatteryStruct *batt = batts[i];

    i_in = getBattIin(batt);
    i_out = getBattIout(batt);
    // i_in = (batt)->i_in;
    // i_out = (batt)->i_out;

    sprintf(tmp,
            "INSERT INTO  tblBattStatus(in_en, out_en, i_in, i_out, v, temp, soc, energy, bank_id, sys_status_id) " \
            " VALUES(%d, %d, %f, %f, %f, %f, %f, %d, %d, " \
            " (SELECT id FROM tblSystemStatus where id = (select max(id) from tblSystemStatus)) ); ",
            (batt)->in_en,
            (batt)->out_en,
            i_in,
            i_out,
            (batt)->v,
            (batt)->temp,
            (batt)->soc,
            (batt)->energy,
            (batt)->bank_id
          );
    // printf("len: %d\n", strlen(tmp));
    // printf("Tmp: %s\n", tmp);

    if(i==0) {
        strcpy(sqlSysStatus, tmp);
    }else{
        strcat(sqlSysStatus, tmp);
    }
  }

  // printf("Sql: %s\n", sqlSysStatus);

  for(int i = 0; i < 5; i++) {
    char *err_msg = 0;
    rc = sqlite3_exec(db, sqlSysStatus, 0, 0, &err_msg);
    if(rc == SQLITE_OK) {
      break;
    }

    fprintf(stderr, "[Retry: %d] Inserting battery status error: %s\n", i, err_msg);
    sqlite3_free(err_msg);
    sleep(1);
  }

  //Check execution result
  if (rc != SQLITE_OK ) {
    sqlite3_close(db);
    return 1;
  }
  sqlite3_close(db); //Close database
  return 0;
}

/// Save a status(V,I,temp...) of battery which read by sensor
// int insert_batt_status(const BatteryStruct **batt){

//   char *err_msg = 0;
//   int rc = sqlite3_open(db_name, &db);

//   // Check if database is available
//   if (rc != SQLITE_OK) {
//       fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
//       sqlite3_close(db); // Close db
//       return 1;
//   }

//   char sqlSysStatus[2000];
//   sprintf(sqlSysStatus,
//               "INSERT INTO  tblBattStatus(in_en, out_en, i_in, i_out, v, temp, soc, energy, bank_id, sys_status_id) " \
//               " VALUES(%d, %d, %f, %f, %f, %f, %f, %d, %d, " \
//               " (SELECT id FROM tblSystemStatus where id = (select max(id) from tblSystemStatus)) ); ",
//               (*batt)->in_en,
//               (*batt)->out_en,
//               (*batt)->i_in,
//               (*batt)->i_out,
//               (*batt)->v,
//               (*batt)->temp,
//               (*batt)->soc,
//               (*batt)->energy,
//               (*batt)->bank_id
//             );
//   for(int i = 0; i < 5; i++) {
//     rc = sqlite3_exec(db, sqlSysStatus, 0, 0, &err_msg);
//     if(rc == SQLITE_OK) {
//       break;
//     }

//     fprintf(stderr, "Inserting battery status error: %s\n", err_msg);
//     sleep(1);
//   }

//   //Check execution result
//   if (rc != SQLITE_OK ) {
//     fprintf(stderr, "Inserting battery status error: %s\n", err_msg);
//     sqlite3_free(err_msg);
//     sqlite3_close(db);

//     return 1;
//   }
// 	//Close database
//   sqlite3_close(db);

//   return 0;
// }

///Update current status
int updateMonitoringStatus(StatusStruct **status_addr){
    sqlite3 *db;
		char *err_msg = 0;
		//Open database
  	int rc = sqlite3_open(db_name, &db);

		// Check if database is available
  	if (rc != SQLITE_OK) {
      		fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
          sqlite3_close(db); // Close db
      		return 1;}

    float i_in1 = getBattIin((*status_addr)->b1);
    float i_in2 = getBattIin((*status_addr)->b2);
    float i_in3 = getBattIin((*status_addr)->b3);
    float i_out1 = getBattIout((*status_addr)->b1);
    float i_out2 = getBattIout((*status_addr)->b2);
    float i_out3 = getBattIout((*status_addr)->b3);

  	char sqlSysStatus[1000];
  	sprintf(sqlSysStatus,
        "DELETE FROM tblCurrentStatus; " \
        "INSERT INTO tblCurrentStatus " \
        "(v_bank1," \
        "v_bank2," \
        "v_bank3," \
        "i_in1," \
        "i_in2," \
        "i_in3," \
        "i_out1," \
        "i_out2," \
        "i_out3," \
        "in_en1," \
        "in_en2," \
        "in_en3," \
        "out_en1," \
        "out_en2," \
        "out_en3," \
        "temp1," \
        "temp2," \
        "temp3," \
        "soc1," \
        "soc2," \
        "soc3," \
        "heatsink1," \
        "heatsink2," \
        "inside_air," \
        "p_pv," \
        "p_load," \
        "date ) " \
	      "VALUES ( %.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f," \
        "%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2d,%.2d,datetime('now', 'localtime')); ",
              (*status_addr)->b1->v,
              (*status_addr)->b2->v,
              (*status_addr)->b3->v,
              i_in1,
              i_in2,
              i_in3,
              i_out1,
              i_out2,
              i_out3,
              // (*status_addr)->b1->i_in,
              // (*status_addr)->b2->i_in,
              // (*status_addr)->b3->i_in,
              // (*status_addr)->b1->i_out,
              // (*status_addr)->b2->i_out,
              // (*status_addr)->b3->i_out,
              (*status_addr)->b1->in_en,
              (*status_addr)->b2->in_en,
              (*status_addr)->b3->in_en,
              (*status_addr)->b1->out_en,
              (*status_addr)->b2->out_en,
              (*status_addr)->b3->out_en,
              (*status_addr)->b1->temp,
              (*status_addr)->b2->temp,
              (*status_addr)->b3->temp,
              (*status_addr)->b1->soc,
              (*status_addr)->b2->soc,
              (*status_addr)->b3->soc,
              (*status_addr)->temp1,
              (*status_addr)->temp2,
              (*status_addr)->temp3,
              (*status_addr)->p_pv,
              (*status_addr)->p_load
            );

  	rc = sqlite3_exec(db, sqlSysStatus, 0, 0, &err_msg);

		//Check execution result
  	if (rc != SQLITE_OK ) {
      		fprintf(stderr, "updateMonitoringStatus error: %s\n", err_msg);
      		sqlite3_free(err_msg);
      		sqlite3_close(db);

      		return 1;
  	}
  	sqlite3_close(db);
  	return 0;
}

int retrieve_socs(SOCsStruct *socs, const TblName_t tbl)
{
  if(!socs) {
    fprintf(stderr, "Invalid input socs\n");
    return 1;
  }

  char tblName[32];
  switch(tbl)
  {
    case TBL_CURRENT_STATUS:
    {
      sprintf(tblName, "tblCurrentStatus");
      break;
    }
    case TBL_CUSTOM_SETTING:
    {
      sprintf(tblName, "tblCustomSetting");
      break;
    }
    default:
    {
      fprintf(stderr, "Invali table type: %d\n", tbl);
      return 1;
    }
  }

  //Open database
  sqlite3 *db;
  int rc = sqlite3_open(db_name, &db);
  if(rc) { // Check if database is available
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db); // Close db
    return 1;
  }

  /* Create SQL statement */
  char sql[100];
  sprintf(sql, "SELECT soc1, soc2, soc3 FROM %s", tblName);

  int error = 1;
  sqlite3_stmt *res;
  const char *tail;
  rc = sqlite3_prepare_v2(db, sql, 100, &res, &tail);
  if(rc!=SQLITE_OK) {
    fprintf(stderr, "SQL error\n");
  }else {
    if(sqlite3_step(res)==SQLITE_ROW) {
      socs->soc1 = (float)sqlite3_column_double(res, 0);
      socs->soc2 = (float)sqlite3_column_double(res, 1);
      socs->soc3 = (float)sqlite3_column_double(res, 2);
      error = 0;
    }else {
      printf("No data\n");
    }
  }
  sqlite3_finalize(res);

  if(tbl==TBL_CUSTOM_SETTING) { // Delete custom setting data after retrived
    char sql2[100];
    sprintf(sql2, "DELETE FROM tblCustomSetting");
    char *err_msg = 0;
	  rc = sqlite3_exec(db, sql2, 0, 0, &err_msg);
    if (rc != SQLITE_OK ) {
      fprintf(stderr, "Delete custom setting error: %s\n", err_msg);
      sqlite3_free(err_msg);
    }
  }

  sqlite3_close(db);
  return error;
}

int retrieve_last_status(StatusStruct **status_addr, const int time_limit)
{
  //Open database
  sqlite3 *db;
  int rc = sqlite3_open(db_name, &db);
  if(rc) { // Check if database is available
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db); // Close db
    return 1;
  }

  char cond[100];
  if(time_limit<0) { // No condition
      sprintf(cond, "1");
  }else{
      sprintf(cond, "date>(datetime('now', 'localtime', '-%d seconds'))", time_limit);
  }

  /* Create SQL statement */
  char sql[500];
  sprintf(sql,
      "SELECT v_bank1,v_bank2,v_bank3," \
      "i_in1,i_in2,i_in3," \
      "i_out1,i_out2,i_out3," \
      "in_en1,in_en2,in_en3," \
      "out_en1,out_en2,out_en3," \
      "temp1,temp2,temp3," \
      "soc1,soc2,soc3," \
      "heatsink1,heatsink2,inside_air," \
      "p_pv,p_load" \
      " FROM tblCurrentStatus" \
      " WHERE %s;", cond);

  int error = 1;
  sqlite3_stmt *res;
  const char *tail;
  rc = sqlite3_prepare_v2(db, sql, 500, &res, &tail);
  if(rc!=SQLITE_OK) {
    fprintf(stderr, "SQL error\n");
  }else {
    if(sqlite3_step(res)==SQLITE_ROW) {
      (*status_addr)->b1->v = (float)sqlite3_column_double(res, 0);
      (*status_addr)->b2->v = (float)sqlite3_column_double(res, 1);
      (*status_addr)->b3->v = (float)sqlite3_column_double(res, 2);
      (*status_addr)->b1->i_in = (float)sqlite3_column_double(res, 3);
      (*status_addr)->b2->i_in = (float)sqlite3_column_double(res, 4);
      (*status_addr)->b3->i_in = (float)sqlite3_column_double(res, 5);
      (*status_addr)->b1->i_out = (float)sqlite3_column_double(res, 6);
      (*status_addr)->b2->i_out = (float)sqlite3_column_double(res, 7);
      (*status_addr)->b3->i_out = (float)sqlite3_column_double(res, 8);
      (*status_addr)->b1->in_en = sqlite3_column_int(res, 9);
      (*status_addr)->b2->in_en = sqlite3_column_int(res, 10);
      (*status_addr)->b3->in_en = sqlite3_column_int(res, 11);
      (*status_addr)->b1->out_en = sqlite3_column_int(res, 12);
      (*status_addr)->b2->out_en = sqlite3_column_int(res, 13);
      (*status_addr)->b3->out_en = sqlite3_column_int(res, 14);
      (*status_addr)->b1->temp = (float)sqlite3_column_double(res, 15);
      (*status_addr)->b2->temp = (float)sqlite3_column_double(res, 16);
      (*status_addr)->b3->temp = (float)sqlite3_column_double(res, 17);
      (*status_addr)->b1->soc = (float)sqlite3_column_double(res, 18);
      (*status_addr)->b2->soc = (float)sqlite3_column_double(res, 19);
      (*status_addr)->b3->soc = (float)sqlite3_column_double(res, 20);
      (*status_addr)->temp1 = (float)sqlite3_column_double(res, 21);
      (*status_addr)->temp2 = (float)sqlite3_column_double(res, 22);
      (*status_addr)->temp3 = (float)sqlite3_column_double(res, 23);
      (*status_addr)->p_pv = sqlite3_column_int(res, 24);
      (*status_addr)->p_load = sqlite3_column_int(res, 25);
      error = 0;
    }else {
        printf("No data\n");
    }
  }
  sqlite3_finalize(res);
  sqlite3_close(db);
  return error;
}

///Update state prediction.
int update_prediction(StatusStruct **status){
    sqlite3 *db;
		char *err_msg = 0;
		//Open database
  	int rc = sqlite3_open(db_name, &db);

		// Check if database is available
  	if (rc != SQLITE_OK) {
      		fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
          sqlite3_close(db); // Close db
      		return 1;
  	}

  	char sqlSysStatus[200];
  	sprintf(sqlSysStatus,
	      "DELETE FROM tblPrediction; " \
              "INSERT INTO tblPrediction " \
              "VALUES(%d, %d );" ,
              (*status)->next_chrg,
              (*status)->next_dischrg
            );

  	rc = sqlite3_exec(db, sqlSysStatus, 0, 0, &err_msg);

		//Check execution result
  	if (rc != SQLITE_OK ) {
      		fprintf(stderr, "Inserting prediction error: %s\n", err_msg);
      		sqlite3_free(err_msg);
      		sqlite3_close(db);

      		return 1;
  	}
  	sqlite3_close(db);
  	return 0;
}

///Update fault
int update_fault(int ocp,int otp, int ovp, int uvp, int modbus){
    sqlite3 *db;
		char *err_msg = 0;
		//Open database
  	int rc = sqlite3_open(db_name, &db);

		// Check if database is available
  	if (rc != SQLITE_OK) {
      		fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
          sqlite3_close(db); // Close db
      		return 1;
  	}

  	char sqlSysStatus[200];
  	sprintf(sqlSysStatus,
	      "DELETE FROM tblFault; " \
              "INSERT INTO tblFault " \
              "VALUES(%d, %d, %d, %d, %d);" ,
              ocp,
              otp,
              ovp,
              uvp,
              modbus
            );

  	rc = sqlite3_exec(db, sqlSysStatus, 0, 0, &err_msg);

		//Check execution result
  	if (rc != SQLITE_OK ) {
      		fprintf(stderr, "Inserting prediction error: %s\n", err_msg);
      		sqlite3_free(err_msg);
      		sqlite3_close(db);

      		return 1;
  	}
  	sqlite3_close(db);
  	return 0;
}

//Call back of retrieving data
static int retrieve_status_callback(void *data, int argc, char **argv, char **azColName) {
   	int i;
		fprintf(stderr, "%s: ", (const char*)data);

		for(i = 0; i<argc; i++) {
      		printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   	}
   	return 0;
}

//Retrieving DBEM status
int retrieve_status() {
    sqlite3 *db;
		char *zErrMsg = 0;
   	int rc;
   	char *sql;
   	const char* data = "Callback function called";
    //Open database
  	rc = sqlite3_open(db_name, &db);

	// Check if database is available
   	if( rc ) {
      		fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
          sqlite3_close(db); // Close db
      		return(0);
   	} else {
      		fprintf(stderr, "Opened database successfully\n");
   	}

   	/* Create SQL statement */
   	sql = "SELECT * FROM tblBattStatus bs INNER JOIN tblSystemStatus ss ON bs.sys_status_id = ss.id;";

   	/* Execute SQL statement */
   	rc = sqlite3_exec(db, sql, retrieve_status_callback, (void*)data, &zErrMsg);
		//Check execution result
   	if( rc != SQLITE_OK ) {
      		fprintf(stderr, "SQL error: %s\n", zErrMsg);
      		sqlite3_free(zErrMsg);
   	} else {
      		fprintf(stdout, "Operation done successfully\n");
   	}
   	sqlite3_close(db);
	return 0;
}

//Call back of retrieving data
static int retrieve_i_conf_callback(void *data, int argc, char **argv, char **azColName) {
    BatteryConfigStruct * conf = (BatteryConfigStruct*)data;
	conf->chg_v = atof(argv[0]);
	conf->chg_i_min = atof(argv[1]);
	conf->chg_i_max = atof(argv[2]);
	conf->dischg_v = atof(argv[3]);
	conf->dischg_i_min = atof(argv[4]);
	conf->dischg_i_max = atof(argv[5]);
    conf->type = atof(argv[6]);
    conf->capacity = atof(argv[7]);
    conf->chg_v_re = atof(argv[8]);
    conf->dischg_v_re = atof(argv[9]);
    return 0;
}

//Retrieving DBEM status
int retrieve_config(BatteryConfigStruct **s, int bank_id) {
        sqlite3 *db;
        char *zErrMsg = 0;
        int rc;
        const char* data = "Callback function called";
        //Open database
        rc = sqlite3_open(db_name, &db);

        // Check if database is available
        if( rc ) {
                fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
                sqlite3_close(db); // Close db
                return 1;
        }

        /* Create SQL statement */
        char sql[500];
        sprintf(sql,
                    "SELECT chg_v,chg_i_min,chg_i_max," \
                    "dischg_v,dischg_i_min,dischg_i_max,type,e_charge,chg_v_re,dischg_v_re " \
                    " FROM tblBattery WHERE id = %d;",
                    bank_id
                  );

        /* Execute SQL statement */
        rc = sqlite3_exec(db, sql, retrieve_i_conf_callback, *s, &zErrMsg);
                //Check execution result
        if( rc != SQLITE_OK ) {
                fprintf(stderr, "SQL error: %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
        }
        sqlite3_close(db);
        return 0;
}

int retrieve_batt_e_charge(int *e_charges) {
  //Open database
  sqlite3 *db;
  int rc = sqlite3_open(db_name, &db);
  if(rc) { // Check if database is available
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db); // Close db
    return 1;
  }

  /* Create SQL statement */
  char sql[200];
  sprintf(sql, "SELECT id,e_charge FROM tblBattery;");

  sqlite3_stmt *res;
  const char *tail;
  rc = sqlite3_prepare_v2(db, sql, 200, &res, &tail);
  if(rc!=SQLITE_OK) {
    fprintf(stderr, "SQL error\n");
  }else {
    while(sqlite3_step(res)==SQLITE_ROW)
    {
      const int bankNum = sqlite3_column_int(res, 0);
      if(bankNum>0 && bankNum<=3) {
        e_charges[bankNum-1] = sqlite3_column_int(res, 1);
      }
    }
  }
  sqlite3_finalize(res);
  sqlite3_close(db);
  return 0;
}

int export_to_csv(char *dstart, char *dend){

    struct stat st = {0};

    if (stat("logs", &st) == -1) {
        mkdir("logs", 0755);
    }

		char sys_cmd[6000];
  	sprintf(sys_cmd,
		"sqlite3 -header -csv 'dbem.db' \"" \

		//Start query
		"SELECT sys_status.date, " \
		"in_en1,i_in1,v1,i_out1,out_en1,temp_batt1,soc1, " \
		"in_en2,i_in2,v2,i_out2,out_en2,temp_batt2,soc2, " \
		"in_en3,i_in3,v3,i_out3,out_en3,temp_batt3,soc3, " \
		"sys_status.p_pv, " \
		"sys_status.p_load " \
		"FROM " \

		"((SELECT in_en AS in_en1,i_in AS i_in1,v as v1,i_out AS i_out1,out_en AS out_en1,temp as temp_batt1,soc as soc1,sys_status_id " \
		"FROM tblBattStatus WHERE bank_id = 1) AS bank1 " \

		"INNER JOIN " \

		"(SELECT in_en AS in_en2,i_in AS i_in2,v as v2,i_out AS i_out2,out_en AS out_en2,temp as temp_batt2,soc as soc2,sys_status_id " \
		"FROM tblBattStatus WHERE bank_id = 2) AS bank2 " \
		"ON bank1.sys_status_id = bank2.sys_status_id " \

		"INNER JOIN " \
		"(SELECT in_en AS in_en3,i_in AS i_in3,v as v3,i_out AS i_out3,out_en AS out_en3,temp as temp_batt3,soc as soc3,sys_status_id " \
		"FROM tblBattStatus WHERE bank_id = 3) AS bank3 " \
		"ON bank1.sys_status_id = bank3.sys_status_id " \

		") AS batt_status INNER JOIN tblSystemStatus sys_status " \
		"ON batt_status.sys_status_id = sys_status.id " \
		"WHERE date BETWEEN '%s' AND '%s'; " \
		"\" > log.csv",dstart,dend);
		system(sys_cmd);

		return 0;
}

// @return table's data count (0 when failed)
int count_table_data(const char *table_name){
  //Open database
  sqlite3 *db;
  int rc = sqlite3_open(db_name, &db);
  if(rc) { // Check if database is available
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db); // Close db
    return 0;
  }

  /* Create SQL statement */
  char sql[200];
  sprintf(sql, "SELECT COUNT(*) FROM %s;", table_name);

  int count = 0;
  sqlite3_stmt *res;
  const char *tail;
  rc = sqlite3_prepare_v2(db, sql, 200, &res, &tail);
  if(rc!=SQLITE_OK) {
    fprintf(stderr, "SQL error\n");
  }else {
    if(sqlite3_step(res)==SQLITE_ROW) {
      count = sqlite3_column_int(res, 0);
    }
  }
  sqlite3_finalize(res);
  sqlite3_close(db);
  return count;
}

int count_current_status()
{
  char *table_name = "tblCurrentStatus";
  return count_table_data(table_name);
}

int count_system_status()
{
  char *table_name = "tblSystemStatus";
  return count_table_data(table_name);
}

