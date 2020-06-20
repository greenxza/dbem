#ifndef DB_MANAGER_H
#define DB_MANAGER_H

#include "status_struct.h"

typedef enum
{
    TBL_CURRENT_STATUS = 0,
    TBL_CUSTOM_SETTING
} TblName_t;

// Initailize database
extern int init_database(char *name);

// Get batt Iin from battery struct
extern float getBattIin(const BatteryStruct *batt);
// Get batt Iout from battery struct
extern float getBattIout(const BatteryStruct *batt);

// Save monitoring status of system
extern int saveStatusToDB(StatusStruct **status_addr);

/**
 * @param socs is output socs
 * @param tbl is table type
 * @return 1 when failed
 */
extern int retrieve_socs(SOCsStruct *socs, const TblName_t tbl);

/**
 * @Retrieve last status from db
 * @param status_addr is status storage loaded
 * @param time_limit is limit time in second (-1 will skip)
 * @return 1 when failed
 */
extern int retrieve_last_status(StatusStruct **status_addr, const int time_limit);

// Get last statua values
extern int retrieve_status();

// Retrieving battery e_charge
extern int retrieve_batt_e_charge(int *e_charges);

// Export csv log from database
extern int export_to_csv(char *dstart, char *dend);

// Update fualt status to the database.
extern int update_fault(int ocp,int otp, int ovp, int uvp, int modbus);

// Get battery configuration (from UI).
extern int retrieve_config(BatteryConfigStruct **s, int bank_id);

// Update monitoring status (Save to database only one row).
extern int updateMonitoringStatus(StatusStruct **status_addr);

extern int get_limit_log(float **samplingRate);

// @return data count in table tblCurrentStatus
extern int count_current_status();

// @return data count in table tblSystemStatus
extern int count_system_status();

#endif
