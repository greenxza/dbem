#ifndef STATUS_STRUCT_H
#define STATUS_STRUCT_H

#include "battery_struct.h"

typedef struct {
    int p_load;                 /**< Power of load (W) */
    int p_pv;                   /**< Power of pv (W) */
    int modbus_err;             /**< Modbus error status */
    time_t prev_log_time;       /**< Previous logged timestamp */
    int next_chrg;              /**< Prediction of next charging (1,2,3) */
    int next_dischrg;           /**< Prediction of next discharging (1,2,3) */
    float temp1;                /**< Temperature inside DBEM case 1 */
    float temp2;                /**< Temperature inside DBEM case 2 */
    float temp3;                /**< Temperature inside DBEM case 3 */
    BatteryStruct *b1;          /**< Status battery group 1 (see battery_struct.h) */
    BatteryStruct *b2;          /**< Status battery group 2 (see battery_struct.h) */
    BatteryStruct *b3;          /**< Status battery group 3 (see battery_struct.h) */
    int now_chg;                /**< Back changing at current moment */
    int now_dischg;
} StatusStruct;                 /**< a structure of dbem parameters */

//structure of custom soc
typedef struct {
    float soc1;
    float soc2;
    float soc3;
} SOCsStruct;

typedef struct {
    BatteryConfigStruct *b1;
    BatteryConfigStruct *b2;
    BatteryConfigStruct *b3;
} ConfigStruct;

#endif
