#ifndef DBEM_IO_H
#define DBEM_IO_H

#include <stdio.h>
#include <modbus.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include "iolib/core.h"
#include "iolib/gpio.h"
#include "iolib/pwm.h"
#include "iolib/ads1015.h"
#include "iolib/adc.h"
#include "db_manager.h"
#include "battery_struct.h"
#include "status_struct.h"

#define DL1_EN_GPIO 66   /* Dummy load 1 (P8_7) */
#define DL2_EN_GPIO 69   /* Dummy load 2 (P8_9)*/
#define DL3_EN_GPIO 45   /* Dummy load 3 (P8_11)*/

#define BANK1_CHG_GPIO 47 /* Charging switch 1 (P8_15) */
#define BANK2_CHG_GPIO 117 /* Charging switch 1 (P9_25) */
#define BANK3_CHG_GPIO 20 /* Charging switch 1 (P9_41) */

#define BANK1_DISCHG_GPIO 44 /* Discharging switch 1 (P8_12) */
#define BANK2_DISCHG_GPIO 26 /* Discharging switch 2 (P8_14)*/
#define BANK3_DISCHG_GPIO 46 /* Discharging switch 3 (P8_16)*/

#define MR_IN_EN_GPIO 27 /* MR switch in (P8_17)*/
#define MR_OUT_EN_GPIO 65 /* MR switch out (P8_18)*/

#define READY_GPIO 67 /* P8_8 */
#define RESET_N 68 /* P8_10 */
#define FAULT_N 61 /* P8_26 */

//Define i2c address --------------------------------------------
#define I2C_V_BANK1_ADDR 48 // i2c address for voltage of battery group 1
#define I2C_V_BANK2_ADDR 48 // i2c address for voltage of battery group 2
#define I2C_V_BANK3_ADDR 49 // i2c address for voltage of battery group 3

#define I2C_CS_CHRG_BANK1_ADDR 48 // i2c address for charging current sensor of battery group 1
#define I2C_CS_CHRG_BANK2_ADDR 49 // i2c address for charging current sensor of battery group 2
#define I2C_CS_CHRG_BANK3_ADDR 49 // i2c address for charging current sensor of battery group 3

#define I2C_CS_DISCHRG_BANK1_ADDR 48 // i2c address for discharging current sensor of battery group 1
#define I2C_CS_DISCHRG_BANK2_ADDR 49 // i2c address for discharging current sensor of battery group 2
#define I2C_CS_DISCHRG_BANK3_ADDR 50 // i2c address for discharging current sensor of battery group 3

#define I2C_TEMP_BANK1_ADDR 51 // i2c address for temp sensor of battery group 1
#define I2C_TEMP_BANK2_ADDR 51 // i2c address for temp sensor of battery group 2
#define I2C_TEMP_BANK3_ADDR 51 // i2c address for temp sensor of battery group 3

#define I2C_TEMP_H1_ADDR 50 // i2c address for temp sensor of heatsink1
#define I2C_TEMP_H2_ADDR 50 // i2c address for temp sensor of heatsink2
#define I2C_TEMP_INA_ADDR 50 // i2c address for temp sensor of inside-air

#define I2C_V_REF_ADC_ADDR 51 // i2c address for temp sensor of inside-air

//Define i2c AIN pin --------------------------------------------
#define I2C_V_BANK1_AIN 0 // i2c pin AIN for voltage of battery group 1
#define I2C_V_BANK2_AIN 3 // i2c pin AIN for voltage of battery group 2
#define I2C_V_BANK3_AIN 2 // i2c pin AIN for voltage of battery group 3

#define I2C_CS_CHRG_BANK1_AIN 1 // i2c pin AIN for charging current sensor of battery group 1
#define I2C_CS_CHRG_BANK2_AIN 0 // i2c pin AIN for charging current sensor of battery group 2
#define I2C_CS_CHRG_BANK3_AIN 3 // i2c pin AIN for charging current sensor of battery group 3

#define I2C_CS_DISCHRG_BANK1_AIN 2 // i2c pin AIN for discharging current sensor of battery group 1
#define I2C_CS_DISCHRG_BANK2_AIN 1 // i2c pin AIN for discharging current sensor of battery group 2
#define I2C_CS_DISCHRG_BANK3_AIN 0 // i2c pin AIN for discharging current sensor of battery group 3

#define I2C_TEMP_BANK1_AIN 0 // i2c pin AIN for temp sensor of battery group 1
#define I2C_TEMP_BANK2_AIN 1 // i2c pin AIN for temp sensor of battery group 2
#define I2C_TEMP_BANK3_AIN 2 // i2c pin AIN for temp sensor of battery group 3

#define I2C_TEMP_H1_AIN 1 // i2c address for temp sensor of heatsink1
#define I2C_TEMP_H2_AIN 2 // i2c address for temp sensor of heatsink2
#define I2C_TEMP_INA_AIN 3 // i2c address for temp sensor of inside-air

#define I2C_V_REF_ADC_AIN 3 // i2c address for temp sensor of inside-air
//----------------------------------------------------------------

#define EN_DL_SLEEP 0.2

// #define MODBUS_TCP_IP "192.168.1.105"
#define MODBUS_TCP_IP "192.168.2.16"
#define MODBUS_TCP_PORT 502
#define DELAY_READ_COMBOX 100000 // MicroSeconds

// Constant For cal I-in and I-out

#define A_CHRG_BANK1 -356.2536667
#define A_CHRG_BANK2 -466.2205336
#define A_CHRG_BANK3 -327.9689834
#define A_DISCHG_BANK1 -327.9689834
#define A_DISCHG_BANK2 -351.2055582
#define A_DISCHG_BANK3 -334.6105669
#define B_CHRG_BANK1 176.788032
#define B_CHRG_BANK2 231.358210
#define B_CHRG_BANK3 164.720782
#define B_DISCHG_BANK1 162.751984
#define B_DISCHG_BANK2 174.282949
#define B_DISCHG_BANK3 168.726040

#define LOW_SOC_INDICATOR 0.5
#define HIGH_SOC_INDICATOR 1.0

#define CHRG_OP_MODE_STANDBY 2
#define CHRG_OP_MODE_OPERATING 3

#define CHRG_CYCLY_3_STAGE 1
#define CHRG_CYCLE_2_STAGE 2

extern int compareSOC(StatusStruct** status);
extern int compareCapacityWithSOC(int comp_soc, int *batt_cap_idx_sorted, int *batt_soc_idx_sorted);
extern int compareBattCurrentChg(int batt_num, ConfigStruct *config, StatusStruct **status);

/* Write GPIO pin
  Parameters:
  - pin: GPIO pin number to control
  - value: turning switch value (on 1 , off 0)
*/
extern int write_gpio(int pin, int value);

/* Write PWM pin
  Parameters:
  - name: pwm file name to control
  - run: to run pwm (start 1, stop 1)
  - period: value of period (nano sec)
  - duty: value of duty (nano sec)

  *** Duty cycle = duty / period ***
*/
extern int drive_pwm(char *name, int run, int period, int duty);

/* Check if battery voltage is too low.
  Parameters:
  - bank_num: battery group number (can only be 1,2,3)
  - v_batt: battery voltage
*/
extern int is_low_ovp_detected(int bank_num,int v_batt);

/* Check if battery voltage is too high.
  Parameters:
  - bank_num: battery group number (can only be 1,2,3)
  - v_batt: battery voltage
*/
extern int is_high_ovp_detected(int bank_num,int v_batt);

/* Check if battery temperature is too high.
  Parameters:
  - bank_num: battery group number (can only be 1,2,3)
  - temp: battery temperature
*/
extern int is_batt_otp_detected(int bank_num,float temp);

/* Check if temperature inside of DBEM is too high.
  Parameters:
  - t1: a value from temp sensor number 1
  - t2: a value from temp sensor number 2
  - t3: a value from temp sensor number 3
*/
extern int is_dbem_otp_detected(float t1,float t2,float t3);

/* Check if battery current is too high.
  Parameters:
  - batt_num: battery group number (can only be 1,2,3)
  - i_in: inlet current
  - i_out: outlet current
*/
extern int is_ocp_detected(int batt_num, float i_in, float i_out);

/* Open circuit (cut) for all charging/discharging paths*/
extern void openAllSwitch();

/* Open circuit (cut) for all discharging paths*/
extern void open_all_output_switch();

/* Open circuit (cut) for all charging paths*/
extern void open_all_input_switch();

extern int open_other_output_switch(int bank_num, StatusStruct **status);

extern int open_other_input_switch(int bank_num, StatusStruct **status);

/* Get charging current of battery group
  Parameters:
  - batt_num: battery group number (can only be 1,2,3)
*/
extern float getBattChrgCurrent(int bank_num, float a, float b);

/* Get discharging current of battery group
  Parameters:
  - batt_num: battery group number (can only be 1,2,3)
*/
extern float getBattDisChrgCurrent(int bank_num, float a, float b);

/* Get bettery temperature
  Parameters:
  - batt_num: battery group number (can only be 1,2,3)
*/
extern float getBattTemp(int bank_num);

/* Control fan speed by considering temp
  Parameters:
  - t1: a value from DBEM's temp sensor number 1
  - t2: a value from DBEM's temp sensor number 2
  - t3: a value from DBEM's temp sensor number 3
*/
extern int controlFanByTemp(float temp1, float temp2, float temp3);

/* Update fault status /This function will protect system automatically after fault occurs
  Parameters:
  - status: A structure of DBEM status
  Return:
  - false:1 , OK:0
*/
extern int updateFaultStatus(StatusStruct **status);

/* Get voltage of battery
  Parameters:
  - batt_num: battery group number (can only be 1,2,3)
*/
extern float getBattVoltage(int bank_num);

/* Get power of load*/
extern int getPLoad(float voltageBatt);

/* Get power of pv*/
extern int getPPV();

/* Get I of pv remian*/
extern float getIPvRemain();

/* Get temperature of HeatSink1*/
extern float getTempHeatSink1();

/* Get temperature of HeatSink2*/
extern float getTempHeatSink2();

/* Get temperature of InsideAir*/
extern float getTempInsideAir();

/* Discharge battery
  Parameters:
  - bank: battery properties structure
  - forced: force discharge even lower ovp/soc detected
*/
extern int dischrgBank(BatteryStruct **bank, int forced);

/* Charge battery
  Parameters:
  - bank: battery properties structure
*/
extern int chrgBank(BatteryStruct **bank);

/* Get the lowest voltage bank
  Parameter:
  - status structure (see status_struct.h)
*/
extern int getBankLowestVoltage(StatusStruct **status);

/* Get bank number to charge
  Parameter:
  - status structure (see status_struct.h)
  Return:
  - in-range bank number
*/
extern int batteryChargingSelect(StatusStruct **status);
extern int batteryDischargingSelect(StatusStruct **status);
extern int getBankToCharge(float i_pv_remain, StatusStruct **status);

/* Get the highest voltage bank
  - status structure (see status_struct.h)
*/
extern int getBankHighestVoltage(StatusStruct **status);

extern int getBankHighestSOC(StatusStruct **status);

extern int getBankLowestSOC(StatusStruct **status);

/* Sort data from biggest to smallest.
  Parameter:
  - index: given array ,will return data of sorted index.
  - values: array of value to be sorted.
*/
extern int sortBiggestToSmallest(int * index, float * values);

extern float getHighestVoltage(StatusStruct **status);

/* Tell Charger to change battery type
  Parameters:
  - battery_type: battery type number (can only be 0,1,2,3)
*/
extern int tellChargerToChangeBatteryType(BatteryConfigStruct **batt);

/* Get Ppv from charger via modbus
  Return:
  - Ppv
*/
extern float readPpv();

/* Get input voltage from charger via modbus
  Return:
  - charger voltage
*/
extern float readChargerIDcVolt();

/* Get input current from charger via modbus
  Return:
  - charger input current
*/
extern float readChargerIDcCurrent();

/* Get output current from charger via modbus
  Return:
  - charger output current
*/
extern float readChargerODcCurrent();

/* Get voltage from charger via modbus
  Return:
  - charger voltage
*/
extern float readChargerVoltage();

/* Get Pload from charger via modbus
  Return:
  - Pload
*/
extern float readLoadAcPower();

/* Get current from charger via modbus
  Return:
  - load current
*/
extern float readLoadAcCurrent();

/* Get net current to/from battery via modbus
  Return
  - net batt current
*/
extern float readBattCurrentNet();

/* Get charge/discharge current from combox
  Param:
  - chgI is charge current read from combox
  - dischgI is discharge current read from combox
*/
extern void getCombBattI(float *chgI, float *dischgI);

/**
 * Read data from modbug (p_pv, p_load, i_in, i_out)
 * @param p_pv
 * @param p_load
 * @param i_in
 * @param i_out
 * @return error status (0 is no error);
 */
extern int readDataFromModbus(float *p_pv, float *p_load, float *i_in, float *i_out);

/* Get modbus value by register address
  Parameter:
  - slave_id: slave id of modbus (ex. 0xAA, 0xA)
  - modbus_adress: modbus address that we need value
  - number_holding: number holding (ex. 1 for 16bit, 2 for 32bit) libmodbus only read 16bit
  - scale: scale depend on modbus address
  - error: error flag set when read modbus error (optional, if not use set to NULL)
  Return:
  - value of mudbus address
*/
extern float readModbus(long slave_id, long modbus_adress, int number_holding, float scale, int *error);

/**
 * Read multiple modbus value (all parameter (except first) is array with same size as count)
 * @param count is modbus value count
 * @param slave_ids
 * @param modbus_addrs
 * @param number_holdings
 * @param scales
 * @param values
 * @return error status (0 is no error)
 */
extern int readMultiModbus(const int count, long *slave_ids, long *modbus_addrs, int *number_holdings, float *scales, float *values);

/**
 * Write multiple modbus value (all parameter (except first) is array with same size as count)
 */
extern int writeMultiModbus(const int count, long *slave_ids, long *modbus_addrs, int *number_holdings, int *values);
extern float writeModbus(long slave_id, long modbus_address, float value);
extern float writeModbus32(long slave_id, long modbus_address, int value);
extern int setChargerOpMode(int mode, int retry);
extern int inverterConfiguration(BatteryConfigStruct **batt);

#endif
