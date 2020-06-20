#ifndef BATTERY_STRUCT_H
#define BATTERY_STRUCT_H

//A structure of battery
typedef struct {
  int bank_id;      //Battery identifier
  int in_en;        //Battery charging enabled (1/0)
  int out_en;       //Battery discharging enabled (1/0)
  float i_in;         //Battery in current (A)
  float i_out;        //Battery out current (A)
  float comb_i_in;    //Current in from combox
  float comb_i_out;   //Current out from combox
  float v;            //Battery voltage (V)
  float temp;         //Temperature of battery (C)
  float soc;         // State of charge (percent or 0.0-1.0)
  float target_soc;    /**< Target SOC to check next state */
  float target_soc_dischg;
  int energy;       //Energy size of battery (A/Hr)
} BatteryStruct;

typedef struct {
  int bank_id;              /**< Battery identifier */
  float capacity;           /**< battery capacity */
  float chg_v;            //Charging V config
  float chg_i_min;        //Charging Imin config
  float chg_i_max;        //Charging Imax config
  float dischg_v;         //Discharging V config
  float dischg_i_min;     //Discharging Imin config
  float dischg_i_max;     //Discharging Imax config
  int type;       //Battery type config
  float chg_v_re;
  float dischg_v_re;        /**< minimum voltage allow for discharing */
} BatteryConfigStruct;

#endif
