#include </usr/include/python2.7/Python.h>
#include <stdlib.h>
#include <string.h>

#define INIT_PYTHON_ONCE
#define READ_ADC_ERROR_VALUE -999999

// Convert a value read from i2C device to Vref
extern float adcToVoltRef(float adc);

// Convert Vref to V battery
extern float voltRefToVoltReal(float volt);

// Get ADC value from i2c number 48
extern float get_adc(int num);

// Get ADC value from i2c number [address]
extern float get_adc_addr(int num,int address);

// Convert voltage to current
extern float voltToCurrent(float volt_out_read,float volt_ref_read, float slope, float cross);

// Convert voltage to resistance value
extern float voltToResistance(float volt);

extern float resistanceToCelsius(float rt);

extern int findIndexCelius(float rt);

extern float vOutToCelsius(float Vout);

extern int findIndexVolt(float Vout);
