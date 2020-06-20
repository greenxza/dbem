#include <stdlib.h>
 #include <stdio.h>
 #include <string.h>
 #include <unistd.h>     //close()
 #include <fcntl.h>     //define O_WONLY and O_RDONLY

 #define MAX_BUF 64     //This is plenty large

//Function declarations
extern float convertAdcToVolt(int adc);
extern void enableAdcPins();
extern void print_all_adc();
extern int readADC(unsigned int pin);
extern float battVoltageDevider(float volt);
