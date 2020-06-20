#include <unistd.h>
#include "dbem_io.h"
#include <time.h>

// #define ENABLE_SIMULATE_MODBUS_DATA
#define WRITE_MODBUS_DELAY_USEC 1000000 // Micro seconds
#define COMBOX_MPPT_SLAVE_ADDR 0xAA // Charger
#define COMBOX_XW_SLAVE_ADDR 0xA // Inverter
#define SOC_COMP_CAP 0.1   /**< compare value of SOC with capacity >*/

modbus_t *connectModbus();

//--------Comparation-------------
struct str{
   float value;
   int index;
};

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int cmp(const void *a,const void *b){
    struct str *a1 = (struct str *)a;
    struct str *a2 = (struct str*)b;
    if((*a1).value > (*a2).value) {
        return -1;
    }
    else if((*a1).value<(*a2).value) {
        return 1;
    }
    else {
        return 0;
    }
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int write_gpio(int pin, int value){
    // Initailize library
    init_bbc_lib();

    //prepare GPIO properties
    gpio_properties *gpio = malloc(sizeof(gpio_properties));
    gpio->nr = pin;
    gpio->direction = OUTPUT_PIN;

    //Open gpio file to write
    int isOpen = gpio_open(gpio);
    if (isOpen == 0) {
        gpio_set_value(gpio, value);
        //Close if the pin is not in use.
        //gpio_close(gpio);
    }
    free(gpio);
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int drive_pwm(char *name,int run, int period, int duty) {
    // Initailize library
    init_bbc_lib();
    //Prepare pwm properties
    pwm_properties  *pwm = malloc(sizeof(pwm_properties));
    pwm->name = name;
    char pname[30];
    int isOpen = 0;
    for(int i = 0; i < 30; i++) {
        sprintf(pname, "%s.%d",name, i);
        //Open pwm file to write
        pwm->name = pname;
        if(pwm_open(pwm)){
            isOpen = 1;
            break;
        }
    }

    if (isOpen == 1) {
        if (pwm_set_run(pwm, 0) < 0) {
            printf("Could not stop pwm port.\n");
            free(pwm);
            return 1;
        }
        if (pwm_set_polarity(pwm, 0) < 0) {
            printf("Could not set polarity.\n");
            free(pwm);
            return 1;
        }
        if (pwm_set_period(pwm, period) < 0) {
            printf("Could not set period.\n");
            free(pwm);
            return 1;
        }
        if (pwm_set_duty(pwm, duty) < 0) {
            printf("Could not set duty.\n");
            free(pwm);
            return 1;
        }
        if (pwm_set_run(pwm, run) < 0) {
            printf("Could not start pwm port.\n");
            free(pwm);
            return 1;
        }
    }
    free(pwm);
    return 0;
}

/** \brief change all switching state to open
 *
 * \param
 * \param
 * \return
 *
 */
void openAllSwitch(){
  //Open circuit , current can pass
  open_all_input_switch();
  open_all_output_switch();
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
void open_all_output_switch(){
    printf("INFO: Open all output\n");
    //Open circuit , current can pass
    write_gpio(BANK1_DISCHG_GPIO, 0);
    write_gpio(BANK2_DISCHG_GPIO, 0);
    write_gpio(BANK3_DISCHG_GPIO, 0);
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
void open_all_input_switch(){
    printf("INFO: Open all input\n");
    //Open circuit , current can pass
    write_gpio(BANK1_CHG_GPIO, 0);
    write_gpio(BANK2_CHG_GPIO, 0);
    write_gpio(BANK3_CHG_GPIO, 0);
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int open_other_output_switch(int bank_num, StatusStruct **status) {
    switch(bank_num) {
        case 1:
            write_gpio(BANK2_DISCHG_GPIO, 0);
            write_gpio(BANK3_DISCHG_GPIO, 0);
            (*status)->b2->out_en = 0;
            (*status)->b3->out_en = 0;
            break;
        case 2:
            write_gpio(BANK1_DISCHG_GPIO, 0);
            write_gpio(BANK3_DISCHG_GPIO, 0);
            (*status)->b1->out_en = 0;
            (*status)->b3->out_en = 0;
            break;
        case 3:
            write_gpio(BANK1_DISCHG_GPIO, 0);
            write_gpio(BANK2_DISCHG_GPIO, 0);
            (*status)->b1->out_en = 0;
            (*status)->b2->out_en = 0;
            break;
        default:
            return 1;
    }
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int open_other_input_switch(int bank_num, StatusStruct **status) {
    switch(bank_num) {
        case 1:
            write_gpio(BANK2_CHG_GPIO, 0);
            write_gpio(BANK3_CHG_GPIO, 0);
            (*status)->b2->in_en = 0;
            (*status)->b3->in_en = 0;
            break;
        case 2:
            write_gpio(BANK1_CHG_GPIO, 0);
            write_gpio(BANK3_CHG_GPIO, 0);
            (*status)->b1->in_en = 0;
            (*status)->b3->in_en = 0;
            break;
        case 3:
            write_gpio(BANK1_CHG_GPIO, 0);
            write_gpio(BANK2_CHG_GPIO, 0);
            (*status)->b1->in_en = 0;
            (*status)->b2->in_en = 0;
            break;
        default:
            return 1;
        }
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int is_low_soc_detected(int bank_num,float soc_batt){
    if(soc_batt < LOW_SOC_INDICATOR){
        //Cut discharging
        switch(bank_num) {
            case 1:
                write_gpio(BANK1_DISCHG_GPIO, 0);
                break;
            case 2:
                write_gpio(BANK2_DISCHG_GPIO, 0);
                break;
            case 3:
                write_gpio(BANK3_DISCHG_GPIO, 0);
                break;
            default:
                break;
        }
        return 1;
    }
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int is_high_soc_detected(int bank_num, float soc_batt){
    if(soc_batt >= HIGH_SOC_INDICATOR){
        //Cut charging
        switch(bank_num) {
        case 1:
            write_gpio(BANK1_CHG_GPIO, 0);
            break;
        case 2:
            write_gpio(BANK2_CHG_GPIO, 0);
            break;
        case 3:
            write_gpio(BANK3_CHG_GPIO, 0);
            break;
        default:
            break;
        }
        return 1;
    }
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int is_low_ovp_detected(int bank_num,int v_batt){
    BatteryConfigStruct *batt;
    batt = malloc(sizeof(BatteryConfigStruct));
    retrieve_config(&batt, bank_num);

    if(v_batt < batt->dischg_v_re) {
        //Cut discharging
        switch(bank_num) {
        case 1:
          write_gpio(BANK1_DISCHG_GPIO, 0);
          break;
        case 2:
          write_gpio(BANK2_DISCHG_GPIO, 0);
          break;
        case 3:
          write_gpio(BANK3_DISCHG_GPIO, 0);
          break;
        default : break;
        }
        free(batt);
        return 1;
    }
    free(batt);
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int is_high_ovp_detected(int bank_num,int v_batt){
    BatteryConfigStruct *batt;
    batt = malloc(sizeof(BatteryConfigStruct));
    retrieve_config(&batt, bank_num);

    if(v_batt > batt->chg_v_re) {
        //Cut charging
        switch(bank_num) {
        case 1:
          write_gpio(BANK1_CHG_GPIO, 0);
          break;
        case 2:
          write_gpio(BANK2_CHG_GPIO, 0);
          break;
        case 3:
          write_gpio(BANK3_CHG_GPIO, 0);
          break;
        default : break;
        }
        free(batt);
        return 1;
    }
    free(batt);
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int is_ocp_detected(int batt_num, float i_in, float i_out){
    int result = 0;
    if(i_in >= 100){
      //write_gpio(BANK1_CHG_GPIO, 0);
      switch(batt_num) {
      case 1:
        write_gpio(BANK1_CHG_GPIO, 0);
        break;
      case 2:
        write_gpio(BANK2_CHG_GPIO, 0);
        break;
      case 3:
        write_gpio(BANK3_CHG_GPIO, 0);
        break;
      default : break;
      }
      result = 1;
    }
    if(i_out >= 100){
      //write_gpio(BANK1_DISCHG_GPIO, 0);
      switch(batt_num) {
      case 1:
        write_gpio(BANK1_DISCHG_GPIO, 0);
        break;
      case 2:
        write_gpio(BANK2_DISCHG_GPIO, 0);
        break;
      case 3:
        write_gpio(BANK3_DISCHG_GPIO, 0);
        break;
      default : break;
      }
      result = 1;
    }
    return result;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int is_batt_otp_detected(int bank_num,float temp){
    if(temp > 45.0){
        return 1;
    }
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int is_dbem_otp_detected(float t1,float t2,float t3){
    if(t1 > 90 || t2 > 90 || t3 > 90){
        //Cut all switches, Open circuit
        openAllSwitch();
        return 1;
    }
    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
static float voltRefToVolCalibate(float a, float b, float Vadc) {
  printf("====Voltage convert====\n");
  printf("a: %f\n", a);
  printf("b: %f\n", b);
  printf("Vadc: %f\n", Vadc);
  printf("Cal: %f\n", (a*Vadc+b));
  return a * Vadc + b;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
float getBattVoltage(int bank_num){

  float volt = 0.0;
  float adc = -999999;
  float ab[2];
  char bank[10];
  FILE *fp;
  char *line = NULL;
  char *strDoub = NULL;
  int index = 0;
  size_t len = 0;
  ssize_t read;
  fp = fopen("/home/machinekit/Desktop/workspace/dbem/configVreftoVbatt1", "r");
  sprintf(bank, "Bank%d", bank_num);
  printf("======= Convert Vadc to Vbatt =======\n");
  while ((read = getline(&line, &len, fp)) != -1) {
    if(strstr(line, bank) != NULL) {
        strDoub = strtok(line, "=");
        while(strDoub != NULL) {
          strDoub = strtok(NULL, ",");
          if(strDoub != NULL) {
            ab[index] = atof(strDoub);
            printf("Value from file(V to V): %f\n", ab[index]);
            index += 1;
          }
        }
        break;
      }
    }

  fclose(fp);
  if(len) {
    free(line);
    // free(strDoub); // Double free
  }

  if(ab[0] == 0.0 || ab[1] == 0.0) {
    volt = -999999;
    printf("Can not get value from configVreftoVbatt\n");
  }

  switch(bank_num) {
  case 1:
      //Enanable load before reading voltage : Active-low switch
      write_gpio(DL1_EN_GPIO, 1);
      sleep(EN_DL_SLEEP);
      adc  = get_adc_addr(I2C_V_BANK1_AIN, I2C_V_BANK1_ADDR);
      write_gpio(DL1_EN_GPIO, 0);
      break;
  case 2:
      //Enanable load before reading voltage : Active-low switch
      write_gpio(DL2_EN_GPIO, 1);
      sleep(EN_DL_SLEEP);
      adc = get_adc_addr(I2C_V_BANK2_AIN, I2C_V_BANK2_ADDR);
      write_gpio(DL2_EN_GPIO, 0);
      break;
  case 3:
      //Enanable load before reading voltage : Active-low switch
      write_gpio(DL3_EN_GPIO, 1);
      sleep(EN_DL_SLEEP);
      adc = get_adc_addr(I2C_V_BANK3_AIN, I2C_V_BANK3_ADDR);
      write_gpio(DL3_EN_GPIO, 0);
      break;
  default : break;
  }

  if(adc <= READ_ADC_ERROR_VALUE){
    volt = -999999; //return -999999 when read adc error
  }else{
    // volt =  voltRefToVoltReal(adcToVoltRef(adc));
    volt = voltRefToVolCalibate(ab[0], ab[1], adcToVoltRef(adc));
  }
  printf("bank: %d volt_ref: %f\n", bank_num,volt);

  return volt;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
float getBattTemp(int bank_num){

  float resist, adc;
  switch(bank_num) {
  case 1:
      //Read a value from ADC-i2c device number 3 pin A1
      adc = get_adc_addr(I2C_TEMP_BANK1_AIN, I2C_TEMP_BANK1_ADDR);
      break;
  case 2:
      //Read a value from ADC-i2c device number 3 pin A2
      adc = get_adc_addr(I2C_TEMP_BANK2_AIN, I2C_TEMP_BANK2_ADDR);
      break;
  case 3:
      //Read a value from ADC-i2c device number 3 pin A3
      adc = get_adc_addr(I2C_TEMP_BANK3_AIN, I2C_TEMP_BANK3_ADDR);
  default : break;
  }

  if(adc <= READ_ADC_ERROR_VALUE){
    return -999999; //return -999999 when read adc error
  }else{
    return vOutToCelsius(adcToVoltRef(adc));
  }
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
float getBattChrgCurrent(int bank_num, float a, float b){

  float current, adc_volt_read;
  float adc_volt_ref;

  switch(bank_num) {
  case 1:
      adc_volt_read =  get_adc_addr(I2C_CS_CHRG_BANK1_AIN, I2C_CS_CHRG_BANK1_ADDR);
      break;
  case 2:
      adc_volt_read =  get_adc_addr(I2C_CS_CHRG_BANK2_AIN, I2C_CS_CHRG_BANK2_ADDR);
      break;
  case 3:
      adc_volt_read =  get_adc_addr(I2C_CS_CHRG_BANK3_AIN, I2C_CS_CHRG_BANK3_ADDR);
      break;
  default :
      adc_volt_ref = get_adc_addr(3,48);
      break;
  }

  if(adc_volt_read <= READ_ADC_ERROR_VALUE){
    return -999999; //return -999999 when read adc error
  }else{
    float volt_out_read =adcToVoltRef(adc_volt_read);
    float volt_ref_read =adcToVoltRef(adc_volt_ref);
    float current = voltToCurrent(volt_out_read, volt_ref_read, a, b);
    if(current<0) { // Set current to 0 when negative
      current = 0;
    }
    return current;
  }
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
float getBattDisChrgCurrent(int bank_num, float a, float b){

  float current, adc_volt_read;
  float adc_volt_ref;

  switch(bank_num) {
  case 1:
      adc_volt_read =  get_adc_addr(I2C_CS_DISCHRG_BANK1_AIN, I2C_CS_DISCHRG_BANK1_ADDR);
      break;
  case 2:
      adc_volt_read =  get_adc_addr(I2C_CS_DISCHRG_BANK2_AIN, I2C_CS_DISCHRG_BANK2_ADDR);
      break;
  case 3:
      adc_volt_read =  get_adc_addr(I2C_CS_DISCHRG_BANK3_AIN, I2C_CS_DISCHRG_BANK3_ADDR);
      break;
  default : break;
  }

  if(adc_volt_read <= READ_ADC_ERROR_VALUE){
    return -999999; //return -999999 when read adc error
  }else{
    float volt_out_read =adcToVoltRef(adc_volt_read);
    float volt_ref_read =adcToVoltRef(adc_volt_ref);
    float current = voltToCurrent(volt_out_read, volt_ref_read, a, b);
    if(current<0) { // Set current to 0 when negative
      current = 0;
    }
    return current;
  }
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int controlFanByTemp(float temp1, float temp2, float temp3){
    if (temp1 >= 38 || temp2 >= 38 || temp3 >= 38 ){
      //100% fan speed
      drive_pwm("pwm_test_P9_31",1, 43478, 43478);
    }else if(temp1 >= 32 || temp2 >= 32 || temp3 >=32) {
      // 50% fan speed
      drive_pwm("pwm_test_P9_31", 1, 43478, 21739);
    }else if(temp1 >= 28 || temp2 >= 28 || temp3 >= 28){
      //25% fan speed
      drive_pwm("pwm_test_P9_31",1, 43478, 0);
    }else{
      //0% fan speed
      drive_pwm("pwm_test_P9_31",1, 43478, 0);
    }
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int updateFaultStatus(StatusStruct **status){
  // Checking OVP
  int ovp1_high = is_high_ovp_detected((*status)->b1->bank_id, (*status)->b1->v);
  int ovp2_high = is_high_ovp_detected((*status)->b2->bank_id, (*status)->b2->v);
  int ovp3_high = is_high_ovp_detected((*status)->b3->bank_id, (*status)->b3->v);

  int ovp1_low = is_low_ovp_detected((*status)->b1->bank_id, (*status)->b1->v);
  int ovp2_low = is_low_ovp_detected((*status)->b2->bank_id, (*status)->b2->v);
  int ovp3_low = is_low_ovp_detected((*status)->b3->bank_id, (*status)->b3->v);

  // Checking OCP
  int ocp1 = is_ocp_detected((*status)->b1->bank_id, (*status)->b1->i_in, (*status)->b1->i_out);
  int ocp2 = is_ocp_detected((*status)->b2->bank_id, (*status)->b2->i_in, (*status)->b2->i_out);
  int ocp3 = is_ocp_detected((*status)->b3->bank_id, (*status)->b3->i_in, (*status)->b3->i_out);
  { // Check OCP from comb_i
    for(int i=0;i<3;i++) {
      int *ocp = NULL;
      BatteryStruct *batt = NULL;
      switch(i)
      {
        case 0:
          batt = (*status)->b1;
          ocp = &ocp1;
          break;
        case 1:
          batt = (*status)->b2;
          ocp = &ocp2;
          break;
        case 2:
          batt = (*status)->b3;
          ocp = &ocp3;
          break;
        default:
          break;
      }

      if(batt==NULL || // No current batt object
      ocp==NULL || // No current
      *ocp==1 || // OCP detected, no need to check bank current from combox
      (batt->in_en==0 && batt->out_en==0)) { // Both Iin and Iout disabled
        continue;
      }

      float i_in = getBattIin(batt); // Get batt Iin
      float i_out = getBattIout(batt); // Get batt Iout
      *ocp = is_ocp_detected(batt->bank_id, i_in, i_out); // Check OCP from current read from combox
    }
  }

  // Checking OTP
  int otp1 = is_batt_otp_detected((*status)->b1->bank_id,(*status)->b1->temp);
  int otp2 = is_batt_otp_detected((*status)->b2->bank_id,(*status)->b2->temp);
  int otp3 = is_batt_otp_detected((*status)->b3->bank_id,(*status)->b3->temp);

  int dbem_otp = is_dbem_otp_detected((*status)->temp1,(*status)->temp2,(*status)->temp3);

  int fault_ocp = ocp1 || ocp2 || ocp3;
  int fault_ovp = ovp1_high || ovp2_high || ovp3_high;
  int fault_uvp = ovp1_low || ovp2_low || ovp3_low;
  int fault_otp = otp1 || otp2 || otp3 || dbem_otp;

  if(fault_ocp || dbem_otp || fault_otp) {
    //Cut all path
    write_gpio(MR_IN_EN_GPIO, 0);
    write_gpio(MR_OUT_EN_GPIO, 0);

    //Disable all UI path
    (*status)->b1->in_en = 0;
    (*status)->b1->out_en = 0;
    (*status)->b2->in_en = 0;
    (*status)->b2->out_en = 0;
    (*status)->b3->in_en = 0;
    (*status)->b3->out_en = 0;
  }

 /* if(fault_ovp) {
    //Cut all inlet path
    write_gpio(MR_IN_EN_GPIO, 0);

    //Disable all UI inlet path
    (*status)->b1->in_en = 0;
    (*status)->b2->in_en = 0;
    (*status)->b3->in_en = 0;
  } */

  /* if(fault_uvp) {
    //Cut all outlet path
    write_gpio(MR_OUT_EN_GPIO, 0);

    //Disable all UI outlet path
    (*status)->b1->out_en = 0;
    (*status)->b2->out_en = 0;
    (*status)->b3->out_en = 0;
  } */

  // int fault_ovp = fault_ovp_high || fault_ovp_low;

  int fault_modbus_connection = 0;
  if((*status)->modbus_err!=0) {
    fault_modbus_connection = 1;
  }
  // if(((*status)->p_pv==-1) && ((*status)->p_load==-1)){
  //   fault_modbus_connection = 1;
  // }

  //Update database for fault status.
  update_fault(fault_ocp ,fault_otp, fault_ovp, fault_uvp, fault_modbus_connection);

  if(!fault_ocp && !fault_otp && ( !(*status)->b1->out_en && !(*status)->b2->out_en) && !(*status)->b3->out_en){
    write_gpio(MR_IN_EN_GPIO, 1);
    write_gpio(MR_OUT_EN_GPIO, 1);
    return 0; //False status is not detected
  }else{
    printf("INFO: fault_ocp=%d, fault_ovp=%d, fault_uvp=%d, fault_otp=%d\n", fault_ocp ,fault_ovp, fault_uvp, fault_otp);
    return 1; //False status is detected
  }
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int getPLoad(float voltageBatt) {
  return readLoadAcPower(); //read from MODBUS
}

int getPPV() {
  return readPpv(); // read from MODBUS
}

float getTempHeatSink1(){

  float resist, adc;
  adc = get_adc_addr(I2C_TEMP_H1_AIN, I2C_TEMP_H1_ADDR);

  if(adc <= READ_ADC_ERROR_VALUE){
    return -999999; //return -999999 when read adc error
  }else{
    return vOutToCelsius(adcToVoltRef(adc));
  }
}

float getTempHeatSink2(){

  float resist, adc;
  adc = get_adc_addr(I2C_TEMP_H2_AIN, I2C_TEMP_H2_ADDR);

  if(adc <= READ_ADC_ERROR_VALUE){
    return -999999; //return -999999 when read adc error
  }else{
    return vOutToCelsius(adcToVoltRef(adc));
  }
}

float getTempInsideAir(){

  float resist, adc;
  adc = get_adc_addr(I2C_TEMP_INA_AIN, I2C_TEMP_INA_ADDR);

  if(adc <= READ_ADC_ERROR_VALUE){
    return -999999; //return -999999 when read adc error
  }else{
    return vOutToCelsius(adcToVoltRef(adc));
  }
}

int dischrgBank(BatteryStruct **bank, int forced){
  printf("Bank of: %d (forced: %d)\n", (*bank)->bank_id, forced);
  if(!forced) { // Not forced discharge check low ovp/soc
    if(is_low_ovp_detected((*bank)->bank_id, (*bank)->v)){
      printf("Discharging low ovp detected: %f\n", (*bank)->v);
      //OVP has been detected: Battery voltage is too low
      (*bank)->out_en = 0;
      return 1;
    }

    if(is_low_soc_detected((*bank)->bank_id, (*bank)->soc)){
      printf("Discharging low soc detected: %f\n", (*bank)->soc);
      //low SOC has been detected
      (*bank)->out_en = 0;
      return 1;
    }
  }

  //Everything OK, enable switch.
  switch((*bank)->bank_id) {
  case 1:
    write_gpio(BANK1_DISCHG_GPIO, 1);
    break;
  case 2:
    write_gpio(BANK2_DISCHG_GPIO, 1);
    break;
  case 3:
    write_gpio(BANK3_DISCHG_GPIO, 1);
    break;
  default : break;
  }
  (*bank)->out_en = 1;
  printf("INFO: Discharge bank %d\n", (*bank)->bank_id);
  return 0;
}

int chrgBank(BatteryStruct **bank){
  ///1) Detect faults status before process the command -------------------
  // Detecting OTP
  if(is_batt_otp_detected((*bank)->bank_id, (*bank)->temp)){
    //OTP has been detected: Battery voltage is too high
    printf("ERROR: Detected OTP\n");
    (*bank)->in_en = 0;
    return 1;
  }

  //Detecting OVP
  if(is_high_ovp_detected((*bank)->bank_id, (*bank)->v)){
    //OVP has been detected
    printf("ERROR: Detected OVP: %f\n",(*bank)->v);
    (*bank)->in_en = 0;
    return 1;
  }

  //Detecting SOC over
  if(is_high_soc_detected((*bank)->bank_id, (*bank)->soc)){
    //SOC high has been detected
    printf("Charging detected high soc: %f\n",(*bank)->soc);
    (*bank)->in_en = 0;
    return 1;
  }

  // Tell Battery capacity, Battery type and Nominal voltage to combox before charging
  BatteryConfigStruct *batt;
  batt = malloc(sizeof(BatteryConfigStruct));
  retrieve_config(&batt, (*bank)->bank_id);
  tellChargerToChangeBatteryType(&batt); //TODO: please change input when battery ready
  free(batt);

  ///2) Everything OK, enable switch. -----------------------------------
  switch((*bank)->bank_id) {
  case 1:

    write_gpio(BANK1_CHG_GPIO, 1); //Close citcuit for Bank1 charging
    break;
  case 2:
    write_gpio(BANK2_CHG_GPIO, 1); //Close citcuit for Bank2 charging
    break;
  case 3:
    write_gpio(BANK3_CHG_GPIO, 1); //Close citcuit for Bank3 charging
    break;
  default : break;
  }
  (*bank)->in_en = 1;
  printf("Charge Bank:%d\n", (*bank)->bank_id);
  return 0;

}

int getBankLowestVoltage(StatusStruct **status){
  int lowest = (*status)->b1->v;
  int id = (*status)->b1->bank_id;

  if (lowest > (*status)->b2->v) {
    lowest = (*status)->b2->v;
    id = (*status)->b2->bank_id;
  }

  if (lowest > (*status)->b3->v) {
    lowest = (*status)->b3->v;
    id = (*status)->b3->bank_id;
  }

  return id;
}

int getBankHighestVoltage(StatusStruct **status){
  int highest = (*status)->b1->v;
  int id = (*status)->b1->bank_id;

  if (highest < (*status)->b2->v) {
    highest = (*status)->b2->v;
    id = (*status)->b2->bank_id;
  }

  if (highest < (*status)->b3->v) {
    highest = (*status)->b3->v;
    id = (*status)->b3->bank_id;
  }
  return id;
}

float getHighestVoltage(StatusStruct **status){
  float highest = (*status)->b1->v;
  if (highest < (*status)->b2->v) {
    highest = (*status)->b2->v;
  }
  if (highest < (*status)->b3->v) {
    highest = (*status)->b3->v;
  }
  return highest;
}

int getBankLowestSOC(StatusStruct **status){
  float lowest = (*status)->b1->soc;
  int id = (*status)->b1->bank_id;

  if (lowest > (*status)->b2->soc) {
    lowest = (*status)->b2->soc;
    id = (*status)->b2->bank_id;
  }

  if (lowest > (*status)->b3->soc) {
    lowest = (*status)->b3->soc;
    id = (*status)->b3->bank_id;
  }

  return id;
}

int getBankHighestSOC(StatusStruct **status){
  float highest = (*status)->b1->soc;
  int id = (*status)->b1->bank_id;

  if (highest < (*status)->b2->soc) {
    highest = (*status)->b2->soc;
    id = (*status)->b2->bank_id;
  }

  if (highest < (*status)->b3->soc) {
    highest = (*status)->b3->soc;
    id = (*status)->b3->bank_id;
  }

  return id;
}

float getIPvRemain(StatusStruct **status){
  // Ipv_remain calculate by Ppv-Pload = Ppv_remain ; P = IV
  // Ipv_remain = (Ppv-Pload)/V
  float Ppv = (float)(*status)->p_pv;
  float Pload = (float)(*status)->p_load;
  float v = 48.0;

  float i_pv_remian = (Ppv-Pload)/v;
  return i_pv_remian;
}

/** \brief get battery group index for charging
 *
 * \param float i_pv_remain
 * \param StatusStruct **status
 * \return get battery group index
 *
 */
int batteryChargingSelect(float i_pv_remain, StatusStruct **status) {
  int battery_to_charge = 0;
  int soc_idx[3] = {1, 2, 3};
  float soc[3] = {(*status)->b1->soc, (*status)->b2->soc, (*status)->b3->soc};
  float soc_no_sort[3] = {(*status)->b1->soc, (*status)->b2->soc, (*status)->b3->soc};
  float target_soc[3] = {(*status)->b1->target_soc, (*status)->b2->target_soc, (*status)->b3->target_soc};
  int i_batt_index[3] = {1, 2, 3};
  float i_batt[3] = {(*status)->b1->comb_i_in, (*status)->b2->comb_i_in, (*status)->b3->comb_i_in};
  int batt_prev_charge = (*status)->now_chg - 1;
  BatteryConfigStruct *battery_config1;
  BatteryConfigStruct *battery_config2;
  BatteryConfigStruct *battery_config3;
  battery_config1 = malloc(sizeof(BatteryConfigStruct));
  battery_config2 = malloc(sizeof(BatteryConfigStruct));
  battery_config3 = malloc(sizeof(BatteryConfigStruct));
  retrieve_config(&battery_config1, 1);
  retrieve_config(&battery_config2, 2);
  retrieve_config(&battery_config3, 3);
  int batt_cap_idx[3] = {1, 2, 3};
  float batt_cap_value[3] = {
    battery_config1->capacity, battery_config2->capacity, battery_config3->capacity};
  float i_min[3] = {
    battery_config1->chg_i_min, battery_config2->chg_i_min, battery_config3->chg_i_min
  };
  float i_max[3] = {
    battery_config1->chg_i_min, battery_config2->chg_i_max, battery_config3->chg_i_max
  };
  free(battery_config1);
  free(battery_config2);
  free(battery_config3);
  sortBiggestToSmallest(batt_cap_idx, batt_cap_value);
  sortBiggestToSmallest(soc_idx, soc);
  if (soc[0] - soc[1] <= SOC_COMP_CAP && soc[1] - soc[2] <= SOC_COMP_CAP) {
    printf("INFO: All battery has similar SOC, charge highest capacity\n");
    return batt_cap_idx[0];
  } else if (soc[1] - soc[2] <= SOC_COMP_CAP) {
    printf("INFO: 2 battery with lowest SOC have similar SOC, charge higher capacity\n");
    return batt_cap_idx[1];
  } else {
    printf("INFO: All SOC has similar value\n");
  }
  if (batt_prev_charge >= 0) {
    printf("INFO: Check SOC and target SOC\n");
    printf(
      "INFO: Bank=%d | SOC=%.10f | target SOC=%.10f\n", 
      batt_prev_charge + 1, 
      soc_no_sort[batt_prev_charge], 
      target_soc[batt_prev_charge]);
      if (soc_no_sort[batt_prev_charge] < target_soc[batt_prev_charge] &&  target_soc[batt_prev_charge] > 0) {
        printf("INFO: Check SOC and target SOC pass\n");
        return 3 + (*status)->now_chg;
      }
  }
  sortBiggestToSmallest(i_batt_index, i_batt);
  int index;
  for (index = 2; index >= 0; index--) {
    printf("INFO: Check battery current within range\n");
    printf(
      "INFO: Bank=%d | I_MIN=%.1f | I_MAX=%.1f \n",
      soc_idx[index],
      i_min[soc_idx[index] - 1],
      i_max[soc_idx[index] - 1]);
    if (i_min[soc_idx[index] - 1] <= i_batt[0] && i_batt[0] <= i_max[soc_idx[index] - 1]) {
      printf("INFO: current is within range\n");
      return soc_idx[index];
    }
  }
  return battery_to_charge;
}

/** \brief select battery with highest state of charge to discharge
 *
 * \param StatusStruct **status
 * \return battery id
 *
 */
int batteryDischargingSelect(StatusStruct **status) {
    int battery_to_discharge = 0;                   /**< battery group identifier for discharging */
    BatteryConfigStruct *battery_config;            /**< a structure of battery configuration  */

    int soc_idx[3] = {1, 2, 3};       /**< battery group identifiers (state of charge) */
    float state_of_charge[3] = {                    /**< state of discharge values */
        (*status)->b1->soc,
        (*status)->b2->soc,
        (*status)->b3->soc};
    float target_soc[3] = {
        (*status)->b1->target_soc_dischg,
        (*status)->b2->target_soc_dischg,
        (*status)->b3->target_soc_dischg
    };
    int batt_prev_charge = (*status)->now_dischg - 1;
    if (batt_prev_charge >= 0) {
        printf("DEBUG: Check SOC and target SOC\n");
        printf("DEBUG: Bank=%d, SOC=%.10f, target SOC=%.10f\n", batt_prev_charge+1, state_of_charge[batt_prev_charge], target_soc[batt_prev_charge]);
        if (state_of_charge[batt_prev_charge] > target_soc[batt_prev_charge] &&  target_soc[batt_prev_charge] < 1) {
            printf("DEBUG: Check SOC and target SOC pass\n");
            return 3 + (*status)->now_chg;
        }
    }
    sortBiggestToSmallest(soc_idx, state_of_charge);

    int battery_current_index[3] = {1, 2, 3};       /**< battery group identifiers (battery's current) */
    float battery_current[3] = {                    /**< battery current value */
        (*status)->b1->comb_i_out,
        (*status)->b2->comb_i_out,
        (*status)->b3->comb_i_out};
    sortBiggestToSmallest(battery_current_index, battery_current);

    int index;                                      /**< loop counter */
    for (index = 0; index < 3; index++) {
        battery_config = malloc(sizeof(BatteryConfigStruct));
        retrieve_config(&battery_config, soc_idx[index]);
        printf("DEBUG: check battery condition\n");
        printf(
            "DEBUG: BT_ID=%d | SOC=%f | I_MIN=%f | I_MAX=%f\n",
            soc_idx[index],
            state_of_charge[index],
            battery_config->dischg_i_min,
            battery_config->dischg_i_max);
        if (battery_config->dischg_i_min <= battery_current[0] && battery_current[0] <= battery_config->dischg_i_max) {
            printf("DEBUG: Result=PASS\n");
            battery_to_discharge = soc_idx[index];
            free(battery_config);
            break;
        }
        else {
            printf("DEBUG: Result=UNMATCH, Check next group\n");
            free(battery_config);
        }
    }
    return battery_to_discharge;
}

/** \brief get battery group suitable to charge
 *
 * \param leftover current from the pv
 * \param StatusStruct **status
 * \return battery group indentifier
 *
 */
int getBankToCharge(float i_pv_remain, StatusStruct **status){
  BatteryConfigStruct *b1,*b2,*b3;

  //allocate memory
  b1 = malloc(sizeof(BatteryConfigStruct));
  b2 = malloc(sizeof(BatteryConfigStruct));
  b3 = malloc(sizeof(BatteryConfigStruct));

  //Retrieve voltage and current configuration
  retrieve_config(&b1, 1);
  retrieve_config(&b2, 2);
  retrieve_config(&b3, 3);

  int bank_to_chrg = 0;

  int bank_cover_i_pv_remain[3];
  float SOCbank_cover_i_pv_remain[3];
  int index_bank_cover_i_pv_remain =0;

  //Find a range of pv current remain
  if( b1->chg_i_min <= i_pv_remain && i_pv_remain <= b1->chg_i_max){
    bank_cover_i_pv_remain[index_bank_cover_i_pv_remain] = 1;
    SOCbank_cover_i_pv_remain[index_bank_cover_i_pv_remain] = (*status)->b1->soc;
    index_bank_cover_i_pv_remain++;
    printf("i_pv_remain in Bank1 \n");
  }
  if (b2->chg_i_min <= i_pv_remain && i_pv_remain <= b2->chg_i_max){
    bank_cover_i_pv_remain[index_bank_cover_i_pv_remain] = 2;
    SOCbank_cover_i_pv_remain[index_bank_cover_i_pv_remain] = (*status)->b2->soc;
    index_bank_cover_i_pv_remain++;
    printf("i_pv_remain in Bank2 \n");
  }
  if (b3->chg_i_min <=i_pv_remain && i_pv_remain <= b3->chg_i_max){
    bank_cover_i_pv_remain[index_bank_cover_i_pv_remain] = 3;
    SOCbank_cover_i_pv_remain[index_bank_cover_i_pv_remain] = (*status)->b3->soc;
    index_bank_cover_i_pv_remain++;
    printf("i_pv_remain in Bank3 \n");
  }

  //Find lowest soc from rage of pv current remain
  if(index_bank_cover_i_pv_remain !=0){
    float low_soc = 70;
    for(int i = 0; i < index_bank_cover_i_pv_remain; i++){
      printf("Find charge Bank:%d SOC:%f\n",  bank_cover_i_pv_remain[i], SOCbank_cover_i_pv_remain[i]);
      if(SOCbank_cover_i_pv_remain[i] <= low_soc){
        low_soc = SOCbank_cover_i_pv_remain[i];
        bank_to_chrg = bank_cover_i_pv_remain[i];
      }
    }
  }

  free(b1);
  free(b2);
  free(b3);

  printf("bank_to_chrg is : %d\n", bank_to_chrg);
  return bank_to_chrg;
}

int sortBiggestToSmallest(int * index, float * values){
  float arr[3]={values[0],values[1],values[2]};
  struct str objects[3];

  for(int i=0;i<3;i++){
    objects[i].value=arr[i];
    objects[i].index=index[i];
  }

  //sort objects array according to value maybe using qsort
  qsort(objects,3,sizeof(objects[0]),cmp);

  for(int i=0;i<3;i++){
    values[i] = objects[i].value;
    index[i] = objects[i].index;
  }
  return 0;
}

float readPpv(){
  return readModbus(0xAA,0x0050, 2, 1, NULL);
}

float readChargerIDcVolt() {
  return readModbus(0xAA, 0x004C, 2, 0.001, NULL);
}

float readChargerIDcCurrent() {
  return readModbus(0xAA, 0x004E, 2, 0.001, NULL);
}

float readChargerODcCurrent() {
  return readModbus(0xAA, 0x005A, 2, 0.001, NULL);
}

float readChargerVoltage(){
  return readModbus(0xAA,0x0058, 2, 0.001, NULL);
}

float readLoadAcCurrent(){
  return readModbus(0xA,0x0096, 2, 0.001, NULL);
}

float readLoadAcPower(){
  return readModbus(0xA,0x009A, 2, 1, NULL);
}

float readBattCurrentNet()
{
  // Combox modbus id is 201 (0xC9)
  int error = 0;
  float i = readModbus(0xC9, 0x009C, 2, 0.001, &error); // Read modbus id 0xC9 addr 0x009C with two 16 bits param signed
  return (error==0)?i:0.0;
}

void getCombBattI(float *chgI, float *dischgI)
{
  // Reset param
  if(chgI) *chgI = 0.0;
  if(dischgI) *dischgI = 0.0;

  int error = 0;
  float netI = readModbus(0xC9, 0x009C, 2, 0.001, &error); // Net current from combox module (signed)
  if(error) {
    printf("Read net current failed\n");
    return;
  }

  printf("Read net current value: %f\n", netI);
  if(netI>=0) { // Net current is positive, assign value to charge current
    if(chgI) {
      *chgI = netI;
    }
  }else { // Net current is negative, assign value to discharge current
    if(dischgI) {
      *dischgI = (netI*-1);
    }
  }
  return;
}

int readDataFromModbus(float *p_pv, float *p_load, float *i_in, float *i_out)
{
  *p_pv = *p_load = *i_in = *i_out = 0.0; // Reset value

#ifdef ENABLE_SIMULATE_MODBUS_DATA
  // For test (default is disable)
  const time_t curr_time = time(NULL);
  const int curr_hour = (int)(curr_time/3600);
  printf("curr_hour: %d\n", curr_hour);
  if(curr_hour%2==0) { // Switch every hour
    printf("Simulate charge process\n");
    *p_pv = 100;
    *p_load = 50;
  }else {
    printf("Simulate discharge process\n");
    *p_pv = 50;
    *p_load = 100;
  }
  return 0;
#endif

  // Read ppv, pload and current
  const int count = 3;
  long slave_ids[] = {COMBOX_MPPT_SLAVE_ADDR, COMBOX_XW_SLAVE_ADDR, 0xC9};
  long modbus_addrs[] = {0x0050, 0x009A, 0x009C};
  int number_holdings[] = {2, 2, 2};
  float scales[] = {1, 1, 0.001};
  float values[count];
  int err = readMultiModbus(count, slave_ids, modbus_addrs, number_holdings, scales, values);
  if(err==0) {
    *p_pv = values[0];
    *p_load = values[1];
    if(values[2]>=0) { // Net current is positive, assign value to charge current
      *i_in = values[2];
    }else { // Net current is negative, assign value to discharge current
      *i_out = (values[2]*-1);
    }
  }else {
    printf("Read multiple modbus failed with err: %d\n", err);
  }
  return err;
}

//tell charger to known battry type before charge battery
int tellChargerToChangeBatteryType(BatteryConfigStruct **batt){
  printf("Check value Nominal: %f, BattCapa: %f\n", (*batt)->chg_v, (*batt)->capacity);
  {
    const int count = 3;
    long slave_ids[] = {0xAA, 0xAA, 0xAA};
    long modbus_addrs[] = {0x00A5, 0x00A8, 0x00A6};
    int number_holdings[] = {1, 1, 2};
    int values[] = {(int)(*batt)->type, (int)(*batt)->capacity, (int)((*batt)->chg_v * 1000)};
    return writeMultiModbus(count, slave_ids, modbus_addrs, number_holdings, values);
  }
  /** Call write multiple modbus instead
  // Battery type
  // Battery type number (can only be 0,1,2,3)
  // sleep(1);
  // printf("Battery type before: %f\n", readModbus(0xAA, 0x00A5, 1, 1, NULL));
  // sleep(1);
  writeModbus(0xAA, 0x00A5, (*batt)->type);
  // sleep(1);
  // printf("Battery type after: %f\n", readModbus(0xAA, 0x00A5, 1, 1, NULL));

  // Battery Capacity
  // readModbus(0xAA, 0x00A8, 1, 1, NULL);
  // sleep(1);
  // printf("Battery Capacity before: %f\n", readModbus(0xAA, 0x00A8, 1, 1, NULL));
  sleep(1);
  writeModbus(0xAA, 0x00A8, (*batt)->capacity);
  // sleep(1);
  // printf("Battery Capacity after: %f\n", readModbus(0xAA, 0x00A8, 1, 1, NULL));

  // Nominal Voltage
  // readModbus(0xAA, 0x00A6, 2, 0.001, NULL);
  // nominal voltage value are 24000(24V) and 48000(48V)
  // sleep(1);
  // printf("Nominal Voltage before: %f\n", readModbus(0xAA, 0x00A6, 2, 0.001, NULL));
  sleep(1);
  writeModbus32(0xAA, 0x00A6, (int)((*batt)->chg_v * 1000));
  // sleep(1);
  // printf("Nominal Voltage after: %f\n", readModbus(0xAA, 0x00A6, 2, 0.001, NULL));
  // sleep(1);
  */

}

void readModbusConfig(char *ip, int *port)
{
  strcpy(ip, MODBUS_TCP_IP); // Default ip
  *port = MODBUS_TCP_PORT; // Default port
  FILE *fp = fopen("/home/machinekit/Desktop/workspace/dbem/configModbus", "r");
  if(!fp) {
    return;
  }

  char * line = NULL;
  size_t len = 0;
  size_t read;
  char * strDoub = NULL;
  while ((read = getline(&line, &len, fp)) != -1)
  {
    if(strstr(line, "MODBUS_TCP_IP") != NULL) {
      strDoub = strtok(line, "=");
      while(strDoub != NULL) {
        strDoub = strtok(NULL, "=");
        if(strDoub != NULL) {
          strcpy(ip, strDoub);
          const size_t len = strlen(ip);
          if(len>0 && ip[len-1]=='\n') ip[len-1] = '\0'; // Remove \n
        }
      }
    }else if(strstr(line, "MODBUS_TCP_PORT") != NULL) {
      strDoub = strtok(line, "=");
      while(strDoub != NULL) {
        strDoub = strtok(NULL, "=");
        if(strDoub != NULL) {
          *port = atoi(strDoub);
        }
      }
    }
  }
  fclose(fp);
  if (line)
    free(line);
}

// const char * getModbusTcpIp(){
//     FILE *fp;
//     char * line = NULL;
//     size_t len = 0;
//     ssize_t read;
//     char * strDoub = NULL;
//     char * ipDefault = "192.168.2.16";
//     fp = fopen("/home/machinekit/Desktop/workspace/dbem/configModbus", "r");
//     while ((read = getline(&line, &len, fp)) != -1) {
//         if(strstr(line, "MODBUS_TCP_IP") != NULL) {
//             strDoub = strtok(line, "=");
//             while(strDoub != NULL) {
//             strDoub = strtok(NULL, "=");
//                 if(strDoub != NULL) {
//                     fclose(fp);
//                     if (line){free(line);}
//                     return strDoub;
//                 }
//             }
//         }
//     }
//     fclose(fp);
//     if (line)
//         free(line);
//     return ipDefault;
// }

// int getModbusTcpPort(){
//     FILE *fp;
//     char * line = NULL;
//     size_t len = 0;
//     ssize_t read;
//     char * strDoub = NULL;
//     int port = 1;
//     fp = fopen("/home/machinekit/Desktop/workspace/dbem/configModbus", "r");
//     while ((read = getline(&line, &len, fp)) != -1) {
//         if(strstr(line, "MODBUS_TCP_PORT") != NULL) {
//             strDoub = strtok(line, "=");
//             while(strDoub != NULL) {
//             strDoub = strtok(NULL, "=");
//                 if(strDoub != NULL) {
//                     port = atoi(strDoub);
//                 }
//             }
//         }
//     }
//     fclose(fp);
//     if (line)
//         free(line);
//     return port;
// }

float readModbus(long slave_id, long modbus_adress, int number_holding, float scale, int *error)
{
  if(error) { // Reset error flag
    *error = 0;
  }

  char modbus_ip[32];
  int modbus_port;
  readModbusConfig(modbus_ip, &modbus_port); // Read modbus config from file

  modbus_t *ctx;
  uint16_t tab_reg[64];
  int rc;
  ctx = modbus_new_tcp(modbus_ip, modbus_port);
  if (modbus_connect(ctx) == -1) {
    printf("Connection failed: %s\n", modbus_strerror(errno));
    modbus_free(ctx);
    if(error) { // Set error flag
            *error = 1;
    }
    return -1;
  }
  modbus_set_slave(ctx, slave_id);
  rc = modbus_read_registers(ctx, modbus_adress, number_holding, tab_reg);
  if (rc == -1) {
      printf("Read register failed: %s\n", modbus_strerror(errno));
      modbus_close(ctx);
      modbus_free(ctx);
      if(error) { // Set error flag
        *error = 1;
      }
      return -1;
  }

  float mb_read;
  float mb_value;
  if(number_holding == 2){
    mb_read = (tab_reg[1] << 16) + tab_reg[0];
    mb_value = mb_read*scale;
  }else{
    mb_value = tab_reg[0]*scale;
  }
  modbus_close(ctx);
  modbus_free(ctx);
  return mb_value;
}

modbus_t *connectModbus()
{
  char modbus_ip[32];
  int modbus_port;
  readModbusConfig(modbus_ip, &modbus_port); // Read modbus config from file

  modbus_t *ctx;
  ctx = modbus_new_tcp(modbus_ip, modbus_port);
  if (modbus_connect(ctx) == -1) {
    printf("Connection failed: %s\n", modbus_strerror(errno));
    modbus_free(ctx);
    return NULL;
  }
  return ctx;
}

int readMultiModbus(const int count, long *slave_ids, long *modbus_addrs, int *number_holdings, float *scales, float *values)
{
  modbus_t *ctx = connectModbus();
  if(ctx==NULL) {
    printf("Read multiple modbus, connection failed\n");
    return 1;
  }

  int i;
  int result = 0;
  for(i=0; i<count; i++) {
    values[i] = 0;
    uint16_t tab_reg[64];
    const long slave_id = slave_ids[i];
    const int number_holding = number_holdings[i];
    modbus_set_slave(ctx, slave_id);
    int rc = modbus_read_registers(ctx, modbus_addrs[i], number_holding, tab_reg);
    if (rc == -1) {
      printf("Read register: %ld failed: %s\n", slave_id, modbus_strerror(errno));
      result = 2;
      break;
    }

    float mb_read;
    float mb_value;
    const float scale = scales[i];
    if(number_holding == 2){
      mb_read = (tab_reg[1] << 16) + tab_reg[0];
      mb_value = mb_read*scale;
    }else{
      mb_value = tab_reg[0]*scale;
    }
    values[i] = mb_value;
  }
  modbus_close(ctx);
  modbus_free(ctx);
  return result;
}

int writeMultiModbus(const int count, long *slave_ids, long *modbus_addrs, int *number_holdings, int *values)
{
  modbus_t *ctx = connectModbus();
  if(ctx==NULL) {
    printf("Write multiple modbus, connection failed\n");
    return 1;
  }

  int i, rc;
  int result = 0;
  for(i=0; i<count; i++) {
    int value = values[i];
    const long slave_id = slave_ids[i];
    const int number_holding = number_holdings[i];
    modbus_set_slave(ctx, slave_id);

    if(number_holding == 2) {
      uint16_t test[2];
      test[0] = (value & 0xFFFF);
      test[1] = ((value >> 16) & 0xFFFF);
      rc = modbus_write_registers(ctx, modbus_addrs[i], 2, test);
    }else {
      rc = modbus_write_register(ctx, modbus_addrs[i], value);
    }

    if (rc == -1) {
      printf("Write register: %ld failed: %s\n", slave_id, modbus_strerror(errno));
      result = 2;
      break;
    }
    usleep(WRITE_MODBUS_DELAY_USEC); // Delay
  }
  modbus_close(ctx);
  modbus_free(ctx);
  return result;
}

float writeModbus(long slave_id, long modbus_address, float value)
{
  char modbus_ip[32];
  int modbus_port;
  readModbusConfig(modbus_ip, &modbus_port); // Read modbus config from file

  modbus_t *ctx;
  float res = 1.0;
  int rc;
  ctx = modbus_new_tcp(modbus_ip, modbus_port);
  if (modbus_connect(ctx) == -1) {
    printf("Connection failed write to modbus: %s\n", modbus_strerror(errno));
    res = -1.0;
  }

  modbus_set_slave(ctx, slave_id);
  rc = modbus_write_register(ctx, modbus_address, value);
  if(rc ==-1){
    printf("Error write: %s\n", modbus_strerror(errno));
    res = -1.0;
  }

  modbus_close(ctx);
  modbus_free(ctx);
  return res;
}

float writeModbus32(long slave_id, long modbus_address, int value)
{
  char modbus_ip[32];
  int modbus_port;
  readModbusConfig(modbus_ip, &modbus_port); // Read modbus config from file

  modbus_t *ctx;
  uint16_t test[2];
  float res = 1.0;
  int rc;
  test[0] = (value & 0xFFFF);
  test[1] = ((value >> 16) & 0xFFFF);
  ctx = modbus_new_tcp(modbus_ip, modbus_port);
  if (modbus_connect(ctx) == -1) {
    printf("Connection failed write to modbus: %s\n", modbus_strerror(errno));
    res = -1.0;
  }

  modbus_set_slave(ctx, slave_id);
  rc = modbus_write_registers(ctx, modbus_address, 2, test);
  if(rc ==-1){
    printf("Error write: %s\n", modbus_strerror(errno));
    res = -1.0;
  }

  modbus_close(ctx);
  modbus_free(ctx);
  return res;
}

int setChargerOpMode(int mode, int retry) {
  const int count = 1;
  long slave_ids[] = {COMBOX_MPPT_SLAVE_ADDR}; // MPPT address
  long modbus_addrs[] = {0xAC}; // Operating mode modbus address
  int number_holdings[] = {1}; // uint16
  int values[] = {mode}; // Mode value
  int result = 0;
  for(int i=0; i<retry; i++) {
    result = writeMultiModbus(count, slave_ids, modbus_addrs, number_holdings, values);
    if(result==0) {
      break;
    }
    printf("Set charger operation mode: %d failed with status: %d\n", mode, result);
  }
  return result;
}

int inverterConfiguration(BatteryConfigStruct **batt) {
    printf("Config charger batt type: %d nominal: %f capacity: %f Imax: %f\n",
        (*batt)->type, (*batt)->chg_v, (*batt)->capacity, (*batt)->chg_i_max);
    // MPPT:  BattType, BattCapacity, BattNominal, ChargeCycle
    // XW:    BattType, BattCapacity, MaxBulkChargeI, MaxAbsorptionI, ChargeCycle
    const int count = 10;
    long slave_ids[] = {
      COMBOX_MPPT_SLAVE_ADDR, COMBOX_MPPT_SLAVE_ADDR, COMBOX_MPPT_SLAVE_ADDR, COMBOX_MPPT_SLAVE_ADDR,
      COMBOX_XW_SLAVE_ADDR, COMBOX_XW_SLAVE_ADDR, COMBOX_XW_SLAVE_ADDR, COMBOX_XW_SLAVE_ADDR, COMBOX_XW_SLAVE_ADDR
    };

    long modbus_addrs[] = {
      0x00A5, 0x00A8, 0x00A6, 0x00B9,
      0x0173, 0x0176, 0x020A, 0x020C,
      0x016E, 0x017c
    };

    int number_holdings[] = {
      1, 1, 2, 1,
      1, 1, 2, 2,
      1, 2
    };

    int mppt_batt_type, xw_batt_type;
    { // Get battery type
      const int batt_type = (int)(*batt)->type;
      if(batt_type==4) { // Type Lithium
        mppt_batt_type = 3; // No Lithium in mppt, use custom type 3
        xw_batt_type = 6; // In xw Lithium is type 6
      }else {
        mppt_batt_type = xw_batt_type = batt_type;
      }
    }

    int capacity = (int)(*batt)->capacity; // Batt capacity
    int volt = (int)((*batt)->chg_v*1000); // Batt voltage
    int i_max = (int)((*batt)->chg_i_max*1000); // Batt I max
    int discharge_vol_limit = (int)((*batt)->dischg_v_re*1000);

    int values[] = {
      mppt_batt_type, capacity, volt, CHRG_CYCLE_2_STAGE,
      xw_batt_type, capacity, i_max, i_max,
      CHRG_CYCLE_2_STAGE, discharge_vol_limit
    };
    return writeMultiModbus(count, slave_ids, modbus_addrs, number_holdings, values);
}
