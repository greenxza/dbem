#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include "db_manager.h"
#include "status_struct.h"
#include "battery_struct.h"
#include "helper.h"
#include "dbem_io.h"
#include "iolib/ads1015.h"

//#define ENABLE_CURRENT_SAMPLING
#define ENABLE_SOC_DELTA                        /**< enable state of charge hysteresis */
#define SELF_DISCHARGE 0.2/100.0                /**< battery self-discharge value */
#define BATT_EFFICIENCY 1                       /**< battery efficiency value */
#define BATT_SOC_DELTA 0.01                     /**< Battery SOC delta value */
#define STATE_OF_CHARGE_DELTA_CHARGING 0.05
#define STATE_OF_CHARGE_DELTA_DISCHARGING 0.05
#define STARTUP_TIME_INDICATOR 60               /**< Seconds that will assume system just reboot */
#define UPDATE_CHARGER_OP_MODE_DELAY 5

#define DBEM_DIRECTORY_PATH  "/home/machinekit/Desktop/workspace/dbem/"     /**< dbem working directory */
#define DATABALE_PATH  "/home/machinekit/Desktop/workspace/dbem/dbem.db"    /**< default database path */
#define MAIN_STATUS_FILE_PATH  "/home/machinekit/Desktop/workspace/dbem/watchdog/main-status.txt"
#define SAVING_SLEEP 60                 /**< sleep duration for saving */
#define MONITORING_SLEEP 3 //10 sec
#define DELTA_T 3                      /**< sleep duration before switching state change */

#define CONFIG_SAVE_STATE_PATH "/home/machinekit/Desktop/workspace/dbem/configSaveState"
#define DEFAULT_LOAD_SAVED_SOC_TIME_LIMIT 1800 // Load saved soc time limit in seconds

// Config VToI data type
enum ConfVToI_t {
    // VToI charge
    VTOI_C_B1_A = 0,
    VTOI_C_B1_B,
    VTOI_C_B2_A,
    VTOI_C_B2_B,
    VTOI_C_B3_A,
    VTOI_C_B3_B,
    // VToI discharge
    VTOI_D_B1_A,
    VTOI_D_B1_B,
    VTOI_D_B2_A,
    VTOI_D_B2_B,
    VTOI_D_B3_A,
    VTOI_D_B3_B,
    // Size
    VTOI_COUNT
};
/**< forward declaration */
void init_system();
void boot_process();
void readAllSensor();
int update_status();
void *active_status(void *vargp);                       /**< thread function for active status to watchdog */
void *monitering_all(void *vargp);                      /**< thread function for monitoring system */
void *saveDatabase(void *vargp);                        /**< thread function for save date to database every period system */
int process_algorithm(                                  /**< charging/discharging algorithm */
    StatusStruct **status);
void lookupSocTable(StatusStruct **status);
static double getSecSinceStartUp(const double def);
float calculateSOC(
    BatteryStruct **batt_addr, float self_discharge,
    float rated_ampere_hour);
void updateUiOpenAllSwitch();
static void recordStartProgram();
static bool resumeSavedState(StatusStruct **storage);
int resetChargerOnDayTime();

StatusStruct *status;                                   /**< dbem status parameters */
bool resumedState = false;                              /**< identifier for resume state status */
bool before_boot_process = true;
pthread_mutex_t status_mutex; // For multi-thread syncronization status

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
int main(int argc, char* argv[]) {
    recordStartProgram();
    init_database(DATABALE_PATH);
    resumedState = false;

    status = malloc(sizeof(StatusStruct));
    status->b1 = malloc(sizeof(BatteryStruct));
    status->b2 = malloc(sizeof(BatteryStruct));
    status->b3 = malloc(sizeof(BatteryStruct));
    status->b1->energy = status->b2->energy = status->b3->energy = 0; // Initialize value
    status->b1->soc = status->b2->soc = status->b3->soc = 0; // Avoid use of uninitialize value
    status->b1->target_soc = status->b2->target_soc = status->b3->target_soc = 0;
    status->b1->target_soc_dischg = status->b2->target_soc_dischg = status->b3->target_soc_dischg = 1;
    status->now_chg = -1;
    status->now_dischg = -1;
    status->prev_log_time = time(NULL); // Current timestamp

#ifdef INIT_PYTHON_ONCE
    setenv("PYTHONPATH",".",1);
    Py_Initialize();
#endif

    boot_process();
    init_system();

    // Thread for monitor sensor value and detect over protection
    pthread_t tid_monitoring;
    if(pthread_create(&tid_monitoring, NULL, monitering_all, NULL)) {
        fprintf(stderr, "Error creating monitoring thread\n");
    }

    // Thread for save data to database every period
    pthread_t tid_saveDatabase;
    if(pthread_create(&tid_saveDatabase, NULL, saveDatabase, NULL)) {
        fprintf(stderr, "Error creating Save Database thread\n");
    }

    // This is main section process algorithm to charge and discharge
    while (1) {
        const int res = process_algorithm(&status);
        sleep(DELTA_T);
    }

    pthread_join( tid_monitoring, NULL);
    pthread_join( tid_saveDatabase, NULL);

    free(status->b1);
    free(status->b2);
    free(status->b3);
    free(status);

#ifdef INIT_PYTHON_ONCE
    Py_Finalize();
#endif

    return 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
void init_system(){
    //Start fans 100% at first start-----------------------------------
    system("chmod a+x dts/compile-dts.sh");
    system("./dts/compile-dts.sh");
    system("/home/machinekit/Desktop/workspace/dbem/setNetwork.sh");
    drive_pwm("pwm_test_P9_31",1, 10000, 0);
    system("echo BB-ADC > /sys/devices/bone_capemgr.9/slots");

    if(resumedState) { // State was resumed skip init system
        // Enable MR in/out
        write_gpio(MR_IN_EN_GPIO, 1);
        write_gpio(MR_OUT_EN_GPIO, 1);

        readAllSensor();
        return;
    }

    openAllSwitch();
    updateUiOpenAllSwitch();
    write_gpio(DL1_EN_GPIO, 0);
    write_gpio(DL2_EN_GPIO, 0);
    write_gpio(DL3_EN_GPIO, 0);
    write_gpio(MR_IN_EN_GPIO, 1);
    write_gpio(MR_OUT_EN_GPIO, 1);
    readAllSensor();
    //Check & update fault status to database if exists
    updateFaultStatus(&status);

    //update last status to sync with GUI
    if(count_current_status()==0) { // Insert 1st record when no current status
        updateMonitoringStatus(&status);
    }

    //insert system status structure to database
    if(count_system_status()==0) { // Insert 1st record when no current status
        saveStatusToDB(&status);
    }

  //Control fan's speed epends on detected temperature
    controlFanByTemp(status->temp1, status->temp2, status->temp3);

    { // Force charge lowest voltage bank
        BatteryStruct *lowestVBank = status->b1;
        for(int i=0; i<2; i++) {
            BatteryStruct *currentBank = (i==0)?status->b2:status->b3;
            if((currentBank->v)<(lowestVBank->v)) {
                lowestVBank = currentBank;
            }
        }

        { // Charge and discharge lowest V bank
            chrgBank(&lowestVBank);
            dischrgBank(&lowestVBank, 1); // Discharge force
        }
    }
}

/** \brief initial state of charge configuration
 *
 * \param
 * \param
 * \return
 *
 */
void boot_process() {
    pthread_mutex_lock(&status_mutex);
    resumedState = false;
    SOCsStruct customSOCs;
    if (retrieve_socs(&customSOCs, TBL_CUSTOM_SETTING)==0) {
        printf("INFO: Boot process with custom SOC state\n");
        status->b1->soc = customSOCs.soc1;
        status->b2->soc = customSOCs.soc2;
        status->b3->soc = customSOCs.soc3;
    }
    else { // Retrieve custom soc failed, check state can resume
        resumedState = resumeSavedState(&status); // Resume saved state from db (success only when app restart after reboot system)
        if(resumedState) {
            printf("INFO: Boot process resumed SOC state\n");
        }
        else { // Reboot without set custom SOC
            if(retrieve_socs(&customSOCs, TBL_CURRENT_STATUS)==0) { // Retrived success
                printf("INFO: Boot process with current SOC state\n");
                status->b1->soc = customSOCs.soc1;
                status->b2->soc = customSOCs.soc2;
                status->b3->soc = customSOCs.soc3;
            }
            else { // No current SOC, get from lookup table
                lookupSocTable(&status);
                printf("INFO: Boot process with lookup SOC in table state\n");
            }
        }
    }
    printf("Boot process SOC B1: %f | B2: %f | B3: %f |\n", status->b1->soc, status->b2->soc, status->b3->soc);
    before_boot_process = false;
    pthread_mutex_unlock(&status_mutex); // Unlock status mutex after finished
}

/** \brief update ui to open all switching states
 *
 * \param
 * \param
 * \return
 *
 */
void updateUiOpenAllSwitch(){
    status->b1->in_en = 0;
    status->b1->out_en = 0;
    status->b2->in_en = 0;
    status->b2->out_en = 0;
    status->b3->in_en = 0;
    status->b3->out_en = 0;
}

/** \brief
 *
 * \param
 * \param
 * \return
 *
 */
float calSOCFromLookupTable(float v) {

    FILE * fp;
    char * line = NULL;
    char * strDoub = NULL;
    char * strDoub2 = NULL;
    char * vSOC = NULL;
    size_t len = 0;
    ssize_t read;
    bool check;
    double a = 0;
    double b = 0;
    float soc = 0.0;

    if(v > 45.48) {
        vSOC = "48V";
    }
    else if(v > 34.11) {
        vSOC = "36V";
    }
    else if(v > 22.748) {
        vSOC = "24V";
    }
    else if(v > 11.374) {
        vSOC = "12V";
    }
    else if(v > 5.681) {
        vSOC = "6V";
    }

    if(vSOC == NULL) {
        printf("\nNominal Voltage out of range.\n");
        vSOC = NULL;
        return -1.0;
    }

    fp = fopen("/home/machinekit/Desktop/workspace/dbem/configSOC", "r");
    check = false;
    while ((read = getline(&line, &len, fp)) != -1) {
        if(check) {
            strDoub = line;
            printf("This SOC: %s \n", line);
            a = strtod(line, &strDoub);
            memmove(&strDoub[0], &strDoub[1], strlen(strDoub) - 1);
            b = strtod(strDoub, &strDoub2);
            printf("Num: %f %f\n", a, b);
            break;
        }
        if(strstr(line, vSOC) != NULL) {
            check = true;
        }
    }

    check = NULL;
    fclose(fp);
    if (line) {
        free(line);
        line = NULL;
        strDoub = NULL;
        strDoub2 = NULL;
        vSOC = NULL;
    }

    if(a == 0 || b == 0) {
        printf("Can not get variable for calculate from configSOC, Please check format or value in file.\n");
        return -1.0;
    }
    printf("Calculated SOC(%f * %f - %f): %f\n",a ,v, b, (a * v + b)/ 100.0);
    soc = (a * v + b) / 100.0;

    if(soc > 1.0) {
        soc = 1.0;
    }
    return soc;
}

/** \brief look up state of charge
 *
 * \param StatusStruct **status_addr
 * \return
 *
 */
void lookupSocTable(StatusStruct **status_addr){
    (*status_addr)->b1->soc = calSOCFromLookupTable((*status_addr)->b1->v);
    (*status_addr)->b2->soc = calSOCFromLookupTable((*status_addr)->b2->v);
    (*status_addr)->b3->soc = calSOCFromLookupTable((*status_addr)->b3->v);
}

/** \brief save parameters to database
 *
 * \param void *vargp
 * \param
 * \return
 *
 */
void *saveDatabase(void *vargp){
    float *samplingRate;
    samplingRate = malloc(sizeof(float));
    status->prev_log_time = time(NULL); // Current timestamp
    while(1) {
        pthread_mutex_lock(&status_mutex); // Lock status mutex before use status
        get_limit_log(&samplingRate);
        printf("Samplingss: %f\n", (*samplingRate));
        if(before_boot_process) {
            printf("Update status after boot_process\n");
        }
        else {
            update_status();
        }
        float sleepMonitor = 60 * (*samplingRate);
        printf("Sleep time montor: %d\n", (int)sleepMonitor);
        pthread_mutex_unlock(&status_mutex); // Unlock status mutex after finished
        sleep((int)sleepMonitor); //SAVING_SLEEP should get from UI setting
    }
    free(samplingRate);
}

/** \brief this function is used for watchdog process
 *
 * \param void *vargp
 * \return
 *
 */
void *active_status(void *vargp) {
    int i=0;
    FILE *fp;
    while (1) {
        //Write 1 (Active) to tell watchdog service
        fp = fopen(MAIN_STATUS_FILE_PATH, "w+");
        if (fp == NULL) {
            printf("Error opening main-status file!\n");
        }
        fprintf(fp, "%d", 1); //write 1(active) to file
        fclose(fp);
        sleep(10);
    }
}

/** \brief Read all config VtoI to params (size VTOI_COUNT)
 *
 * \param float *values
 * \return
 *
 */
static void readConfigVtoI(float *values) {
    FILE *fp = fopen("/home/machinekit/Desktop/workspace/dbem/configVtoI", "r");
    if(!fp) {
        printf("Invalid ConfigVtoI file\n");
        return;
    }

    static const int statCount = 2; // Charge and discharge
    static const int bankCount = 3; // Bank count
    static const int posCount = 2; // Parse number position only a and b

    char * line = NULL;
    char * strDoub = NULL;
    size_t len = 0;

    int bankIndex = 0;
    int statIndex = -1; // Status index charge:0 | discharge:1
    while (getline(&line, &len, fp) != -1) {
        if(strstr(line, "Charge")!=NULL) {
            statIndex = 0;
            bankIndex = 0;
            continue;
        }
        else if(strstr(line, "Discharge")!=NULL) {
            statIndex = 1;
            bankIndex = 0;
            continue;
        }

        if(statIndex < 0 || statIndex >= statCount || bankIndex < 0 || bankIndex >= bankCount) {
            continue;
        }

        int pos = 0;
        strDoub = strtok(line, "=");
        while(strDoub != NULL) {
            strDoub = strtok(NULL, ",");
            if(strDoub != NULL) {
                const int index = (statIndex * (statCount*bankCount)) + (bankIndex * statCount) + (pos);
                values[index] = atof(strDoub);
                if(++pos >= posCount) { // Allow only 2 position (a and b)
                    break;
                }
            }
        }
        bankIndex++;
    }

    fclose(fp);
    if(line) {
        free(line);
    }
}

/** \brief average current
 *
 * \param float currents[]
 * \param float new_current
 * \return average value of current
 *
 */
float average_last5_current(float currents[], float new_current) {
    int num_arr = 5;
    float sum = 0.0;

    for (int index = 0; index < num_arr; index++) {
        printf("%f ", currents[index]);
    }

    //put new current to last index array fist in last out

    for (int k = 0; k < num_arr - 1; k++) {
        currents[k] = currents[k + 1];
    }
    currents[num_arr - 1] = new_current;

    //average current array
    for (int i = 0; i < num_arr; i++) {
        sum += currents[i];
    }
    float average = sum / num_arr;
    return average;
}

/** \brief monitoring function
 *
 * \param void *vargp
 * \return
 *
 */
void *monitering_all(void *vargp){
    // these variables below are used to store a reading data temporary to prevent reading error
    float p_pv, p_load;
    float v1,v2,v3;
    float i_in1,i_in2,i_in3;
    float i_out1,i_out2,i_out3;
    float comb_i_in, comb_i_out;
    float temp1,temp2,temp3;
    float temp_heatsink1,temp_heatsink2,temp_inside_air;
    float confVtoI[VTOI_COUNT];

    float arr_i_in1[5] = {0.0};
    float arr_i_out1[5] = {0.0};
    float arr_i_in2[5] = {0.0};
    float arr_i_out2[5] = {0.0};
    float arr_i_in3[5] = {0.0};
    float arr_i_out3[5] = {0.0};

    int v_counter = 0;
    while(1) {
        //tempolary variables ---------------------------------------
        // these variables below are used to store a reading data temporary to prevent reading error
        readConfigVtoI(confVtoI); // Read config VtoI from file

        pthread_mutex_lock(&status_mutex); // Lock status mutex, before use status

        i_in1 = getBattChrgCurrent(1, confVtoI[VTOI_C_B1_A], confVtoI[VTOI_C_B1_B]);
        i_out1 = getBattDisChrgCurrent(1, confVtoI[VTOI_D_B1_A], confVtoI[VTOI_D_B1_B]);
        temp1 = getBattTemp(1);

        i_in2 = getBattChrgCurrent(2, confVtoI[VTOI_C_B2_A], confVtoI[VTOI_C_B2_B]);
        i_out2 = getBattDisChrgCurrent(2, confVtoI[VTOI_D_B2_A], confVtoI[VTOI_D_B2_B]);
        temp2 = getBattTemp(2);

        i_in3 = getBattChrgCurrent(3, confVtoI[VTOI_C_B3_A], confVtoI[VTOI_C_B3_B]);
        i_out3 = getBattDisChrgCurrent(3, confVtoI[VTOI_D_B3_A], confVtoI[VTOI_D_B3_B]);
        temp3 = getBattTemp(3);

        temp_heatsink1 = getTempHeatSink1();
        temp_heatsink2 = getTempHeatSink2();
        temp_inside_air = getTempInsideAir();

        if(v_counter <= 0) {
            v1 = getBattVoltage(1);
            v2 = getBattVoltage(2);
            v3 = getBattVoltage(3);
            v_counter = 3;

            // pthread_mutex_lock(&status_mutex); // Lock status mutex, before use status
            if(v1 != READ_ADC_ERROR_VALUE) {
                status->b1->v = v1;
            }
            if(v2 != READ_ADC_ERROR_VALUE) {
                status->b2->v = v2;
            }
            if(v3 != READ_ADC_ERROR_VALUE ) {
                status->b3->v = v3;
            }
        }
        else {
            v_counter--;
            // pthread_mutex_lock(&status_mutex); // Lock status mutex, before use status
        }

        //Bank 1 ------------------------------------------------------
        status->b1->bank_id = 1;
        if(i_in1 != READ_ADC_ERROR_VALUE) {
#ifdef ENABLE_CURRENT_SAMPLING
            float avg_i_in1 = average_last5_current(arr_i_in1, i_in1);
            status->b1->i_in = avg_i_in1;
#else
            status->b1->i_in = i_in1;
#endif
        }
        if(i_out1 != READ_ADC_ERROR_VALUE ) {
#ifdef ENABLE_CURRENT_SAMPLING
            float avg_i_out1 = average_last5_current(arr_i_out1, i_out1);
            status->b1->i_out = avg_i_out1;
#else
            status->b1->i_out = i_out1;
#endif
        }
        if(temp1 != READ_ADC_ERROR_VALUE ) {
            status->b1->temp = temp1;
        }

        //Bank 2 ------------------------------------------------------
        status->b2->bank_id = 2;
        if(i_in2 != READ_ADC_ERROR_VALUE) {
#ifdef ENABLE_CURRENT_SAMPLING
            float avg_i_in2 = average_last5_current(arr_i_in2, i_in2);
            status->b2->i_in = avg_i_in2;
#else
            status->b2->i_in = i_in2;
#endif
        }
        if(i_out2 != READ_ADC_ERROR_VALUE ) {
#ifdef ENABLE_CURRENT_SAMPLING
            float avg_i_out2 = average_last5_current(arr_i_out2, i_out2);
            status->b2->i_out = avg_i_out2;
#else
            status->b2->i_out = i_out2;
#endif
        }
        if(temp2 != READ_ADC_ERROR_VALUE ) {
            status->b2->temp = temp2;
        }

        //Bank 3 ------------------------------------------------------
        status->b3->bank_id = 3;
        if(i_in3 != READ_ADC_ERROR_VALUE) {
#ifdef ENABLE_CURRENT_SAMPLING
            float avg_i_in3 = average_last5_current(arr_i_in3, i_in3);
            status->b3->i_in = avg_i_in3;
#else
            status->b3->i_in = i_in3;
#endif
        }
        if(i_out3 != READ_ADC_ERROR_VALUE ){
#ifdef ENABLE_CURRENT_SAMPLING
            float avg_i_out3 = average_last5_current(arr_i_out3, i_out3);
            status->b3->i_out = avg_i_out3;
#else
            status->b3->i_out = i_out3;
#endif
        }
        if(temp3 != READ_ADC_ERROR_VALUE ){
            status->b3->temp = temp3;
        }

        // Get temp inside of DBEM case
        if(temp_heatsink1 != READ_ADC_ERROR_VALUE ){
            status->temp1 = temp_heatsink1;
        }
        if(temp_heatsink2 != READ_ADC_ERROR_VALUE ){
            status->temp2 = temp_heatsink2;
        }
        if(temp_inside_air != READ_ADC_ERROR_VALUE ){
            status->temp3 = temp_inside_air;
        }

        { // Read modbus
            status->modbus_err = readDataFromModbus(&p_pv, &p_load, &comb_i_in, &comb_i_out);
            status->p_pv = p_pv;
            status->p_load = p_load;
            status->b1->comb_i_in = status->b2->comb_i_in = status->b3->comb_i_in = comb_i_in;
            status->b1->comb_i_out = status->b2->comb_i_out = status->b3->comb_i_out = comb_i_out;
        }

        // printf("Before read PPV \n");
        // p_pv = getPPV();
        // printf("After read PPV and Before read pload\n");
        // usleep(DELAY_READ_COMBOX); // Read combox delay
        // p_load = getPLoad(0.0);
        // printf("After read pload\n");

        // // Get charge/discharge current from combox
        // comb_i_in = comb_i_out = 0.0; // Reset
        // usleep(DELAY_READ_COMBOX); // Read combox delay
        // getCombBattI(&comb_i_in, &comb_i_out); // Get charge/discharge current from combox
        // status->b1->comb_i_in = status->b2->comb_i_in = status->b3->comb_i_in = comb_i_in;
        // status->b1->comb_i_out = status->b2->comb_i_out = status->b3->comb_i_out = comb_i_out;

        // status->p_pv = p_pv < 0? 0.0 : p_pv;
        // status->p_load = p_load < 0? 0.0 : p_load;

        if(before_boot_process) {
            printf("Update monitoring status after boot_process\n");
        }
        else {
            updateMonitoringStatus(&status);
        }

        //Control fans depends on temp
        controlFanByTemp(status->temp1, status->temp2, status->temp3);

        // if (status->p_pv <= 0 && status->p_load <= 0) {
        //   printf("Script monitering all in this round cause PPV(%d) less than 0 or PLoad(%d) less than 0\n", status->p_pv, status->p_load);
        //   pthread_mutex_unlock(&status_mutex); // Unlock status mutex before sleep
        //   sleep(MONITORING_SLEEP);
        //   continue;
        // }
        BatteryConfigStruct *battery_config1;
        BatteryConfigStruct *battery_config2;
        BatteryConfigStruct *battery_config3;

        battery_config1 = malloc(sizeof(BatteryConfigStruct));
        battery_config2 = malloc(sizeof(BatteryConfigStruct));
        battery_config3 = malloc(sizeof(BatteryConfigStruct));

        retrieve_config(&battery_config1, 1);
        retrieve_config(&battery_config2, 2);
        retrieve_config(&battery_config3, 3);
        // printf("readChargerIDcVolt: %f\n", readChargerIDcVolt());
        printf("Monitoring: V1batt=%f | V2batt=%f | V3batt=%f | \n", status->b1->v, status->b2->v, status->b3->v);
        printf("Monitoring: V1=%f | V2=%f | V3=%f |\n", v1,v2,v3);
        printf("Monitoring: From sensor I_in1=%f | I_in2=%f | I_in3=%f \n", status->b1->i_in, status->b2->i_in, status->b3->i_in);
        printf("Monitoring: From sensor I_out1=%f | I_out2=%f | I_out3=%f \n", status->b1->i_out, status->b2->i_out, status->b3->i_out);
        printf("Monitoring: Temp_B1=%f | Temp_B2=%f | Temp_B3=%f |\n", temp1,temp2,temp3);
        printf("Monitoring: Temp_H1=%f | Temp_H2=%f | Temp_Air=%f |\n", temp_heatsink1,temp_heatsink2,temp_inside_air);
        printf("Monitoring: Modbus status: %d\n", status->modbus_err);
        printf("Monitoring: Ppv=%d | Pload=%d |\n", status->p_pv, status->p_load);
        printf("Monitoring: Combox Iin=%f Iout=%f\n", comb_i_in, comb_i_out);
        printf("Monitoring: SOC1=%f | SOC2=%f | SOC3=%f |\n", status->b1->soc,status->b2->soc,status->b3->soc);
        printf(
            "Monitoring: I_COMB=%f | I_COMB=%f | I_COMB=%f |\n",
            status->b1->comb_i_in,status->b2->comb_i_in,status->b3->comb_i_in);
        printf(
            "Monitoring: I_MIN_CHG=%f | I_MIN_CHG=%f | I_MIN_CHG=%f |\n",
            battery_config1->chg_i_min, battery_config2->chg_i_min, battery_config3->chg_i_min);
        printf(
            "Monitoring: I_MAX_CHG=%f | I_MAX_CHG=%f | I_MAX_CHG=%f |\n",
            battery_config1->chg_i_max, battery_config2->chg_i_max, battery_config3->chg_i_max);
        printf(
            "Monitoring: I_MIN_DCHG=%f | I_MIN_DCHG=%f | I_MIN_DCHG=%f |\n",
            battery_config1->dischg_i_min, battery_config2->dischg_i_min, battery_config3->dischg_i_min);
        printf(
            "Monitoring: I_MAX_DCHG=%f | I_MAX_DCHG=%f | I_MAX_DCHG=%f |\n",
            battery_config1->dischg_i_max, battery_config2->dischg_i_max, battery_config3->dischg_i_max);
        free(battery_config1);
        free(battery_config2);
        free(battery_config3);

        int false_status = updateFaultStatus(&status);
        printf("Monitoring in_en: B1=%d B2=%d B3=%d\n", status->b1->in_en, status->b2->in_en, status->b3->in_en);
        printf("Monitoring out_en: B1=%d B2=%d B3=%d\n", status->b1->out_en, status->b2->out_en, status->b3->out_en);
        pthread_mutex_unlock(&status_mutex); // Unlock status mutex before sleep
        sleep(MONITORING_SLEEP);
    }
}

/** \brief update status to database
 *
 * \param
 * \param
 * \return
 *
 */
int update_status() {
  //insert system status structure to database
    saveStatusToDB(&status);
    return 0;
}

/** \brief update soc of every battery group
 *
 * \param StatusStruct **status
 * \return
 *
 */
void updateAllSOC(StatusStruct **status) {
    int e_charges [] = {9, 9, 5}; // Default e_charges value
    retrieve_batt_e_charge(e_charges);
    // printf("e_charge bank#1: %d bank#2: %d bank#3: %d\n", e_charges[0], e_charges[1], e_charges[2]);

    // SoC is limit to 1.0
    (*status)->b1->soc = MIN(1.0, calculateSOC(&((*status)->b1), SELF_DISCHARGE, e_charges[0]));
    (*status)->b2->soc = MIN(1.0, calculateSOC(&((*status)->b2), SELF_DISCHARGE, e_charges[1]));
    (*status)->b3->soc = MIN(1.0, calculateSOC(&((*status)->b3), SELF_DISCHARGE, e_charges[2]));
    // printf("SOC B1:%f | B2:%f | B3:%f |\n", (*status)->b1->soc, (*status)->b2->soc, (*status)->b3->soc);
}

/** \brief sort state of charge from highest to lowest
 *
 * \param
 * \param
 * \return sort index
 *
 */
int *sortSocDESC(StatusStruct **status) {
    float soc_bank[3] = {(*status)->b1->soc, (*status)->b2->soc, (*status)->b3->soc};
    int *sort_index = (int*)malloc(sizeof(int)*3);
    sort_index[0] = 1;
    sort_index[1] = 2;
    sort_index[2] = 3;
    sortBiggestToSmallest(sort_index, soc_bank);
    printf("INFO: Sorted SOC B%d:%.10f B%d:%.10f B%d:%.10f\n",
           sort_index[0], soc_bank[0], sort_index[1], soc_bank[1], sort_index[2], soc_bank[2]);
    return sort_index;
}

/** \brief update prediction bank
 *
 * \param StatusStruct **status:
 * \param
 * \return
 *
 */
void updatePredictBank(StatusStruct **status) {
    {
        int *sorted_index = sortSocDESC(status);
        BatteryStruct *batts[3]; // Sorted batts by SoC DESC
        for(int i=0;i<3;i++) {
            switch(sorted_index[i]) {
                case 1:
                    batts[i] = (*status)->b1;
                    break;
                case 2:
                    batts[i] = (*status)->b2;
                    break;
                case 3:
                    batts[i] = (*status)->b3;
                    break;
                default:
                    break;
            }
        }
        free(sorted_index);

        int next_dischrg = batts[0]->out_en?batts[1]->bank_id:batts[0]->bank_id; // Highest SoC is discharging, discharge next high
        int next_chrg = batts[2]->in_en?batts[1]->bank_id:batts[2]->bank_id; // Lowest SoC is charging, charg next low
        printf("INFO: Update predict next charge=%d, next discharge=%d\n", next_chrg, next_dischrg);
        (*status)->next_dischrg = next_dischrg;
        (*status)->next_chrg = next_chrg;
    }

    // int next_dischrg = getBankHighestSOC(status);
    // int next_chrg = getBankLowestSOC(status);
    // (*status)->next_dischrg = next_dischrg;
    // (*status)->next_chrg = next_chrg;
    // printf("Status next_dischrg:%d next_chrg:%d \n", (*status)->next_dischrg, (*status)->next_chrg);
}

/** \brief check power before charge or discharge
 *
 * \param float pv_power
 * \param float load_power
 * \return
 *
 */
int preCondition(float pv_power, float load_power) {
    if (pv_power <= 0 && load_power <= 0) {
        printf("INFO: No PV and load power\n");
        resetChargerOnDayTime();
        return 1;
    }
    else {
        return 0;
    }
}

 /** \brief function to perform charging and discharging
  *
  * \param status is system parameters
  * \param deltaTime is seconds from previous loop (use for calculate SoC)
  * \return 2 when no need to sleep before next loop
  *
  */
int process_algorithm(StatusStruct **status) {
    pthread_mutex_lock(&status_mutex);
    updateAllSOC(status);
    float pv_power = (*status)->p_pv;
    float load_power = (*status)->p_load;
    bool compare_power = load_power > pv_power;
    printf("INFO: SOC B1:%f | B2:%f | B3:%f |\n",
        (*status)->b1->soc, (*status)->b2->soc, (*status)->b3->soc);
    if (preCondition(pv_power, load_power)) {
        updatePredictBank(status);
        pthread_mutex_unlock(&status_mutex);
        return 0;
    }

    if(compare_power) {
        printf("INFO: Start discharging process\n");
        write_gpio(MR_OUT_EN_GPIO, 1);
        open_all_input_switch();
        (*status)->b1->in_en = 0;
        (*status)->b2->in_en = 0;
        (*status)->b3->in_en = 0;

        int bank_to_dischrg = batteryDischargingSelect(status);
        BatteryStruct *batt = NULL;
        switch(bank_to_dischrg) {
            case 1:
                batt = (*status)->b1;
                break;
            case 2:
                batt = (*status)->b2;
                break;
            case 3:
                batt = (*status)->b3;
                break;
            case 4:
                batt = (*status)->b1;
                break;
            case 5:
                batt = (*status)->b2;
                break;
            case 6:
                batt = (*status)->b3;
                break;
            default:
                break;
        }

        if(batt==NULL) {
            printf("ERROR: Invalid battery group#%d to discharge\n", bank_to_dischrg);
            open_all_output_switch();
            (*status)->b1->out_en = 0;
            (*status)->b2->out_en = 0;
            (*status)->b3->out_en = 0;
            write_gpio(MR_IN_EN_GPIO, 0);
            updatePredictBank(status);
            pthread_mutex_unlock(&status_mutex);
            return 1;
        }
        BatteryConfigStruct *battery_config;
        battery_config = malloc(sizeof(BatteryConfigStruct));
        retrieve_config(&battery_config, batt->bank_id);
        inverterConfiguration(&battery_config);
        free(battery_config);

        if(pv_power <= 0) { // 20190625 Requirement: update charger mode by time when ppv is 0
            if(resetChargerOnDayTime()) {
                write_gpio(MR_IN_EN_GPIO, 0);
            }
            else {
                printf("INFO: Reset charger and enable input relay\n");
                write_gpio(MR_IN_EN_GPIO, 1);
                chrgBank(&batt);
            }
        }
        else {
            write_gpio(MR_IN_EN_GPIO, 0);
        }
        dischrgBank(&batt, 0); // Discharge not force
        if (bank_to_dischrg <= 3) {
            float update_target_soc = batt->soc - (float)BATT_SOC_DELTA;
            if (update_target_soc <= 0) {
                update_target_soc = 1;
            }
            (*status)->b1->target_soc_dischg = (*status)->b2->target_soc_dischg = (*status)->b3->target_soc_dischg = update_target_soc;
            printf("INFO: Update target SOC: %.10f\n", batt->target_soc_dischg);
            (*status)->now_dischg = batt->bank_id;
            printf("INFO: Update new discharging status to bank %d\n", batt->bank_id);
        } else {
            printf("INFO: Remain state");
        }
        open_other_output_switch(batt->bank_id, status);
        updatePredictBank(status);
        pthread_mutex_unlock(&status_mutex);
        return 2;
    }
    else {
        printf("INFO: Start charging process.\n");
        float i_pv_remain = getIPvRemain(status);
        printf("INFO: Ramain PV current: %.2f\n", i_pv_remain);
        int bank_to_chrg = batteryChargingSelect(i_pv_remain, status);
        write_gpio(MR_IN_EN_GPIO, 1);
        write_gpio(MR_OUT_EN_GPIO, 1);
        BatteryStruct *batt = NULL;
        switch(bank_to_chrg) {
            case 1:
                batt = (*status)->b1;
                break;
            case 2:
                batt = (*status)->b2;
                break;
            case 3:
                batt = (*status)->b3;
                break;
            case 4:
                batt = (*status)->b1;
                break;
            case 5:
                batt = (*status)->b2;
                break;
            case 6:
                batt = (*status)->b3;
                break;
            default:
                break;
        }
        if(batt==NULL) {
            printf("ERROR: Invalid battery bank#%d to charge\n", bank_to_chrg);
            open_all_input_switch();
            (*status)->b1->in_en = 0;
            (*status)->b2->in_en = 0;
            (*status)->b3->in_en = 0;
            write_gpio(MR_IN_EN_GPIO, 0);
            updatePredictBank(status); //For state prediction (the next dischrg will be the biggest voltage)
            pthread_mutex_unlock(&status_mutex);
            return 1;
        }
        chrgBank(&batt);
        dischrgBank(&batt, 1); // Discharge force
        if (bank_to_chrg <= 3) {
            (*status)->b1->target_soc = (*status)->b2->target_soc = (*status)->b3->target_soc = MIN(batt->soc + (float)BATT_SOC_DELTA, 1.0);
            printf("INFO: Update target SOC: %.10f\n", batt->target_soc);
            (*status)->now_chg = batt->bank_id;
            printf("INFO: Update new charging status to bank %d\n", batt->bank_id);
            (*status)->now_dischg = 0;
        } else {
            printf("INFO: Remain state");
        }
        sleep(1);
        open_other_input_switch(batt->bank_id, status);
        open_other_output_switch(batt->bank_id, status);
        updatePredictBank(status); //For state prediction (the next dischrg will be the biggest voltage)
        pthread_mutex_unlock(&status_mutex); // Unlock status mutex before sleep
        return 2;
    }
    pthread_mutex_unlock(&status_mutex); // Unlock status mutex before sleep
    return 0;
}

/** \brief state of charge calculation
 *
 * \param BatteryStruct **batt_addr:
 * \param float self_discharge:
 * \param float battery_efficiency:
 * \param float rated_ampere_hour:
 * \param const int deltaTime:
 * \return value of state of charge
 *
 */
float calculateSOC(
        BatteryStruct **batt_addr, float self_discharge,
        float rated_ampere_hour) {
    float soc;
    float soc_prev = (*batt_addr)->soc;
    float Vbatt = (*batt_addr)->v;
    float i_in = getBattIin(*batt_addr);
    float i_out = getBattIout(*batt_addr);
    BatteryConfigStruct *battery_config;
    battery_config = malloc(sizeof(BatteryConfigStruct));
    retrieve_config(&battery_config, (*batt_addr)->bank_id);
    float Ibatt = i_in - i_out;
    float Pbatt = Vbatt*Ibatt; //P = V ⋅ I
    float Vrate = battery_config->chg_v;
    float first = 1 - ((self_discharge*(float)DELTA_T) / (24*3600));
    float seconds1 = (float)BATT_EFFICIENCY * Pbatt * (float)DELTA_T / 3600;
    float seconds2 = rated_ampere_hour * Vrate;
    float seconds = seconds1/seconds2;
  // For BBB connection refused from combox soc_prev get -1
    if(soc_prev < 0) {
        soc_prev = 0;
    }
    soc = (soc_prev * first) + seconds;
    printf(
        "INFO: calculate SOC on bank %d soc_prev=%.10f, soc=%.10f\n",
        (*batt_addr)->bank_id, soc_prev, soc);
    printf("INFO: ===Info Value[SOC]===\n");
    printf("INFO: first=%.10f\n", first);
    printf("INFO: seconds=%.10f\n", seconds);
    free(battery_config);
    return soc;
}

/** \brief read sensors
 *
 * \param
 * \param
 * \return
 *
 */
void readAllSensor() {
    float confVtoI[VTOI_COUNT];
    readConfigVtoI(confVtoI); // Read config VtoI from file

    status->b1->bank_id = 1;
    if(!resumedState) { // Resumed state will resume in_en and out_en from db
        status->b1->in_en = 0;
        status->b1->out_en = 0;
    }
    status->b1->i_in = getBattChrgCurrent(1, confVtoI[VTOI_C_B1_A], confVtoI[VTOI_C_B1_B]);
    status->b1->i_out = getBattDisChrgCurrent(1, confVtoI[VTOI_D_B1_A], confVtoI[VTOI_D_B1_B]);
    status->b1->v = getBattVoltage(1);
    status->b1->temp = getBattTemp(1);

    status->b2->bank_id = 2;
    if(!resumedState) { // Resumed state will resume in_en and out_en from db
        status->b2->in_en = 0;
        status->b2->out_en = 0;
    }
    status->b2->i_in = getBattChrgCurrent(2, confVtoI[VTOI_C_B2_A], confVtoI[VTOI_C_B2_B]);
    status->b2->i_out = getBattDisChrgCurrent(2, confVtoI[VTOI_D_B2_A], confVtoI[VTOI_D_B2_B]);
    status->b2->v = getBattVoltage(2);
    status->b2->temp = getBattTemp(2);

    status->b3->bank_id = 3;
    if(!resumedState) { // Resumed state will resume in_en and out_en from db
        status->b3->in_en = 0;
        status->b3->out_en = 0;
    }
    status->b3->i_in = getBattChrgCurrent(3, confVtoI[VTOI_C_B3_A], confVtoI[VTOI_C_B3_B]);
    status->b3->i_out = getBattDisChrgCurrent(3, confVtoI[VTOI_D_B3_A], confVtoI[VTOI_D_B3_B]);
    status->b3->v = getBattVoltage(3);
    status->b3->temp = getBattTemp(3);

    // Get temp inside of DBEM case
    status->temp1 = getTempHeatSink1();
	status->temp2 = getTempHeatSink2();
	status->temp3 = getTempInsideAir();
    status->prev_log_time = time(NULL); // Current timestamp

    { // Read modbus
        float p_pv, p_load, comb_i_in, comb_i_out;
        status->modbus_err = readDataFromModbus(&p_pv, &p_load, &comb_i_in, &comb_i_out);
        status->p_pv = p_pv;
        status->p_load = p_load;
        status->b1->comb_i_in = status->b2->comb_i_in = status->b3->comb_i_in = comb_i_in;
        status->b1->comb_i_out = status->b2->comb_i_out = status->b3->comb_i_out = comb_i_out;
    }

  // float p_pv = getPPV();
  // status->p_pv = (p_pv<0?0.0:p_pv);
  // usleep(DELAY_READ_COMBOX); // Read combox delay
  // // sleep(1); //when read modbus should delay 1 second
  // float p_load = getPLoad(0.0);
  // status->p_load = (p_load<0?0.0:p_load);
  // float comb_i_in = 0.0;
  // float comb_i_out = 0.0;
  // usleep(DELAY_READ_COMBOX); // Read combox delay
  // getCombBattI(&comb_i_in, &comb_i_out); // Get charge/discharge current from combox
  // status->b1->comb_i_in = status->b2->comb_i_in = status->b3->comb_i_in = comb_i_in;
  // status->b1->comb_i_out = status->b2->comb_i_out = status->b3->comb_i_out = comb_i_out;
}

/** \brief get uptime
 *
 * \param const double def
 * \return seconds since system start up
 *
 */
static double getSecSinceStartUp(const double def) {
    double sec = def;
    FILE *file = fopen("/proc/uptime", "r");
    if(file) {
        char *line = NULL;
        size_t len = 0;
        if(getline(&line, &len, file)!=-1) {
            sec = atof(line);
        }
        free(line);
        fclose(file);
    }
    return sec;
}

/** \brief Record timestamp when program starts
 *
 * \return
 *
 */
static void recordStartProgram() {
    const char *startProcessPath = "/home/machinekit/Desktop/workspace/dbem/startProcess";

    struct stat st;
    if(stat(startProcessPath, &st) == 0) {
        const int size = st.st_size;
        if(size > 4096) { // Backup file when larger than 4Kb
            char backUpPath[100];
            strcpy(backUpPath, startProcessPath);
            strcat(backUpPath, ".backup");
            rename(startProcessPath, backUpPath);
        }
    }

    FILE *file = fopen(startProcessPath, "a+");
    if(file) {
        time_t rawtime;
        struct tm *timeinfo;
        time (&rawtime);
        timeinfo = localtime (&rawtime);
        fprintf(file, "Start at: %s\n", asctime(timeinfo));
        fclose(file);
    }
    else {
        printf("Cannot open file\n");
    }
}

/** \brief saved state from db
 *
 * \param StatusStruct **storage
 * \return true when state was load else false
 *
 */
static bool resumeSavedState(StatusStruct **storage) {
    bool loaded = false;
    const int timeLimit = (int)getSecSinceStartUp(DEFAULT_LOAD_SAVED_SOC_TIME_LIMIT); // Seconds since system start up
    StatusStruct *state = malloc(sizeof(StatusStruct)); // Load state
    state->b1 = malloc(sizeof(BatteryStruct));
    state->b2 = malloc(sizeof(BatteryStruct));
    state->b3 = malloc(sizeof(BatteryStruct));
    if(retrieve_last_status(&state, timeLimit)==0) { // Load last state saved after boot up
        printf("Resume state from db\n");
        (*storage)->b1->soc = state->b1->soc;
        (*storage)->b2->soc = state->b2->soc;
        (*storage)->b3->soc = state->b3->soc;
        // Charge state
        (*storage)->b1->in_en = state->b1->in_en;
        (*storage)->b2->in_en = state->b2->in_en;
        (*storage)->b3->in_en = state->b3->in_en;
        // Discharge state
        (*storage)->b1->out_en = state->b1->out_en;
        (*storage)->b2->out_en = state->b2->out_en;
        (*storage)->b3->out_en = state->b3->out_en;
        loaded = true;
    }
    free(state->b3);
    free(state->b2);
    free(state->b1);
    free(state);
    return loaded;
}

/** \brief reset charger
 *
 * \return 0 if reset, 1 if out of range
 *
 */
int resetChargerOnDayTime() {
    int hour = 0;
    get_time_of_day(&hour, NULL, NULL);
    if(hour < 5 || hour > 18) {
        return 1;
    }
    setChargerOpMode(CHRG_OP_MODE_STANDBY, 3); // Set charger operating to standby (retry limit 31 times)
    sleep(UPDATE_CHARGER_OP_MODE_DELAY); // Sleep before set operating
    setChargerOpMode(CHRG_OP_MODE_OPERATING, 3); // Set charger operating to operating (retry limit 3 times)
    return 0;
}
