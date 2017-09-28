// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
#include <WProgram.h>
#include "fastio.h"

#define STATUS_OK 0
#define STATUS_SD 1
#define STATUS_ERROR 2
const char* status_str[] = { "Ok", "SD", "Error"};
    
enum ErrorCodes { 
    ERROR_CODE_NO_ERROR = 0, 
    ERROR_CODE_HOTEND_TEMPERATURE, 
    ERROR_CODE_BED_TEMPERATURE, 
    ERROR_CODE_HOTEND_TEMPERATURE_HIGH, 
    ERROR_CODE_BED_TEMPERATURE_HIGH,
    ERROR_CODE_USER_ABORT
  };

const char* error_code_str[] = { "No Error", "Extruder Low", "Bed Low", "Extruder High", "Bed High", "User Abort" };


extern "C" void __cxa_pure_virtual();
void __cxa_pure_virtual(){};
void get_command();
void process_commands();
void reset_status();

void manage_inactivity(byte debug);

void manage_heater();
void wait_for_temp();
int temp2analogu(int celsius, const short table[][2], int numtemps, int source);
int analog2tempu(int raw, const short table[][2], int numtemps, int source);
#ifdef HEATER_USES_THERMISTOR
    #define HEATERSOURCE 1
#endif
#ifdef HEATER_USES_AD595
    #define HEATERSOURCE 2
#endif
#ifdef HEATER_USES_MAX6675
    #define HEATERSOURCE 3
#endif
#ifdef BED_USES_THERMISTOR
    #define BEDSOURCE 1
#endif
#ifdef BED_USES_AD595
    #define BEDSOURCE 2
#endif
#ifdef BED_USES_MAX6675
    #define BEDSOURCE 3
#endif

#define temp2analogh( c ) temp2analogu((c),temptable,NUMTEMPS,HEATERSOURCE)
#define temp2analogBed( c ) temp2analogu((c),bedtemptable,BNUMTEMPS,BEDSOURCE)
#define analog2temp( c ) analog2tempu((c),temptable,NUMTEMPS,HEATERSOURCE)
#define analog2tempBed( c ) analog2tempu((c),bedtemptable,BNUMTEMPS,BEDSOURCE)
#if X_ENABLE_PIN > -1
#define  enable_x() WRITE(X_ENABLE_PIN, X_ENABLE_ON)
#define disable_x() WRITE(X_ENABLE_PIN,!X_ENABLE_ON)
#else
#define enable_x() ;
#define disable_x() ;
#endif
#if Y_ENABLE_PIN > -1
#define  enable_y() WRITE(Y_ENABLE_PIN, Y_ENABLE_ON)
#define disable_y() WRITE(Y_ENABLE_PIN,!Y_ENABLE_ON)
#else
#define enable_y() ;
#define disable_y() ;
#endif
#if Z_ENABLE_PIN > -1
#define  enable_z() WRITE(Z_ENABLE_PIN, Z_ENABLE_ON)
#define disable_z() WRITE(Z_ENABLE_PIN,!Z_ENABLE_ON)
#else
#define enable_z() ;
#define disable_z() ;
#endif
#if E_ENABLE_PIN > -1
#define  enable_e() WRITE(E_ENABLE_PIN, E_ENABLE_ON)
#define disable_e() WRITE(E_ENABLE_PIN,!E_ENABLE_ON)
#else
#define enable_e() ;
#define disable_e() ;
#endif

//defines needed for V3 I2C device

#define V3_I2C_DEVICE          0x48

// Nozzle RGB LED

#define V3_NOZZLE_RED          211    // nozzle RGB LED Red
#define V3_NOZZLE_RED_FLASH    212    // nozzle RGB LED Red Flashing
#define V3_NOZZLE_GREEN        213    // nozzle RGB LED Green 
#define V3_NOZZLE_GREEN_FLASH  214    // nozzle RGB LED Green Flashing
#define V3_NOZZLE_BLUE         215    // nozzle RGB LED Blue
#define V3_NOZZLE_BLUE_FLASH   216    // nozzle RGB LED Blue Flashing
#define V3_NOZZLE_WHITE        217    // nozzle RGB LED White
#define V3_NOZZLE_WHITE_FLASH  218    // nozzle RGB LED White Flashing
#define V3_NOZZLE_ORANGE       219    // nozzle RGB LED Orange
#define V3_NOZZLE_ORANGE_FLASH 220    // nozzle RGB LED Orange Flashing
#define V3_NOZZLE_LED_OFF      221    // nozzle RGB LED off

// front button RGB LEDs

#define V3_BUTTON_RED          222    // button RGB LED Red
#define V3_BUTTON_RED_FLASH    223    // button RGB LED Red Flashing
#define V3_BUTTON_GREEN        224    // button RGB LED Green
#define V3_BUTTON_GREEN_FLASH  225    // button RGB LED Green Flashing
#define V3_BUTTON_BLUE         226    // button RGB LED Blue
#define V3_BUTTON_BLUE_FLASH   227    // button RGB LED Blue Flashing
#define V3_BUTTON_WHITE        228    // button RGB LED White
#define V3_BUTTON_WHITE_FLASH  229    // button RGB LED White Flashing
#define V3_BUTTON_ORANGE       230    // button RGB LED Orange
#define V3_BUTTON_ORANGE_FLASH 231    // button RGB LED Orange Flashing
#define V3_BUTTON_LED_OFF      232    // button RGB LED off ? I havent tested this yet

// beeper

#define V3_3_SHORT_BEEP        233    // Short Beep x 3
#define V3_LONG_BEEP           234    // Long Beep x 1 ( 3 sec)
#define V3_BEEP_FOR_3_MIN      235    // Beep every sec, 3 min.
#define V3_BEEP_OFF            236    // Beep Off
#define V3_SHORT_BEEP          239    // Short Beep x 1

//Hood Switch

#define V3_HOODSWITCH_ENABLE   237    // Hood Switch Enable
#define V3_HOODSWITCH_DISABLE  238    // Hood Switch Disable


void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void linear_move(unsigned long steps_remaining[]);
void do_step(int axis);
void kill(byte debug);

