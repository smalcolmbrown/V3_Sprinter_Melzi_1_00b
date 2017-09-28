// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL
// last update date: 2017-4-26
// Additions and bug fixes by Suusi Malcolm-Brown 
// https://github.com/smalcolmbrown/V3_Sprinter_Melzi_1_00b/

#include "fastio.h"
//#include "Configuration.h"
//#include "pins.h"
#include "Configuration.h"
#include "Sprinter.h"
#include "SerialManager.h"
#include <EEPROM.h>
#include <Wire.h>

#ifdef SDSUPPORT
#include "SdFat.h"
#endif

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G4  - Dwell S<seconds> or P<milliseconds>
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M80  - Turn on Power Supply
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move,
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M115	- Capabilities string
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000)
// M203 - Adjust Z height

//0x03, 0x01, 0x10
char PauseID = 0x03; //rp3d.com pause id
char HSW_Enable = 0x01; //rp3d.com M237, M238
int FSW_Counter = 0; //rp3d.com Front Switch Counter
int FSW_status = 1; //rp3d.com Front Switch Status



//Printer status variables
int status = STATUS_OK; //
int error_code = ERROR_CODE_NO_ERROR; //0=Nothing, 1=Heater thermistor error

//Led counter (for blinking the led in different timings)
int led_counter = 0;

//Stepper Movement Variables

char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
bool move_direction[NUM_AXIS];
unsigned long axis_previous_micros[NUM_AXIS];
unsigned long previous_micros = 0, previous_millis_heater, previous_millis_bed_heater, previous_millis_PauseID;
unsigned long move_steps_to_take[NUM_AXIS];
#ifdef RAMP_ACCELERATION
unsigned long axis_max_interval[NUM_AXIS];
unsigned long axis_steps_per_sqr_second[NUM_AXIS];
unsigned long axis_travel_steps_per_sqr_second[NUM_AXIS];
unsigned long max_interval;
unsigned long steps_per_sqr_second, plateau_steps;
#endif
boolean acceleration_enabled = false, accelerating = false;
unsigned long interval;
float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
unsigned long steps_taken[NUM_AXIS];
long axis_interval[NUM_AXIS]; // for speed delay
bool home_all_axis = true;
int feedrate = 1500, next_feedrate, saved_feedrate;
float time_for_move;
long gcode_N, gcode_LastN;
bool relative_mode = false;  //Determines Absolute or Relative Coordinates
bool relative_mode_e = false;  //Determines Absolute or Relative E Codes while in Absolute Coordinates mode. E is always relative in Relative Coordinates mode.
long timediff = 0;
//experimental feedrate calc
float d = 0;
float axis_diff[NUM_AXIS] = {0, 0, 0, 0};
#ifdef STEP_DELAY_RATIO
  long long_step_delay_ratio = STEP_DELAY_RATIO * 100;
#endif

#define Z_ADJUST_BYTE 0
#define Z_MAX_LENGTH_EEPROM 1
float Z_MAX_LENGTH_M240 = 120.00;
union data {
  float v;
  unsigned char fchar[4];
} fvalue;


// comm variables
#define MAX_CMD_SIZE 96
#define BUFSIZE 8
char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
bool fromsd[BUFSIZE];
int bufindr = 0;
int bufindw = 0;
int buflen = 0;
int i = 0;
char serial_char;
int serial_count = 0;
boolean comment_mode = false;
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

// Manage heater variables. For a thermistor or AD595 thermocouple, raw values refer to the
// reading from the analog pin. For a MAX6675 thermocouple, the raw value is the temperature in 0.25
// degree increments (i.e. 100=25 deg).

int target_raw = 0;
int current_raw = 0;
int target_bed_raw = 0;
int current_bed_raw = 0;
int tt = 0, bt = 0;
#ifdef PIDTEMP
  int temp_iState = 0;
  int temp_dState = 0;
  int pTerm;
  int iTerm;
  int dTerm;
  int output;
  int error;
  int temp_iState_min = -pid_i_max / Ki;
  int temp_iState_max = pid_i_max / Ki;
#endif
#ifdef SMOOTHING
  uint32_t nma = 0;
#endif
#ifdef WATCHPERIOD
  int watch_raw = -1000;
  unsigned long watchmillis = 0;
#endif
#ifdef MINTEMP
  int minttemp = temp2analogh(MINTEMP);
#endif
#ifdef MAXTEMP
int maxttemp = temp2analogh(MAXTEMP);
#endif
#ifdef MAXTEMPBED
int maxbtemp = temp2analogh(MAXTEMPBED);
#endif

//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = 0;
unsigned long stepper_inactive_time = 0;

#ifdef SDSUPPORT
  Sd2Card card;
  SdVolume volume;
  SdFile root;
  SdFile file;
  uint32_t filesize = 0;
  uint32_t sdpos = 0;
  bool sdmode = false;
  bool sdactive = false;
  bool savetosd = false;
  int16_t n;

  void initsd(){
  sdactive = false;
  #if SDSS >- 1
    if(root.isOpen())
        root.close();
    if (!card.init(SPI_FULL_SPEED,SDSS)){
    //if (!card.init(SPI_HALF_SPEED,SDSS))
          SerialMgr.cur()->println("SD init fail");
    }
    else if (!volume.init(&card))
          SerialMgr.cur()->println("volume.init failed");
    else if (!root.openRoot(&volume))
          SerialMgr.cur()->println("openRoot failed");
    else{
          sdactive = true;
          //print_disk_info();

          #ifdef SDINITFILE
            file.close();
            if(file.open(&root, "init.g", O_READ)){
                sdpos = 0;
                filesize = file.fileSize();
                sdmode = true;
            }
            #endif
    }
    #endif
  }

  inline void write_command(char *buf){
      char* begin = buf;
      char* npos = 0;
      char* end = buf + strlen(buf) - 1;

      file.writeError = false;
      if((npos = strchr(buf, 'N')) != NULL){
          begin = strchr(npos, ' ') + 1;
          end = strchr(npos, '*') - 1;
      }
      end[1] = '\r';
      end[2] = '\n';
      end[3] = '\0';
      //SerialMgr.cur()->println(begin);
      file.write(begin);
      if (file.writeError){
          SerialMgr.cur()->println("error writing to file");
      }
  }
#endif


void setup()
{
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(BAUDRATE);
  Serial.println("start");

  #ifdef BLUETOOTH
	BLUETOOTH_SERIAL.begin(BAUDRATE);
  #endif

  for(int i = 0; i < BUFSIZE; i++){
      fromsd[i] = false;
  }

  #if LED_PIN > -1
    SET_OUTPUT(LED_PIN);
    WRITE(LED_PIN,HIGH);
  #endif

  //Initialize Dir Pins
  #if X_DIR_PIN > -1
    SET_OUTPUT(X_DIR_PIN);
  #endif
  #if Y_DIR_PIN > -1
    SET_OUTPUT(Y_DIR_PIN);
  #endif
  #if Z_DIR_PIN > -1
    SET_OUTPUT(Z_DIR_PIN);
  #endif
  #if E_DIR_PIN > -1
    SET_OUTPUT(E_DIR_PIN);
  #endif

  //Initialize Enable Pins - steppers default to disabled.

  #if (X_ENABLE_PIN > -1)
    SET_OUTPUT(X_ENABLE_PIN);
  if(!X_ENABLE_ON) WRITE(X_ENABLE_PIN,HIGH);
  #endif
  #if (Y_ENABLE_PIN > -1)
    SET_OUTPUT(Y_ENABLE_PIN);
  if(!Y_ENABLE_ON) WRITE(Y_ENABLE_PIN,HIGH);
  #endif
  #if (Z_ENABLE_PIN > -1)
    SET_OUTPUT(Z_ENABLE_PIN);
  if(!Z_ENABLE_ON) WRITE(Z_ENABLE_PIN,HIGH);
  #endif
  #if (E_ENABLE_PIN > -1)
    SET_OUTPUT(E_ENABLE_PIN);
  if(!E_ENABLE_ON) WRITE(E_ENABLE_PIN,HIGH);
  #endif

  //endstops and pullups
  #ifdef ENDSTOPPULLUPS
  #if X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);
    WRITE(X_MIN_PIN,HIGH);
  #endif
  #if X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN);
    WRITE(X_MAX_PIN,HIGH);
  #endif
  #if Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);
    WRITE(Y_MIN_PIN,HIGH);
  #endif
  #if Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN);
    WRITE(Y_MAX_PIN,HIGH);
  #endif
  #if Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);
    WRITE(Z_MIN_PIN,HIGH);
  #endif
  #if Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);
    WRITE(Z_MAX_PIN,HIGH);
  #endif
  #else
  #if X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);
  #endif
  #if X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN);
  #endif
  #if Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);
  #endif
  #if Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN);
  #endif
  #if Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);
  #endif
  #if Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);
  #endif
/*  #if PROBE_PIN > -1
    SET_INPUT(PROBE_PIN);
  #endif
*/
  #endif

  #if (FAN_PIN > -1)
    SET_OUTPUT(FAN_PIN);
    WRITE(FAN_PIN,FAN_INIT);
  #endif
  #if (HEATER_0_PIN > -1)
    SET_OUTPUT(HEATER_0_PIN);
  #endif
  #if (HEATER_1_PIN > -1)
    SET_OUTPUT(HEATER_1_PIN);
  #endif

//Initialize Step Pins
  #if (X_STEP_PIN > -1)
    SET_OUTPUT(X_STEP_PIN);
  #endif
  #if (Y_STEP_PIN > -1)
    SET_OUTPUT(Y_STEP_PIN);
  #endif
  #if (Z_STEP_PIN > -1)
    SET_OUTPUT(Z_STEP_PIN);
  #endif
  #if (E_STEP_PIN > -1)
    SET_OUTPUT(E_STEP_PIN);
  #endif
  #ifdef RAMP_ACCELERATION
  for(int i=0; i < NUM_AXIS; i++){
        axis_max_interval[i] = 100000000.0 / (max_start_speed_units_per_second[i] * axis_steps_per_unit[i]);
        axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
        axis_travel_steps_per_sqr_second[i] = max_travel_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
    }
  #endif

#ifdef HEATER_USES_MAX6675
  SET_OUTPUT(SCK_PIN);
  WRITE(SCK_PIN,0);

  SET_OUTPUT(MOSI_PIN);
  WRITE(MOSI_PIN,1);

  SET_INPUT(MISO_PIN);
  WRITE(MISO_PIN,1);

  SET_OUTPUT(MAX6675_SS);
  WRITE(MAX6675_SS,1);
#endif

#ifdef SDSUPPORT

  //power to SD reader
  #if SDPOWER > -1
    SET_OUTPUT(SDPOWER);
    WRITE(SDPOWER,HIGH);
  #endif
  initsd();

#endif

for(int i = 0; i < 4; i++) fvalue.fchar[i] = EEPROM.read(i + Z_MAX_LENGTH_EEPROM);
Serial.print("PrinterHeight: ");
Serial.println(fvalue.v);
Z_MAX_LENGTH_M240 = fvalue.v;

}//end setup()


void loop()
{
  check_heater();
  #ifdef BLUETOOTH
	if(BLUETOOTH_SERIAL.available() && !serial_count)
  	{
  		SerialMgr.ChangeSerial(&BLUETOOTH_SERIAL);
  	}
  else {
    if(Serial.available() && !serial_count) {
        SerialMgr.ChangeSerial(&Serial);
      }
    }
  #endif

  //  SerialMgr.cur()->println("ok");
  if( (buflen<3) && (PauseID == 0x03) && (FSW_status == 1) )
    //  if(buflen<3)
  	get_command();

  if(buflen){
    #ifdef SDSUPPORT
        if(savetosd){
            if(strstr(cmdbuffer[bufindr],"M29") == NULL){
                write_command(cmdbuffer[bufindr]);
                SerialMgr.cur()->println("ok");
            }else{
                file.sync();
                file.close();
                savetosd = false;
                SerialMgr.cur()->println("Done saving file.");
            }
        }else{
            process_commands();
        }
    #else
        process_commands();
    #endif
        buflen = (buflen-1);
        bufindr = (bufindr + 1)%BUFSIZE;
  }
  //check heater every n milliseconds
      manage_heater();
      manage_inactivity(1);
      check_PauseID();

} //end loop()

void check_PauseID()
{
  if((millis() - previous_millis_PauseID) < 100 ) //check the PauseID every 100ms
    return;
  previous_millis_PauseID = millis();

                            //char b = 0x03;
                            Wire.beginTransmission(0x48);
                            Wire.send(237);
                            Wire.endTransmission();
                            //SerialMgr.cur()->println("M237 OK");

                            Wire.requestFrom(0x48, 1);    // request 1 bytes from slave device #0x48
                            while(Wire.available())    // slave may send less than requested
                            {
                              char c = Wire.receive(); // receive a byte as character
                              //SerialMgr.cur()->println(c, HEX);         // print the character
                              PauseID = c & 0x03;
                            }

                            if (HSW_Enable == 0x00){
                              PauseID = PauseID | 0x01;
                            }
//                            if ( (HSW_Enable == 0x01) && ((PauseID & 0x01) == 0x00) ){
//                             // 212,223; red flashing
//                                  Wire.beginTransmission(0x48);
//                                  Wire.send(212);
//                                  Wire.endTransmission();
//
//                                  Wire.beginTransmission(0x48);
//                                  Wire.send(223);
//                                  Wire.endTransmission();
//
//                                  Wire.beginTransmission(0x48);
//                                  Wire.send(235);
//                                  Wire.endTransmission();
//                            }

//                            if ( (HSW_Enable == 0x01) && ((PauseID & 0x01) == 0x01) ){
//                              //front blue
//                              Wire.beginTransmission(0x48);
//                              Wire.send(226);
//                              Wire.endTransmission();
//
//                              //beep off
//                              Wire.beginTransmission(0x48);
//                              Wire.send(236);
//                              Wire.endTransmission();
//                              //nozzle white
//                              Wire.beginTransmission(0x48);
//                              Wire.send(217);
//                              Wire.endTransmission();
//                            }
//Front Switch
//Long Press: “Emergency Stop” to Main Chip.
                              if (FSW_Counter > 30){

                                  //beep long x1
                                  Wire.beginTransmission(0x48);
                                  Wire.send(234);
                                  Wire.endTransmission();

                                  //disable x,y,z,e
                                 //FSW_Counter = 31; //forever stop
                                 while(1){
                                 PauseID = 0;
                                 disable_x(); disable_y(); disable_z(); disable_e();//stop motors
                                 analogWrite(FAN_PIN, 0); WRITE(FAN_PIN, LOW);//stop fan
                                 //target_bed_raw = temp2analogBed(0);target_raw = temp2analogh(0);//stop hotend heater, bed
                                 WRITE(HEATER_0_PIN,LOW);WRITE(HEATER_1_PIN,LOW);//stop hotend heater, bed
                                 SerialMgr.cur()->println("Emergency Stop");
                                 }

                               }


//Short Press:”Pause” to Main chip.

                            if( (FSW_Counter > 5) && ( FSW_Counter <= 30 ) ){
                                  //beep short x1
                                Wire.beginTransmission(0x48);
                                Wire.send(239);
                                Wire.endTransmission();

                              if ( (PauseID & 0x02) == 0x02 ){//released the Front Switch
                                //PauseID = ~PauseID;
                                SerialMgr.cur()->println("Pause/Unpause");

                                FSW_status = ~FSW_status;
                                FSW_Counter = 0;
                              }
                              //else PauseID = 0x03;

                            }

                           if( FSW_Counter <= 5){
                            if ( (PauseID & 0x02) == 0x02 ){//released the Front Switch
                              FSW_Counter = 0;
                            }
                           }
                            if ( (PauseID & 0x02) == 0 ){
                              FSW_Counter++;
                             // PauseID = 0x03;
                            }
}

inline void get_command()
{
//  SerialMgr.cur()->println("get_command");
  while( SerialMgr.cur()->available() > 0  && buflen < BUFSIZE) {
    serial_char = SerialMgr.cur()->read();
    if(serial_char == '\n' || serial_char == '\r' || serial_char == ':' || serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) return; //if empty line
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
    fromsd[bufindw] = false;
  if(strstr(cmdbuffer[bufindw], "N") != NULL)
  {
    strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
    gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
    if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) ) {
      SerialMgr.cur()->print("Serial Error: Line Number is not Last Line Number+1, Last Line:");
      SerialMgr.cur()->println(gcode_LastN);
      //SerialMgr.cur()->println(gcode_N);
      FlushSerialRequestResend();
      serial_count = 0;
      return;
    }

    if(strstr(cmdbuffer[bufindw], "*") != NULL)
    {
      byte checksum = 0;
      byte count = 0;
      while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
      strchr_pointer = strchr(cmdbuffer[bufindw], '*');

      if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
        SerialMgr.cur()->print("Error: checksum mismatch, Last Line:");
        SerialMgr.cur()->println(gcode_LastN);
        FlushSerialRequestResend();
        serial_count = 0;
        return;
      }
      //if no errors, continue parsing
    }
    else
    {
      SerialMgr.cur()->print("Error: No Checksum with line number, Last Line:");
      SerialMgr.cur()->println(gcode_LastN);
      FlushSerialRequestResend();
      serial_count = 0;
      return;
    }

    gcode_LastN = gcode_N;
    //if no errors, continue parsing
  }
  else  // if we don't receive 'N' but still see '*'
  {
    if((strstr(cmdbuffer[bufindw], "*") != NULL))
    {
      SerialMgr.cur()->print("Error: No Line Number with checksum, Last Line:");
      SerialMgr.cur()->println(gcode_LastN);
      serial_count = 0;
      return;
    }
  }
	if((strstr(cmdbuffer[bufindw], "G") != NULL)){
		strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
		switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
		case 0:
		case 1:
              #ifdef SDSUPPORT
              if(savetosd)
                break;
              #endif
			  SerialMgr.cur()->println("ok");
			  break;
		default:
			break;
		}

	}
        bufindw = (bufindw + 1)%BUFSIZE;
        buflen += 1;

      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
#ifdef SDSUPPORT
if(!sdmode || serial_count!=0){
    return;
}
  while( filesize > sdpos  && buflen < BUFSIZE) {
    n = file.read();
    serial_char = (char)n;
    if(serial_char == '\n' || serial_char == '\r' || serial_char == ':' || serial_count >= (MAX_CMD_SIZE - 1) || n == -1)
    {
        sdpos = file.curPosition();
        if(sdpos >= filesize){
            sdmode = false;
            SerialMgr.cur()->println("Done printing file");
        }
      if(!serial_count) return; //if empty line
      cmdbuffer[bufindw][serial_count] = 0; //terminate string
      if(!comment_mode){
        fromsd[bufindw] = true;
        buflen += 1;
        bufindw = (bufindw + 1)%BUFSIZE;
      }
      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
}
#endif

}


inline float code_value() { return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); }
inline long code_value_long() { return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); }
inline bool code_seen(char code_string[]) { return (strstr(cmdbuffer[bufindr], code_string) != NULL); }  //Return True if the string was found

inline bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

inline void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;


  if(code_seen('G'))
  {
    switch((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        #if (defined DISABLE_CHECK_DURING_ACC) || (defined DISABLE_CHECK_DURING_MOVE) || (defined DISABLE_CHECK_DURING_TRAVEL)
          manage_heater();
        #endif
        get_coordinates(); // For X Y Z E F
        prepare_move();
        previous_millis_cmd = millis();
        //ClearToSend();
        return;
        //break;
      case 4: // G4 dwell
        codenum = 0;
        if(code_seen('P')) codenum = code_value(); // milliseconds to wait
        if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
        codenum += millis();  // keep track of when we started waiting
        while(millis()  < codenum ){
          manage_heater();
        }
        break;
      case 28: //G28 Home all Axis one at a time

      // M225 - green flash //rp3d
                  Wire.beginTransmission(0x48);
                  Wire.send(225);
                  Wire.endTransmission();

                  //nozzle white
                  Wire.beginTransmission(0x48);
                  Wire.send(217);
                  Wire.endTransmission();

                  // beep short x3
                  Wire.beginTransmission(0x48);
                  Wire.send(233);
                  Wire.endTransmission();
                  delay(2000);    //wait for beep end
        saved_feedrate = feedrate;
        for(int i=0; i < NUM_AXIS; i++) {
          destination[i] = current_position[i];
        }
        feedrate = 0;

        home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

        if((home_all_axis) || (code_seen(axis_codes[0]))) {
          if ((X_MIN_PIN > -1 && X_HOME_DIR==-1) || (X_MAX_PIN > -1 && X_HOME_DIR==1)){
            current_position[0] = 0;
            destination[0] = 1.5 * X_MAX_LENGTH * X_HOME_DIR;
            feedrate = homing_feedrate[0];
            prepare_move();

            current_position[0] = 0;
            destination[0] = -5 * X_HOME_DIR;
            prepare_move();

            destination[0] = 10 * X_HOME_DIR;
            prepare_move();

            //for X offset 20160329
            if(X_OFFSET > 0){
              destination[0] = X_OFFSET;
              prepare_move();
            }

            current_position[0] = (X_HOME_DIR == -1) ? 0 : X_MAX_LENGTH;
            destination[0] = current_position[0];

            feedrate = 0;
          }
        }

        if((home_all_axis) || (code_seen(axis_codes[1]))) {
          if ((Y_MIN_PIN > -1 && Y_HOME_DIR==-1) || (Y_MAX_PIN > -1 && Y_HOME_DIR==1)){
            current_position[1] = 0;
            destination[1] = 1.5 * Y_MAX_LENGTH * Y_HOME_DIR;
            feedrate = homing_feedrate[1];
            prepare_move();

            current_position[1] = 0;
            destination[1] = -5 * Y_HOME_DIR;
            prepare_move();

            destination[1] = 10 * Y_HOME_DIR;
            prepare_move();

            //for Y offset 20160329
            if(Y_OFFSET > 0){
              destination[1] = Y_OFFSET;
              prepare_move();
            }

            current_position[1] = (Y_HOME_DIR == -1) ? 0 : Y_MAX_LENGTH;
            destination[1] = current_position[1];
            feedrate = 0;
          }
        }

        if((home_all_axis) || (code_seen(axis_codes[2]))) {
          if ((Z_MIN_PIN > -1 && Z_HOME_DIR==-1) || (Z_MAX_PIN > -1 && Z_HOME_DIR==1)){
            current_position[2] = 0;
            destination[2] = 1.5 * Z_MAX_LENGTH * Z_HOME_DIR;
            feedrate = homing_feedrate[2];
            prepare_move();

            current_position[2] = 0;
            destination[2] = -2 * Z_HOME_DIR;
            prepare_move();

            destination[2] = 10 * Z_HOME_DIR;
            prepare_move();

            current_position[2] = (Z_HOME_DIR == -1) ? (float)byteToint(EEPROM.read(Z_ADJUST_BYTE))/100 : Z_MAX_LENGTH_M240;
            //current_position[2] = (Z_HOME_DIR == -1) ? 0 : Z_MAX_LENGTH;
            //destination[2] = current_position[2];
            feedrate = 0;

        }
        }

        feedrate = saved_feedrate;
        previous_millis_cmd = millis();
        // M226 - blue on front
                  Wire.beginTransmission(0x48);
                  Wire.send(226);
                  Wire.endTransmission();

                  //beep long x1
                  Wire.beginTransmission(0x48);
                  Wire.send(234);
                  Wire.endTransmission();
                  //nozzle white
                  Wire.beginTransmission(0x48);
                  Wire.send(217);
                  Wire.endTransmission();
        break;
/*      case 29:
        saved_feedrate = feedrate;
        if (PROBE_PIN > -1 && Z_HOME_DIR==-1){
          current_position[2] = 0;
          destination[2] = 1.5 * Z_MAX_LENGTH * Z_HOME_DIR;
          feedrate = homing_feedrate[2];
          prepare_move();

          //move up in small increments until switch makes
          int z=0;
          current_position[2] = 0;
          SerialMgr.cur()->print("ZMIN=");
          SerialMgr.cur()->println(READ(PROBE_PIN));
          while(READ(PROBE_PIN) == true && z<50){
            SerialMgr.cur()->print("ZMIN=");
            SerialMgr.cur()->println(READ(PROBE_PIN));
            destination[2] = current_position[2] - Z_INCREMENT * Z_HOME_DIR;
            prepare_move();
            z++;
          }

          SerialMgr.cur()->print("Z=");
          SerialMgr.cur()->println(current_position[2]);
          //current_position[2] = (Z_HOME_DIR == -1) ? 0 : Z_MAX_LENGTH;
          //destination[2] = current_position[2];
          feedrate = 0;
        }
        break;*/
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;
      case 92: // G92
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) current_position[i] = code_value();
        }
        break;

    }
  }

  else if(code_seen('M'))
  {

    switch( (int)code_value() )
    {
      case 4: //Ask for status
        SerialMgr.cur()->print("S:");
        SerialMgr.cur()->print(status);
        SerialMgr.cur()->print(", ");
        SerialMgr.cur()->println(status_str[status]);
        if (status == STATUS_ERROR)
        {
            SerialMgr.cur()->print("EC:");
            SerialMgr.cur()->print(error_code);
            SerialMgr.cur()->print(", ");
            SerialMgr.cur()->println(error_code_str[error_code]);
        }
        break;
      case 5: //Reset errors
        status = STATUS_OK;
        error_code = ERROR_CODE_NO_ERROR;
        break;

#ifdef SDSUPPORT

      case 20: // M20 - list SD card
        SerialMgr.cur()->println("Begin file list");
        root.ls();
        SerialMgr.cur()->println("End file list");
        break;
      case 21: // M21 - init SD card
        sdmode = false;
        initsd();
        break;
      case 22: //M22 - release SD card
        sdmode = false;
        sdactive = false;
        break;
      case 23: //M23 - Select file
        if(sdactive){
            sdmode = false;
            file.close();
            starpos = (strchr(strchr_pointer + 4,'*'));
            if(starpos!=NULL)
                *(starpos-1)='\0';
            if (file.open(&root, strchr_pointer + 4, O_READ)) {
                SerialMgr.cur()->print("File opened:");
                SerialMgr.cur()->print(strchr_pointer + 4);
                SerialMgr.cur()->print(" Size:");
                SerialMgr.cur()->println(file.fileSize());
                sdpos = 0;
                filesize = file.fileSize();
                SerialMgr.cur()->println("File selected");
            }
            else{
                SerialMgr.cur()->println("file.open failed");
            }
        }
        break;
      case 24: //M24 - Start SD print
        if(sdactive){
            sdmode = true;
        }
        break;
      case 25: //M25 - Pause SD print
        if(sdmode){
            sdmode = false;
        }
        break;
      case 26: //M26 - Set SD index
        if(sdactive && code_seen('S')){
            sdpos = code_value_long();
            file.seekSet(sdpos);
        }
        break;
      case 27: //M27 - Get SD status
        if(sdactive){
            SerialMgr.cur()->print("SD printing byte ");
            SerialMgr.cur()->print(sdpos);
            SerialMgr.cur()->print("/");
            SerialMgr.cur()->println(filesize);
        }else{
            SerialMgr.cur()->println("Not SD printing");
        }
        break;
            case 28: //M28 - Start SD write
        if(sdactive){
          char* npos = 0;
            file.close();
            sdmode = false;
            starpos = (strchr(strchr_pointer + 4,'*'));
            if(starpos != NULL){
              npos = strchr(cmdbuffer[bufindr], 'N');
              strchr_pointer = strchr(npos,' ') + 1;
              *(starpos-1) = '\0';
            }
      if (!file.open(&root, strchr_pointer+4, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
            {
            SerialMgr.cur()->print("open failed, File: ");
            SerialMgr.cur()->print(strchr_pointer + 4);
            SerialMgr.cur()->print(".");
            }else{
            savetosd = true;
            SerialMgr.cur()->print("Writing to file: ");
            SerialMgr.cur()->println(strchr_pointer + 4);
            }
        }
        break;
      case 29: //M29 - Stop SD write
        //processed in write to file routine above
        //savetosd = false;
        break;
#endif
      case 104: // M104
                  //front yellow flashing
                  Wire.beginTransmission(0x48);
                  Wire.send(231);
                  Wire.endTransmission();

                  //nozzle  yellow flashing
                  Wire.beginTransmission(0x48);
                  Wire.send(220);
                  Wire.endTransmission();
        if (code_seen('S')) target_raw = temp2analogh(code_value());
        if (error_code == ERROR_CODE_HOTEND_TEMPERATURE)
        {
            wait_for_temp(); //if we have had a nozzle error, we should wait even though not wait command
        }
        else
        {
            #ifdef WATCHPERIOD
                if(target_raw > current_raw){
                    watchmillis = max(1,millis());
                    watch_raw = current_raw;
                }else{
                    watchmillis = 0;
                }
            #endif
        }
                  Wire.beginTransmission(0x48);
                  Wire.send(226);
                  Wire.endTransmission();

                  //nozzle white
                  Wire.beginTransmission(0x48);
                  Wire.send(217);
                  Wire.endTransmission();

                  //long beep x1
                  Wire.beginTransmission(0x48);
                  Wire.send(234);
                  Wire.endTransmission();
        break;
      case 140: // M140 set bed temp
                  //front yellow flashing
                  Wire.beginTransmission(0x48);
                  Wire.send(231);
                  Wire.endTransmission();

                  //nozzle  yellow flashing
                  Wire.beginTransmission(0x48);
                  Wire.send(220);
                  Wire.endTransmission();
        #if TEMP_1_PIN > -1 || defined BED_USES_AD595
            if (code_seen('S')) target_bed_raw = temp2analogBed(code_value());
        #endif
                  Wire.beginTransmission(0x48);
                  Wire.send(226);
                  Wire.endTransmission();

                  //nozzle white
                  Wire.beginTransmission(0x48);
                  Wire.send(217);
                  Wire.endTransmission();

                  //long beep x1
                  Wire.beginTransmission(0x48);
                  Wire.send(234);
                  Wire.endTransmission();
        break;
      case 105: // M105
        #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX6675)|| defined HEATER_USES_AD595
          tt = analog2temp(current_raw);
        #endif
        #if TEMP_1_PIN > -1 || defined BED_USES_AD595
          bt = analog2tempBed(current_bed_raw);
        #endif
        #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX6675) || defined HEATER_USES_AD595
            SerialMgr.cur()->print("ok T:");
            SerialMgr.cur()->print(tt);
          #if TEMP_1_PIN > -1 || defined BED_USES_AD595
            SerialMgr.cur()->print(" B:");
            SerialMgr.cur()->println(bt);
#ifdef DEBUG_PID
            SerialMgr.cur()->print(" R:");
            SerialMgr.cur()->print(output);
            SerialMgr.cur()->print(" E:");
            SerialMgr.cur()->print(error);
            SerialMgr.cur()->print(" P:");
            SerialMgr.cur()->print(pTerm);
            SerialMgr.cur()->print(" I:");
            SerialMgr.cur()->print(iTerm);
            SerialMgr.cur()->print(" D:");
            SerialMgr.cur()->print(dTerm);
            SerialMgr.cur()->print(" iState:");
            SerialMgr.cur()->println(temp_iState);
#endif
          #else
            SerialMgr.cur()->println();
          #endif
        #else
          #error No temperature source available
        #endif
        return;
        //break;
      /*
      case 205:
          SerialMgr.cur()->print("ok o:");
          SerialMgr.cur()->print(output);
          SerialMgr.cur()->print(", p:");
          SerialMgr.cur()->print(pTerm);
          SerialMgr.cur()->print(", i:");
          SerialMgr.cur()->print(iTerm);
          SerialMgr.cur()->print(", d:");
          SerialMgr.cur()->print(dTerm);
          return;
      */
      case 109: // M109 - Wait for extruder heater to reach target.
                  //front yellow flashing
                  Wire.beginTransmission(0x48);
                  Wire.send(231);
                  Wire.endTransmission();

                  //nozzle  yellow flashing
                  Wire.beginTransmission(0x48);
                  Wire.send(220);
                  Wire.endTransmission();
        if (code_seen('S'))
            target_raw = temp2analogh(code_value() - nzone);
        wait_for_temp();
                          //front blue
                  Wire.beginTransmission(0x48);
                  Wire.send(226);
                  Wire.endTransmission();

                  //nozzle white
                  Wire.beginTransmission(0x48);
                  Wire.send(217);
                  Wire.endTransmission();

                  //long beep x1
                  Wire.beginTransmission(0x48);
                  Wire.send(234);
                  Wire.endTransmission();

        break;
      case 190: // M190 - Wait bed for heater to reach target.

                  //front yellow flashing
                  Wire.beginTransmission(0x48);
                  Wire.send(231);
                  Wire.endTransmission();

                  //nozzle  yellow flashing
                  Wire.beginTransmission(0x48);
                  Wire.send(220);
                  Wire.endTransmission();

      #if TEMP_1_PIN > -1
        if (code_seen('S')) target_bed_raw = temp2analogh(code_value());
        codenum = millis();
        while(current_bed_raw < target_bed_raw) {
          if( (millis()-codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
          {
            tt=analog2temp(current_raw);
            SerialMgr.cur()->print("T:");
            SerialMgr.cur()->print( tt );
            SerialMgr.cur()->print(" B:");
            SerialMgr.cur()->println( analog2temp(current_bed_raw) );
            codenum = millis();
          }
            manage_heater();
        }
      #endif
                  //front blue
                  Wire.beginTransmission(0x48);
                  Wire.send(226);
                  Wire.endTransmission();

                  //nozzle white
                  Wire.beginTransmission(0x48);
                  Wire.send(217);
                  Wire.endTransmission();

                  //long beep x1
                  Wire.beginTransmission(0x48);
                  Wire.send(234);
                  Wire.endTransmission();
      break;
      #if FAN_PIN > -1
      case 106: //M106 Fan On
        if (code_seen('S')){
            WRITE(FAN_PIN, HIGH);
            analogWrite(FAN_PIN, constrain(code_value(),0,255) );
        }
        else
            WRITE(FAN_PIN, HIGH);
        break;
      case 107: //M107 Fan Off
        analogWrite(FAN_PIN, 0);

        WRITE(FAN_PIN, LOW);
        break;
      #endif
      #if (PS_ON_PIN > -1)
      case 80: // M81 - ATX Power On
        SET_OUTPUT(PS_ON_PIN); //GND
        break;
      case 81: // M81 - ATX Power Off
        SET_INPUT(PS_ON_PIN); //Floating
        break;
      #endif
      case 82:
        axis_relative_modes[3] = false;
        break;
      case 83:
        axis_relative_modes[3] = true;
        break;
      case 84: //M84
        if(code_seen('S')){ stepper_inactive_time = code_value() * 1000; }
        else{ disable_x(); disable_y(); disable_z(); disable_e(); }

                  //front Green
                  Wire.beginTransmission(0x48);
                  Wire.send(224);
                  Wire.endTransmission();

                  //nozzle white
                  Wire.beginTransmission(0x48);
                  Wire.send(217);
                  Wire.endTransmission();

//                  // beep short x3
//                  Wire.beginTransmission(0x48);
//                  Wire.send(233);
//                  Wire.endTransmission();
        break;
      case 85: // M85
        code_seen('S');
        max_inactive_time = code_value() * 1000;
        break;
      case 92: // M92
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) axis_steps_per_unit[i] = code_value();
        }

        //Update start speed intervals and axis order. TODO: refactor axis_max_interval[] calculation into a function, as it
        // should also be used in setup() as well
        #ifdef RAMP_ACCELERATION
          long temp_max_intervals[NUM_AXIS];
          for(int i=0; i < NUM_AXIS; i++) {
            axis_max_interval[i] = 100000000.0 / (max_start_speed_units_per_second[i] * axis_steps_per_unit[i]);//TODO: do this for
                  // all steps_per_unit related variables
          }
        #endif
        break;
      case 115: // M115
        //SerialMgr.cur()->print("FIRMWARE_NAME:Sprinter FIRMWARE_URL:http%%3A/github.com/kliment/Sprinter/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 UUID:");
        SerialMgr.cur()->print("FIRMWARE_NAME:rp3d.com FIRMWARE_URL:http://rp3d.com/  PROTOCOL_VERSION:1.0 MACHINE_TYPE:rp3d EXTRUDER_COUNT:1 UUID:");
        SerialMgr.cur()->println(uuid);
        break;
      case 114: // M114
	SerialMgr.cur()->print("X:");
        SerialMgr.cur()->print(current_position[0]);
	SerialMgr.cur()->print("Y:");
        SerialMgr.cur()->print(current_position[1]);
	SerialMgr.cur()->print("Z:");
        SerialMgr.cur()->print(current_position[2]);
	SerialMgr.cur()->print("E:");
        SerialMgr.cur()->println(current_position[3]);
        break;
      case 119: // M119
      	#if (X_MIN_PIN > -1)
      	SerialMgr.cur()->print("x_min:");
        SerialMgr.cur()->print((READ(X_MIN_PIN)^ENDSTOPS_INVERTING)?"H ":"L ");
      	#endif
      	#if (X_MAX_PIN > -1)
      	SerialMgr.cur()->print("x_max:");
        SerialMgr.cur()->print((READ(X_MAX_PIN)^ENDSTOPS_INVERTING)?"H ":"L ");
      	#endif
      	#if (Y_MIN_PIN > -1)
      	SerialMgr.cur()->print("y_min:");
        SerialMgr.cur()->print((READ(Y_MIN_PIN)^ENDSTOPS_INVERTING)?"H ":"L ");
      	#endif
      	#if (Y_MAX_PIN > -1)
      	SerialMgr.cur()->print("y_max:");
        SerialMgr.cur()->print((READ(Y_MAX_PIN)^ENDSTOPS_INVERTING)?"H ":"L ");
      	#endif
      	#if (Z_MIN_PIN > -1)
      	SerialMgr.cur()->print("z_min:");
        SerialMgr.cur()->print((READ(Z_MIN_PIN)^ENDSTOPS_INVERTING)?"H ":"L ");
      	#endif
      	#if (Z_MAX_PIN > -1)
      	SerialMgr.cur()->print("z_max:");
        SerialMgr.cur()->print((READ(Z_MAX_PIN)^ENDSTOPS_INVERTING)?"H ":"L ");
      	#endif
        SerialMgr.cur()->println("");
      	break;
      #ifdef RAMP_ACCELERATION
      //TODO: update for all axis, use for loop
      case 201: // M201
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) axis_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
        }
        break;
      case 202: // M202
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
        }
        break;
      #endif

      /*
      M203: Record Z adjustment
Example: M203 Z-0.75
This records a Z offset in non-volatile memory in RepRap's microcontroller where it remains active until next set,
even when the power is turned off and on again.
If the first layer is too close to the bed, you need to effectively move the bed down, so the Z value will be negative.
If the nozzle is too far from the bed during the first layer, the Z value should be positive to raise the bed.
The maximum adjustment is +/-1.27mm.
      */
      case 203: // M203 - set Z height adjustment
        if(code_seen('Z')){
          EEPROM.write(Z_ADJUST_BYTE,code_value()*100);
        }
      break;

//rp3d.com

/*

*/

      case 240: // M240 - set Z_MAX_LENGTH

        if(code_seen('Z')){
                //数据拆分
                fvalue.v = code_value();

                unsigned char *fpointer;
                fpointer = fvalue.fchar;
                for(int i = 0; i < 4; i++) {
                  EEPROM.write(i + Z_MAX_LENGTH_EEPROM,*fpointer);
                  fpointer++;
                }

                //数据还原
                for(int i = 0; i < 4; i++) fvalue.fchar[i] = EEPROM.read(i + Z_MAX_LENGTH_EEPROM);
      	        Serial.print("PrinterHeight: ");
                Serial.println(fvalue.v);
                Z_MAX_LENGTH_M240 = fvalue.v;
//                Z_MAX_LENGTH = fvalue.v;

        } else {
                for(int i = 0; i < 4; i++) fvalue.fchar[i] = EEPROM.read(i + Z_MAX_LENGTH_EEPROM);
      	        Serial.print("PrinterHeight: ");
                Serial.println(fvalue.v);
                Z_MAX_LENGTH_M240 = fvalue.v;
        }
      break;

//head led
      case 211: // M211 - red on
                  Wire.beginTransmission(0x48); // transmit to device #0x48
                  Wire.send(211);        // sends 211, Red LED on
                  Wire.endTransmission();    // stop transmitting
                  //return;
                  SerialMgr.cur()->println("M211 OK");
      break;
      case 212: // M212 - red flash
                  Wire.beginTransmission(0x48); // transmit to device #0x48
                  Wire.send(212);        // sends 212, Red LED flashing
                  Wire.endTransmission();    // stop transmitting
                  //return;
                  SerialMgr.cur()->println("M212 OK");
      break;
      case 213: // M213 - Green on
                  Wire.beginTransmission(0x48);
                  Wire.send(213);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M213 OK");
      break;
      case 214: // M214 - green flash
                  Wire.beginTransmission(0x48);
                  Wire.send(214);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M214 OK");
      break;
      case 215: // M215 - blue on
                  Wire.beginTransmission(0x48);
                  Wire.send(215);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M215 OK");
      break;
      case 216: // M216 - blue flash
                  Wire.beginTransmission(0x48);
                  Wire.send(216);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M216 OK");
      break;
      case 217: // M217 - white led on
                  Wire.beginTransmission(0x48);
                  Wire.send(217);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M217 OK");
      break;
      case 218: // M218 - white led flash
                  Wire.beginTransmission(0x48);
                  Wire.send(218);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M218 OK");
      break;
      case 219: // M219 - Orange LED ON
                  Wire.beginTransmission(0x48);
                  Wire.send(219);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M219 OK");
      break;
      case 220: // M220 - Orange LED flash
                  Wire.beginTransmission(0x48);
                  Wire.send(220);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M220 OK");
      break;
      case 221: // M221 - Head LED OFF.
                  Wire.beginTransmission(0x48);
                  Wire.send(221);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M221 OK");
      break;
//end head led

//front SW led
      case 222: // M222 - red on
                  Wire.beginTransmission(0x48); // transmit to device #0x48
                  Wire.send(222);        //
                  Wire.endTransmission();    // stop transmitting
                  //return;
                  SerialMgr.cur()->println("M222 OK");
      break;
      case 223: // M223 - red flash
                  Wire.beginTransmission(0x48); // transmit to device #0x48
                  Wire.send(223);        //
                  Wire.endTransmission();    // stop transmitting
                  //return;
                  SerialMgr.cur()->println("M223 OK");
      break;
      case 224: // M224 - Green on
                  Wire.beginTransmission(0x48);
                  Wire.send(224);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M224 OK");
      break;
      case 225: // M225 - green flash
                  Wire.beginTransmission(0x48);
                  Wire.send(225);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M225 OK");
      break;
      case 226: // M226 - blue on
                  Wire.beginTransmission(0x48);
                  Wire.send(226);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M226 OK");
      break;
      case 227: // M227 - blue flash
                  Wire.beginTransmission(0x48);
                  Wire.send(227);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M227 OK");
      break;
      case 228: // M228 - white led on
                  Wire.beginTransmission(0x48);
                  Wire.send(228);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M228 OK");
      break;
      case 229: // M229 - white led flash
                  Wire.beginTransmission(0x48);
                  Wire.send(229);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M229 OK");
      break;
      case 230: // M230 - Orange LED ON
                  Wire.beginTransmission(0x48);
                  Wire.send(230);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M230 OK");
      break;
      case 231: // M231 - Orange LED flash
                  Wire.beginTransmission(0x48);
                  Wire.send(231);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M231 OK");
      break;
      case 232: // M221 - Head LED OFF.
                  Wire.beginTransmission(0x48);
                  Wire.send(232);
                  Wire.endTransmission();
                  //return;
                  SerialMgr.cur()->println("M232 OK");
      break;

//Beep
//233	Short Beep x 3
//234	Long Beep x 1 ( 3 sec)
//235	Beep every sec, 3 min.
//236	Beep Off
//239	Short Beep x 1
      case 233:
                  Wire.beginTransmission(0x48);
                  Wire.send(233);
                  Wire.endTransmission();
                  SerialMgr.cur()->println("M233 OK");
      break;
      case 234:
                  Wire.beginTransmission(0x48);
                  Wire.send(234);
                  Wire.endTransmission();
                  SerialMgr.cur()->println("M234 OK");
      break;
      case 235:
                  Wire.beginTransmission(0x48);
                  Wire.send(235);
                  Wire.endTransmission();
                  SerialMgr.cur()->println("M235 OK");
      break;
      case 236:
                  Wire.beginTransmission(0x48);
                  Wire.send(236);
                  Wire.endTransmission();
                  SerialMgr.cur()->println("M236 OK");
      break;
      case 239:
                  Wire.beginTransmission(0x48);
                  Wire.send(239);
                  Wire.endTransmission();
                  SerialMgr.cur()->println("M239 OK");
      break;

//Hood Switch
//237	HSW Enable
//238	HSW Disable
      case 237:
                  HSW_Enable = 0x01;
                  SerialMgr.cur()->println("M237 OK");
      break;
      case 238:
                  HSW_Enable = 0x00;
                  SerialMgr.cur()->println("M238 OK");
      break;
//Front Switch
//Long Press: “Emergency Stop” to Main Chip.
//Short Press:”Pause” to Main chip.


#ifdef PIDTEMP
      case 301: // M301
        if(code_seen('P')) Kp = code_value();
        if(code_seen('I')) Ki = code_value();
        if(code_seen('D')) Kd = code_value();
        if(code_seen('F')) pid_max = code_value();
        if(code_seen('Z')) nzone = code_value();
        if(code_seen('W')) pid_i_max = code_value();
        SerialMgr.cur()->print("Kp ");SerialMgr.cur()->println(Kp);
        SerialMgr.cur()->print("Ki ");SerialMgr.cur()->println(Ki);
        SerialMgr.cur()->print("Kd ");SerialMgr.cur()->println(Kd);
        SerialMgr.cur()->print("PID_MAX ");SerialMgr.cur()->println(pid_max);
        SerialMgr.cur()->print("PID_I_MAX ");SerialMgr.cur()->println(pid_i_max);
        SerialMgr.cur()->print("NZONE ");SerialMgr.cur()->println(nzone);
        temp_iState_min = -pid_i_max / Ki;
        temp_iState_max = pid_i_max / Ki;
        break;
#endif //PIDTEMP
      default:
        ClearToSend();
        return;
    }

  }
  else{
      SerialMgr.cur()->println("Unknown command:");
      SerialMgr.cur()->println(cmdbuffer[bufindr]);
  }

  ClearToSend();

}

int byteToint(byte value){
    if(value>>7)
    {
      return (255-value+1)*-1;
    }
    return value;
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  SerialMgr.cur()->flush();
  SerialMgr.cur()->print("Resend:");
  SerialMgr.cur()->println(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
  previous_millis_cmd = millis();
  if (status == STATUS_ERROR)
  {
    SerialMgr.cur()->print("EC:");
    SerialMgr.cur()->println(error_code);
    SerialMgr.cur()->print(", ");
    SerialMgr.cur()->print(error_code_str[error_code]);
  }
  else
  {
    #ifdef SDSUPPORT
    if(fromsd[bufindr])
      return;
    #endif
  }
//  SerialMgr.cur()->println("Clear to Send ok");
  SerialMgr.cur()->println("ok");
}

inline void get_coordinates()
{
  for(int i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
    else destination[i] = current_position[i];                                                       //Are these else lines really needed?
  }
  if(code_seen('F')) {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void prepare_move()
{
  //Find direction
  for(int i=0; i < NUM_AXIS; i++) {
    if(destination[i] >= current_position[i]) move_direction[i] = 1;
    else move_direction[i] = 0;
  }


  if (min_software_endstops) {
    if (destination[0] < 0) destination[0] = 0.0;
    if (destination[1] < 0) destination[1] = 0.0;
    if (destination[2] < 0) destination[2] = 0.0;
  }

  if (max_software_endstops) {
    if (destination[0] > X_MAX_LENGTH) destination[0] = X_MAX_LENGTH;
    if (destination[1] > Y_MAX_LENGTH) destination[1] = Y_MAX_LENGTH;
    if (destination[2] > Z_MAX_LENGTH) destination[2] = Z_MAX_LENGTH;
  }

  for(int i=0; i < NUM_AXIS; i++) {
    axis_diff[i] = destination[i] - current_position[i];
    move_steps_to_take[i] = abs(axis_diff[i]) * axis_steps_per_unit[i];
  }
  if(feedrate < 10)
      feedrate = 10;

  //Feedrate calc based on XYZ travel distance
  float xy_d;
  //Check for cases where only one axis is moving - handle those without float sqrt
  if(abs(axis_diff[0]) > 0 && abs(axis_diff[1]) == 0 && abs(axis_diff[2])==0)
    d=abs(axis_diff[0]);
  else if(abs(axis_diff[0]) == 0 && abs(axis_diff[1]) > 0 && abs(axis_diff[2])==0)
    d=abs(axis_diff[1]);
  else if(abs(axis_diff[0]) == 0 && abs(axis_diff[1]) == 0 && abs(axis_diff[2])>0)
    d=abs(axis_diff[2]);
  //two or three XYZ axes moving
  else if(abs(axis_diff[0]) > 0 || abs(axis_diff[1]) > 0) { //X or Y or both
    xy_d = sqrt(axis_diff[0] * axis_diff[0] + axis_diff[1] * axis_diff[1]);
    //check if Z involved - if so interpolate that too
    d = (abs(axis_diff[2]>0))?sqrt(xy_d * xy_d + axis_diff[2] * axis_diff[2]):xy_d;
  }
  else if(abs(axis_diff[3]) > 0)
    d = abs(axis_diff[3]);
  else{ //zero length move
  #ifdef DEBUG_PREPARE_MOVE

      log_message("_PREPARE_MOVE - No steps to take!");

  #endif
    return;
    }
  time_for_move = (d / (feedrate / 60000000.0) );
  //Check max feedrate for each axis is not violated, update time_for_move if necessary
  for(int i = 0; i < NUM_AXIS; i++) {
    if(move_steps_to_take[i] && abs(axis_diff[i]) / (time_for_move / 60000000.0) > max_feedrate[i]) {
      time_for_move = time_for_move / max_feedrate[i] * (abs(axis_diff[i]) / (time_for_move / 60000000.0));
    }
  }
  //Calculate the full speed stepper interval for each axis
  for(int i=0; i < NUM_AXIS; i++) {
    if(move_steps_to_take[i]) axis_interval[i] = time_for_move / move_steps_to_take[i] * 100;
  }

  #ifdef DEBUG_PREPARE_MOVE
    log_float("_PREPARE_MOVE - Move distance on the XY plane", xy_d);
    log_float("_PREPARE_MOVE - Move distance on the XYZ space", d);
    log_int("_PREPARE_MOVE - Commanded feedrate", feedrate);
    log_float("_PREPARE_MOVE - Constant full speed move time", time_for_move);
    log_float_array("_PREPARE_MOVE - Destination", destination, NUM_AXIS);
    log_float_array("_PREPARE_MOVE - Current position", current_position, NUM_AXIS);
    log_ulong_array("_PREPARE_MOVE - Steps to take", move_steps_to_take, NUM_AXIS);
    log_long_array("_PREPARE_MOVE - Axes full speed intervals", axis_interval, NUM_AXIS);
  #endif

  unsigned long move_steps[NUM_AXIS];
  for(int i=0; i < NUM_AXIS; i++)
    move_steps[i] = move_steps_to_take[i];
  linear_move(move_steps); // make the move
}

inline void linear_move(unsigned long axis_steps_remaining[]) // make linear move with preset speeds and destinations, see G0 and G1
{
  //Determine direction of movement
  if (destination[0] > current_position[0]) WRITE(X_DIR_PIN,!INVERT_X_DIR);
  else WRITE(X_DIR_PIN,INVERT_X_DIR);
  if (destination[1] > current_position[1]) WRITE(Y_DIR_PIN,!INVERT_Y_DIR);
  else WRITE(Y_DIR_PIN,INVERT_Y_DIR);
  if (destination[2] > current_position[2]) WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
  else WRITE(Z_DIR_PIN,INVERT_Z_DIR);
  if (destination[3] > current_position[3]) WRITE(E_DIR_PIN,!INVERT_E_DIR);
  else WRITE(E_DIR_PIN,INVERT_E_DIR);
  movereset:
  #if (X_MIN_PIN > -1)
    if(!move_direction[0]) if(READ(X_MIN_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[0]=0;
  #endif
  #if (Y_MIN_PIN > -1)
    if(!move_direction[1]) if(READ(Y_MIN_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[1]=0;
  #endif
  #if (Z_MIN_PIN > -1)
    if(!move_direction[2]) if(READ(Z_MIN_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[2]=0;
  #endif
  #if (X_MAX_PIN > -1)
    if(move_direction[0]) if(READ(X_MAX_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[0]=0;
  #endif
  #if (Y_MAX_PIN > -1)
    if(move_direction[1]) if(READ(Y_MAX_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[1]=0;
  #endif
  # if(Z_MAX_PIN > -1)
    if(move_direction[2]) if(READ(Z_MAX_PIN) != ENDSTOPS_INVERTING) axis_steps_remaining[2]=0;
  #endif


  //Only enable axis that are moving. If the axis doesn't need to move then it can stay disabled depending on configuration.
  // TODO: maybe it's better to refactor into a generic enable(int axis) function, that will probably take more ram,
  // but will reduce code size
  if(axis_steps_remaining[0]) enable_x();
  if(axis_steps_remaining[1]) enable_y();
  if(axis_steps_remaining[2]) enable_z();
  if(axis_steps_remaining[3]) enable_e();

    //Define variables that are needed for the Bresenham algorithm. Please note that  Z is not currently included in the Bresenham algorithm.
  unsigned long delta[] = {axis_steps_remaining[0], axis_steps_remaining[1], axis_steps_remaining[2], axis_steps_remaining[3]}; //TODO: implement a "for" to support N axes
  long axis_error[NUM_AXIS];
  int primary_axis;
  if(delta[1] > delta[0] && delta[1] > delta[2] && delta[1] > delta[3]) primary_axis = 1;
  else if (delta[0] >= delta[1] && delta[0] > delta[2] && delta[0] > delta[3]) primary_axis = 0;
  else if (delta[2] >= delta[0] && delta[2] >= delta[1] && delta[2] > delta[3]) primary_axis = 2;
  else primary_axis = 3;
  unsigned long steps_remaining = delta[primary_axis];
  unsigned long steps_to_take = steps_remaining;
  for(int i=0; i < NUM_AXIS; i++){
       if(i != primary_axis) axis_error[i] = delta[primary_axis] / 2;
       steps_taken[i]=0;
    }
  interval = axis_interval[primary_axis];
  bool is_print_move = delta[3] > 0;
  #ifdef DEBUG_BRESENHAM
    log_int("_BRESENHAM - Primary axis", primary_axis);
    log_int("_BRESENHAM - Primary axis full speed interval", interval);
    log_ulong_array("_BRESENHAM - Deltas", delta, NUM_AXIS);
    log_long_array("_BRESENHAM - Errors", axis_error, NUM_AXIS);
  #endif

  //If acceleration is enabled, do some Bresenham calculations depending on which axis will lead it.
  #ifdef RAMP_ACCELERATION
    long max_speed_steps_per_second;
    long min_speed_steps_per_second;
    max_interval = axis_max_interval[primary_axis];
    #ifdef DEBUG_RAMP_ACCELERATION
     log_ulong_array("_RAMP_ACCELERATION - Teoric step intervals at move start", axis_max_interval, NUM_AXIS);
    #endif
    unsigned long new_axis_max_intervals[NUM_AXIS];
    max_speed_steps_per_second = 100000000 / interval;
    min_speed_steps_per_second = 100000000 / max_interval; //TODO: can this be deleted?
    //Calculate start speeds based on moving axes max start speed constraints.
    int slowest_start_axis = primary_axis;
    unsigned long slowest_start_axis_max_interval = max_interval;
    for(int i = 0; i < NUM_AXIS; i++)
      if (axis_steps_remaining[i] >0 &&
            i != primary_axis &&
            axis_max_interval[i] * axis_steps_remaining[i]/ axis_steps_remaining[slowest_start_axis] > slowest_start_axis_max_interval) {
        slowest_start_axis = i;
        slowest_start_axis_max_interval = axis_max_interval[i];
      }
    for(int i = 0; i < NUM_AXIS; i++)
      if(axis_steps_remaining[i] >0) {
        // multiplying slowest_start_axis_max_interval by axis_steps_remaining[slowest_start_axis]
        // could lead to overflows when we have long distance moves (say, 390625*390625 > sizeof(unsigned long))
        float steps_remaining_ratio = (float) axis_steps_remaining[slowest_start_axis] / axis_steps_remaining[i];
        new_axis_max_intervals[i] = slowest_start_axis_max_interval * steps_remaining_ratio;

        if(i == primary_axis) {
          max_interval = new_axis_max_intervals[i];
          min_speed_steps_per_second = 100000000 / max_interval;
        }
      }
    //Calculate slowest axis plateau time
    float slowest_axis_plateau_time = 0;
    for(int i=0; i < NUM_AXIS ; i++) {
      if(axis_steps_remaining[i] > 0) {
        if(is_print_move && axis_steps_remaining[i] > 0) slowest_axis_plateau_time = max(slowest_axis_plateau_time,
              (100000000.0 / axis_interval[i] - 100000000.0 / new_axis_max_intervals[i]) / (float) axis_steps_per_sqr_second[i]);
        else if(axis_steps_remaining[i] > 0) slowest_axis_plateau_time = max(slowest_axis_plateau_time,
              (100000000.0 / axis_interval[i] - 100000000.0 / new_axis_max_intervals[i]) / (float) axis_travel_steps_per_sqr_second[i]);
      }
    }
    //Now we can calculate the new primary axis acceleration, so that the slowest axis max acceleration is not violated
    steps_per_sqr_second = (100000000.0 / axis_interval[primary_axis] - 100000000.0 / new_axis_max_intervals[primary_axis]) / slowest_axis_plateau_time;
    plateau_steps = (long) ((steps_per_sqr_second / 2.0 * slowest_axis_plateau_time + min_speed_steps_per_second) * slowest_axis_plateau_time);
    #ifdef DEBUG_RAMP_ACCELERATION
      log_int("_RAMP_ACCELERATION - Start speed limiting axis", slowest_start_axis);
      log_ulong("_RAMP_ACCELERATION - Limiting axis start interval", slowest_start_axis_max_interval);
      log_ulong_array("_RAMP_ACCELERATION - Actual step intervals at move start", new_axis_max_intervals, NUM_AXIS);
    #endif
  #endif

  unsigned long steps_done = 0;
  #ifdef RAMP_ACCELERATION
  plateau_steps *= 1.01; // This is to compensate we use discrete intervals
  acceleration_enabled = true;
  unsigned long full_interval = interval;
  if(interval > max_interval) acceleration_enabled = false;
  boolean decelerating = false;
  #endif

  unsigned long start_move_micros = micros();
  for(int i = 0; i < NUM_AXIS; i++) {
    axis_previous_micros[i] = start_move_micros * 100;
  }

  #ifdef DISABLE_CHECK_DURING_TRAVEL
    //If the move time is more than allowed in DISABLE_CHECK_DURING_TRAVEL, let's
    // consider this a print move and perform heat management during it
    if(time_for_move / 1000 > DISABLE_CHECK_DURING_TRAVEL) is_print_move = true;
    //else, if the move is a retract, consider it as a travel move for the sake of this feature
    else if(delta[3]>0 && delta[0] + delta[1] + delta[2] == 0) is_print_move = false;
    #ifdef DEBUG_DISABLE_CHECK_DURING_TRAVEL
      log_bool("_DISABLE_CHECK_DURING_TRAVEL - is_print_move", is_print_move);
    #endif
  #endif

  #ifdef DEBUG_MOVE_TIME
    unsigned long startmove = micros();
  #endif

  //move until no more steps remain
  while(axis_steps_remaining[0] + axis_steps_remaining[1] + axis_steps_remaining[2] + axis_steps_remaining[3] > 0) {
    #ifdef DISABLE_CHECK_DURING_ACC
      if(!accelerating && !decelerating) {
        //If more that HEATER_CHECK_INTERVAL ms have passed since previous heating check, adjust temp
        #ifdef DISABLE_CHECK_DURING_TRAVEL
          if(is_print_move)
        #endif
            manage_heater();
      }
    #else
      #ifdef DISABLE_CHECK_DURING_MOVE
        {} //Do nothing
      #else
        //If more that HEATER_CHECK_INTERVAL ms have passed since previous heating check, adjust temp
        #ifdef DISABLE_CHECK_DURING_TRAVEL
          if(is_print_move)
        #endif
            manage_heater();
      #endif
    #endif
    #ifdef RAMP_ACCELERATION
    //If acceleration is enabled on this move and we are in the acceleration segment, calculate the current interval
    if (acceleration_enabled && steps_done == 0) {
        interval = max_interval;
    } else if (acceleration_enabled && steps_done <= plateau_steps) {
        long current_speed = (long) ((((long) steps_per_sqr_second) / 10000)
	    * ((micros() - start_move_micros)  / 100) + (long) min_speed_steps_per_second);
	    interval = 100000000 / current_speed;
      if (interval < full_interval) {
        accelerating = false;
      	interval = full_interval;
      }
      if (steps_done >= steps_to_take / 2) {
	plateau_steps = steps_done;
	max_speed_steps_per_second = 100000000 / interval;
	accelerating = false;
      }
    } else if (acceleration_enabled && steps_remaining <= plateau_steps) { //(interval > minInterval * 100) {
      if (!accelerating) {
        start_move_micros = micros();
        accelerating = true;
        decelerating = true;
      }
      long current_speed = (long) ((long) max_speed_steps_per_second - ((((long) steps_per_sqr_second) / 10000)
          * ((micros() - start_move_micros) / 100)));
      interval = 100000000 / current_speed;
      if (interval > max_interval)
	interval = max_interval;
    } else {
      //Else, we are just use the full speed interval as current interval
      interval = full_interval;
      accelerating = false;
    }
    #endif

    //If there are x or y steps remaining, perform Bresenham algorithm
    if(axis_steps_remaining[primary_axis]) {
      #if (X_MIN_PIN > -1)
        if(!move_direction[0]) if(READ(X_MIN_PIN) != ENDSTOPS_INVERTING) if(primary_axis==0) break; else if(axis_steps_remaining[0]) axis_steps_remaining[0]=0;
      #endif
      #if (Y_MIN_PIN > -1)
        if(!move_direction[1]) if(READ(Y_MIN_PIN) != ENDSTOPS_INVERTING) if(primary_axis==1) break; else if(axis_steps_remaining[1]) axis_steps_remaining[1]=0;
      #endif
      #if (X_MAX_PIN > -1)
        if(move_direction[0]) if(READ(X_MAX_PIN) != ENDSTOPS_INVERTING) if(primary_axis==0) break; else if(axis_steps_remaining[0]) axis_steps_remaining[0]=0;
      #endif
      #if (Y_MAX_PIN > -1)
        if(move_direction[1]) if(READ(Y_MAX_PIN) != ENDSTOPS_INVERTING) if(primary_axis==1) break; else if(axis_steps_remaining[1]) axis_steps_remaining[1]=0;
      #endif
      #if (Z_MIN_PIN > -1)
        if(!move_direction[2]) if(READ(Z_MIN_PIN) != ENDSTOPS_INVERTING) if(primary_axis==2) break; else if(axis_steps_remaining[2]) axis_steps_remaining[2]=0;
      #endif
      #if (Z_MAX_PIN > -1)
        if(move_direction[2]) if(READ(Z_MAX_PIN) != ENDSTOPS_INVERTING) if(primary_axis==2) break; else if(axis_steps_remaining[2]) axis_steps_remaining[2]=0;
      #endif
      timediff = micros() * 100 - axis_previous_micros[primary_axis];
      if(timediff<0){//check for overflow
        axis_previous_micros[primary_axis]=micros()*100;
        timediff=interval/2; //approximation
      }
      while(((unsigned long)timediff) >= interval && axis_steps_remaining[primary_axis] > 0) {
        steps_done++;
        steps_remaining--;
        axis_steps_remaining[primary_axis]--; timediff -= interval;
        do_step(primary_axis);
        axis_previous_micros[primary_axis] += interval;
        for(int i=0; i < NUM_AXIS; i++) if(i != primary_axis && axis_steps_remaining[i] > 0) {
          axis_error[i] = axis_error[i] - delta[i];
          if(axis_error[i] < 0) {
            do_step(i); axis_steps_remaining[i]--;
            axis_error[i] = axis_error[i] + delta[primary_axis];
          }
        }
        #ifdef STEP_DELAY_RATIO
        if(timediff >= interval) delayMicroseconds(long_step_delay_ratio * interval / 10000);
        #endif
        #ifdef STEP_DELAY_MICROS
        if(timediff >= interval) delayMicroseconds(STEP_DELAY_MICROS);
        #endif
      }
    }
  }
  #ifdef DEBUG_MOVE_TIME
    log_ulong("_MOVE_TIME - This move took", micros()-startmove);
  #endif

  if(DISABLE_X) disable_x();
  if(DISABLE_Y) disable_y();
  if(DISABLE_Z) disable_z();
  if(DISABLE_E) disable_e();

  // Update current position partly based on direction, we probably can combine this with the direction code above...
  for(int i=0; i < NUM_AXIS; i++) {
    if (destination[i] > current_position[i]) current_position[i] = current_position[i] + steps_taken[i] /  axis_steps_per_unit[i];
    else current_position[i] = current_position[i] - steps_taken[i] / axis_steps_per_unit[i];
  }
  //current_position[3]=0;
}

void do_step(int axis) {
  switch(axis){
  case 0:
    WRITE(X_STEP_PIN, HIGH);
    break;
  case 1:
    WRITE(Y_STEP_PIN, HIGH);
    break;
  case 2:
    WRITE(Z_STEP_PIN, HIGH);
    break;
  case 3:
    WRITE(E_STEP_PIN, HIGH);
    break;
  }
  steps_taken[axis]+=1;
  WRITE(X_STEP_PIN, LOW);
  WRITE(Y_STEP_PIN, LOW);
  WRITE(Z_STEP_PIN, LOW);
  WRITE(E_STEP_PIN, LOW);
}

#define HEAT_INTERVAL 250
#ifdef HEATER_USES_MAX6675
unsigned long max6675_previous_millis = 0;
int max6675_temp = 2000;

int read_max6675()
{
  if (millis() - max6675_previous_millis < HEAT_INTERVAL)
    return max6675_temp;

  max6675_previous_millis = millis();

  max6675_temp = 0;

  #ifdef	PRR
    PRR &= ~(1<<PRSPI);
  #elif defined PRR0
    PRR0 &= ~(1<<PRSPI);
  #endif

  SPCR = (1<<MSTR) | (1<<SPE) | (1<<SPR0);

  // enable TT_MAX6675
  WRITE(MAX6675_SS, 0);

  // ensure 100ns delay - a bit extra is fine
  delay(1);

  // read MSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp = SPDR;
  max6675_temp <<= 8;

  // read LSB
  SPDR = 0;
  for (;(SPSR & (1<<SPIF)) == 0;);
  max6675_temp |= SPDR;

  // disable TT_MAX6675
  WRITE(MAX6675_SS, 1);

  if (max6675_temp & 4)
  {
    // thermocouple open
    max6675_temp = 2000;
  }
  else
  {
    max6675_temp = max6675_temp >> 3;
  }

  return max6675_temp;
}
#endif

void reset_status()
{
    status = STATUS_OK;
    error_code = ERROR_CODE_NO_ERROR;
}

void wait_for_temp()
{
    unsigned long codenum; //throw away variable

    if (error_code == ERROR_CODE_HOTEND_TEMPERATURE)
    {
        reset_status();
    }

    #ifdef WATCHPERIOD
        if(target_raw>current_raw)
        {
            watchmillis = max(1,millis());
            watch_raw = current_raw;
        }
        else
        {
            watchmillis = 0;
        }
    #endif
    codenum = millis();
    while ((current_raw < target_raw) && (status == STATUS_OK))
    {
        if( (millis() - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
        {
            SerialMgr.cur()->print("T:");
            SerialMgr.cur()->println( analog2temp(current_raw) );
            codenum = millis();
        }
        manage_heater();
    }
}

void manage_heater()
{
  if((millis() - previous_millis_heater) < HEATER_CHECK_INTERVAL )
    return;
  previous_millis_heater = millis();
  #ifdef HEATER_USES_THERMISTOR
    current_raw = analogRead(TEMP_0_PIN);
    #ifdef DEBUG_HEAT_MGMT
      log_int("_HEAT_MGMT - analogRead(TEMP_0_PIN)", current_raw);
      log_int("_HEAT_MGMT - NUMTEMPS", NUMTEMPS);
    #endif
    // When using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target,
    // this switches it up so that the reading appears lower than target for the control logic.
    current_raw = 1023 - current_raw;
  #elif defined HEATER_USES_AD595
    current_raw = analogRead(TEMP_0_PIN);
  #elif defined HEATER_USES_MAX6675
    current_raw = read_max6675();
  #endif
  #ifdef SMOOTHING
  if (!nma) nma = SMOOTHFACTOR * current_raw;
  nma = (nma + current_raw) - (nma / SMOOTHFACTOR);
  current_raw = nma / SMOOTHFACTOR;
  #endif
  #ifdef WATCHPERIOD
    if(watchmillis && millis() - watchmillis > WATCHPERIOD){
        if(watch_raw + 1 >= current_raw){
            target_raw = 0;
            WRITE(HEATER_0_PIN,LOW);
        }else{
            watchmillis = 0;
        }
    }
  #endif
  #ifdef MINTEMP
    if(current_raw <= minttemp)
    {
      status = STATUS_ERROR;
      error_code = ERROR_CODE_HOTEND_TEMPERATURE;
      target_raw = 0;
      BBB();
    }
  #endif
  #ifdef MAXTEMP
    if(current_raw >= maxttemp)
    {
      status = STATUS_ERROR;
      error_code = ERROR_CODE_HOTEND_TEMPERATURE_HIGH;
      target_raw = 0;
      BBB();
    }
  #endif
  #if (TEMP_0_PIN > -1) || defined (HEATER_USES_MAX6675) || defined (HEATER_USES_AD595)
    #ifdef PIDTEMP
      error = target_raw - current_raw;
      pTerm = Kp * error;
      temp_iState += error;
      temp_iState = constrain(temp_iState, temp_iState_min, temp_iState_max);
      iTerm = Ki * temp_iState;
      dTerm = Kd * (current_raw - temp_dState);
      temp_dState = current_raw;
      output=constrain(pTerm + iTerm - dTerm, 0, pid_max);
      analogWrite(HEATER_0_PIN, output);
    #else
      if(current_raw >= target_raw)
      {
        WRITE(HEATER_0_PIN,LOW);
      }
      else
      {
        WRITE(HEATER_0_PIN,HIGH);
      }
    #endif
  #endif

  //LED handling is put in here for convinient handling
  #if LED_PIN>-1
    if (status==STATUS_ERROR) //on error, blink fast
    {
        TOGGLE(LED_PIN);
    }
    else if (target_raw > minttemp) //on heated hotend, blink slow
    {
        if ((led_counter++) > 4)
        {
            led_counter = 0;
            TOGGLE(LED_PIN);
        }
    }
    else
    {
        WRITE(LED_PIN,HIGH); //In idle, just on
    }
  #endif



  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();
  #ifndef TEMP_1_PIN
    return;
  #endif
  #if TEMP_1_PIN == -1
    return;
  #else

  #ifdef BED_USES_THERMISTOR

    current_bed_raw = analogRead(TEMP_1_PIN);
    #ifdef DEBUG_HEAT_MGMT
      log_int("_HEAT_MGMT - analogRead(TEMP_1_PIN)", current_bed_raw);
      log_int("_HEAT_MGMT - BNUMTEMPS", BNUMTEMPS);
    #endif
    // If using thermistor, when the heater is colder than targer temp, we get a higher analog reading than target,
    // this switches it up so that the reading appears lower than target for the control logic.
    current_bed_raw = 1023 - current_bed_raw;

    #ifdef MINTEMP
      if(current_bed_raw <= minttemp)
      {
        status = STATUS_ERROR;
        error_code = ERROR_CODE_BED_TEMPERATURE;
        target_bed_raw = 0;
        BBB();
      }
    #endif
    #ifdef MAXTEMPBED
      if(current_bed_raw >= maxbtemp)
      {
        status = STATUS_ERROR;
        error_code = ERROR_CODE_BED_TEMPERATURE_HIGH;
        target_bed_raw = 0;
        BBB();
      }
    #endif

  #elif defined BED_USES_AD595
    current_bed_raw = analogRead(TEMP_1_PIN);

  #endif


    if(current_bed_raw >= target_bed_raw)
    {
      WRITE(HEATER_1_PIN,LOW);
    }
    else
    {
      WRITE(HEATER_1_PIN,HIGH);
    }
    #endif
}


int temp2analogu(int celsius, const short table[][2], int numtemps, int source) {
  #if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
  if(source==1){
    int raw = 0;
    byte i;

    for (i=1; i<numtemps; i++)
    {
      if (table[i][1] < celsius)
      {
        raw = table[i-1][0] +
          (celsius - table[i-1][1]) *
          (table[i][0] - table[i-1][0]) /
          (table[i][1] - table[i-1][1]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == numtemps) raw = table[i-1][0];

    return 1023 - raw;
    }
  #elif defined (HEATER_USES_AD595) || defined (BED_USES_AD595)
    if(source==2)
        return celsius * 1024 / (500);
  #elif defined (HEATER_USES_MAX6675) || defined (BED_USES_MAX6675)
    if(source==3)
        return celsius * 4;
  #endif
  return -1;
}

int analog2tempu(int raw,const short table[][2], int numtemps, int source) {
  #if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
    if(source==1){
    int celsius = 0;
    byte i;

    raw = 1023 - raw;

    for (i=1; i<numtemps; i++)
    {
      if (table[i][0] > raw)
      {
        celsius  = table[i-1][1] +
          (raw - table[i-1][0]) *
          (table[i][1] - table[i-1][1]) /
          (table[i][0] - table[i-1][0]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == numtemps) celsius = table[i-1][1];

    return celsius;
    }
  #elif defined (HEATER_USES_AD595) || defined (BED_USES_AD595)
    if(source==2)
        return raw * 500 / 1024;
  #elif defined (HEATER_USES_MAX6675) || defined (BED_USES_MAX6675)
    if(source==3)
        return raw / 4;
  #endif
  return -1;
}


inline void kill()
{
  #if TEMP_0_PIN > -1
  target_raw=0;
  WRITE(HEATER_0_PIN,LOW);
  #endif
  #if TEMP_1_PIN > -1
  target_bed_raw=0;
  if(HEATER_1_PIN > -1) WRITE(HEATER_1_PIN,LOW);
  #endif
  disable_x();
  disable_y();
  disable_z();
  disable_e();

  if(PS_ON_PIN > -1) pinMode(PS_ON_PIN,INPUT);

}

inline void manage_inactivity(byte debug) {
if( (millis()-previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill();
if( (millis()-previous_millis_cmd) >  stepper_inactive_time ) if(stepper_inactive_time) { disable_x(); disable_y(); disable_z(); disable_e(); }
}

#ifdef DEBUG
void log_message(char*   message) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->println(message);
}

void log_bool(char* message, bool value) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": "); SerialMgr.cur()->println(value);
}

void log_int(char* message, int value) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": "); SerialMgr.cur()->println(value);
}

void log_long(char* message, long value) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": "); SerialMgr.cur()->println(value);
}

void log_float(char* message, float value) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": "); SerialMgr.cur()->println(value);
}

void log_uint(char* message, unsigned int value) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": "); SerialMgr.cur()->println(value);
}

void log_ulong(char* message, unsigned long value) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": "); SerialMgr.cur()->println(value);
}

void log_int_array(char* message, int value[], int array_lenght) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": {");
  for(int i=0; i < array_lenght; i++){
    SerialMgr.cur()->print(value[i]);
    if(i != array_lenght-1) SerialMgr.cur()->print(", ");
  }
  SerialMgr.cur()->println("}");
}

void log_long_array(char* message, long value[], int array_lenght) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": {");
  for(int i=0; i < array_lenght; i++){
    SerialMgr.cur()->print(value[i]);
    if(i != array_lenght-1) SerialMgr.cur()->print(", ");
  }
  SerialMgr.cur()->println("}");
}

void log_float_array(char* message, float value[], int array_lenght) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": {");
  for(int i=0; i < array_lenght; i++){
    SerialMgr.cur()->print(value[i]);
    if(i != array_lenght-1) SerialMgr.cur()->print(", ");
  }
  SerialMgr.cur()->println("}");
}

void log_uint_array(char* message, unsigned int value[], int array_lenght) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": {");
  for(int i=0; i < array_lenght; i++){
    SerialMgr.cur()->print(value[i]);
    if(i != array_lenght-1) SerialMgr.cur()->print(", ");
  }
  SerialMgr.cur()->println("}");
}

void log_ulong_array(char* message, unsigned long value[], int array_lenght) {
  SerialMgr.cur()->print("DEBUG"); SerialMgr.cur()->print(message); SerialMgr.cur()->print(": {");
  for(int i=0; i < array_lenght; i++){
    SerialMgr.cur()->print(value[i]);
    if(i != array_lenght-1) SerialMgr.cur()->print(", ");
  }
  SerialMgr.cur()->println("}");
}
#endif

//////////////////////////////////////////////////////////////////////////
//
// ErrorBleepCodes() 
//
// If this is called then we are in trouble. 
// There are 4 error codes:
// HOTEND low temperture  - 1 Long beep followed by 1 short beep
// BED low temperture     - 1 long beep followed by 2 short beeps
// HOTEND high temperture - 1 long beep followed by 3 short beeps
// BED high temperture    - 1 long beep followed by 4 short beeps
// Suusi M-M 2017/08/01
//
//////////////////////////////////////////////////////////////////////////

void ErrorBleepCodes(){
  
  V3_I2C_Command( V3_LONG_BEEP, false ) ;                         // beep long
  delay(2000);
  
  for (int i = 1 ; i <= error_code ; i++) {
    V3_I2C_Command( V3_SHORT_BEEP, false ) ;                      // beep short x1
    delay(1000);                                                  // 1 second delay
  }
  
#ifdef LCD_SUPPORTED
  StatusScreen();                                                 // display X Y Z and error on LCD
#endif
  
  SerialMgr.cur()->print("EC:");                                  // Now send error code and error string to the terminal
  SerialMgr.cur()->print(error_code);
  SerialMgr.cur()->print(", ");
  SerialMgr.cur()->println(error_code_str[error_code]);
  
  delay(2000);                                                    // delay 2 seconds
}

//////////////////////////////////////////////////////////////////////////
//
// BBB() 
// 
// switches everything off and loops forever
// new with the firmware update 2017/04/01
//
//////////////////////////////////////////////////////////////////////////

void BBB(){
  // BBB short beep x3 chris 2017-04-01
  // sorry chris i dont think so the code you used was for 1 short beep
  // but in practice it was 1 continus beep due to the ininte loop
                                                              
  V3_I2C_Command( V3_NOZZLE_RED_FLASH, false ) ;                  // nozzle RGB LED Red Flashing
  V3_I2C_Command( V3_BUTTON_RED_FLASH, false ) ;                  // button RGB LED Red Flashing
  while(1){
                                                                  // loop forever
    pinMode(HEATER_0_PIN, OUTPUT);
    pinMode(HEATER_1_PIN, OUTPUT);
    target_bed_raw = 0;                                           // stop bed heater
    target_raw = 0;                                               // stop hotend heater
    digitalWrite(HEATER_0_PIN,LOW);                               // stop hotend heater
    digitalWrite(HEATER_1_PIN,LOW);                               // stop bed heater 
    
//    V3_I2C_Command( V3_SHORT_BEEP, false ) ;                      // beep short x1
    ErrorBleepCodes();                                            // give an audiabe clue to the error
    
    disable_x(); disable_y(); disable_z(); disable_e();           //stop motors
    analogWrite(FAN_PIN, 0);                                      //stop fan
    WRITE(FAN_PIN, LOW); 
  }
}

//////////////////////////////////////////////////////////////////////////
//
// check_heater() 
//
// Check the Extruder and Bed are within safe limits
//
// calls  BBB() if there is an error
//
// new with the firmware update 2017/04/01
//
//////////////////////////////////////////////////////////////////////////

void check_heater()
{
  // check the nozzle temparatue is within safe limits
  current_raw = analogRead(TEMP_0_PIN);
  current_raw = 1023 - current_raw;                               // NTC
  if(current_raw <= minttemp)
  {
    // Temperature below MINTEMP 
    status = STATUS_ERROR;
    error_code = ERROR_CODE_HOTEND_TEMPERATURE;
    target_raw = 0;                                               // switch off the Extruder target
    BBB();                                                        // call the error handler
  }
  if(current_raw >= maxttemp) 
  {
    // Temperature above MAXTEMP 
    status = STATUS_ERROR;
    error_code = ERROR_CODE_HOTEND_TEMPERATURE_HIGH;
    target_raw = 0;                                               // switch off the Extruder target
    BBB();                                                        // call the error handler
  }

  // check the bed temparature is within the safe limits
  current_bed_raw = analogRead(TEMP_1_PIN);
  current_bed_raw = 1023 - current_bed_raw;                       // NTC
  if(current_bed_raw <= minttemp) 
  {
    // bed temperature below MINTEMP
    status = STATUS_ERROR;
    error_code = ERROR_CODE_BED_TEMPERATURE;
    target_bed_raw = 0;                                           // switch off the Extruder target
    BBB();
  }
  if(current_bed_raw >= maxbtemp) 
  {
    // bed temparature above MAXTEMPBED
    status = STATUS_ERROR;
    error_code = ERROR_CODE_BED_TEMPERATURE_HIGH;
    target_bed_raw = 0;                                           // switch off the Extruder target
    BBB();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// void V3_I2C_Command ( int iCommand, boolean bEchoCommand )
//
// 
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void V3_I2C_Command ( int iCommand, boolean bEchoCommand )
{
  char szChar[10];                            // workspace for echo string
  Wire.beginTransmission( V3_I2C_DEVICE );    // open comms with V3 I2C device
  Wire.send(iCommand);                        // Send command
  Wire.endTransmission();                     // end transmission
  if( bEchoCommand ) {
    sprintf( szChar, "M%d OK", iCommand );    // create echo string
    SerialMgr.cur()->println(szChar);         // echo result to serial manager
  }
}

