#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// Uncomment ONE of the next three lines - the one for your RepRap machine
#define V3

// change to 0 if you do not need a offset 20160329
#define X_OFFSET 8
#define Y_OFFSET 12

// BASIC SETTINGS: select your board type, thermistor type, axis scaling, and endstop configuration

//// The following define selects which electronics board you have. Please choose the one that matches your setup
// MEGA/RAMPS up to 1.2  = 3,
// RAMPS 1.3 = 33
// Gen6 = 5,
// Sanguinololu up to 1.1 = 6
// Sanguinololu 1.2 and above, and Melzi = 62
#define MOTHERBOARD 62

//// Thermistor settings:
// 1 is 100k thermistor
// 2 is 200k thermistor
// 3 is mendel-parts thermistor
// 4 is 10k thermistor
#define THERMISTORHEATER 1
#define THERMISTORBED 1

//// Calibration variables
// X, Y, Z, E steps per unit - Metric Prusa Mendel with Wade extruder:
//float axis_steps_per_unit[] = {91.4286, 91.4286,4000,910};
//float axis_steps_per_unit[] = {80.00, 80.00,2560,910};

//// Calibration variables
// X, Y, Z, E steps per unit - String with extruder
//float axis_steps_per_unit[] = {78.82, 78.82,78.82,99.6150}; //V3-Z-belt
//float axis_steps_per_unit[] = {78.82, 78.82,400,99.6150}; //V3-Z-screw
float axis_steps_per_unit[] = {78.82, 78.82,2560,99.6150}; //V3-Z-screw2

//// Endstop Settings
//#define ENDSTOPPULLUPS 1 // Comment this out (using // at the start of the line) to disable the endstop pullup resistors
// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const bool ENDSTOPS_INVERTING = true; //set to true to invert the logic of the endstops
//If your axes are only moving in one direction, make sure the endstops are connected properly.
//If your axes move in one direction ONLY when the endstops are triggered, set ENDSTOPS_INVERTING to true here

// This determines the communication speed of the printer
#define BAUDRATE 115200

// Comment out (using // at the start of the line) to disable SD support:
#define SDSUPPORT 1

// Uncomment to make run init.g from SD on boot
#define SDINITFILE

// Comment out (using // at the start of the line) to disable Bluetooth support:
//#define BLUETOOTH
#ifdef BLUETOOTH
	#define BLUETOOTH_SERIAL Serial1
#endif


#ifndef V3
#error Uncomment one of #define V3 at the start of the file Configuration.h
#endif


//// ADVANCED SETTINGS - to tweak parameters

//#include "thermistortables.h"

// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
#define X_ENABLE_ON 0
#define Y_ENABLE_ON 0
#define Z_ENABLE_ON 0
#define E_ENABLE_ON 0

// Disables axis when it's not being used.
const bool DISABLE_X = false;
const bool DISABLE_Y = false;
const bool DISABLE_Z = false;
const bool DISABLE_E = false;

// Inverting axis direction
const bool INVERT_X_DIR = false;
const bool INVERT_Y_DIR = false;
const bool INVERT_Z_DIR = false;
const bool INVERT_E_DIR = true;

//// ENDSTOP SETTINGS:
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR 1

const bool min_software_endstops = false; //If true, axis won't move to coordinates less than zero.
const bool max_software_endstops = true;  //If true, axis won't move to coordinates greater than the defined lengths below.

#ifdef V3
const int X_MAX_LENGTH = 140;
const int Y_MAX_LENGTH = 140;
const int Z_MAX_LENGTH = 130;
#endif

/*#define Z_INCREMENT 0.05*/

//// MOVEMENT SETTINGS
const int NUM_AXIS = 4; // The axis order in all axis related arrays is X, Y, Z, E
//float max_feedrate[] = {3000, 3000, 800, 10000}; //V3 Z-belt
float max_feedrate[] = {3000, 3000, 350, 10000}; //V3 Z-screw
//float homing_feedrate[] = {1500,1500,800}; //V3 Z-blet
float homing_feedrate[] = {1500,1500,350}; //V3 Z-screw
bool axis_relative_modes[] = {false, false, false, false};

// Min step delay in microseconds. If you are experiencing missing steps, try to raise the delay microseconds, but be aware this
// If you enable this, make sure STEP_DELAY_RATIO is disabled.
//#define STEP_DELAY_MICROS 1

// Step delay over interval ratio. If you are still experiencing missing steps, try to uncomment the following line, but be aware this
// If you enable this, make sure STEP_DELAY_MICROS is disabled. (except for Gen6: both need to be enabled.)
//#define STEP_DELAY_RATIO 0.25

// Comment this to disable ramp acceleration
#define RAMP_ACCELERATION 1

//// Acceleration settings
#ifdef RAMP_ACCELERATION
// X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
float max_start_speed_units_per_second[] = {25.0,25.0,25.0,10.0};
//long max_acceleration_units_per_sq_second[] = {1000,1000,1000,250}; // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts. V3 Z-blet
long max_acceleration_units_per_sq_second[] = {1000,1000,50,250}; // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts. V3 Z-screw
//long max_travel_acceleration_units_per_sq_second[] = {500,500,500,500}; // X, Y, Z max acceleration in mm/s^2 for travel moves. V3 Z-blet
long max_travel_acceleration_units_per_sq_second[] = {500,500,50,500}; // X, Y, Z max acceleration in mm/s^2 for travel moves. V3 Z-screw
#endif

// Machine UUID
// This may be useful if you have multiple machines and wish to identify them by using the M115 command.
// By default we set it to zeros.
char uuid[] = "00000000-0000-0000-0000-000000000000";


//// AD595 THERMOCOUPLE SUPPORT UNTESTED... USE WITH CAUTION!!!!

//// PID settings:
// Uncomment the following line to enable PID support. This is untested and could be disastrous. Be careful.
#define PIDTEMP 1
#ifdef PIDTEMP
int pid_max = 255; // limits current to nozzle
int pid_i_max = 80;//130;//125;
double Kp = 2;//1.10;
double Ki = 0.01;
double Kd = 20.0;//0.01;
#endif

//M109 target window - machine will deem to have reached target temperature when nozzle reaches Temp = target - NZONE.
int nzone = 5;//2;

//#define DEBUG_PID

#define FAN_INIT 200

// How often should the heater check for new temp readings, in milliseconds
#define HEATER_CHECK_INTERVAL 112
#define BED_CHECK_INTERVAL 5000
// Comment the following line to enable heat management during acceleration
//#define DISABLE_CHECK_DURING_ACC
#ifndef DISABLE_CHECK_DURING_ACC
  // Uncomment the following line to disable heat management during moves
  //#define DISABLE_CHECK_DURING_MOVE
#endif
// Uncomment the following line to disable heat management during travel moves (and extruder-only moves, eg: retracts), strongly recommended if you are missing steps mid print.
// Probably this should remain commented if are using PID.
// It also defines the max milliseconds interval after which a travel move is not considered so for the sake of this feature.
//#define DISABLE_CHECK_DURING_TRAVEL 1000

//// Temperature smoothing - only uncomment this if your temp readings are noisy (Gen6 without EvdZ's 5V hack)
//#define SMOOTHING 1
//#define SMOOTHFACTOR 16 //best to use a power of two here - determines how many values are averaged together by the smoothing algorithm

//// Experimental watchdog and minimal temp
// The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
//#define WATCHPERIOD 5000 //5 seconds

//// The minimal temperature defines the temperature below which the heater will not be enabled
#define MINTEMP 5

//// Experimental max temp
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define MAXTEMP 270
#define MAXTEMPBED 130

// Select one of these only to define how the nozzle temp is read.
#define HEATER_USES_THERMISTOR
//#define HEATER_USES_AD595
//#define HEATER_USES_MAX6675

// Select one of these only to define how the bed temp is read.
#define BED_USES_THERMISTOR
//#define BED_USES_AD595

// Uncomment the following line to enable debugging. You can better control debugging below the following line
//#define DEBUG
#ifdef DEBUG
  //#define DEBUG_PREPARE_MOVE //Enable this to debug prepare_move() function
  //#define DEBUG_BRESENHAM //Enable this to debug the Bresenham algorithm
  //#define DEBUG_RAMP_ACCELERATION //Enable this to debug all constant acceleration info
  //#define DEBUG_MOVE_TIME //Enable this to time each move and print the result
  //#define DEBUG_HEAT_MGMT //Enable this to debug heat management. WARNING, this will cause axes to jitter!
  //#define DEBUG_DISABLE_CHECK_DURING_TRAVEL //Debug the namesake feature, see above in this file
#endif

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://reprap.org/wiki/Thermistor)
// Made with the online thermistor table generator by nathan7 at http://calculator.josefprusa.cz/
// r0: 100000
// t0: 25
// r1: 0
// r2: 10000
// beta: 3950
// max adc: 1023
#define NUMTEMPS 100
short temptable[NUMTEMPS][2] = {
	{	1	,	500	},
	{	12	,	300	},
	{	14	,	288	},
	{	23	,	254	},
	{	31	,	234	},
	{	40	,	220	},
	{	66	,	192	},
	{	80	,	182	},
	{	92	,	175	},
	{	109	,	167	},
	{	120	,	162	},
	{	124	,	160	},
	{	131	,	157	},
	{	142	,	153	},
	{	147	,	151	},
	{	156	,	148	},
	{	163	,	146	},
	{	181	,	141	},
	{	196	,	137	},
	{	218	,	132	},
	{	278	,	120	},
	{	295	,	117	},
	{	307	,	115	},
	{	319	,	113	},
	{	325	,	112	},
	{	338	,	110	},
	{	345	,	109	},
	{	352	,	108	},
	{	359	,	107	},
	{	366	,	106	},
	{	373	,	105	},
	{	380	,	104	},
	{	387	,	103	},
	{	394	,	102	},
	{	401	,	101	},
	{	409	,	100	},
	{	416	,	99	},
	{	424	,	98	},
	{	432	,	97	},
	{	439	,	96	},
	{	447	,	95	},
	{	455	,	94	},
	{	462	,	93	},
	{	470	,	92	},
	{	478	,	91	},
	{	486	,	90	},
	{	494	,	89	},
	{	502	,	88	},
	{	511	,	87	},
	{	519	,	86	},
	{	527	,	85	},
	{	535	,	84	},
	{	543	,	83	},
	{	552	,	82	},
	{	560	,	81	},
	{	568	,	80	},
	{	576	,	79	},
	{	585	,	78	},
	{	593	,	77	},
	{	601	,	76	},
	{	610	,	75	},
	{	618	,	74	},
	{	626	,	73	},
	{	634	,	72	},
	{	643	,	71	},
	{	651	,	70	},
	{	659	,	69	},
	{	667	,	68	},
	{	675	,	67	},
	{	691	,	65	},
	{	707	,	63	},
	{	714	,	62	},
	{	722	,	61	},
	{	737	,	59	},
	{	744	,	58	},
	{	752	,	57	},
	{	766	,	55	},
	{	773	,	54	},
	{	787	,	52	},
	{	794	,	51	},
	{	807	,	49	},
	{	814	,	48	},
	{	826	,	46	},
	{	833	,	45	},
	{	845	,	43	},
	{	856	,	41	},
	{	862	,	40	},
	{	872	,	38	},
	{	883	,	36	},
	{	892	,	34	},
	{	902	,	32	},
	{	915	,	29	},
	{	923	,	27	},
	{	935	,	24	},
	{	942	,	22	},
	{	952	,	19	},
	{	963	,	15	},
	{	973	,	11	},
	{	981	,	7	},
	{	990	,	2	},
};
// Thermistor lookup table for RS thermistor 198-961
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=10000 --beta=3960 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 10000
// beta: 3960
// max adc: 1023
/*#define NUMTEMPS 30
short temptable[NUMTEMPS][2] = {
   {1, 704},
   {15, 280},
   {21, 266},
   {41, 234},
   {61, 208},
   {81, 191},
   {101, 178},
   {121, 168},
   {141, 159},
   {161, 152},
   {181, 146},
   {221, 135},
   {261, 126},
   {301, 118},
   {341, 111},
   {381, 105},
   {421, 99},
   {461, 94},
   {501, 88},
   {541, 83},
   {581, 78},
   {621, 73},
   {661, 68},
   {741, 58},
   {781, 52},
   {821, 46},
   {861, 40},
   {901, 32},
   {981, 7},
   {1008, 0}
};
*/
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=3960 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 4700
// beta: 3960
// max adc: 1023
/*#define NUMTEMPS 30
short temptable[NUMTEMPS][2] = {
   {1, 929},
   {36, 299},
   {71, 246},
   {106, 217},
   {141, 198},
   {176, 184},
   {211, 173},
   {246, 163},
   {281, 154},
   {316, 147},
   {351, 140},
   {386, 134},
   {421, 128},
   {456, 122},
   {491, 117},
   {526, 112},
   {561, 107},
   {596, 102},
   {631, 97},
   {666, 91},
   {701, 86},
   {736, 81},
   {771, 76},
   {806, 70},
   {841, 63},
   {876, 56},
   {911, 48},
   {946, 38},
   {981, 23},
   {1016, 0}
};
*/
#ifdef REPRAPPRO_HUXLEY
//bed temp table, 100k EPCOS
#define BNUMTEMPS 20
const short bedtemptable[BNUMTEMPS][2] = {
   {1, 549},
   {11, 274},
   {21, 228},
   {31, 204},
   {41, 188},
   {51, 176},
   {61, 166},
   {71, 159},
   {81, 152},
   {91, 146},
   {101, 141},
   {111, 137},
   {121, 133},
   {131, 131},
   {141, 129},
   {151, 127},
   {161, 125},
   {171, 123},
   {181, 120},
   {191, 119},
   {201, 117},
   {211, 115},
   {221, 113},
   {231, 112},
   {241, 110},
   {251, 109},
   {261, 108},
   {271, 107},
   {281, 106},
   {291, 105},
   {301, 104},
   {311, 103},
   {321, 102},
   {331, 101},
   {341, 100},
   {351, 99},
   {361, 98},
   {371, 97},
   {381, 96},
   {391, 95},
   {401, 94},
   {411, 93},
   {421, 92},
   {431, 91},
   {441, 90},
   {451, 89},
   {461, 88},
   {471, 87},
   {481, 86},
   {491, 85},
   {501, 84},
   {511, 83},
   {521, 82},
   {531, 81},
   {541, 80},
   {551, 79},
   {561, 78},
   {571, 77},
   {581, 76},
   {591, 75},
   {601, 74},
   {611, 73},
   {621, 72},
   {631, 71},
   {641, 70},
   {651, 69},
   {661, 68},
   {671, 67},
   {681, 66},
   {691, 65},
   {701, 63},
   {711, 62},
   {721, 61},
   {731, 59},
   {741, 58},
   {751, 57},
   {761, 55},
   {771, 54},
   {781, 52},
   {791, 51},
   {801, 49},
   {811, 48},
   {821, 46},
   {831, 45},
   {841, 43},
   {851, 41},
   {861, 40},
   {871, 38},
   {881, 36},
   {891, 34},
   {901, 32},
   {911, 29},
   {921, 27},
   {931, 24},
   {941, 22},
   {951, 19},
   {961, 15},
   {971, 11},
   {981, 7},
   {991, 2},
};
#endif

//#ifdef REPRAPPRO_MENDEL
#ifdef RP3D.COM_PANGU
// Thermistor lookup table for RepRap Temperature Sensor Boards (http://reprap.org/wiki/Thermistor)
// Made with the online thermistor table generator by nathan7 at http://calculator.josefprusa.cz/
// r0: 100000
// t0: 25
// r1: 0
// r2: 10000
// beta: 3950
// max adc: 1023
#define BNUMTEMPS 100
const short bedtemptable[BNUMTEMPS][2] = {
   {1, 549},
   {11, 274},
   {21, 228},
   {31, 204},
   {41, 188},
   {51, 176},
   {61, 166},
   {71, 159},
   {81, 152},
   {91, 146},
   {101, 141},
   {111, 137},
   {121, 133},
   {131, 131},
   {141, 129},
   {151, 127},
   {161, 125},
   {171, 123},
   {181, 120},
   {191, 119},
   {201, 117},
   {211, 115},
   {221, 113},
   {231, 112},
   {241, 110},
   {251, 109},
   {261, 108},
   {271, 107},
   {281, 106},
   {291, 105},
   {301, 104},
   {311, 103},
   {321, 102},
   {331, 101},
   {341, 100},
   {351, 99},
   {361, 98},
   {371, 97},
   {381, 96},
   {391, 95},
   {401, 94},
   {411, 93},
   {421, 92},
   {431, 91},
   {441, 90},
   {451, 89},
   {461, 88},
   {471, 87},
   {481, 86},
   {491, 85},
   {501, 84},
   {511, 83},
   {521, 82},
   {531, 81},
   {541, 80},
   {551, 79},
   {561, 78},
   {571, 77},
   {581, 76},
   {591, 75},
   {601, 74},
   {611, 73},
   {621, 72},
   {631, 71},
   {641, 70},
   {651, 69},
   {661, 68},
   {671, 67},
   {681, 66},
   {691, 65},
   {701, 63},
   {711, 62},
   {721, 61},
   {731, 59},
   {741, 58},
   {751, 57},
   {761, 55},
   {771, 54},
   {781, 52},
   {791, 51},
   {801, 49},
   {811, 48},
   {821, 46},
   {831, 45},
   {841, 43},
   {851, 41},
   {861, 40},
   {871, 38},
   {881, 36},
   {891, 34},
   {901, 32},
   {911, 29},
   {921, 27},
   {931, 24},
   {941, 22},
   {951, 19},
   {961, 15},
   {971, 11},
   {981, 7},
   {991, 2},
};

// Thermistor lookup table for RS thermistor 198-961
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=10000 --beta=3960 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 10000
// beta: 3960
// max adc: 1023
/*#define BNUMTEMPS 30
const short bedtemptable[BNUMTEMPS][2] = {
   {1, 704},
   {15, 280},
   {21, 266},
   {41, 234},
   {61, 208},
   {81, 191},
   {101, 178},
   {121, 168},
   {141, 159},
   {161, 152},
   {181, 146},
   {221, 135},
   {261, 126},
   {301, 118},
   {341, 111},
   {381, 105},
   {421, 99},
   {461, 94},
   {501, 88},
   {541, 83},
   {581, 78},
   {621, 73},
   {661, 68},
   {741, 58},
   {781, 52},
   {821, 46},
   {861, 40},
   {901, 32},
   {981, 7},
   {1008, 0}
};
*/
/*
// RS thermistor 484-0149; EPCOS B57550G103J
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=10000 --t0=25 --r1=0 --r2=4700 --beta=3480 --max-adc=1023
// r0: 10000
// t0: 25
// r1: 0
// r2: 4700
// beta: 3480
// max adc: 1023
#define BNUMTEMPS 20
const short bedtemptable[BNUMTEMPS][2] = {
   {1, 599},
   {54, 160},
   {107, 123},
   {160, 103},
   {213, 90},
   {266, 79},
   {319, 70},
   {372, 62},
   {425, 55},
   {478, 49},
   {531, 43},
   {584, 37},
   {637, 31},
   {690, 25},
   {743, 19},
   {796, 12},
   {849, 5},
   {902, -3},
   {955, -16},
   {1008, -42}
};
*/
#endif

#ifdef V3
// Thermistor lookup table for RepRap Temperature Sensor Boards (http://reprap.org/wiki/Thermistor)
// Made with the online thermistor table generator by nathan7 at http://calculator.josefprusa.cz/
// r0: 100000
// t0: 25
// r1: 0
// r2: 10000
// beta: 3950
// max adc: 1023
#define BNUMTEMPS 100
const short bedtemptable[BNUMTEMPS][2] = {
	{	1	,	500	},
	{	12	,	300	},
	{	14	,	288	},
	{	23	,	254	},
	{	31	,	234	},
	{	40	,	220	},
	{	66	,	192	},
	{	80	,	182	},
	{	92	,	175	},
	{	109	,	167	},
	{	120	,	162	},
	{	124	,	160	},
	{	131	,	157	},
	{	142	,	153	},
	{	147	,	151	},
	{	156	,	148	},
	{	163	,	146	},
	{	181	,	141	},
	{	196	,	137	},
	{	218	,	132	},
	{	278	,	120	},
	{	295	,	117	},
	{	307	,	115	},
	{	319	,	113	},
	{	325	,	112	},
	{	338	,	110	},
	{	345	,	109	},
	{	352	,	108	},
	{	359	,	107	},
	{	366	,	106	},
	{	373	,	105	},
	{	380	,	104	},
	{	387	,	103	},
	{	394	,	102	},
	{	401	,	101	},
	{	409	,	100	},
	{	416	,	99	},
	{	424	,	98	},
	{	432	,	97	},
	{	439	,	96	},
	{	447	,	95	},
	{	455	,	94	},
	{	462	,	93	},
	{	470	,	92	},
	{	478	,	91	},
	{	486	,	90	},
	{	494	,	89	},
	{	502	,	88	},
	{	511	,	87	},
	{	519	,	86	},
	{	527	,	85	},
	{	535	,	84	},
	{	543	,	83	},
	{	552	,	82	},
	{	560	,	81	},
	{	568	,	80	},
	{	576	,	79	},
	{	585	,	78	},
	{	593	,	77	},
	{	601	,	76	},
	{	610	,	75	},
	{	618	,	74	},
	{	626	,	73	},
	{	634	,	72	},
	{	643	,	71	},
	{	651	,	70	},
	{	659	,	69	},
	{	667	,	68	},
	{	675	,	67	},
	{	691	,	65	},
	{	707	,	63	},
	{	714	,	62	},
	{	722	,	61	},
	{	737	,	59	},
	{	744	,	58	},
	{	752	,	57	},
	{	766	,	55	},
	{	773	,	54	},
	{	787	,	52	},
	{	794	,	51	},
	{	807	,	49	},
	{	814	,	48	},
	{	826	,	46	},
	{	833	,	45	},
	{	845	,	43	},
	{	856	,	41	},
	{	862	,	40	},
	{	872	,	38	},
	{	883	,	36	},
	{	892	,	34	},
	{	902	,	32	},
	{	915	,	29	},
	{	923	,	27	},
	{	935	,	24	},
	{	942	,	22	},
	{	952	,	19	},
	{	963	,	15	},
	{	973	,	11	},
	{	981	,	7	},
	{	990	,	2	},
};

// Thermistor lookup table for RS thermistor 198-961
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=10000 --beta=3960 --max-adc=1023
// r0: 100000
// t0: 25
// r1: 0
// r2: 10000
// beta: 3960
// max adc: 1023
/*#define BNUMTEMPS 30
const short bedtemptable[BNUMTEMPS][2] = {
   {1, 704},
   {15, 280},
   {21, 266},
   {41, 234},
   {61, 208},
   {81, 191},
   {101, 178},
   {121, 168},
   {141, 159},
   {161, 152},
   {181, 146},
   {221, 135},
   {261, 126},
   {301, 118},
   {341, 111},
   {381, 105},
   {421, 99},
   {461, 94},
   {501, 88},
   {541, 83},
   {581, 78},
   {621, 73},
   {661, 68},
   {741, 58},
   {781, 52},
   {821, 46},
   {861, 40},
   {901, 32},
   {981, 7},
   {1008, 0}
};
*/
/*
// RS thermistor 484-0149; EPCOS B57550G103J
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=10000 --t0=25 --r1=0 --r2=4700 --beta=3480 --max-adc=1023
// r0: 10000
// t0: 25
// r1: 0
// r2: 4700
// beta: 3480
// max adc: 1023
#define BNUMTEMPS 20
const short bedtemptable[BNUMTEMPS][2] = {
   {1, 599},
   {54, 160},
   {107, 123},
   {160, 103},
   {213, 90},
   {266, 79},
   {319, 70},
   {372, 62},
   {425, 55},
   {478, 49},
   {531, 43},
   {584, 37},
   {637, 31},
   {690, 25},
   {743, 19},
   {796, 12},
   {849, 5},
   {902, -3},
   {955, -16},
   {1008, -42}
};
*/
#endif
/****************************************************************************************
* Sanguinololu pin assignment
*
****************************************************************************************/
#if MOTHERBOARD == 62
#define MOTHERBOARD 6
#define SANGUINOLOLU_V_1_2
#endif
#if MOTHERBOARD == 6
#define KNOWN_BOARD 1
#ifndef __AVR_ATmega644P__
#ifndef __AVR_ATmega1284P__
#error Oops!  Make sure you have the appropriate 'Sanguino' selected from the 'Tools -> Boards' menu.
#endif
#endif

#define X_STEP_PIN         15
#define X_DIR_PIN          21
#define X_MIN_PIN          18
#define X_MAX_PIN           -2

#define Y_STEP_PIN         22
#define Y_DIR_PIN          23
#define Y_MIN_PIN          19
#define Y_MAX_PIN          -1

#define Z_STEP_PIN         3
#define Z_DIR_PIN          2
#define Z_MIN_PIN          -1
#define Z_MAX_PIN          20

#define E_STEP_PIN         1
#define E_DIR_PIN          0

/*#define PROBE_PIN          11*/

#define LED_PIN            27

#define FAN_PIN            4

#define PS_ON_PIN          -1
#define KILL_PIN           -1

#define HEATER_0_PIN       13 // (extruder)

#ifdef SANGUINOLOLU_V_1_2

#define HEATER_1_PIN       12 // (bed)
#define X_ENABLE_PIN       14
#define Y_ENABLE_PIN       14
#define Z_ENABLE_PIN       26
#define E_ENABLE_PIN       14

#else

#define HEATER_1_PIN       14  // (bed)
#define X_ENABLE_PIN       -1
#define Y_ENABLE_PIN       -1
#define Z_ENABLE_PIN       -1
#define E_ENABLE_PIN       -1

#endif

#define TEMP_0_PIN          7   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
#define TEMP_1_PIN          6   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
#define SDPOWER          -1
#define SDSS          31

#ifndef KNOWN_BOARD
#error Unknown MOTHERBOARD value in configuration.h
#endif

#endif
#endif
