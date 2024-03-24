#ifndef __PARAMETERS_h
#define __PARAMETERS_h

#define EEPROM_WRITE_UINT8 0x0A
#define EEPROM_WRITE_UINT16 0x0B
#define EEPROM_WRITE_UINT32 0x0C
#define EEPROM_WRITE_INT16 0x0D
#define EEPROM_WRITE_BOOL 0x0E
#define EEPROM_WRITE_FLOAT 0x0F

// uint8
#define DEFAULT_CONFIG_FLAGS 48
#define DEFAULT_UPDATE_RATE 10
#define DEFAULT_PWM_DIV 64
#define DEFAULT_DRIVE_MODE 3
#define DEFAULT_MONITOR_RATE 100
#define DEFAULT_ALLOWED_SKIP 3

#define PARAM_CONFIG_FLAGS 0
#define PARAM_UPDATE_RATE 1
#define PARAM_PWM_DIV 2
#define PARAM_DRIVE_MODE 3
#define PARAM_MONITOR_RATE 4
#define PARAM_ALLOWED_SKIP 5

// uint16
#define DEFAULT_PWM_SCALE 256
#define DEFAULT_PWM_FRQ 50
#define DEFAULT_MAX_IDLE_SECONDS 3600
#define DEFAULT_UPPER_LIMIT 255
#define DEFAULT_INTEGRAL_LIMIT 224
#define DEFAULT_ENCODER_PPR 48

#define PARAM_PWM_SCALE 0
#define PARAM_PWM_FRQ 1
#define PARAM_MAX_IDLE_SECONDS 2
#define PARAM_UPPER_LIMIT 3
#define PARAM_INTEGRAL_LIMIT 4
#define PARAM_ENCODER_PPR 5

// uint32
#define DEFAULT_RTC_TRIM 0x7FFF

#define PARAM_RTC_TRIM 0

// int16
#define DEFAULT_LEFT_FORWARD_DEADZONE 8
#define DEFAULT_LEFT_REVERSE_DEADZONE 8
#define DEFAULT_RIGHT_FORWARD_DEADZONE 8
#define DEFAULT_RIGHT_REVERSE_DEADZONE 8

#define PARAM_LEFT_FORWARD_DEADZONE 0
#define PARAM_LEFT_REVERSE_DEADZONE 1
#define PARAM_RIGHT_FORWARD_DEADZONE 2
#define PARAM_RIGHT_REVERSE_DEADZONE 3

// float
#define DEFAULT_GEAR_RATIO (65.0F)
#define DEFAULT_WHEEL_DIA (0.0685F)
#define DEFAULT_BASE_WIDTH (0.160F)
#define DEFAULT_MAIN_AMP_LIMIT (3.6F)
#define DEFAULT_BAT_VOLTS_HIGH (15.0F)
#define DEFAULT_BAT_VOLTS_LOW (6.0F)
#define DEFAULT_MAX_RPM (90.0F)
#define DEFAULT_LEFT_AMP_LIMIT (1.6F)
#define DEFAULT_RIGHT_AMP_LIMIT (1.6F)
#define DEFAULT_LEFT_KP (0.32F)
#define DEFAULT_LEFT_KI (0.64F)
#define DEFAULT_LEFT_KD (0.01F)
#define DEFAULT_RIGHT_KP (0.32F)
#define DEFAULT_RIGHT_KI (0.64F)
#define DEFAULT_RIGHT_KD (0.01F)
#define DEFAULT_GAIN (1.0F)
#define DEFAULT_TRIM (0.0F)
#define DEFAULT_MOTOR_CONSTANT (1.0F)

#define PARAM_GEAR_RATIO 0
#define PARAM_WHEEL_DIA 1
#define PARAM_BASE_WIDTH 2
#define PARAM_MAIN_AMP_LIMIT 3
#define PARAM_BAT_VOLTS_HIGH 4
#define PARAM_BAT_VOLTS_LOW 5
#define PARAM_MAX_RPM 6
#define PARAM_LEFT_AMP_LIMIT 7
#define PARAM_RIGHT_AMP_LIMIT 8
#define PARAM_LEFT_KP 9
#define PARAM_LEFT_KI 10
#define PARAM_LEFT_KD 11
#define PARAM_RIGHT_KP 12
#define PARAM_RIGHT_KI 13
#define PARAM_RIGHT_KD 14
#define PARAM_GAIN 15
#define PARAM_TRIM 16
#define PARAM_MOTOR_CONSTANT 17

#define DEFAULT_AUTO_SYNC false
#define PARAM_AUTO_SYNC 0

// uint8 array
#define SIZE_PARAMS_UINT8 6
uint8_t params_uint8[SIZE_PARAMS_UINT8] = {
                                DEFAULT_CONFIG_FLAGS,
                                DEFAULT_UPDATE_RATE,
                                DEFAULT_PWM_DIV,
                                DEFAULT_DRIVE_MODE,
                                DEFAULT_MONITOR_RATE,
                                DEFAULT_ALLOWED_SKIP
                            };

const char *names_uint8[] = { "CONFIG_FLAGS",
                              "UPDATE_RATE",
                              "PWM_DIV",
                              "DRIVE_MODE",
                              "MONITOR_RATE", 
                              "ALLOWED_SKIP"
                            };

// uint16 array
#define SIZE_PARAMS_UINT16 6
uint16_t params_uint16[SIZE_PARAMS_UINT16] = {
                                DEFAULT_PWM_SCALE,
                                DEFAULT_PWM_FRQ,
                                DEFAULT_MAX_IDLE_SECONDS,
                                DEFAULT_UPPER_LIMIT,
                                DEFAULT_INTEGRAL_LIMIT,
                                DEFAULT_ENCODER_PPR
                            };

const char *names_uint16[] = { "PWM_SCALE",
                               "PWM_FRQ",
                               "MAX_IDLE_SECONDS",
                               "UPPER_LIMIT",
                               "INTEGRAL_LIMIT",
                               "ENCODER_PPR" };

// uint32 array
#define SIZE_PARAMS_UINT32 1
uint32_t params_uint32[SIZE_PARAMS_UINT32] = {
                               DEFAULT_RTC_TRIM
                            };

const char *names_uint32[] = { "RTC_TRIM" };

// int16 array
#define SIZE_PARAMS_INT16 4
int16_t params_int16[SIZE_PARAMS_INT16] = {
                                DEFAULT_LEFT_FORWARD_DEADZONE,
                                DEFAULT_LEFT_REVERSE_DEADZONE,
                                DEFAULT_RIGHT_FORWARD_DEADZONE,
                                DEFAULT_RIGHT_REVERSE_DEADZONE
                            };

const char *names_int16[] = { "LEFT_FORWARD_DEADZONE",
                              "LEFT_REVERSE_DEADZONE",
                              "RIGHT_FORWARD_DEADZONE",
                              "RIGHT_REVERSE_DEADZONE" };

// float array
#define SIZE_PARAMS_FLOAT 18
float params_float[SIZE_PARAMS_FLOAT] = {
                               DEFAULT_GEAR_RATIO,
                               DEFAULT_WHEEL_DIA,
                               DEFAULT_BASE_WIDTH,
                               DEFAULT_MAIN_AMP_LIMIT,
                               DEFAULT_BAT_VOLTS_HIGH,
                               DEFAULT_BAT_VOLTS_LOW,
                               DEFAULT_MAX_RPM,
                               DEFAULT_LEFT_AMP_LIMIT,
                               DEFAULT_RIGHT_AMP_LIMIT,
                               DEFAULT_LEFT_KP,
                               DEFAULT_LEFT_KI,
                               DEFAULT_LEFT_KD,
                               DEFAULT_RIGHT_KP,
                               DEFAULT_RIGHT_KI,
                               DEFAULT_RIGHT_KD,
                               DEFAULT_GAIN,
                               DEFAULT_TRIM,
                               DEFAULT_MOTOR_CONSTANT
                            };

const char *names_float[] = { "GEAR_RATIO",
                              "WHEEL_DIA",
                              "BASE_WIDTH",
                              "MAIN_AMP_LMT",
                              "BAT_VOLTS_HIGH",
                              "BAT_VOLTS_LOW",
                              "MAX_RPM",
                              "LEFT_AMP_LMT",
                              "RIGHT_AMP_LMT",
                              "LEFT_KP",
                              "LEFT_KI",
                              "LEFT_KD",
                              "RIGHT_KP",
                              "RIGHT_KI",
                              "RIGHT_KD",
                              "GAIN",
                              "TRIM",
                              "MOTOR_CONSTANT" };

#define SIZE_PARAMS_BOOL 1
bool params_bool[SIZE_PARAMS_BOOL] = { DEFAULT_AUTO_SYNC };
const char *names_bool[] = { "AUTO_SYNC" };

// calculated parameters
uint16_t PULSE_PER_REV;
double COMMAND_TIMEOUT_SECS;
float WHEEL_CIRCUMFERENCE;
float TICKS_PER_METER;
float ROUNDS_PER_MINUTE;
float LINEAR_RPM;
float ANGULAR_RPM;
double UPDATE_PERIOD;
double MONITOR_PERIOD;

// cached and calculated parameters for diff drive
float MAX_RPM;
float GAIN;
float TRIM;
float MOTOR_CONSTANT;
float MOTOR_CONSTANT_LEFT;
float MOTOR_CONSTANT_RIGHT;

// these are imaginary derived from config_flags
bool LEFT_REVERSE;
bool RIGHT_REVERSE;
bool LEFT_SWAP;
bool RIGHT_SWAP;
bool LEFT_ENC_AB;
bool RIGHT_ENC_AB;
bool MODE1;
bool MODE2;

#endif

// main parameters