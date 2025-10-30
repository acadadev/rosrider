#ifndef __PARAMETERS_h
#define __PARAMETERS_h

#define PARAM_WRITE     0x01                        // write parameter to eeprom
#define PARAM_OVERRIDE  0x02                        // override parameter and make it instantly apply

#define I2C_WRITE_RESULT_SUCCESS 0x00               // param operation success
#define I2C_WRITE_RESULT_UNCHANGED 0x01             // param not changed
#define I2C_WRITE_RESULT_CHECKSUM 0x03              // param operation checksum fail
#define I2C_WRITE_RESULT_OVERRIDE 0xFF              // param override operation success

// parameter types, or addresses
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
#define DEFAULT_I2C_ADDRESS 0x3C
#define DEFAULT_OUTPUT_FILTER_TYPE 0
#define DEFAULT_INNER_FILTER_TYPE 0

#define PARAM_CONFIG_FLAGS 0
#define PARAM_UPDATE_RATE 1
#define PARAM_PWM_DIV 2
#define PARAM_DRIVE_MODE 3
#define PARAM_MONITOR_RATE 4
#define PARAM_ALLOWED_SKIP 5
#define PARAM_I2C_ADDRESS 6
#define PARAM_OUTPUT_FILTER_TYPE 7
#define PARAM_INNER_FILTER_TYPE 8

// uint16
#define DEFAULT_PWM_SCALE 256
#define DEFAULT_PWM_FRQ 50
#define DEFAULT_MAX_IDLE_SECONDS 3600
#define DEFAULT_UPPER_LIMIT 244
#define DEFAULT_INTEGRAL_LIMIT 128
#define DEFAULT_ENCODER_PPR 48
#define DEFAULT_INA219_CAL 8192
#define DEFAULT_ADC_CS_DIV 32
#define DEFAULT_CURRENT_INTEGRAL_LIMIT 128

#define PARAM_PWM_SCALE 0
#define PARAM_PWM_FRQ 1
#define PARAM_MAX_IDLE_SECONDS 2
#define PARAM_UPPER_LIMIT 3
#define PARAM_INTEGRAL_LIMIT 4
#define PARAM_ENCODER_PPR 5
#define PARAM_INA219_CAL 6
#define PARAM_ADC_CS_DIV 7
#define PARAM_CURRENT_INTEGRAL_LIMIT 8

// uint32
#define DEFAULT_RTC_TRIM 0x7FFF

#define PARAM_RTC_TRIM 0

// int16
#define DEFAULT_LEFT_FORWARD_DEADZONE 12
#define DEFAULT_LEFT_REVERSE_DEADZONE 12
#define DEFAULT_RIGHT_FORWARD_DEADZONE 12
#define DEFAULT_RIGHT_REVERSE_DEADZONE 12
#define DEFAULT_CS_LEFT_OFFSET 0
#define DEFAULT_CS_RIGHT_OFFSET 0

#define PARAM_LEFT_FORWARD_DEADZONE 0
#define PARAM_LEFT_REVERSE_DEADZONE 1
#define PARAM_RIGHT_FORWARD_DEADZONE 2
#define PARAM_RIGHT_REVERSE_DEADZONE 3
#define PARAM_CS_LEFT_OFFSET 4
#define PARAM_CS_RIGHT_OFFSET 5

// float
#define DEFAULT_GEAR_RATIO (65.0F)
#define DEFAULT_WHEEL_DIA (0.0685F)
#define DEFAULT_BASE_WIDTH (0.10F)
#define DEFAULT_MAIN_AMP_LIMIT (3.6F)
#define DEFAULT_BAT_VOLTS_HIGH (15.0F)
#define DEFAULT_BAT_VOLTS_LOW (6.0F)
#define DEFAULT_MAX_RPM (90.0F)
#define DEFAULT_LEFT_AMP_LIMIT (1.6F)
#define DEFAULT_RIGHT_AMP_LIMIT (1.6F)
#define DEFAULT_LEFT_KP (1.0F)
#define DEFAULT_LEFT_KI (1.0F)
#define DEFAULT_LEFT_KD (0.0F)
#define DEFAULT_RIGHT_KP (1.0F)
#define DEFAULT_RIGHT_KI (1.0F)
#define DEFAULT_RIGHT_KD (0.0F)
#define DEFAULT_GAIN (1.0F)
#define DEFAULT_TRIM (0.0F)
#define DEFAULT_MOTOR_CONSTANT (1.0F)
#define DEFAULT_TANH_DIV (2.0F)
#define DEFAULT_SIGM_DIV (10.0F)
#define DEFAULT_CURRENT_KP (1.0F)
#define DEFAULT_CURRENT_KI (1.0F)

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
#define PARAM_TANH_DIV 18
#define PARAM_SIGM_DIV 19
#define PARAM_CURRENT_KP 20
#define PARAM_CURRENT_KI 21

#define DEFAULT_AUTOSYNC true
#define DEFAULT_ADCSYNC true
#define DEFAULT_FASTADC false
#define DEFAULT_CASCADED false
#define DEFAULT_BACKEMF false
#define DEFAULT_IDLE_BRAKE false
#define DEFAULT_CASCADE_FILTER false
#define DEFAULT_AUTO_BIAS false

#define PARAM_AUTOSYNC 0
#define PARAM_ADCSYNC 1
#define PARAM_FASTADC 2
#define PARAM_CASCADED 3
#define PARAM_BACKEMF 4
#define PARAM_IDLE_BRAKE 5
#define PARAM_CASCADE_FILTER 6
#define PARAM_AUTO_BIAS 7

// uint8 array
#define SIZE_PARAMS_UINT8 9
uint8_t params_uint8[SIZE_PARAMS_UINT8] = {
                                DEFAULT_CONFIG_FLAGS,
                                DEFAULT_UPDATE_RATE,
                                DEFAULT_PWM_DIV,
                                DEFAULT_DRIVE_MODE,
                                DEFAULT_MONITOR_RATE,
                                DEFAULT_ALLOWED_SKIP,
                                DEFAULT_I2C_ADDRESS,
                                DEFAULT_OUTPUT_FILTER_TYPE,
                                DEFAULT_INNER_FILTER_TYPE
                            };

const char *names_uint8[] = { "CONFIG_FLAGS",
                              "UPDATE_RATE",
                              "PWM_DIV",
                              "DRIVE_MODE",
                              "MONITOR_RATE", 
                              "ALLOWED_SKIP",
                              "I2C_ADDRESS",
                              "OUTPUT_FILTER_TYPE",
                              "INNER_FILTER_TYPE"
                            };

// uint16 array
#define SIZE_PARAMS_UINT16 9
uint16_t params_uint16[SIZE_PARAMS_UINT16] = {
                                DEFAULT_PWM_SCALE,
                                DEFAULT_PWM_FRQ,
                                DEFAULT_MAX_IDLE_SECONDS,
                                DEFAULT_UPPER_LIMIT,
                                DEFAULT_INTEGRAL_LIMIT,
                                DEFAULT_ENCODER_PPR,
                                DEFAULT_INA219_CAL,
                                DEFAULT_ADC_CS_DIV,
                                DEFAULT_CURRENT_INTEGRAL_LIMIT
                            };

const char *names_uint16[] = { "PWM_SCALE",
                               "PWM_FRQ",
                               "MAX_IDLE_SECONDS",
                               "UPPER_LIMIT",
                               "INTEGRAL_LIMIT",
                               "ENCODER_PPR",
                               "INA219_CAL",
                               "ADC_CS_DIV",
                               "CURRENT_INTEGRAL_LIMIT" };

// uint32 array
#define SIZE_PARAMS_UINT32 1
uint32_t params_uint32[SIZE_PARAMS_UINT32] = {
                               DEFAULT_RTC_TRIM
                            };

const char *names_uint32[] = { "RTC_TRIM" };

// int16 array
#define SIZE_PARAMS_INT16 6
int16_t params_int16[SIZE_PARAMS_INT16] = {
                                DEFAULT_LEFT_FORWARD_DEADZONE,
                                DEFAULT_LEFT_REVERSE_DEADZONE,
                                DEFAULT_RIGHT_FORWARD_DEADZONE,
                                DEFAULT_RIGHT_REVERSE_DEADZONE,
                                DEFAULT_CS_LEFT_OFFSET,
                                DEFAULT_CS_RIGHT_OFFSET
                            };

const char *names_int16[] = { "LEFT_FORWARD_DEADZONE",
                              "LEFT_REVERSE_DEADZONE",
                              "RIGHT_FORWARD_DEADZONE",
                              "RIGHT_REVERSE_DEADZONE",
                              "CS_LEFT_OFFSET",
                              "CS_RIGHT_OFFSET" };

// float array
#define SIZE_PARAMS_FLOAT 22
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
                               DEFAULT_MOTOR_CONSTANT,
                               DEFAULT_TANH_DIV,
                               DEFAULT_SIGM_DIV,
                               DEFAULT_CURRENT_KP,
                               DEFAULT_CURRENT_KI
                            };

const char *names_float[] = { "GEAR_RATIO",
                              "WHEEL_DIA",
                              "BASE_WIDTH",
                              "MAIN_AMP_LIMIT",
                              "BAT_VOLTS_HIGH",
                              "BAT_VOLTS_LOW",
                              "MAX_RPM",
                              "LEFT_AMP_LIMIT",
                              "RIGHT_AMP_LIMIT",
                              "LEFT_KP",
                              "LEFT_KI",
                              "LEFT_KD",
                              "RIGHT_KP",
                              "RIGHT_KI",
                              "RIGHT_KD",
                              "GAIN",
                              "TRIM",
                              "MOTOR_CONSTANT",
                              "TANH_DIV",
                              "SIGM_DIV",
                              "CURRENT_KP",
                              "CURRENT_KI" };

#define SIZE_PARAMS_BOOL 8
bool params_bool[SIZE_PARAMS_BOOL] = { DEFAULT_AUTOSYNC,
                                       DEFAULT_ADCSYNC,
                                       DEFAULT_FASTADC,
                                       DEFAULT_CASCADED,
                                       DEFAULT_BACKEMF,
                                       DEFAULT_IDLE_BRAKE,
                                       DEFAULT_CASCADE_FILTER,
                                       DEFAULT_AUTO_BIAS };
const char *names_bool[] = { "AUTOSYNC",
                             "ADCSYNC",
                             "FASTADC",
                             "CASCADED",
                             "BACKEMF",
                             "IDLE_BRAKE",
                             "CASCADE_FILTER",
                             "AUTO_BIAS" };

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

// these are imaginary derived from config_flags
bool LEFT_REVERSE;
bool RIGHT_REVERSE;
bool LEFT_SWAP;
bool RIGHT_SWAP;
bool LEFT_ENC_AB;
bool RIGHT_ENC_AB;
bool MODE1;
bool MODE2;

enum class CParamDataType {
    C_TYPE_UINT8,
    C_TYPE_UINT16,
    C_TYPE_INT16,
    C_TYPE_FLOAT,
    C_TYPE_BOOL
};

struct ParamMetadata {
    CParamDataType c_type;          // The target C data type
    uint8_t param_index;            // Parameter Index
    uint8_t fp_index;               // Function Pointer Index
};

#define FP_DRIVE_MODE 0
#define FP_PWM_DIV 1
#define FP_OUTPUT_FILTER_TYPE 2
#define FP_INNER_FILTER_TYPE 3

#define FP_PWM_SCALE 0
#define FP_PWM_FRQ 1
#define FP_MAX_IDLE_SECONDS 2
#define FP_INTEGRAL_LIMIT 3
#define FP_UPPER_LIMIT 4
#define FP_ADC_CS_DIV 5
#define FP_CURRENT_INTEGRAL_LIMIT 6

#define FP_LEFT_FORWARD_DEADZONE 0
#define FP_LEFT_REVERSE_DEADZONE 1
#define FP_RIGHT_FORWARD_DEADZONE 2
#define FP_RIGHT_REVERSE_DEADZONE 3
#define FP_CS_LEFT_OFFSET 4
#define FP_CS_RIGHT_OFFSET 5

#define FP_MAX_RPM 0

#define FP_LEFT_KP 1
#define FP_RIGHT_KP 2
#define FP_LEFT_KI 3
#define FP_RIGHT_KI 4
#define FP_LEFT_KD 5
#define FP_RIGHT_KD 6

#define FP_GAIN 7
#define FP_TRIM 8
#define FP_MOTOR_CONSTANT 9

#define FP_BAT_VOLTS_LOW 10
#define FP_BAT_VOLTS_HIGH 11
#define FP_MAIN_AMP_LIMIT 12
#define FP_LEFT_AMP_LIMIT 13
#define FP_RIGHT_AMP_LIMIT 14

#define FP_TANH_DIV 15
#define FP_SIGM_DIV 16

#define FP_CURRENT_KP 17
#define FP_CURRENT_KI 18

#define FP_AUTOSYNC 0
#define FP_ADCSYNC 1
#define FP_FASTADC 2
#define FP_CASCADED 3
#define FP_BACKEMF 4
#define FP_IDLE_BRAKE 5
#define FP_CASCADE_FILTER 6
#define FP_AUTO_BIAS 7

const std::map<std::string, ParamMetadata> ParamMap = {

    {"DRIVE_MODE",              { CParamDataType::C_TYPE_UINT8,  PARAM_DRIVE_MODE, FP_DRIVE_MODE}},
    {"PWM_DIV",                 { CParamDataType::C_TYPE_UINT8,  PARAM_PWM_DIV, FP_PWM_DIV}},
    {"OUTPUT_FILTER_TYPE",      { CParamDataType::C_TYPE_UINT8,  PARAM_OUTPUT_FILTER_TYPE, FP_OUTPUT_FILTER_TYPE}},
    {"INNER_FILTER_TYPE",       { CParamDataType::C_TYPE_UINT8,  PARAM_INNER_FILTER_TYPE, FP_INNER_FILTER_TYPE}},

    {"PWM_SCALE",               { CParamDataType::C_TYPE_UINT16, PARAM_PWM_SCALE, FP_PWM_SCALE}},
    {"PWM_FRQ",                 { CParamDataType::C_TYPE_UINT16, PARAM_PWM_FRQ, FP_PWM_FRQ}},
    {"MAX_IDLE_SECONDS",        { CParamDataType::C_TYPE_UINT16, PARAM_MAX_IDLE_SECONDS, FP_MAX_IDLE_SECONDS}},
    {"INTEGRAL_LIMIT",          { CParamDataType::C_TYPE_UINT16, PARAM_INTEGRAL_LIMIT, FP_INTEGRAL_LIMIT}},
    {"UPPER_LIMIT",             { CParamDataType::C_TYPE_UINT16, PARAM_UPPER_LIMIT, FP_UPPER_LIMIT}},
    {"ADC_CS_DIV",              { CParamDataType::C_TYPE_UINT16, PARAM_ADC_CS_DIV, FP_ADC_CS_DIV}},
    {"CURRENT_INTEGRAL_LIMIT",  { CParamDataType::C_TYPE_UINT16, PARAM_CURRENT_INTEGRAL_LIMIT, FP_CURRENT_INTEGRAL_LIMIT}},

    {"LEFT_FORWARD_DEADZONE",   { CParamDataType::C_TYPE_INT16,  PARAM_LEFT_FORWARD_DEADZONE, FP_LEFT_FORWARD_DEADZONE}},
    {"LEFT_REVERSE_DEADZONE",   { CParamDataType::C_TYPE_INT16,  PARAM_LEFT_REVERSE_DEADZONE, FP_LEFT_REVERSE_DEADZONE}},
    {"RIGHT_FORWARD_DEADZONE",  { CParamDataType::C_TYPE_INT16,  PARAM_RIGHT_FORWARD_DEADZONE, FP_RIGHT_FORWARD_DEADZONE}},
    {"RIGHT_REVERSE_DEADZONE",  { CParamDataType::C_TYPE_INT16,  PARAM_RIGHT_REVERSE_DEADZONE, FP_RIGHT_REVERSE_DEADZONE}},
    {"CS_LEFT_OFFSET",          { CParamDataType::C_TYPE_INT16,  PARAM_CS_LEFT_OFFSET, FP_CS_LEFT_OFFSET}},
    {"CS_RIGHT_OFFSET",         { CParamDataType::C_TYPE_INT16,  PARAM_CS_RIGHT_OFFSET, FP_CS_RIGHT_OFFSET}},

    {"MAX_RPM",                 { CParamDataType::C_TYPE_FLOAT,  PARAM_MAX_RPM, FP_MAX_RPM}},

    {"LEFT_KP",                 { CParamDataType::C_TYPE_FLOAT,  PARAM_LEFT_KP, FP_LEFT_KP}},
    {"RIGHT_KP",                { CParamDataType::C_TYPE_FLOAT,  PARAM_RIGHT_KP, FP_RIGHT_KP}},
    {"LEFT_KI",                 { CParamDataType::C_TYPE_FLOAT,  PARAM_LEFT_KI, FP_LEFT_KI}},
    {"RIGHT_KI",                { CParamDataType::C_TYPE_FLOAT,  PARAM_RIGHT_KI, FP_RIGHT_KI}},
    {"LEFT_KD",                 { CParamDataType::C_TYPE_FLOAT,  PARAM_LEFT_KD, FP_LEFT_KD}},
    {"RIGHT_KD",                { CParamDataType::C_TYPE_FLOAT,  PARAM_RIGHT_KD, FP_RIGHT_KD}},

    {"GAIN",                    { CParamDataType::C_TYPE_FLOAT,  PARAM_GAIN, FP_GAIN}},
    {"TRIM",                    { CParamDataType::C_TYPE_FLOAT,  PARAM_TRIM, FP_TRIM}},
    {"MOTOR_CONSTANT",          { CParamDataType::C_TYPE_FLOAT,  PARAM_MOTOR_CONSTANT, FP_MOTOR_CONSTANT}},

    {"BAT_VOLTS_LOW",           { CParamDataType::C_TYPE_FLOAT,  PARAM_BAT_VOLTS_LOW, FP_BAT_VOLTS_LOW}},
    {"BAT_VOLTS_HIGH",          { CParamDataType::C_TYPE_FLOAT,  PARAM_BAT_VOLTS_HIGH, FP_BAT_VOLTS_HIGH}},
    {"MAIN_AMP_LIMIT",          { CParamDataType::C_TYPE_FLOAT,  PARAM_MAIN_AMP_LIMIT, FP_MAIN_AMP_LIMIT}},
    {"LEFT_AMP_LIMIT",          { CParamDataType::C_TYPE_FLOAT,  PARAM_LEFT_AMP_LIMIT, FP_LEFT_AMP_LIMIT}},
    {"RIGHT_AMP_LIMIT",         { CParamDataType::C_TYPE_FLOAT,  PARAM_RIGHT_AMP_LIMIT, FP_RIGHT_AMP_LIMIT}},

    {"TANH_DIV",                { CParamDataType::C_TYPE_FLOAT,  PARAM_TANH_DIV, FP_TANH_DIV}},
    {"SIGM_DIV",                { CParamDataType::C_TYPE_FLOAT,  PARAM_SIGM_DIV, FP_SIGM_DIV}},

    {"CURRENT_KP",              { CParamDataType::C_TYPE_FLOAT,  PARAM_CURRENT_KP, FP_CURRENT_KP}},
    {"CURRENT_KI",              { CParamDataType::C_TYPE_FLOAT,  PARAM_CURRENT_KI, FP_CURRENT_KI}},

    {"AUTOSYNC",                { CParamDataType::C_TYPE_BOOL,  PARAM_AUTOSYNC, FP_AUTOSYNC}},
    {"ADCSYNC",                 { CParamDataType::C_TYPE_BOOL,  PARAM_ADCSYNC, FP_ADCSYNC}},
    {"FASTADC",                 { CParamDataType::C_TYPE_BOOL,  PARAM_FASTADC, FP_FASTADC}},
    {"CASCADED",                { CParamDataType::C_TYPE_BOOL,  PARAM_CASCADED, FP_CASCADED}},
    {"BACKEMF",                 { CParamDataType::C_TYPE_BOOL,  PARAM_BACKEMF, FP_BACKEMF}},
    {"IDLE_BRAKE",              { CParamDataType::C_TYPE_BOOL,  PARAM_IDLE_BRAKE, FP_IDLE_BRAKE}},
    {"CASCADE_FILTER",          { CParamDataType::C_TYPE_BOOL,  PARAM_CASCADE_FILTER, FP_CASCADE_FILTER}},
    {"AUTO_BIAS",               { CParamDataType::C_TYPE_BOOL,  PARAM_AUTO_BIAS, FP_AUTO_BIAS}},

};

const ParamMetadata* get_param_metadata(const std::string& param_name) {
    auto it = ParamMap.find(param_name);
    if (it != ParamMap.end()) { return &(it->second); }
    return nullptr;
}

#endif

// main parameters