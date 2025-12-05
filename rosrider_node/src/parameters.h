#ifndef __PARAMETERS_h
#define __PARAMETERS_h

#define   UPDATE_SUCCESS 0x00
#define UPDATE_UNCHANGED 0x10
#define  UPDATE_WR_ERROR 0x21
#define  UPDATE_CHECKSUM 0x22
#define    UPDATE_DENIED 0x30
#define  UPDATE_FP_ERROR 0x40
#define UPDATE_MOD_STATE 0x41
#define    UPDATE_REBOOT 0x80                               // notice: this is returned in write result array

// parameter types, or addresses
#define EEPROM_WRITE_UINT8 0x0A
#define EEPROM_WRITE_UINT16 0x0B
#define EEPROM_WRITE_UINT32 0x0C
#define EEPROM_WRITE_INT16 0x0D
#define EEPROM_WRITE_BOOL 0x0E
#define EEPROM_WRITE_FLOAT 0x0F

// uint8
#define DEFAULT_CONFIG_FLAGS 48
#define DEFAULT_UPDATE_RATE 20
#define DEFAULT_PWM_DIV 32
#define DEFAULT_DRIVE_MODE 3
#define DEFAULT_MONITOR_RATE 100
#define DEFAULT_ALLOWED_SKIP 3
#define DEFAULT_I2C_ADDRESS 0x3C
#define DEFAULT_OUTPUT_FILTER_TYPE 0
#define DEFAULT_CS_WAVEFORM_DIVIDER 16

#define PARAM_CONFIG_FLAGS 0
#define PARAM_UPDATE_RATE 1
#define PARAM_PWM_DIV 2
#define PARAM_DRIVE_MODE 3
#define PARAM_MONITOR_RATE 4
#define PARAM_ALLOWED_SKIP 5
#define PARAM_I2C_ADDRESS 6
#define PARAM_OUTPUT_FILTER_TYPE 7
#define PARAM_CS_WAVEFORM_DIVIDER 8

// uint16
#define DEFAULT_PWM_SCALE 256
#define DEFAULT_PWM_FRQ 50
#define DEFAULT_MAX_IDLE_SECONDS 1800
#define DEFAULT_UPPER_LIMIT 192
#define DEFAULT_INNER_LIMIT 192
#define DEFAULT_ENCODER_PPR 48
#define DEFAULT_INA219_CAL 8192
#define DEFAULT_ADC_SPEED 16000

#define PARAM_PWM_SCALE 0
#define PARAM_PWM_FRQ 1
#define PARAM_MAX_IDLE_SECONDS 2
#define PARAM_UPPER_LIMIT 3
#define PARAM_INNER_LIMIT 4
#define PARAM_ENCODER_PPR 5
#define PARAM_INA219_CAL 6
#define PARAM_ADC_SPEED 7

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
#define DEFAULT_RIGHT_AMP_LIMIT (1.6F) // notice: do not add or remove before this point
#define DEFAULT_LEFT_KP (2.4F)
#define DEFAULT_LEFT_KI (1.8F)
#define DEFAULT_LEFT_KD (0.0F)
#define DEFAULT_RIGHT_KP (2.4F)
#define DEFAULT_RIGHT_KI (1.8F)
#define DEFAULT_RIGHT_KD (0.0F)
#define DEFAULT_GAIN (1.0F)
#define DEFAULT_TRIM (0.0F)
#define DEFAULT_MOTOR_CONSTANT (1.0F)
#define DEFAULT_TANH_DIV (2.0F)
#define DEFAULT_SIGM_DIV (10.0F)
#define DEFAULT_CURRENT_KP (2.4F)
#define DEFAULT_CURRENT_KI (1.2F)
#define DEFAULT_CURRENT_OBSERVED_MULTIPLIER (1.0F)
#define DEFAULT_KB (0.5F)
#define DEFAULT_TORQUE_CONSTANT (0.01F)
#define DEFAULT_R_ARM (2.0F)
#define DEFAULT_L_ARM (0.001F)
#define DEFAULT_K_FF_VEL (0.16F)
#define DEFAULT_K_FF_ACCEL (0.12F)
#define DEFAULT_STATIC_KICK (6.0F)
#define DEFAULT_COULOMB_RUN (3.0F)
#define DEFAULT_STRIBECK_WIDTH (64.0F)
#define DEFAULT_VISCOUS_FRICTION (0.001F)
#define DEFAULT_IR_COMP_GAIN (2.0F)
#define DEFAULT_VISCOUS_FRICTION_LIMIT (1.2F)
#define DEFAULT_EB_FF_LIMIT (12.0F)

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
#define PARAM_CURRENT_OBSERVED_MULTIPLIER 22
#define PARAM_KB 23
#define PARAM_TORQUE_CONSTANT 24
#define PARAM_R_ARM 25
#define PARAM_L_ARM 26
#define PARAM_K_FF_VEL 27
#define PARAM_K_FF_ACCEL 28
#define PARAM_STATIC_KICK 29
#define PARAM_COULOMB_RUN 30
#define PARAM_STRIBECK_WIDTH 31
#define PARAM_VISCOUS_FRICTION 32
#define PARAM_IR_COMP_GAIN 33
#define PARAM_VISCOUS_FRICTION_LIMIT 34
#define PARAM_EB_FF_LIMIT 35

#define DEFAULT_AUTO_SYNC true
#define DEFAULT_ADC_SYNC true
#define DEFAULT_CASCADED false
#define DEFAULT_IR_COMP false
#define DEFAULT_CASCADE_FILTER false
#define DEFAULT_AUTO_BIAS true
#define DEFAULT_ADC_MULTIPHASE false
#define DEFAULT_ADC_BIPHASE false
#define DEFAULT_OUTER_FEEDFORWARD false
#define DEFAULT_OUTER_SCV false
#define DEFAULT_OMEGA_FILTER false
#define DEFAULT_VOLTAGE_FILTER false
#define DEFAULT_AUTO_BRAKE false
#define DEFAULT_INDUCTIVE_COMPENSATION false

#define PARAM_AUTO_SYNC 0
#define PARAM_ADC_SYNC 1
#define PARAM_CASCADED 2
#define PARAM_IR_COMP 3
#define PARAM_CASCADE_FILTER 4
#define PARAM_AUTO_BIAS 5
#define PARAM_ADC_MULTIPHASE 6
#define PARAM_ADC_BIPHASE 7
#define PARAM_OUTER_FEEDFORWARD 8
#define PARAM_OUTER_SCV 9
#define PARAM_OMEGA_FILTER 10
#define PARAM_VOLTAGE_FILTER 11
#define PARAM_AUTO_BRAKE 12
#define PARAM_INDUCTIVE_COMPENSATION 13

#define SIZE_PARAMS_UINT8 9
#define SIZE_PARAMS_UINT16 8
#define SIZE_PARAMS_UINT32 1
#define SIZE_PARAMS_INT16 6
#define SIZE_PARAMS_FLOAT 36
#define SIZE_PARAMS_BOOL 14

// uint8 array
uint8_t params_uint8[SIZE_PARAMS_UINT8] = {
                                DEFAULT_CONFIG_FLAGS,
                                DEFAULT_UPDATE_RATE,
                                DEFAULT_PWM_DIV,
                                DEFAULT_DRIVE_MODE,
                                DEFAULT_MONITOR_RATE,
                                DEFAULT_ALLOWED_SKIP,
                                DEFAULT_I2C_ADDRESS,
                                DEFAULT_OUTPUT_FILTER_TYPE,
                                DEFAULT_CS_WAVEFORM_DIVIDER
                            };

const char *names_uint8[] = { "CONFIG_FLAGS",
                              "UPDATE_RATE",
                              "PWM_DIV",
                              "DRIVE_MODE",
                              "MONITOR_RATE", 
                              "ALLOWED_SKIP",
                              "I2C_ADDRESS",
                              "OUTPUT_FILTER_TYPE",
                              "CS_WAVEFORM_DIVIDER"
                            };

// uint16 array
uint16_t params_uint16[SIZE_PARAMS_UINT16] = {
                                DEFAULT_PWM_SCALE,
                                DEFAULT_PWM_FRQ,
                                DEFAULT_MAX_IDLE_SECONDS,
                                DEFAULT_UPPER_LIMIT,
                                DEFAULT_INNER_LIMIT,
                                DEFAULT_ENCODER_PPR,
                                DEFAULT_INA219_CAL,
                                DEFAULT_ADC_SPEED
                            };

const char *names_uint16[] = { "PWM_SCALE",
                               "PWM_FRQ",
                               "MAX_IDLE_SECONDS",
                               "UPPER_LIMIT",
                               "INNER_LIMIT",
                               "ENCODER_PPR",
                               "INA219_CAL",
                               "ADC_SPEED" };

// uint32 array
uint32_t params_uint32[SIZE_PARAMS_UINT32] = {
                               DEFAULT_RTC_TRIM
                            };

const char *names_uint32[] = { "RTC_TRIM" };

// int16 array
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
                               DEFAULT_CURRENT_KI,
                               DEFAULT_CURRENT_OBSERVED_MULTIPLIER,
                               DEFAULT_KB,
                               DEFAULT_TORQUE_CONSTANT,
                               DEFAULT_R_ARM,
                               DEFAULT_L_ARM,
                               DEFAULT_K_FF_VEL,
                               DEFAULT_K_FF_ACCEL,
                               DEFAULT_STATIC_KICK,
                               DEFAULT_COULOMB_RUN,
                               DEFAULT_STRIBECK_WIDTH,
                               DEFAULT_VISCOUS_FRICTION,
                               DEFAULT_IR_COMP_GAIN,
                               DEFAULT_VISCOUS_FRICTION_LIMIT,
                               DEFAULT_EB_FF_LIMIT
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
                              "CURRENT_KI",
                              "CURRENT_MULTIPLIER",
                              "KB",
                              "TORQUE_CONSTANT",
                              "R_ARM",
                              "L_ARM",
                              "K_FF_VEL",
                              "K_FF_ACCEL",
                              "STATIC_KICK",
                              "COULOMB_RUN",
                              "STRIBECK_WIDTH",
                              "VISCOUS_FRICTION",
                              "IR_COMP_GAIN",
                              "VISCOUS_FRICTION_LIMIT",
                              "EB_FF_LIMIT" };

bool params_bool[SIZE_PARAMS_BOOL] = { DEFAULT_AUTO_SYNC,
                                       DEFAULT_ADC_SYNC,
                                       DEFAULT_CASCADED,
                                       DEFAULT_IR_COMP,
                                       DEFAULT_CASCADE_FILTER,
                                       DEFAULT_AUTO_BIAS,
                                       DEFAULT_ADC_MULTIPHASE,
                                       DEFAULT_ADC_BIPHASE,
                                       DEFAULT_OUTER_FEEDFORWARD,
                                       DEFAULT_OUTER_SCV,
                                       DEFAULT_OMEGA_FILTER,
                                       DEFAULT_VOLTAGE_FILTER,
                                       DEFAULT_AUTO_BRAKE,
                                       DEFAULT_INDUCTIVE_COMPENSATION };

const char *names_bool[] = { "AUTO_SYNC",
                             "ADC_SYNC",
                             "CASCADED",
                             "IR_COMP",
                             "CASCADE_FILTER",
                             "AUTO_BIAS",
                             "ADC_MULTIPHASE",
                             "ADC_BIPHASE",
                             "OUTER_FEEDFORWARD",
                             "OUTER_SCV",
                             "OMEGA_FILTER",
                             "VOLTAGE_FILTER",
                             "AUTO_BRAKE",
                             "INDUCTIVE_COMP" };

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
    C_TYPE_UINT32,
    C_TYPE_FLOAT,
    C_TYPE_BOOL
};

struct ParamMetadata {
    CParamDataType c_type;          // The target C data type
    uint8_t param_index;            // Parameter Index
};

const std::map<std::string, ParamMetadata> ParamMap = {

    {"CONFIG_FLAGS",            { CParamDataType::C_TYPE_UINT8,  PARAM_CONFIG_FLAGS } },
    {"UPDATE_RATE",             { CParamDataType::C_TYPE_UINT8,  PARAM_UPDATE_RATE } },
    {"PWM_DIV",                 { CParamDataType::C_TYPE_UINT8,  PARAM_PWM_DIV } },
    {"DRIVE_MODE",              { CParamDataType::C_TYPE_UINT8,  PARAM_DRIVE_MODE } },
    {"MONITOR_RATE",             { CParamDataType::C_TYPE_UINT8,  PARAM_MONITOR_RATE } },
    {"ALLOWED_SKIP",             { CParamDataType::C_TYPE_UINT8,  PARAM_ALLOWED_SKIP } },
    {"OUTPUT_FILTER_TYPE",      { CParamDataType::C_TYPE_UINT8,  PARAM_OUTPUT_FILTER_TYPE } },
    {"CS_WAVEFORM_DIV",         { CParamDataType::C_TYPE_UINT8,  PARAM_CS_WAVEFORM_DIVIDER } },

    {"PWM_SCALE",               { CParamDataType::C_TYPE_UINT16, PARAM_PWM_SCALE } },
    {"PWM_FRQ",                 { CParamDataType::C_TYPE_UINT16, PARAM_PWM_FRQ } },
    {"MAX_IDLE_SECONDS",        { CParamDataType::C_TYPE_UINT16, PARAM_MAX_IDLE_SECONDS } },
    {"UPPER_LIMIT",             { CParamDataType::C_TYPE_UINT16, PARAM_UPPER_LIMIT } },
    {"INNER_LIMIT",             { CParamDataType::C_TYPE_UINT16, PARAM_INNER_LIMIT } },
    {"ENCODER_PPR",             { CParamDataType::C_TYPE_UINT16, PARAM_ENCODER_PPR } },
    {"INA219_CAL",              { CParamDataType::C_TYPE_UINT16, PARAM_INA219_CAL } },
    {"ADC_SPEED",               { CParamDataType::C_TYPE_UINT16, PARAM_ADC_SPEED } },

    {"LEFT_FORWARD_DEADZONE",   { CParamDataType::C_TYPE_INT16,  PARAM_LEFT_FORWARD_DEADZONE } },
    {"LEFT_REVERSE_DEADZONE",   { CParamDataType::C_TYPE_INT16,  PARAM_LEFT_REVERSE_DEADZONE } },
    {"RIGHT_FORWARD_DEADZONE",  { CParamDataType::C_TYPE_INT16,  PARAM_RIGHT_FORWARD_DEADZONE } },
    {"RIGHT_REVERSE_DEADZONE",  { CParamDataType::C_TYPE_INT16,  PARAM_RIGHT_REVERSE_DEADZONE } },
    {"CS_LEFT_OFFSET",          { CParamDataType::C_TYPE_INT16,  PARAM_CS_LEFT_OFFSET } },
    {"CS_RIGHT_OFFSET",         { CParamDataType::C_TYPE_INT16,  PARAM_CS_RIGHT_OFFSET } },

    {"RTC_TRIM",                { CParamDataType::C_TYPE_UINT32,  PARAM_RTC_TRIM } },

    {"GEAR_RATIO",              { CParamDataType::C_TYPE_FLOAT,  PARAM_GEAR_RATIO } },
    {"WHEEL_DIA",               { CParamDataType::C_TYPE_FLOAT,  PARAM_WHEEL_DIA } },
    {"BASE_WIDTH",              { CParamDataType::C_TYPE_FLOAT,  PARAM_BASE_WIDTH } },
    {"MAIN_AMP_LIMIT",          { CParamDataType::C_TYPE_FLOAT,  PARAM_MAIN_AMP_LIMIT } },
    {"BAT_VOLTS_HIGH",          { CParamDataType::C_TYPE_FLOAT,  PARAM_BAT_VOLTS_HIGH } },
    {"BAT_VOLTS_LOW",           { CParamDataType::C_TYPE_FLOAT,  PARAM_BAT_VOLTS_LOW } },
    {"MAX_RPM",                 { CParamDataType::C_TYPE_FLOAT,  PARAM_MAX_RPM } },
    {"LEFT_AMP_LIMIT",          { CParamDataType::C_TYPE_FLOAT,  PARAM_LEFT_AMP_LIMIT } },
    {"RIGHT_AMP_LIMIT",         { CParamDataType::C_TYPE_FLOAT,  PARAM_RIGHT_AMP_LIMIT } },
    {"LEFT_KP",                 { CParamDataType::C_TYPE_FLOAT,  PARAM_LEFT_KP } },
    {"RIGHT_KP",                { CParamDataType::C_TYPE_FLOAT,  PARAM_RIGHT_KP } },
    {"LEFT_KI",                 { CParamDataType::C_TYPE_FLOAT,  PARAM_LEFT_KI } },
    {"RIGHT_KI",                { CParamDataType::C_TYPE_FLOAT,  PARAM_RIGHT_KI } },
    {"LEFT_KD",                 { CParamDataType::C_TYPE_FLOAT,  PARAM_LEFT_KD } },
    {"RIGHT_KD",                { CParamDataType::C_TYPE_FLOAT,  PARAM_RIGHT_KD } },
    {"GAIN",                    { CParamDataType::C_TYPE_FLOAT,  PARAM_GAIN } },
    {"TRIM",                    { CParamDataType::C_TYPE_FLOAT,  PARAM_TRIM } },
    {"MOTOR_CONSTANT",          { CParamDataType::C_TYPE_FLOAT,  PARAM_MOTOR_CONSTANT } },
    {"TANH_DIV",                { CParamDataType::C_TYPE_FLOAT,  PARAM_TANH_DIV } },
    {"SIGM_DIV",                { CParamDataType::C_TYPE_FLOAT,  PARAM_SIGM_DIV } },
    {"CURRENT_KP",              { CParamDataType::C_TYPE_FLOAT,  PARAM_CURRENT_KP } },
    {"CURRENT_KI",              { CParamDataType::C_TYPE_FLOAT,  PARAM_CURRENT_KI } },
    {"CURRENT_MULTIPLIER",      { CParamDataType::C_TYPE_FLOAT,  PARAM_CURRENT_OBSERVED_MULTIPLIER } },
    {"K_FB_WINDUP",             { CParamDataType::C_TYPE_FLOAT,  PARAM_KB } },
    {"TORQUE_CONSTANT",         { CParamDataType::C_TYPE_FLOAT,  PARAM_TORQUE_CONSTANT } },
    {"R_ARM",                   { CParamDataType::C_TYPE_FLOAT,  PARAM_R_ARM } },
    {"L_ARM",                   { CParamDataType::C_TYPE_FLOAT,  PARAM_L_ARM } },
    {"K_FF_VEL",                { CParamDataType::C_TYPE_FLOAT,  PARAM_K_FF_VEL } },
    {"K_FF_ACCEL",              { CParamDataType::C_TYPE_FLOAT,  PARAM_K_FF_ACCEL } },
    {"STATIC_KICK",             { CParamDataType::C_TYPE_FLOAT, PARAM_STATIC_KICK } },
    {"COULOMB_RUN",             { CParamDataType::C_TYPE_FLOAT, PARAM_COULOMB_RUN } },
    {"STRIBECK_WIDTH",          { CParamDataType::C_TYPE_FLOAT, PARAM_STRIBECK_WIDTH } },
    {"VISCOUS_FRICTION",        { CParamDataType::C_TYPE_FLOAT, PARAM_VISCOUS_FRICTION } },
    {"IR_COMP_GAIN",            { CParamDataType::C_TYPE_FLOAT, PARAM_IR_COMP_GAIN } },
    {"VISCOUS_FRICTION_LIMIT",  { CParamDataType::C_TYPE_FLOAT, PARAM_VISCOUS_FRICTION_LIMIT } },
    {"EB_FF_LIMIT",             { CParamDataType::C_TYPE_FLOAT, PARAM_EB_FF_LIMIT } },

    {"AUTO_SYNC",               { CParamDataType::C_TYPE_BOOL,  PARAM_AUTO_SYNC } },
    {"ADC_SYNC",                { CParamDataType::C_TYPE_BOOL,  PARAM_ADC_SYNC } },
    {"CASCADED",                { CParamDataType::C_TYPE_BOOL,  PARAM_CASCADED } },
    {"IR_COMP",                 { CParamDataType::C_TYPE_BOOL,  PARAM_IR_COMP } },
    {"CASCADE_FILTER",          { CParamDataType::C_TYPE_BOOL,  PARAM_CASCADE_FILTER } },
    {"AUTO_BIAS",               { CParamDataType::C_TYPE_BOOL,  PARAM_AUTO_BIAS } },
    {"ADC_MULTIPHASE",          { CParamDataType::C_TYPE_BOOL,  PARAM_ADC_MULTIPHASE } },
    {"ADC_BIPHASE",             { CParamDataType::C_TYPE_BOOL,  PARAM_ADC_BIPHASE } },
    {"OUTER_FEEDFORWARD",       { CParamDataType::C_TYPE_BOOL,  PARAM_OUTER_FEEDFORWARD } },
    {"OUTER_SCV",               { CParamDataType::C_TYPE_BOOL,  PARAM_OUTER_SCV } },
    {"OMEGA_FILTER",            { CParamDataType::C_TYPE_BOOL,  PARAM_OMEGA_FILTER } },
    {"VOLTAGE_FILTER",          { CParamDataType::C_TYPE_BOOL,  PARAM_VOLTAGE_FILTER } },
    {"AUTO_BRAKE",              { CParamDataType::C_TYPE_BOOL,  PARAM_AUTO_BRAKE } },
    {"INDUCTIVE_COMP",          { CParamDataType::C_TYPE_BOOL,  PARAM_INDUCTIVE_COMPENSATION } },
};

const ParamMetadata* get_param_metadata(const std::string& param_name) {
    auto it = ParamMap.find(param_name);
    if (it != ParamMap.end()) { return &(it->second); }
    return nullptr;
}

#endif

// main parameters