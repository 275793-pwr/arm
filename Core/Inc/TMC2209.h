/*
 * TMC2209.h
 *
 * Authors:
 * Peter Polidoro peter@polidoro.io
 * Refactored by Cline
 */

#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
#include "stdint.h" // Include for fixed-width integer types
#include "stdbool.h" // Include for bool type
// Serial Settings
#define BYTE_MAX_VALUE 0xFF
#define BITS_PER_BYTE 8

#define ECHO_DELAY_INC_MICROSECONDS 1
#define ECHO_DELAY_MAX_MICROSECONDS 4000

#define REPLY_DELAY_INC_MICROSECONDS 1
#define REPLY_DELAY_MAX_MICROSECONDS 10000

#define STEPPER_DRIVER_FEATURE_OFF 0
#define STEPPER_DRIVER_FEATURE_ON 1

#define MAX_READ_RETRIES 5
#define READ_RETRY_DELAY_MS 20

// Datagrams
#define WRITE_READ_REPLY_DATAGRAM_SIZE 8
#define DATA_SIZE 4

#define SYNC 0b101
#define RW_READ 0
#define RW_WRITE 1
#define READ_REPLY_SERIAL_ADDRESS 0b11111111

#define READ_REQUEST_DATAGRAM_SIZE 4

// General Configuration Registers
#define ADDRESS_GCONF 0x00
#define ADDRESS_GSTAT 0x01
#define ADDRESS_IFCNT 0x02
#define ADDRESS_REPLYDELAY 0x03
#define ADDRESS_IOIN 0x06
#define VERSION 0x21

// Velocity Dependent Driver Feature Control Register Set
#define ADDRESS_IHOLD_IRUN 0x10
#define PERCENT_MIN 0
#define PERCENT_MAX 100
#define CURRENT_SETTING_MIN 0
#define CURRENT_SETTING_MAX 31
#define HOLD_DELAY_MIN 0
#define HOLD_DELAY_MAX 15
#define IHOLD_DEFAULT 16
#define IRUN_DEFAULT 31
#define IHOLDDELAY_DEFAULT 1

#define ADDRESS_TPOWERDOWN 0x11
#define TPOWERDOWN_DEFAULT 20

#define ADDRESS_TSTEP 0x12

#define ADDRESS_TPWMTHRS 0x13
#define TPWMTHRS_DEFAULT 0

#define ADDRESS_VACTUAL 0x22
#define VACTUAL_DEFAULT 0
#define VACTUAL_STEP_DIR_INTERFACE 0

// CoolStep and StallGuard Control Register Set
#define ADDRESS_TCOOLTHRS 0x14
#define TCOOLTHRS_DEFAULT 0
#define ADDRESS_SGTHRS 0x40
#define SGTHRS_DEFAULT 0
#define ADDRESS_SG_RESULT 0x41

#define ADDRESS_COOLCONF 0x42
#define COOLCONF_DEFAULT 0
#define SEIMIN_UPPER_CURRENT_LIMIT 20
#define SEIMIN_LOWER_SETTING 0
#define SEIMIN_UPPER_SETTING 1
#define SEMIN_OFF 0
#define SEMIN_MIN 1
#define SEMIN_MAX 15
#define SEMAX_MIN 0
#define SEMAX_MAX 15

// Microstepping Control Register Set
#define ADDRESS_MSCNT 0x6A
#define ADDRESS_MSCURACT 0x6B

// Driver Register Set
#define ADDRESS_CHOPCONF 0x6C
#define CHOPPER_CONFIG_DEFAULT 0x10000053
#define TBL_DEFAULT 0b10
#define HEND_DEFAULT 0
#define HSTART_DEFAULT 5
#define TOFF_DEFAULT 3
#define TOFF_DISABLE 0
#define MRES_256 0b0000
#define MRES_128 0b0001
#define MRES_064 0b0010
#define MRES_032 0b0011
#define MRES_016 0b0100
#define MRES_008 0b0101
#define MRES_004 0b0110
#define MRES_002 0b0111
#define MRES_001 0b1000
#define DOUBLE_EDGE_DISABLE 0
#define DOUBLE_EDGE_ENABLE 1
#define VSENSE_DISABLE 0
#define VSENSE_ENABLE 1

#define MICROSTEPS_PER_STEP_MIN 1
#define MICROSTEPS_PER_STEP_MAX 256

#define ADDRESS_DRV_STATUS 0x6F
#define CURRENT_SCALING_MAX 31

#define ADDRESS_PWMCONF 0x70
#define PWM_CONFIG_DEFAULT 0xC10D0024
#define PWM_OFFSET_MIN 0
#define PWM_OFFSET_MAX 255
#define PWM_OFFSET_DEFAULT 0x24
#define PWM_GRAD_MIN 0
#define PWM_GRAD_MAX 255
#define PWM_GRAD_DEFAULT 0x14

#define ADDRESS_PWM_SCALE 0x71
#define ADDRESS_PWM_AUTO 0x72

typedef enum {
    SERIAL_ADDRESS_0 = 0,
    SERIAL_ADDRESS_1 = 1,
    SERIAL_ADDRESS_2 = 2,
    SERIAL_ADDRESS_3 = 3,
} TMC2209_SerialAddress_t;

typedef enum {
    NORMAL = 0,
    FREEWHEELING = 1,
    STRONG_BRAKING = 2,
    BRAKING = 3,
} TMC2209_StandstillMode_t;

typedef enum {
    CURRENT_INCREMENT_1 = 0,
    CURRENT_INCREMENT_2 = 1,
    CURRENT_INCREMENT_4 = 2,
    CURRENT_INCREMENT_8 = 3,
} TMC2209_CurrentIncrement_t;

typedef enum {
    MEASUREMENT_COUNT_32 = 0,
    MEASUREMENT_COUNT_8 = 1,
    MEASUREMENT_COUNT_2 = 2,
    MEASUREMENT_COUNT_1 = 3,
} TMC2209_MeasurementCount_t;

typedef struct {
    uint32_t over_temperature_warning : 1;
    uint32_t over_temperature_shutdown : 1;
    uint32_t short_to_ground_a : 1;
    uint32_t short_to_ground_b : 1;
    uint32_t low_side_short_a : 1;
    uint32_t low_side_short_b : 1;
    uint32_t open_load_a : 1;
    uint32_t open_load_b : 1;
    uint32_t over_temperature_120c : 1;
    uint32_t over_temperature_143c : 1;
    uint32_t over_temperature_150c : 1;
    uint32_t over_temperature_157c : 1;
    uint32_t reserved0 : 4;
    uint32_t current_scaling : 5;
    uint32_t reserved1 : 9;
    uint32_t stealth_chop_mode : 1;
    uint32_t standstill : 1;
} TMC2209_Status_t;

typedef struct {
    uint32_t reset : 1;
    uint32_t drv_err : 1;
    uint32_t uv_cp : 1;
    uint32_t reserved : 29;
} TMC2209_GlobalStatus_t;

typedef struct {
    bool is_communicating;
    bool is_setup;
    bool software_enabled;
    uint16_t microsteps_per_step;
    bool inverse_motor_direction_enabled;
    bool stealth_chop_enabled;
    uint8_t standstill_mode;
    uint8_t irun_percent;
    uint8_t irun_register_value;
    uint8_t ihold_percent;
    uint8_t ihold_register_value;
    uint8_t iholddelay_percent;
    uint8_t iholddelay_register_value;
    bool automatic_current_scaling_enabled;
    bool automatic_gradient_adaptation_enabled;
    uint8_t pwm_offset;
    uint8_t pwm_gradient;
    bool cool_step_enabled;
    bool analog_current_scaling_enabled;
    bool internal_sense_resistors_enabled;
} TMC2209_Settings_t;


// Unions for register access (moved from .c to .h for struct definition)
typedef union {
    struct {
        uint64_t sync : 4;
        uint64_t reserved : 4;
        uint64_t serial_address : 8;
        uint64_t register_address : 7;
        uint64_t rw : 1;
        uint64_t data : 32;
        uint64_t crc : 8;
    };
    uint64_t bytes;
} TMC2209_WriteReadReplyDatagram_t;

typedef union {
    struct {
        uint32_t sync : 4;
        uint32_t reserved : 4;
        uint32_t serial_address : 8;
        uint32_t register_address : 7;
        uint32_t rw : 1;
        uint32_t crc : 8;
    };
    uint32_t bytes;
} TMC2209_ReadRequestDatagram_t;

typedef union {
    struct {
        uint32_t i_scale_analog : 1;
        uint32_t internal_rsense : 1;
        uint32_t enable_spread_cycle : 1;
        uint32_t shaft : 1;
        uint32_t index_otpw : 1;
        uint32_t index_step : 1;
        uint32_t pdn_disable : 1;
        uint32_t mstep_reg_select : 1;
        uint32_t multistep_filt : 1;
        uint32_t test_mode : 1;
        uint32_t reserved : 22;
    };
    uint32_t bytes;
} TMC2209_GlobalConfig_t;

typedef union {
    struct {
        TMC2209_GlobalStatus_t global_status;
    };
    uint32_t bytes;
} TMC2209_GlobalStatusUnion_t;

typedef union {
    struct {
        uint32_t reserved_0 : 8;
        uint32_t replydelay : 4;
        uint32_t reserved_1 : 20;
    };
    uint32_t bytes;
} TMC2209_ReplyDelay_t;

typedef union {
    struct {
        uint32_t enn : 1;
        uint32_t reserved_0 : 1;
        uint32_t ms1 : 1;
        uint32_t ms2 : 1;
        uint32_t diag : 1;
        uint32_t reserved_1 : 1;
        uint32_t pdn_serial : 1;
        uint32_t step : 1;
        uint32_t spread_en : 1;
        uint32_t dir : 1;
        uint32_t reserved_2 : 14;
        uint32_t version : 8;
    };
    uint32_t bytes;
} TMC2209_Input_t;

typedef union {
    struct {
        uint32_t ihold : 5;
        uint32_t reserved_0 : 3;
        uint32_t irun : 5;
        uint32_t reserved_1 : 3;
        uint32_t iholddelay : 4;
        uint32_t reserved_2 : 12;
    };
    uint32_t bytes;
} TMC2209_DriverCurrent_t;

typedef union {
    struct {
        uint32_t semin : 4;
        uint32_t reserved_0 : 1;
        uint32_t seup : 2;
        uint32_t reserved_1 : 1;
        uint32_t semax : 4;
        uint32_t reserved_2 : 1;
        uint32_t sedn : 2;
        uint32_t seimin : 1;
        uint32_t reserved_3 : 16;
    };
    uint32_t bytes;
} TMC2209_CoolConfig_t;

typedef union {
    struct {
        uint32_t toff : 4;
        uint32_t hstart : 3;
        uint32_t hend : 4;
        uint32_t reserved_0 : 4;
        uint32_t tbl : 2;
        uint32_t vsense : 1;
        uint32_t reserved_1 : 6;
        uint32_t mres : 4;
        uint32_t interpolation : 1;
        uint32_t double_edge : 1;
        uint32_t diss2g : 1;
        uint32_t diss2vs : 1;
    };
    uint32_t bytes;
} TMC2209_ChopperConfig_t;

typedef union {
    struct {
        TMC2209_Status_t status;
    };
    uint32_t bytes;
} TMC2209_DriveStatus_t;

typedef union {
    struct {
        uint32_t pwm_offset : 8;
        uint32_t pwm_grad : 8;
        uint32_t pwm_freq : 2;
        uint32_t pwm_autoscale : 1;
        uint32_t pwm_autograd : 1;
        uint32_t freewheel : 2;
        uint32_t reserved : 2;
        uint32_t pwm_reg : 4;
        uint32_t pwm_lim : 4;
    };
    uint32_t bytes;
} TMC2209_PwmConfig_t;

typedef union {
    struct {
        uint32_t pwm_scale_sum : 8;
        uint32_t reserved_0 : 8;
        uint32_t pwm_scale_auto : 9;
        uint32_t reserved_1 : 7;
    };
    uint32_t bytes;
} TMC2209_PwmScale_t;

typedef union {
    struct {
        uint32_t pwm_offset_auto : 8;
        uint32_t reserved_0 : 8;
        uint32_t pwm_gradient_auto : 8;
        uint32_t reserved_1 : 8;
    };
    uint32_t bytes;
} TMC2209_PwmAuto_t;


// Handle struct
typedef struct {
    UART_HandleTypeDef * huart;
    uint8_t serial_address;
    int16_t hardware_enable_pin; // Assuming this was intended for a GPIO pin number

    // Register values (corresponding to the unions above)
    TMC2209_GlobalConfig_t global_config;
    TMC2209_DriverCurrent_t driver_current;
    TMC2209_ChopperConfig_t chopper_config;
    TMC2209_PwmConfig_t pwm_config;
    TMC2209_CoolConfig_t cool_config;

    uint8_t toff; // Corresponds to toff_ in the class
    bool cool_step_enabled; // Corresponds to cool_step_enabled_ in the class

} TMC2209_HandleTypeDef;


// Public functions
TMC2209_HandleTypeDef TMC2209_create(UART_HandleTypeDef * huart, TMC2209_SerialAddress_t serial_address);
void TMC2209_init(TMC2209_HandleTypeDef *htmc);

// unidirectional methods
void TMC2209_enable(TMC2209_HandleTypeDef *htmc);
void TMC2209_disable(TMC2209_HandleTypeDef *htmc);
void TMC2209_setMicrostepsPerStep(TMC2209_HandleTypeDef *htmc, uint16_t microsteps_per_step);
void TMC2209_setMicrostepsPerStepPowerOfTwo(TMC2209_HandleTypeDef *htmc, uint8_t exponent);
void TMC2209_setRunCurrent(TMC2209_HandleTypeDef *htmc, uint8_t percent);
void TMC2209_setHoldCurrent(TMC2209_HandleTypeDef *htmc, uint8_t percent);
void TMC2209_setHoldDelay(TMC2209_HandleTypeDef *htmc, uint8_t percent);
void TMC2209_setAllCurrentValues(TMC2209_HandleTypeDef *htmc, uint8_t run_current_percent, uint8_t hold_current_percent, uint8_t hold_delay_percent);
void TMC2209_setRMSCurrent(TMC2209_HandleTypeDef *htmc, uint16_t mA, float rSense, float holdMultiplier);
void TMC2209_enableDoubleEdge(TMC2209_HandleTypeDef *htmc);
void TMC2209_disableDoubleEdge(TMC2209_HandleTypeDef *htmc);
void TMC2209_enableVSense(TMC2209_HandleTypeDef *htmc);
void TMC2209_disableVSense(TMC2209_HandleTypeDef *htmc);
void TMC2209_enableInverseMotorDirection(TMC2209_HandleTypeDef *htmc);
void TMC2209_disableInverseMotorDirection(TMC2209_HandleTypeDef *htmc);
void TMC2209_setStandstillMode(TMC2209_HandleTypeDef *htmc, TMC2209_StandstillMode_t mode);
void TMC2209_enableAutomaticCurrentScaling(TMC2209_HandleTypeDef *htmc);
void TMC2209_disableAutomaticCurrentScaling(TMC2209_HandleTypeDef *htmc);
void TMC2209_enableAutomaticGradientAdaptation(TMC2209_HandleTypeDef *htmc);
void TMC2209_disableAutomaticGradientAdaptation(TMC2209_HandleTypeDef *htmc);
void TMC2209_setPwmOffset(TMC2209_HandleTypeDef *htmc, uint8_t pwm_amplitude);
void TMC2209_setPwmGradient(TMC2209_HandleTypeDef *htmc, uint8_t pwm_amplitude);
void TMC2209_setPowerDownDelay(TMC2209_HandleTypeDef *htmc, uint8_t power_down_delay);
void TMC2209_setReplyDelay(TMC2209_HandleTypeDef *htmc, uint8_t delay);
void TMC2209_moveAtVelocity(TMC2209_HandleTypeDef *htmc, int32_t microsteps_per_period);
void TMC2209_moveUsingStepDirInterface(TMC2209_HandleTypeDef *htmc);
void TMC2209_enableStealthChop(TMC2209_HandleTypeDef *htmc);
void TMC2209_disableStealthChop(TMC2209_HandleTypeDef *htmc);
void TMC2209_setStealthChopDurationThreshold(TMC2209_HandleTypeDef *htmc, uint32_t duration_threshold);
void TMC2209_setStallGuardThreshold(TMC2209_HandleTypeDef *htmc, uint8_t stall_guard_threshold);
void TMC2209_enableCoolStep(TMC2209_HandleTypeDef *htmc, uint8_t lower_threshold, uint8_t upper_threshold);
void TMC2209_disableCoolStep(TMC2209_HandleTypeDef *htmc);
void TMC2209_setCoolStepCurrentIncrement(TMC2209_HandleTypeDef *htmc, TMC2209_CurrentIncrement_t current_increment);
void TMC2209_setCoolStepMeasurementCount(TMC2209_HandleTypeDef *htmc, TMC2209_MeasurementCount_t measurement_count);
void TMC2209_setCoolStepDurationThreshold(TMC2209_HandleTypeDef *htmc, uint32_t duration_threshold);
void TMC2209_enableAnalogCurrentScaling(TMC2209_HandleTypeDef *htmc);
void TMC2209_disableAnalogCurrentScaling(TMC2209_HandleTypeDef *htmc);
void TMC2209_useExternalSenseResistors(TMC2209_HandleTypeDef *htmc);
void TMC2209_useInternalSenseResistors(TMC2209_HandleTypeDef *htmc);

// bidirectional methods
uint8_t TMC2209_getVersion(TMC2209_HandleTypeDef *htmc);
bool TMC2209_isCommunicating(TMC2209_HandleTypeDef *htmc);
bool TMC2209_isSetupAndCommunicating(TMC2209_HandleTypeDef *htmc);
bool TMC2209_isCommunicatingButNotSetup(TMC2209_HandleTypeDef *htmc);
bool TMC2209_hardwareDisabled(TMC2209_HandleTypeDef *htmc);
uint16_t TMC2209_getMicrostepsPerStep(TMC2209_HandleTypeDef *htmc);
TMC2209_Settings_t TMC2209_getSettings(TMC2209_HandleTypeDef *htmc);
TMC2209_Status_t TMC2209_getStatus(TMC2209_HandleTypeDef *htmc);
TMC2209_GlobalStatus_t TMC2209_getGlobalStatus(TMC2209_HandleTypeDef *htmc);
void TMC2209_clearReset(TMC2209_HandleTypeDef *htmc);
void TMC2209_clearDriveError(TMC2209_HandleTypeDef *htmc);
uint8_t TMC2209_getInterfaceTransmissionCounter(TMC2209_HandleTypeDef *htmc);
uint32_t TMC2209_getInterstepDuration(TMC2209_HandleTypeDef *htmc);
uint16_t TMC2209_getStallGuardResult(TMC2209_HandleTypeDef *htmc);
uint8_t TMC2209_getPwmScaleSum(TMC2209_HandleTypeDef *htmc);
int16_t TMC2209_getPwmScaleAuto(TMC2209_HandleTypeDef *htmc);
uint8_t TMC2209_getPwmOffsetAuto(TMC2209_HandleTypeDef *htmc);
uint8_t TMC2209_getPwmGradientAuto(TMC2209_HandleTypeDef *htmc);
uint16_t TMC2209_getMicrostepCounter(TMC2209_HandleTypeDef *htmc);


#endif /* TMC2209_H */
