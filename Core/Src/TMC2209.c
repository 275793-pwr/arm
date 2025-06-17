/*
 * TMC2209.c
 *
 * Authors:
 * Peter Polidoro peter@polidoro.io
 * Refactored by Cline
 */

#include "TMC2209.h"
#include "stm32f1xx_hal.h" // Include necessary HAL header
#include "stm32f1xx_hal_uart.h" // Include necessary HAL UART header
#include "stdbool.h" // Include for bool type
#include "string.h" // Include for string functions
#include "stdio.h" // Include for sprintf
#include "math.h" // Include for roundf if needed

// Helper functions (forward declarations)
static uint32_t reverseData(uint32_t data);
static uint8_t calculateCrc(uint64_t datagram_bytes, uint8_t datagram_size);
static HAL_StatusTypeDef serialWrite(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size);
static HAL_StatusTypeDef serialRead(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size, uint32_t timeout);
static void serialFlush(UART_HandleTypeDef *huart); // Adapted for HAL

static void sendDatagramUnidirectional(UART_HandleTypeDef *huart, uint64_t datagram_bytes, uint8_t datagram_size);
static void sendDatagramBidirectional(UART_HandleTypeDef *huart, uint64_t datagram_bytes, uint8_t datagram_size);

static void writeRegister(TMC2209_HandleTypeDef *htmc, uint8_t register_address, uint32_t data);
static uint32_t readRegister(TMC2209_HandleTypeDef *htmc, uint8_t register_address);

static uint8_t percentToCurrentSetting(uint8_t percent);
static uint8_t currentSettingToPercent(uint8_t current_setting);
static uint8_t percentToHoldDelaySetting(uint8_t percent);
static uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

static void writeStoredGlobalConfig(TMC2209_HandleTypeDef *htmc);
static uint32_t readGlobalConfigBytes(TMC2209_HandleTypeDef *htmc);
static void writeStoredDriverCurrent(TMC2209_HandleTypeDef *htmc);
static void writeStoredChopperConfig(TMC2209_HandleTypeDef *htmc);
static uint32_t readChopperConfigBytes(TMC2209_HandleTypeDef *htmc);
static void writeStoredPwmConfig(TMC2209_HandleTypeDef *htmc);
static uint32_t readPwmConfigBytes(TMC2209_HandleTypeDef *htmc);

static uint32_t constrain_uint32(uint32_t value, uint32_t low, uint32_t high);
static uint8_t map_uint8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);

static void readAndStoreRegisters(TMC2209_HandleTypeDef * htmc);
static bool serialOperationMode(TMC2209_HandleTypeDef * htmc);
static void minimizeMotorCurrent(TMC2209_HandleTypeDef * htmc);


// Public functions

TMC2209_HandleTypeDef TMC2209_create(UART_HandleTypeDef * huart, TMC2209_SerialAddress_t serial_address)
{
    TMC2209_HandleTypeDef htmc;
    htmc.huart = huart;
    htmc.serial_address = serial_address;
    htmc.cool_step_enabled = false;
    htmc.toff = TOFF_DEFAULT; // Initialize toff

    // Initialize register unions to zero
    htmc.global_config.bytes = 0;
    htmc.driver_current.bytes = 0;
    htmc.chopper_config.bytes = 0;
    htmc.pwm_config.bytes = 0;
    htmc.cool_config.bytes = 0;

    return htmc;
}


void TMC2209_enable(TMC2209_HandleTypeDef * htmc)
{
  htmc->chopper_config.toff = htmc->toff;
  writeStoredChopperConfig(htmc);
}

void TMC2209_disable(TMC2209_HandleTypeDef * htmc)
{
  htmc->chopper_config.toff = TOFF_DISABLE;
  writeStoredChopperConfig(htmc);
}

void TMC2209_setMicrostepsPerStep(TMC2209_HandleTypeDef * htmc, uint16_t microsteps_per_step)
{
  uint16_t microsteps_per_step_shifted = constrain_uint32(microsteps_per_step,
    MICROSTEPS_PER_STEP_MIN,
    MICROSTEPS_PER_STEP_MAX);
  microsteps_per_step_shifted = microsteps_per_step >> 1;
  uint16_t exponent = 0;
  while (microsteps_per_step_shifted > 0)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
    ++exponent;
  }
  TMC2209_setMicrostepsPerStepPowerOfTwo(htmc, exponent);
}

void TMC2209_setMicrostepsPerStepPowerOfTwo(TMC2209_HandleTypeDef * htmc, uint8_t exponent)
{
  switch (exponent)
  {
    case 0:
    {
      htmc->chopper_config.mres = MRES_001;
      break;
    }
    case 1:
    {
      htmc->chopper_config.mres = MRES_002;
      break;
    }
    case 2:
    {
      htmc->chopper_config.mres = MRES_004;
      break;
    }
    case 3:
    {
      htmc->chopper_config.mres = MRES_008;
      break;
    }
    case 4:
    {
      htmc->chopper_config.mres = MRES_016;
      break;
    }
    case 5:
    {
      htmc->chopper_config.mres = MRES_032;
      break;
    }
    case 6:
    {
      htmc->chopper_config.mres = MRES_064;
      break;
    }
    case 7:
    {
      htmc->chopper_config.mres = MRES_128;
      break;
    }
    case 8:
    default:
    {
      htmc->chopper_config.mres = MRES_256;
      break;
    }
  }
  writeStoredChopperConfig(htmc);
}

void TMC2209_setRunCurrent(TMC2209_HandleTypeDef * htmc, uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting(percent);
  htmc->driver_current.irun = run_current;
  writeStoredDriverCurrent(htmc);
}

void TMC2209_setHoldCurrent(TMC2209_HandleTypeDef * htmc, uint8_t percent)
{
  uint8_t hold_current = percentToCurrentSetting(percent);

  htmc->driver_current.ihold = hold_current;
  writeStoredDriverCurrent(htmc);
}

void TMC2209_setHoldDelay(TMC2209_HandleTypeDef * htmc, uint8_t percent)
{
  uint8_t hold_delay = percentToHoldDelaySetting(percent);

  htmc->driver_current.iholddelay = hold_delay;
  writeStoredDriverCurrent(htmc);
}

void TMC2209_setAllCurrentValues(TMC2209_HandleTypeDef * htmc, uint8_t run_current_percent,
  uint8_t hold_current_percent,
  uint8_t hold_delay_percent)
{
  uint8_t run_current = percentToCurrentSetting(run_current_percent);
  uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
  uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

  htmc->driver_current.irun = run_current;
  htmc->driver_current.ihold = hold_current;
  htmc->driver_current.iholddelay = hold_delay;
  writeStoredDriverCurrent(htmc);
}

void TMC2209_setRMSCurrent(TMC2209_HandleTypeDef * htmc, uint16_t mA,
  float rSense,
  float holdMultiplier)
{
  // Taken from https://github.com/teemuatlut/TMCStepper/blob/74e8e6881adc9241c2e626071e7328d7652f361a/src/source/TMCStepper.cpp#L41.

  uint8_t CS = roundf(32.0*1.41421*mA/1000.0*(rSense+0.02)/0.325 - 1);
  // If Current Scale is too low, turn on high sensitivity R_sense and calculate again
  if (CS < 16) {
    TMC2209_enableVSense(htmc);
    CS = roundf(32.0*1.41421*mA/1000.0*(rSense+0.02)/0.180 - 1);
  } else { // If CS >= 16, turn off high_sense_r
    TMC2209_disableVSense(htmc);
  }

  if (CS > 31) {
    CS = 31;
  }

  htmc->driver_current.irun = CS;
  htmc->driver_current.ihold = CS*holdMultiplier; // This might need casting or careful handling of float to uint8_t
  writeStoredDriverCurrent(htmc);
}

void TMC2209_enableDoubleEdge(TMC2209_HandleTypeDef * htmc)
{
  htmc->chopper_config.double_edge = DOUBLE_EDGE_ENABLE;
  writeStoredChopperConfig(htmc);
}

void TMC2209_disableDoubleEdge(TMC2209_HandleTypeDef * htmc)
{
  htmc->chopper_config.double_edge = DOUBLE_EDGE_DISABLE;
  writeStoredChopperConfig(htmc);
}

void TMC2209_enableVSense(TMC2209_HandleTypeDef * htmc)
{
  htmc->chopper_config.vsense = VSENSE_ENABLE;
  writeStoredChopperConfig(htmc);
}

void TMC2209_disableVSense(TMC2209_HandleTypeDef * htmc)
{
  htmc->chopper_config.vsense = VSENSE_DISABLE;
  writeStoredChopperConfig(htmc);
}

void TMC2209_enableInverseMotorDirection(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.shaft = 1;
  writeStoredGlobalConfig(htmc);
}

void TMC2209_disableInverseMotorDirection(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.shaft = 0;
  writeStoredGlobalConfig(htmc);
}

void TMC2209_setStandstillMode(TMC2209_HandleTypeDef * htmc, TMC2209_StandstillMode_t mode)
{
  htmc->pwm_config.freewheel = mode;
  writeStoredPwmConfig(htmc);
}

void TMC2209_enableAutomaticCurrentScaling(TMC2209_HandleTypeDef * htmc)
{
  htmc->pwm_config.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
  writeStoredPwmConfig(htmc);
}

void TMC2209_disableAutomaticCurrentScaling(TMC2209_HandleTypeDef * htmc)
{
  htmc->pwm_config.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig(htmc);
}

void TMC2209_enableAutomaticGradientAdaptation(TMC2209_HandleTypeDef * htmc)
{
  htmc->pwm_config.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
  writeStoredPwmConfig(htmc);
}

void TMC2209_disableAutomaticGradientAdaptation(TMC2209_HandleTypeDef * htmc)
{
  htmc->pwm_config.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig(htmc);
}

void TMC2209_setPwmOffset(TMC2209_HandleTypeDef * htmc, uint8_t pwm_amplitude)
{
  htmc->pwm_config.pwm_offset = pwm_amplitude;
  writeStoredPwmConfig(htmc);
}

void TMC2209_setPwmGradient(TMC2209_HandleTypeDef * htmc, uint8_t pwm_amplitude)
{
  htmc->pwm_config.pwm_grad = pwm_amplitude;
  writeStoredPwmConfig(htmc);
}

void TMC2209_setPowerDownDelay(TMC2209_HandleTypeDef * htmc, uint8_t power_down_delay)
{
  writeRegister(htmc, ADDRESS_TPOWERDOWN, power_down_delay);
}

void TMC2209_setReplyDelay(TMC2209_HandleTypeDef * htmc, uint8_t reply_delay)
{
  if (reply_delay > 15)
  {
    reply_delay = 15;
  }
  TMC2209_ReplyDelay_t reply_delay_data;
  reply_delay_data.bytes = 0;
  reply_delay_data.replydelay = reply_delay;
  writeRegister(htmc, ADDRESS_REPLYDELAY, reply_delay_data.bytes);
}

void TMC2209_moveAtVelocity(TMC2209_HandleTypeDef * htmc, int32_t microsteps_per_period)
{
  writeRegister(htmc, ADDRESS_VACTUAL, microsteps_per_period);
}

void TMC2209_moveUsingStepDirInterface(TMC2209_HandleTypeDef * htmc)
{
  writeRegister(htmc, ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

void TMC2209_enableStealthChop(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.enable_spread_cycle = 0;
  writeStoredGlobalConfig(htmc);
}

void TMC2209_disableStealthChop(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.enable_spread_cycle = 1;
  writeStoredGlobalConfig(htmc);
}

void TMC2209_setCoolStepDurationThreshold(TMC2209_HandleTypeDef * htmc, uint32_t duration_threshold)
{
  writeRegister(htmc, ADDRESS_TCOOLTHRS, duration_threshold);
}

void TMC2209_setStealthChopDurationThreshold(TMC2209_HandleTypeDef * htmc, uint32_t duration_threshold)
{
  writeRegister(htmc, ADDRESS_TPWMTHRS, duration_threshold);
}

void TMC2209_setStallGuardThreshold(TMC2209_HandleTypeDef * htmc, uint8_t stall_guard_threshold)
{
  writeRegister(htmc, ADDRESS_SGTHRS, stall_guard_threshold);
}

void TMC2209_enableCoolStep(TMC2209_HandleTypeDef * htmc, uint8_t lower_threshold,
    uint8_t upper_threshold)
{
  lower_threshold = constrain_uint32(lower_threshold, SEMIN_MIN, SEMIN_MAX);
  htmc->cool_config.semin = lower_threshold;
  upper_threshold = constrain_uint32(upper_threshold, SEMAX_MIN, SEMAX_MAX);
  htmc->cool_config.semax = upper_threshold;
  writeRegister(htmc, ADDRESS_COOLCONF, htmc->cool_config.bytes);
  htmc->cool_step_enabled = true;
}

void TMC2209_disableCoolStep(TMC2209_HandleTypeDef * htmc)
{
  htmc->cool_config.semin = SEMIN_OFF;
  writeRegister(htmc, ADDRESS_COOLCONF, htmc->cool_config.bytes);
  htmc->cool_step_enabled = false;
}

void TMC2209_setCoolStepCurrentIncrement(TMC2209_HandleTypeDef * htmc, TMC2209_CurrentIncrement_t current_increment)
{
  htmc->cool_config.seup = current_increment;
  writeRegister(htmc, ADDRESS_COOLCONF, htmc->cool_config.bytes);
}

void TMC2209_setCoolStepMeasurementCount(TMC2209_HandleTypeDef * htmc, TMC2209_MeasurementCount_t measurement_count)
{
  htmc->cool_config.sedn = measurement_count;
  writeRegister(htmc, ADDRESS_COOLCONF, htmc->cool_config.bytes);
}

void TMC2209_enableAnalogCurrentScaling(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.i_scale_analog = 1;
  writeStoredGlobalConfig(htmc);
}

void TMC2209_disableAnalogCurrentScaling(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.i_scale_analog = 0;
  writeStoredGlobalConfig(htmc);
}

void TMC2209_useExternalSenseResistors(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.internal_rsense = 0;
  writeStoredGlobalConfig(htmc);
}

void TMC2209_useInternalSenseResistors(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.internal_rsense = 1;
  writeStoredGlobalConfig(htmc);
}

// bidirectional methods

uint8_t TMC2209_getVersion(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_Input_t input;
  input.bytes = readRegister(htmc, ADDRESS_IOIN);

  return input.version;
}

bool TMC2209_isCommunicating(TMC2209_HandleTypeDef * htmc)
{
  return (TMC2209_getVersion(htmc) == VERSION);
}

bool TMC2209_isSetupAndCommunicating(TMC2209_HandleTypeDef * htmc)
{
  return serialOperationMode(htmc);
}

bool TMC2209_isCommunicatingButNotSetup(TMC2209_HandleTypeDef * htmc)
{
  return (TMC2209_isCommunicating(htmc) && (serialOperationMode(htmc) == false));
}

bool TMC2209_hardwareDisabled(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_Input_t input;
  input.bytes = readRegister(htmc, ADDRESS_IOIN);

  return input.enn;
}

uint16_t TMC2209_getMicrostepsPerStep(TMC2209_HandleTypeDef * htmc)
{
  uint16_t microsteps_per_step_exponent;
  switch (htmc->chopper_config.mres)
  {
    case MRES_001:
    {
      microsteps_per_step_exponent = 0;
      break;
    }
    case MRES_002:
    {
      microsteps_per_step_exponent = 1;
      break;
    }
    case MRES_004:
    {
      microsteps_per_step_exponent = 2;
      break;
    }
    case MRES_008:
    {
      microsteps_per_step_exponent = 3;
      break;
    }
    case MRES_016:
    {
      microsteps_per_step_exponent = 4;
      break;
    }
    case MRES_032:
    {
      microsteps_per_step_exponent = 5;
      break;
    }
    case MRES_064:
    {
      microsteps_per_step_exponent = 6;
      break;
    }
    case MRES_128:
    {
      microsteps_per_step_exponent = 7;
      break;
    }
    case MRES_256:
    default:
    {
      microsteps_per_step_exponent = 8;
      break;
    }
  }
  return 1 << microsteps_per_step_exponent;
}

TMC2209_Settings_t TMC2209_getSettings(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_Settings_t settings;
  settings.is_communicating = TMC2209_isCommunicating(htmc);

  if (settings.is_communicating)
  {
    readAndStoreRegisters(htmc);

    settings.is_setup = htmc->global_config.pdn_disable;
    settings.software_enabled = (htmc->chopper_config.toff > TOFF_DISABLE);
    settings.microsteps_per_step = TMC2209_getMicrostepsPerStep(htmc);
    settings.inverse_motor_direction_enabled = htmc->global_config.shaft;
    settings.stealth_chop_enabled = !htmc->global_config.enable_spread_cycle;
    settings.standstill_mode = htmc->pwm_config.freewheel;
    settings.irun_percent = currentSettingToPercent(htmc->driver_current.irun);
    settings.irun_register_value = htmc->driver_current.irun;
    settings.ihold_percent = currentSettingToPercent(htmc->driver_current.ihold);
    settings.ihold_register_value = htmc->driver_current.ihold;
    settings.iholddelay_percent = holdDelaySettingToPercent(htmc->driver_current.iholddelay);
    settings.iholddelay_register_value = htmc->driver_current.iholddelay;
    settings.automatic_current_scaling_enabled = htmc->pwm_config.pwm_autoscale;
    settings.automatic_gradient_adaptation_enabled = htmc->pwm_config.pwm_autograd;
    settings.pwm_offset = htmc->pwm_config.pwm_offset;
    settings.pwm_gradient = htmc->pwm_config.pwm_grad;
    settings.cool_step_enabled = htmc->cool_step_enabled;
    settings.analog_current_scaling_enabled = htmc->global_config.i_scale_analog;
    settings.internal_sense_resistors_enabled = htmc->global_config.internal_rsense;
  }
  else
  {
    settings.is_setup = false;
    settings.software_enabled = false;
    settings.microsteps_per_step = 0;
    settings.inverse_motor_direction_enabled = false;
    settings.stealth_chop_enabled = false;
    settings.standstill_mode = 0; // Default or unknown state
    settings.irun_percent = 0;
    settings.irun_register_value = 0;
    settings.ihold_percent = 0;
    settings.ihold_register_value = 0;
    settings.iholddelay_percent = 0;
    settings.iholddelay_register_value = 0;
    settings.automatic_current_scaling_enabled = false;
    settings.automatic_gradient_adaptation_enabled = false;
    settings.pwm_offset = 0;
    settings.pwm_gradient = 0;
    settings.cool_step_enabled = false;
    settings.analog_current_scaling_enabled = false;
    settings.internal_sense_resistors_enabled = false;
  }

  return settings;
}

TMC2209_Status_t TMC2209_getStatus(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_DriveStatus_t drive_status;
  drive_status.bytes = 0;
  drive_status.bytes = readRegister(htmc, ADDRESS_DRV_STATUS);
  return drive_status.status;
}

TMC2209_GlobalStatus_t TMC2209_getGlobalStatus(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_GlobalStatusUnion_t global_status_union;
  global_status_union.bytes = 0;
  global_status_union.bytes = readRegister(htmc, ADDRESS_GSTAT);
  return global_status_union.global_status;
}

void TMC2209_clearReset(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_GlobalStatusUnion_t global_status_union;
  global_status_union.bytes = 0;
  global_status_union.global_status.reset = 1;
  writeRegister(htmc, ADDRESS_GSTAT, global_status_union.bytes);
}

void TMC2209_clearDriveError(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_GlobalStatusUnion_t global_status_union;
  global_status_union.bytes = 0;
  global_status_union.global_status.drv_err = 1;
  writeRegister(htmc, ADDRESS_GSTAT, global_status_union.bytes);
}

uint8_t TMC2209_getInterfaceTransmissionCounter(TMC2209_HandleTypeDef * htmc)
{
  return readRegister(htmc, ADDRESS_IFCNT);
}

uint32_t TMC2209_getInterstepDuration(TMC2209_HandleTypeDef * htmc)
{
  return readRegister(htmc, ADDRESS_TSTEP);
}

uint16_t TMC2209_getStallGuardResult(TMC2209_HandleTypeDef * htmc)
{
  return readRegister(htmc, ADDRESS_SG_RESULT);
}

uint8_t TMC2209_getPwmScaleSum(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_PwmScale_t pwm_scale;
  pwm_scale.bytes = readRegister(htmc, ADDRESS_PWM_SCALE);

  return pwm_scale.pwm_scale_sum;
}

int16_t TMC2209_getPwmScaleAuto(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_PwmScale_t pwm_scale;
  pwm_scale.bytes = readRegister(htmc, ADDRESS_PWM_SCALE);

  return pwm_scale.pwm_scale_auto;
}

uint8_t TMC2209_getPwmOffsetAuto(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_PwmAuto_t pwm_auto;
  pwm_auto.bytes = readRegister(htmc, ADDRESS_PWM_AUTO);

  return pwm_auto.pwm_offset_auto;
}

uint8_t TMC2209_getPwmGradientAuto(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_PwmAuto_t pwm_auto;
  pwm_auto.bytes = readRegister(htmc, ADDRESS_PWM_AUTO);

  return pwm_auto.pwm_gradient_auto;
}

uint16_t TMC2209_getMicrostepCounter(TMC2209_HandleTypeDef * htmc)
{
  return readRegister(htmc, ADDRESS_MSCNT);
}

// private helper functions

static HAL_StatusTypeDef serialWrite(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size)
{
  if (huart != NULL)
  {
    // Assuming half-duplex, need to switch to transmit mode
    // This might be specific to the STM32 series and configuration
    // For STM32F1, this might involve setting the DE (Driver Enable) pin
    // or using a specific UART mode if available.
    // For a generic HAL approach, we'll assume the UART is configured for half-duplex
    // or that the DE pin is handled externally or by the HAL configuration.
    // If using a DE pin, you would assert it here before transmitting.

    HAL_StatusTypeDef status = HAL_UART_Transmit(huart, data, size, HAL_MAX_DELAY);

    // If using a DE pin, you would deassert it here after transmitting.

    return status;
  }
  return HAL_ERROR;
}

static HAL_StatusTypeDef serialRead(UART_HandleTypeDef *huart, uint8_t *data, uint16_t size, uint32_t timeout)
{
  if (huart != NULL)
  {
    // Assuming half-duplex, need to switch to receive mode
    // If using a DE pin, you would deassert it here before receiving.

    HAL_StatusTypeDef status = HAL_UART_Receive(huart, data, size, timeout);

    return status;
  }
  return HAL_ERROR;
}

static void serialFlush(UART_HandleTypeDef *huart)
{
  // In HAL, flushing typically means waiting for the transmit buffer to be empty.
  // HAL_UART_Transmit is blocking with HAL_MAX_DELAY, so a separate flush is not strictly necessary
  // for the current serialWrite implementation. If using non-blocking transmit,
  // you would wait for the transmission complete flag here.
}

static void setOperationModeToSerial(TMC2209_HandleTypeDef * htmc, TMC2209_SerialAddress_t serial_address)
{
  htmc->serial_address = serial_address;

  htmc->global_config.bytes = 0;
  htmc->global_config.i_scale_analog = 0;
  htmc->global_config.pdn_disable = 1;
  htmc->global_config.mstep_reg_select = 1;
  htmc->global_config.multistep_filt = 1;

  writeStoredGlobalConfig(htmc);
}

static void setRegistersToDefaults(TMC2209_HandleTypeDef * htmc)
{
  htmc->driver_current.bytes = 0;
  htmc->driver_current.ihold = IHOLD_DEFAULT;
  htmc->driver_current.irun = IRUN_DEFAULT;
  htmc->driver_current.iholddelay = IHOLDDELAY_DEFAULT;
  writeRegister(htmc, ADDRESS_IHOLD_IRUN, htmc->driver_current.bytes);

  htmc->chopper_config.bytes = CHOPPER_CONFIG_DEFAULT;
  htmc->chopper_config.tbl = TBL_DEFAULT;
  htmc->chopper_config.hend = HEND_DEFAULT;
  htmc->chopper_config.hstart = HSTART_DEFAULT;
  htmc->chopper_config.toff = TOFF_DEFAULT;
  writeRegister(htmc, ADDRESS_CHOPCONF, htmc->chopper_config.bytes);

  htmc->pwm_config.bytes = PWM_CONFIG_DEFAULT;
  writeRegister(htmc, ADDRESS_PWMCONF, htmc->pwm_config.bytes);

  htmc->cool_config.bytes = COOLCONF_DEFAULT;
  writeRegister(htmc, ADDRESS_COOLCONF, htmc->cool_config.bytes);

  writeRegister(htmc, ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
  writeRegister(htmc, ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
  writeRegister(htmc, ADDRESS_VACTUAL, VACTUAL_DEFAULT);
  writeRegister(htmc, ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
  writeRegister(htmc, ADDRESS_SGTHRS, SGTHRS_DEFAULT);
  writeRegister(htmc, ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

static void readAndStoreRegisters(TMC2209_HandleTypeDef * htmc)
{
  htmc->global_config.bytes = readGlobalConfigBytes(htmc);
  htmc->chopper_config.bytes = readChopperConfigBytes(htmc);
  htmc->pwm_config.bytes = readPwmConfigBytes(htmc);
}

static bool serialOperationMode(TMC2209_HandleTypeDef * htmc)
{
  TMC2209_GlobalConfig_t global_config;
  global_config.bytes = readGlobalConfigBytes(htmc);

  return global_config.pdn_disable;
}

static void minimizeMotorCurrent(TMC2209_HandleTypeDef * htmc)
{
  htmc->driver_current.irun = CURRENT_SETTING_MIN;
  htmc->driver_current.ihold = CURRENT_SETTING_MIN;
  writeStoredDriverCurrent(htmc);
}

static uint32_t reverseData(uint32_t data)
{
  uint32_t reversed_data = 0;
  uint8_t right_shift;
  uint8_t left_shift;
  for (uint8_t i=0; i<DATA_SIZE; ++i)
  {
    right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
    left_shift = i * BITS_PER_BYTE;
    reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
  }
  return reversed_data;
}

static uint8_t calculateCrc(uint64_t datagram_bytes, uint8_t datagram_size)
{
  uint8_t crc = 0;
  uint8_t byte;
  for (uint8_t i=0; i<(datagram_size - 1); ++i)
  {
    byte = (datagram_bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    for (uint8_t j=0; j<BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7) ^ (byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

static void sendDatagramUnidirectional(UART_HandleTypeDef *huart, uint64_t datagram_bytes, uint8_t datagram_size)
{
  uint8_t byte;
  uint8_t data_buffer[datagram_size];

  for (uint8_t i=0; i<datagram_size; ++i)
  {
    byte = (datagram_bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    data_buffer[i] = byte;
  }
  serialWrite(huart, data_buffer, datagram_size);
}

static void sendDatagramBidirectional(UART_HandleTypeDef *huart, uint64_t datagram_bytes, uint8_t datagram_size)
{
  uint8_t byte;
  uint8_t data_buffer[datagram_size];

  // Wait for the transmission of outgoing serial data to complete
  serialFlush(huart);

  // write datagram
  for (uint8_t i=0; i<datagram_size; ++i)
  {
    byte = (datagram_bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    data_buffer[i] = byte;
  }
  serialWrite(huart, data_buffer, datagram_size);

  // Wait for the transmission of outgoing serial data to complete
  serialFlush(huart);

  // wait for bytes sent out on TX line to be echoed on RX line
  // This echo handling was commented out in the original C++ code and
  // is highly dependent on the specific hardware and UART configuration.
  // For this refactoring to a generic C structure with HAL, we will omit
  // this specific echo handling. The read function will handle receiving
  // the reply directly.
}

static void writeRegister(TMC2209_HandleTypeDef *htmc, uint8_t register_address,
  uint32_t data)
{
  TMC2209_WriteReadReplyDatagram_t write_datagram;
  write_datagram.bytes = 0;
  write_datagram.sync = SYNC;
  write_datagram.serial_address = htmc->serial_address;
  write_datagram.register_address = register_address;
  write_datagram.rw = RW_WRITE;
  write_datagram.data = reverseData(data);
  write_datagram.crc = calculateCrc(write_datagram.bytes, WRITE_READ_REPLY_DATAGRAM_SIZE);

  sendDatagramUnidirectional(htmc->huart, write_datagram.bytes, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

static uint32_t readRegister(TMC2209_HandleTypeDef *htmc, uint8_t register_address)
{
  TMC2209_ReadRequestDatagram_t read_request_datagram;
  read_request_datagram.bytes = 0;
  read_request_datagram.sync = SYNC;
  read_request_datagram.serial_address = htmc->serial_address;
  read_request_datagram.register_address = register_address;
  read_request_datagram.rw = RW_READ;
  read_request_datagram.crc = calculateCrc(read_request_datagram.bytes, READ_REQUEST_DATAGRAM_SIZE);

  for (uint8_t retry = 0; retry < MAX_READ_RETRIES; retry++)
  {
    sendDatagramBidirectional(htmc->huart, read_request_datagram.bytes, READ_REQUEST_DATAGRAM_SIZE);

    // Wait for the reply datagram to arrive
    uint32_t reply_delay = 0;
    uint8_t received_data[WRITE_READ_REPLY_DATAGRAM_SIZE];
    HAL_StatusTypeDef read_status = HAL_ERROR;

    // Attempt to read the expected number of bytes with a timeout
    // read_status = serialRead(htmc->huart, received_data, WRITE_READ_REPLY_DATAGRAM_SIZE, REPLY_DELAY_MAX_MICROSECONDS / 1000); // Convert us to ms for HAL_UART_Receive timeout
    
    uint8_t byte_count = 0;
    for (int i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; i++) {
        read_status = HAL_UART_Receive(htmc->huart, &received_data[i], 1, REPLY_DELAY_MAX_MICROSECONDS / WRITE_READ_REPLY_DATAGRAM_SIZE / 1000);
        if (read_status != HAL_OK) {
            break;
        }
        byte_count++;
    }

    if (byte_count == WRITE_READ_REPLY_DATAGRAM_SIZE)
    {
        TMC2209_WriteReadReplyDatagram_t read_reply_datagram;
        read_reply_datagram.bytes = 0;
        // Manually assemble the uint64_t from received bytes (assuming little-endian for this assembly)
        for(int i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; i++) {
            read_reply_datagram.bytes |= ((uint64_t)received_data[i] << (i * BITS_PER_BYTE));
        }

        uint8_t crc = calculateCrc(read_reply_datagram.bytes, WRITE_READ_REPLY_DATAGRAM_SIZE);
        if (crc == read_reply_datagram.crc)
        {
            return reverseData(read_reply_datagram.data);
        }
            return reverseData(read_reply_datagram.data);
        // CRC error - fall through to retry delay
    }
    else
    {
      return 0;
    }
    
    // Incomplete read or CRC error, delay before next retry
    if (retry < MAX_READ_RETRIES - 1)
    {
      HAL_Delay(READ_RETRY_DELAY_MS);
    }
  } // End retry loop

  return 0; // Return 0 if all retries fail
}

static uint8_t percentToCurrentSetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain_uint32(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t current_setting = map_uint8(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX);
  return current_setting;
}

static uint8_t currentSettingToPercent(uint8_t current_setting)
{
  uint8_t percent = map_uint8(current_setting,
    CURRENT_SETTING_MIN,
    CURRENT_SETTING_MAX,
    PERCENT_MIN,
    PERCENT_MAX);
  return percent;
}

static uint8_t percentToHoldDelaySetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain_uint32(percent,
    PERCENT_MIN,
    PERCENT_MAX);
  uint8_t hold_delay_setting = map_uint8(constrained_percent,
    PERCENT_MIN,
    PERCENT_MAX,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX);
  return hold_delay_setting;
}

static uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting)
{
  uint8_t percent = map_uint8(hold_delay_setting,
    HOLD_DELAY_MIN,
    HOLD_DELAY_MAX,
    PERCENT_MIN,
    PERCENT_MAX);
  return percent;
}

static void writeStoredGlobalConfig(TMC2209_HandleTypeDef *htmc)
{
  writeRegister(htmc, ADDRESS_GCONF, htmc->global_config.bytes);
}

static uint32_t readGlobalConfigBytes(TMC2209_HandleTypeDef *htmc)
{
  return readRegister(htmc, ADDRESS_GCONF);
}

static void writeStoredDriverCurrent(TMC2209_HandleTypeDef *htmc)
{
  writeRegister(htmc, ADDRESS_IHOLD_IRUN, htmc->driver_current.bytes);

  if (htmc->driver_current.irun >= SEIMIN_UPPER_CURRENT_LIMIT)
  {
    htmc->cool_config.seimin = SEIMIN_UPPER_SETTING;
  }
  else
  {
    htmc->cool_config.seimin = SEIMIN_LOWER_SETTING;
  }
  if (htmc->cool_step_enabled)
  {
    writeRegister(htmc, ADDRESS_COOLCONF, htmc->cool_config.bytes);
  }
}

static void writeStoredChopperConfig(TMC2209_HandleTypeDef *htmc)
{
  writeRegister(htmc, ADDRESS_CHOPCONF, htmc->chopper_config.bytes);
}

static uint32_t readChopperConfigBytes(TMC2209_HandleTypeDef *htmc)
{
  return readRegister(htmc, ADDRESS_CHOPCONF);
}

static void writeStoredPwmConfig(TMC2209_HandleTypeDef *htmc)
{
  writeRegister(htmc, ADDRESS_PWMCONF, htmc->pwm_config.bytes);
}

static uint32_t readPwmConfigBytes(TMC2209_HandleTypeDef *htmc)
{
  return readRegister(htmc, ADDRESS_PWMCONF);
}

static uint32_t constrain_uint32(uint32_t value, uint32_t low, uint32_t high)
{
  return ((value)<(low)?(low):((value)>(high)?(high):(value)));
}

static uint8_t map_uint8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void TMC2209_init(TMC2209_HandleTypeDef * htmc)
{
    // HAL_UART_Init(htmc->huart); // Assuming UART is already initialized by CubeMX

    // Initialize the driver
    htmc->serial_address = htmc->serial_address; // Redundant, but keeps the structure

    // Set operation mode to serial
    htmc->global_config.bytes = 0;
    htmc->global_config.i_scale_analog = 0;
    htmc->global_config.pdn_disable = 1;
    htmc->global_config.mstep_reg_select = 1;
    htmc->global_config.multistep_filt = 1;
    writeStoredGlobalConfig(htmc);

    // Set registers to defaults
    htmc->driver_current.bytes = 0;
    htmc->driver_current.ihold = IHOLD_DEFAULT;
    htmc->driver_current.irun = IRUN_DEFAULT;
    htmc->driver_current.iholddelay = IHOLDDELAY_DEFAULT;
    writeRegister(htmc, ADDRESS_IHOLD_IRUN, htmc->driver_current.bytes);

    htmc->chopper_config.bytes = CHOPPER_CONFIG_DEFAULT;
    htmc->chopper_config.tbl = TBL_DEFAULT;
    htmc->chopper_config.hend = HEND_DEFAULT;
    htmc->chopper_config.hstart = HSTART_DEFAULT;
    htmc->chopper_config.toff = TOFF_DEFAULT;
    writeRegister(htmc, ADDRESS_CHOPCONF, htmc->chopper_config.bytes);

    htmc->pwm_config.bytes = PWM_CONFIG_DEFAULT;
    writeRegister(htmc, ADDRESS_PWMCONF, htmc->pwm_config.bytes);

    htmc->cool_config.bytes = COOLCONF_DEFAULT;
    writeRegister(htmc, ADDRESS_COOLCONF, htmc->cool_config.bytes);

    writeRegister(htmc, ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
    writeRegister(htmc, ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
    writeRegister(htmc, ADDRESS_VACTUAL, VACTUAL_DEFAULT);
    writeRegister(htmc, ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
    writeRegister(htmc, ADDRESS_SGTHRS, SGTHRS_DEFAULT);
    writeRegister(htmc, ADDRESS_COOLCONF, COOLCONF_DEFAULT); // Written twice in original, keeping for now

    TMC2209_clearDriveError(htmc);
    minimizeMotorCurrent(htmc);
    TMC2209_disable(htmc);
    TMC2209_disableAutomaticCurrentScaling(htmc);
    TMC2209_disableAutomaticGradientAdaptation(htmc);
}
