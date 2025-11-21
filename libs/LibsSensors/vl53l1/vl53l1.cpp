#include "vl53l1.h"
#include "vl53l1_const.h"

VL53L1::VL53L1()
{ 

}

VL53L1::~VL53L1()
{

}


int VL53L1::init(I2C_Interface *i2c_)
{
    this->i2c = i2c_;

    if (_read_reg_16bit(IDENTIFICATION__MODEL_ID) != 0xEACC) 
    { 
        //sensor not responding
        return -1;
    }

    //software reset
    _write_reg(SOFT_RESET, 0x00);
    delay_us(100);
    _write_reg(SOFT_RESET, 0x01);

    delay_us(1000);

    uint16_t loops = 0xffff;
    while ((_read_reg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0)
    {
        loops--;  
        if (loops == 0)  
        {
            return -2;
        }
    }

    // switch to 2.8V mode
    _write_reg(PAD_I2C_HV__EXTSUP_CONFIG, _read_reg(PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);


    fast_osc_frequency  = _read_reg_16bit(OSC_MEASURED__FAST_OSC__FREQUENCY);
    osc_calibrate_val   = _read_reg_16bit(RESULT__OSC_CALIBRATE_VAL);


    // static config
    // API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do
    // that? (seems like it would disable 2V8 mode)
    _write_reg_16bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset
    _write_reg(GPIO__TIO_HV_STATUS, 0x02);
    _write_reg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
    _write_reg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
    _write_reg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
    _write_reg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
    _write_reg(ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
    _write_reg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default

    // general config
    _write_reg_16bit(SYSTEM__THRESH_RATE_HIGH, 0x0000);
    _write_reg_16bit(SYSTEM__THRESH_RATE_LOW, 0x0000);
    _write_reg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

    // timing config
    // most of these settings will be determined later by distance and timing
    // budget configuration
    _write_reg_16bit(RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
    _write_reg_16bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

    // dynamic config

    _write_reg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
    _write_reg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
    _write_reg(SD_CONFIG__QUANTIFIER, 2); // tuning parm default

    // VL53L1_preset_mode_standard_ranging() end

    // from VL53L1_preset_mode_timed_ranging_*
    // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
    // and things don't seem to work if we don't set GPH back to 0 (which the API
    // does here).
    _write_reg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
    _write_reg(SYSTEM__SEED_CONFIG, 1); // tuning parm default

    // from VL53L1_config_low_power_auto_mode
    _write_reg(SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
    _write_reg_16bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
    _write_reg(DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

    // VL53L1_set_preset_mode() end

    // default to long range, 33 ms timing budget
    // note that this is different than what the API defaults to
    // the minimum timing budget is 20 ms for short distance mode and 33 ms for
    set_distance_mode(Long);
    _set_measurement_timing_budget(33000);

    // VL53L1_StaticInit() end

    // the API triggers this change in VL53L1_init_and_start_range() once a
    // measurement is started; assumes MM1 and MM2 are disabled
    _write_reg_16bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM,
    _read_reg_16bit(MM_CONFIG__OUTER_OFFSET_MM) * 4);


    // Start continuous ranging measurements, with the given inter-measurement
    // 33ms period
    _write_reg_32bit(SYSTEM__INTERMEASUREMENT_PERIOD, 33 * osc_calibrate_val);
    _write_reg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
    _write_reg(SYSTEM__MODE_START, 0x40); // mode_range__timed

    this->result.range_mm = 0;

    read();

     

    return 0;
}



VL53L1_RangingData VL53L1::read()
{
    VL53L1_ResultBuffer raw_result;

    i2c->start();
    i2c->write(VL53L1_I2C_ADR);
    i2c->write((uint8_t)(RESULT__RANGE_STATUS >> 8)); // reg high byte
    i2c->write((uint8_t)(RESULT__RANGE_STATUS));      // reg low byte
    

    i2c->start();
    i2c->write(VL53L1_I2C_ADR|0x01);


    raw_result.range_status = i2c->read(1);

    // report_status: not used
    i2c->read(1); 

    raw_result.stream_count = i2c->read(1);

    raw_result.dss_actual_effective_spads_sd0  = (uint16_t)i2c->read(1) << 8; // high byte
    raw_result.dss_actual_effective_spads_sd0 |=           i2c->read(1);      // low byte

    i2c->read(1); // peak_signal_count_rate_mcps_sd0: not used
    i2c->read(1);

    raw_result.ambient_count_rate_mcps_sd0  = (uint16_t)i2c->read(1) << 8; // high byte
    raw_result.ambient_count_rate_mcps_sd0 |=           i2c->read(1);      // low byte

    i2c->read(1); // sigma_sd0: not used
    i2c->read(1);

    i2c->read(1); // phase_sd0: not used
    i2c->read(1);

    raw_result.final_crosstalk_corrected_range_mm_sd0  = (uint16_t)i2c->read(1) << 8; // high byte
    raw_result.final_crosstalk_corrected_range_mm_sd0 |=           i2c->read(1);      // low byte

    raw_result.peak_signal_count_rate_crosstalk_corrected_mcps_sd0  = (uint16_t)i2c->read(1) << 8; // high byte
    raw_result.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |=           i2c->read(0);      // low byte

    i2c->stop();

   

    _update_dss(raw_result);

    _write_reg(SYSTEM__INTERRUPT_CLEAR, 0x01); 

    VL53L1_RangingData result = _get_ranging_data(raw_result);

    if (this->result.range_status == 0)
    {
        this->result.range_mm       = result.range_mm;
    }

    this->result.range_status   = result.range_status;
    this->result.peak_signal_count_rate_MCPS = result.peak_signal_count_rate_MCPS;
    this->result.ambient_count_rate_MCPS = result.ambient_count_rate_MCPS;

    return this->result;
}



// get range, status, rates from results buffer
// based on VL53L1_GetRangingMeasurementData()
VL53L1_RangingData VL53L1::_get_ranging_data(VL53L1_ResultBuffer raw_result)
{
    VL53L1_RangingData ranging_data;
    // VL53L1_copy_sys_and_core_results_to_range_results() begin

    uint16_t range = raw_result.final_crosstalk_corrected_range_mm_sd0;

    // "apply correction gain"
    // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
    // Basically, this appears to scale the result by 2011/2048, or about 98%
    // (with the 1024 added for proper rounding).
    ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

    // VL53L1_copy_sys_and_core_results_to_range_results() end

    // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
    // mostly based on ConvertStatusLite()
    switch(raw_result.range_status)
    {
        case 17: // MULTCLIPFAIL
        case 2: // VCSELWATCHDOGTESTFAILURE
        case 1: // VCSELCONTINUITYTESTFAILURE
        case 3: // NOVHVVALUEFOUND
        // from SetSimpleData()
        ranging_data.range_status = HardwareFail;
        break;

        case 13: // USERROICLIP
        // from SetSimpleData()
        ranging_data.range_status = MinRangeFail;
        break;

        case 18: // GPHSTREAMCOUNT0READY
        ranging_data.range_status = SynchronizationInt;
        break;

        case 5: // RANGEPHASECHECK
        ranging_data.range_status =  OutOfBoundsFail;
        break;

        case 4: // MSRCNOTARGET
        ranging_data.range_status = SignalFail;
        break;

        case 6: // SIGMATHRESHOLDCHECK
        ranging_data.range_status = SigmaFail;
        break;

        case 7: // PHASECONSISTENCY
        ranging_data.range_status = WrapTargetFail;
        break;

        case 12: // RANGEIGNORETHRESHOLD
        ranging_data.range_status = XtalkSignalFail;
        break;

        case 8: // MINCLIP
        ranging_data.range_status = RangeValidMinRangeClipped;
        break;

        case 9: // RANGECOMPLETE
        // from VL53L1_copy_sys_and_core_results_to_range_results()
        if (raw_result.stream_count == 0)
        {
            ranging_data.range_status = RangeValidNoWrapCheckFail;
        }
        else
        {
            ranging_data.range_status = RangeValid;
        }
        break;

        default:
        ranging_data.range_status = None;
    }

  // from SetSimpleData()
  ranging_data.peak_signal_count_rate_MCPS  = _rate_to_float(raw_result.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  ranging_data.ambient_count_rate_MCPS      = _rate_to_float(raw_result.ambient_count_rate_mcps_sd0);

  return ranging_data;
}


// perform Dynamic SPAD Selection calculation/update
// based on VL53L1_low_power_auto_update_DSS()
void VL53L1::_update_dss(VL53L1_ResultBuffer raw_results)
{
  uint16_t spadCount = raw_results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    // "Calc total rate per spad"

    uint32_t totalRatePerSpad =
      (uint32_t)raw_results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      raw_results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // "shift up to take advantage of 32 bits"
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      // "get the target rate and shift up by 16"
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

      // "clip to 16 bit"
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // "override DSS config"
      _write_reg_16bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

      return;
    } 
  }

  // If we reached this point, it means something above would have resulted in a
  // divide by zero.
  // "We want to gracefully set a spad target, not just exit with an error"

   // "set target to mid point"
   _write_reg_16bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}


// set distance mode to Short, Medium, or Long
bool VL53L1::set_distance_mode(DistanceMode mode)
{
  // save existing timing budget
  uint32_t budget_us = _get_measurement_timing_budget();

  switch (mode)
  {
    case Short:
      // from VL53L1_preset_mode_standard_ranging_short_range()

      // timing config
      _write_reg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      _write_reg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      _write_reg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

      // dynamic config
      _write_reg(SD_CONFIG__WOI_SD0, 0x07);
      _write_reg(SD_CONFIG__WOI_SD1, 0x05);
      _write_reg(SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
      _write_reg(SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

      break;

    case Medium:
      // from VL53L1_preset_mode_standard_ranging()

      // timing config
      _write_reg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
      _write_reg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
      _write_reg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

      // dynamic config
      _write_reg(SD_CONFIG__WOI_SD0, 0x0B);
      _write_reg(SD_CONFIG__WOI_SD1, 0x09);
      _write_reg(SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
      _write_reg(SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

      break;

    case Long: // long
      // from VL53L1_preset_mode_standard_ranging_long_range()

      // timing config
      _write_reg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      _write_reg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      _write_reg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

      // dynamic config
      _write_reg(SD_CONFIG__WOI_SD0, 0x0F);
      _write_reg(SD_CONFIG__WOI_SD1, 0x0D);
      _write_reg(SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
      _write_reg(SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

      break;

    default:
      // unrecognized mode - do nothing
      return false;
  }

  // reapply timing budget
  _set_measurement_timing_budget(budget_us);

 
  return true;
}


// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
void VL53L1::_setup_manual_calibration()
{
  // "save original vhv configs"
  uint8_t saved_vhv_init    = _read_reg(VHV_CONFIG__INIT);
  uint8_t saved_vhv_timeout = _read_reg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

  // "disable VHV init"
  _write_reg(VHV_CONFIG__INIT, saved_vhv_init & 0x7F);

  // "set loop bound to tuning param"
  _write_reg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
    (saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

  // "override phasecal"
  _write_reg(PHASECAL_CONFIG__OVERRIDE, 0x01);
  _write_reg(CAL_CONFIG__VCSEL_START, _read_reg(PHASECAL_RESULT__VCSEL_START));
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate
// measurements.
// based on VL53L1__Set_measurement_timing_budgetMicroSeconds()
void VL53L1::_set_measurement_timing_budget(uint32_t budget_us)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS


  uint32_t range_config_timeout_us = budget_us -= TimingGuard;

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values() begin

  uint32_t macro_period_us;

  // "Update Macro Period for Range A VCSEL Period"
  macro_period_us = _calc_macro_period(_read_reg(RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Update Phase timeout - uses Timing A"
  // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg().
  uint32_t phasecal_timeout_mclks = _timeout_us_to_Mclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
  _write_reg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout"
  // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
  // actually ends up with a slightly different value because it gets assigned,
  // retrieved, recalculated with a different macro period, and reassigned,
  // but it probably doesn't matter because it seems like the MM ("mode
  // mitigation"?) sequence steps are disabled in low power auto mode anyway.
  _write_reg_16bit(MM_CONFIG__TIMEOUT_MACROP_A, _encodeTimeout(
    _timeout_us_to_Mclks(1, macro_period_us)));

  // "Update Range Timing A timeout"
  _write_reg_16bit(RANGE_CONFIG__TIMEOUT_MACROP_A, _encodeTimeout(
    _timeout_us_to_Mclks(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  macro_period_us = _calc_macro_period(_read_reg(RANGE_CONFIG__VCSEL_PERIOD_B));

  // "Update MM Timing B timeout"
  // (See earlier comment about MM Timing A timeout.)
  _write_reg_16bit(MM_CONFIG__TIMEOUT_MACROP_B, _encodeTimeout(
    _timeout_us_to_Mclks(1, macro_period_us)));

  // "Update Range Timing B timeout"
  _write_reg_16bit(RANGE_CONFIG__TIMEOUT_MACROP_B, _encodeTimeout(
    _timeout_us_to_Mclks(range_config_timeout_us, macro_period_us)));

  // VL53L1_calc_timeout_register_values() end
}


// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t VL53L1::_decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t VL53L1::_encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else 
  { 
    return 0; 
    }
}


// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
uint32_t VL53L1::_get_measurement_timing_budget()
{ 
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = _calc_macro_period(_read_reg(RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Get Range Timing A timeout"

  uint32_t range_config_timeout_us = _timeout_Mclks_to_us(_decodeTimeout(
    _read_reg_16bit(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

  // VL53L1_get_timeouts_us() end

  return  2 * range_config_timeout_us + TimingGuard;
} 

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t VL53L1::_timeout_Mclks_to_us(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_mclks()
uint32_t VL53L1::_timeout_us_to_Mclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t VL53L1::_calc_macro_period(uint8_t vcsel_period)
{
  // from VL53L1_calc_pll_period_us()
  // fast osc frequency in 4.12 format; PLL period in 0.24 format
  uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

  // from VL53L1_decode_vcsel_period()
  uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

  // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
  uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
  macro_period_us >>= 6;
  macro_period_us *= vcsel_period_pclks;
  macro_period_us >>= 6;

  return macro_period_us;
}

float VL53L1::_rate_to_float(uint16_t count_rate_fixed) 
{
    return (float)count_rate_fixed / (1 << 7); 
}


void VL53L1::_write_reg(uint16_t reg_adr, uint8_t value)
{
    i2c->start();
    i2c->write(VL53L1_I2C_ADR);
    i2c->write((uint8_t)(reg_adr >> 8)); // reg high byte
    i2c->write((uint8_t)(reg_adr));      // reg low byte
    i2c->write(value);
    i2c->stop();
}

uint8_t VL53L1::_read_reg(uint16_t reg_adr)
{
    uint8_t value;

    i2c->start();
    i2c->write(VL53L1_I2C_ADR);  // slave address, write command
    i2c->write((uint8_t)(reg_adr >> 8)); // reg high byte
    i2c->write((uint8_t)(reg_adr));      // reg low byte

    i2c->start();
    i2c->write(VL53L1_I2C_ADR|0x01); // slave address, read command
    value = i2c->read(0);   // read data
    i2c->stop();

    return value;
}

void VL53L1::_write_reg_16bit(uint16_t reg_adr, uint16_t value)
{
    i2c->start();
    i2c->write(VL53L1_I2C_ADR);
    i2c->write((uint8_t)(reg_adr >> 8)); // reg high byte
    i2c->write((uint8_t)(reg_adr));      // reg low byte
    i2c->write((uint8_t)(value >> 8)); // value high byte
    i2c->write((uint8_t)(value));      // value low byte
    i2c->stop();
}


uint16_t VL53L1::_read_reg_16bit(uint16_t reg_adr)
{
    uint16_t value;

    i2c->start();
    i2c->write(VL53L1_I2C_ADR);  // slave address, write command
    i2c->write((uint8_t)(reg_adr >> 8)); // reg high byte
    i2c->write((uint8_t)(reg_adr));      // reg low byte

    i2c->start();
    i2c->write(VL53L1_I2C_ADR|0x01); // slave address, read command

    value  = (uint16_t)i2c->read(1) << 8; // value high byte
    value |=           i2c->read(0);      // value low byte

    i2c->stop();

    return value;
}

void VL53L1::_write_reg_32bit(uint16_t reg_adr, uint32_t value)
{
    i2c->start();
    i2c->write(VL53L1_I2C_ADR);
    i2c->write((uint8_t)(reg_adr >> 8)); // reg high byte
    i2c->write((uint8_t)(reg_adr));      // reg low byte
    i2c->write((uint8_t)(value >> 24)); // value highest byte
    i2c->write((uint8_t)(value >> 16)); // value highest byte
    i2c->write((uint8_t)(value >> 8)); // value highest byte
    i2c->write((uint8_t)(value));      // value lowest byte
    i2c->stop();
}

void VL53L1::delay_us(uint32_t time)
{
    uint32_t loops = time*48/4;
    while (loops--)
    {
        __asm("nop");
    }
}