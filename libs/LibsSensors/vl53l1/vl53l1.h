#ifndef _VL53L1_H_
#define _VL53L1_H_

#include <stdint.h>
#include <i2c_interface.h>
#include "vl53l1_const.h"

//0b0101 0010
#define VL53L1_I2C_ADR ((uint8_t)0x52)



class VL53L1
{
    public:
        VL53L1();
        virtual ~VL53L1();

        int init(I2C_Interface *i2c_);


        VL53L1_RangingData read();
        bool set_distance_mode(DistanceMode mode);


    private:

        VL53L1_RangingData _get_ranging_data(VL53L1_ResultBuffer raw_result);
        void _update_dss(VL53L1_ResultBuffer raw_results);

        
        void delay_us(uint32_t time);
        void _setup_manual_calibration();
        void _set_measurement_timing_budget(uint32_t budget_us);
        uint32_t _decodeTimeout(uint16_t reg_val);
        uint16_t _encodeTimeout(uint32_t timeout_mclks);
        uint32_t _get_measurement_timing_budget();
        uint32_t _timeout_Mclks_to_us(uint32_t timeout_mclks, uint32_t macro_period_us);
        uint32_t _timeout_us_to_Mclks(uint32_t timeout_us, uint32_t macro_period_us);

        uint32_t _calc_macro_period(uint8_t vcsel_period);

        float       _rate_to_float(uint16_t count_rate_fixed);

        void        _write_reg(uint16_t reg_adr, uint8_t value);
        uint8_t     _read_reg(uint16_t reg_adr);
        void        _write_reg_16bit(uint16_t reg_adr, uint16_t value);
        uint16_t    _read_reg_16bit(uint16_t reg_adr);
        void        _write_reg_32bit(uint16_t reg_adr, uint32_t value);



    private:
        I2C_Interface *i2c; 

        uint16_t fast_osc_frequency, osc_calibrate_val;
        
        static const uint16_t TargetRate = 0x0A00;
        static const uint32_t TimingGuard = 4528;

        VL53L1_RangingData result;
 };

#endif