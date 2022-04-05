/** Based on Analog Devices AD7142 datasheet rev. B https://www.analog.com/media/en/technical-documentation/data-sheets/AD7142.pdf
* Started 28/01/2022 by Remko Proesmans
* Code structure analogous to https://github.com/disk91/LIS2DH
* Maintained at https://github.com/RemkoPr/AD7142_Arduino
* Changelog:
*     ... - ongoing development release
* NOTE: THIS IS ONLY A PARIAL RELEASE. 
*/

#ifndef _AD7142_H_
#define _AD7142_H_


#define AD7142_I2C_BASE_ADDRESS           0x02C // 0101 100

// set the profile (AD7142_PROFILE) to determine which settings are used from settings.h
#define AD7142_PROFILE_5X5                0
#define AD7142_PROFILE_SINGLE             1
#define AD7142_PROFILE_DEBUG_PROCH        2
#define AD7142_PROFILE_DEBUG_BEMIS        3
#define AD7142_PROFILE                    AD7142_PROFILE_5X5 

// ***********
//  Registers
// ***********

// Bank 1
#define AD7142_PWR_CONTROL                0x000
#define AD7142_STAGE_CAL_EN               0x001
#define AD7142_AMB_COMP_CTRL0             0x002
#define AD7142_AMB_COMP_CTRL1             0x003
#define AD7142_AMB_COMP_CTRL2             0x004
#define AD7142_STAGE_LOW_INT_EN           0x005
#define AD7142_STAGE_HIGH_INT_EN          0x006
#define AD7142_STAGE_COMPLETE_INT_EN      0x007
#define AD7142_STAGE_LOW_LIMIT_INT        0x008
#define AD7142_STAGE_HIGH_LIMIT_INT       0x009
#define AD7142_STAGE_COMPLETE_LIMIT_INT   0x00A
#define AD7142_ADC_RESULT_S0              0x00B
#define AD7142_ADC_RESULT_S1              0x00C
#define AD7142_ADC_RESULT_S2              0x00D
#define AD7142_ADC_RESULT_S3              0x00E
#define AD7142_ADC_RESULT_S4              0x00F
#define AD7142_ADC_RESULT_S5              0x010
#define AD7142_ADC_RESULT_S6              0x011
#define AD7142_ADC_RESULT_S7              0x012
#define AD7142_ADC_RESULT_S8              0x013
#define AD7142_ADC_RESULT_S9              0x014
#define AD7142_ADC_RESULT_S10             0x015
#define AD7142_ADC_RESULT_S11             0x016
#define AD7142_REVISION_CODE_DEVID_ADDR   0x017 // default value: 0xE622 = 1110 0110 0010 0010
#define AD7142_STAGE_PROXIMITY_STATUS     0x042

// Bank 2: CIN connection setup per STAGE
/* 
 *  To access the STAGEX connection setup registers,
 *   add X * AD7142_STAGEX_BANK2_ADDR_OFFSET to the respective
 *   STAGE0 register address
*/
#define AD7142_STAGE0_CONNECTION_CIN6_0   0x080
#define AD7142_STAGE0_CONNECTION_CIN13_7  0x081
#define AD7142_STAGE0_AFE_OFFSET          0x082
#define AD7142_STAGE0_SENSITIVITY         0x083
#define AD7142_STAGE0_OFFSET_LOW          0x084
#define AD7142_STAGE0_OFFSET_HIGH         0x085
#define AD7142_STAGE0_OFFSET_HIGH_CLAMP   0x086
#define AD7142_STAGE0_OFFSET_LOW_CLAMP    0x087
#define AD7142_STAGEX_BANK2_ADDR_OFFSET   0x008 

// Bank 3
/* 
 *  To access the STAGEX results registers,
 *   add X * AD7142_STAGEX_BANK3_ADDR_OFFSET to the respective
 *   STAGE0 register address
*/
#define AD7142_STAGE0_CONV_DATA           0x0E0
#define AD7142_STAGE0_FF_WORD0            0x0E1
#define AD7142_STAGE0_FF_WORD1            0x0E2
#define AD7142_STAGE0_FF_WORD2            0x0E3
#define AD7142_STAGE0_FF_WORD3            0x0E4
#define AD7142_STAGE0_FF_WORD4            0x0E5
#define AD7142_STAGE0_FF_WORD5            0x0E6
#define AD7142_STAGE0_FF_WORD6            0x0E7
#define AD7142_STAGE0_FF_WORD7            0x0E8
#define AD7142_STAGE0_SF_WORD0            0x0E9
#define AD7142_STAGE0_SF_WORD1            0x0EA
#define AD7142_STAGE0_SF_WORD2            0x0EB
#define AD7142_STAGE0_SF_WORD3            0x0EC
#define AD7142_STAGE0_SF_WORD4            0x0ED
#define AD7142_STAGE0_SF_WORD5            0x0EE
#define AD7142_STAGE0_SF_WORD6            0x0EF
#define AD7142_STAGE0_SF_WORD7            0x0F0
#define AD7142_STAGE0_SF_AMBIENT          0x0F1
#define AD7142_STAGE0_FF_AVG              0x0F2
#define AD7142_STAGE0_PEAK_DETECT_WORD0   0x0F3
#define AD7142_STAGE0_PEAK_DETECT_WORD1   0x0F4
#define AD7142_STAGE0_MAX_WORD0           0x0F5
#define AD7142_STAGE0_MAX_WORD1           0x0F6
#define AD7142_STAGE0_MAX_WORD2           0x0F7
#define AD7142_STAGE0_MAX_WORD3           0x0F8
#define AD7142_STAGE0_MAX_AVG             0x0F9
#define AD7142_STAGE0_HIGH_THRESHOLD      0x0FA
#define AD7142_STAGE0_MAX_TEMP            0x0FB
#define AD7142_STAGE0_MIN_WORD0           0x0FC
#define AD7142_STAGE0_MIN_WORD1           0x0FD
#define AD7142_STAGE0_MIN_WORD2           0x0FE
#define AD7142_STAGE0_MIN_WORD3           0x0FF
#define AD7142_STAGE0_MIN_AVG             0x100
#define AD7142_STAGE0_LOW_THRESHOLD       0x101
#define AD7142_STAGE0_MIN_TEMP            0x102
                                       // 0x103 unused
#define AD7142_STAGEX_BANK3_ADDR_OFFSET   0x024 

// ***********
//  Settings
//   Bank 1
// ***********

// PWR_CONTROL bit options, datasheet Table 20
#define AD7142_POWER_MODE_FULL            0x00
#define AD7142_POWER_MODE_LOW             0x02
#define AD7142_POWER_MODE_SHTDN           0x01
#define AD7142_POWER_MODE_SHTDN_          0x03

#define AD7142_LP_CONV_DELAY_200          0x00 // 200ms
#define AD7142_LP_CONV_DELAY_400          0x01 // 400ms
#define AD7142_LP_CONV_DELAY_600          0x02 // 600ms
#define AD7142_LP_CONV_DELAY_800          0x03 // 800ms

#define AD7142_DECIMATION_256             0x00
#define AD7142_DECIMATION_128             0x01
#define AD7142_DECIMATION_NONE            0x02
#define AD7142_DECIMATION_NONE_           0x03

#define AD7142_INT_POL_ACTIVE_LOW         0x00
#define AD7142_INT_POL_ACTIVE_HIGH        0x01

#define AD7142_EXCITATION_SOURCE_ENABLE   0x00 // pin 15
#define AD7142_EXCITATION_SOURCE_DISABLE  0x01

#define AD7142_SRC_ENABLE                 0x00 // pin 16
#define AD7142_SRC_DISABLE                0x01

#define AD7142_CDC_BIAS_NORMAL            0x00
#define AD7142_CDC_BIAS_PLUS20            0x01
#define AD7142_CDC_BIAS_PLUS35            0x02
#define AD7142_CDC_BIAS_PLUS50            0x03

// STAGE_CAL_EN bit options, datasheet Table 21
#define AD7142_STAGEX_CAL_EN_DISABLE      0x00
#define AD7142_STAGEX_CAL_EN_ENABLE       0x01

#define AD7142_AVG_FP_SKIP_3              0x00
#define AD7142_AVG_FP_SKIP_7              0x01
#define AD7142_AVG_FP_SKIP_15             0x02
#define AD7142_AVG_FP_SKIP_31             0x03

#define AD7142_AVG_LP_SKIP_NONE           0x00
#define AD7142_AVG_LP_SKIP_1              0x01
#define AD7142_AVG_LP_SKIP_2              0x10
#define AD7142_AVG_LP_SKIP_3              0x11

// AMB_COMP_CTRL0 bit options, datasheet Table 22
#define AD7142_FF_SKIP_CNT_MAX			  12
#define AD7142_FP_PROXIMITY_CNT_MAX		  15
#define AD7142_LP_PROXIMITY_CNT_MAX		  15
#define AD7142_PWR_DOWN_TIMEOUT_MAX		  2
#define AD7142_FORCED_CAL_FALSE		  	  0x00
#define AD7142_FORCED_CAL_TRUE		  	  0x01
#define AD7142_CONV_RESET_FALSE			  0x00
#define AD7142_CONV_RESET_TRUE			  0x01

// AMB_COMP_CTRL1 bit options, datasheet Table 23
#define AD7142_PROXIMITY_RECAL_LVL_MAX		  255
#define AD7142_PROXIMITY_DETECTION_RATE_MAX	  63
#define AD7142_SLOW_FILTER_UPDATE_LVL_MAX  	  3

// AMB_COMP_CTRL2 bit options, datasheet Table 24
#define AD7142_FP_PROXIMITY_RECAL_MAX	  1023
#define AD7142_LP_PROXIMITY_RECAL_MAX	  63

// STAGE_LOW_INT_EN bit options, datasheet Table 25
#define AD7142_STAGEX_LOW_INT_DISABLE     0x00
#define AD7142_STAGEX_LOW_INT_ENABLE      0x01 //INT asserted if STAGEX low threshold is exceeded

#define AD7142_GPIO_SETUP_DISABLE         0x00
#define AD7142_GPIO_SETUP_INPUT           0x01
#define AD7142_GPIO_SETUP_OUTPUT_ACTIVE_LOW     0x02
#define AD7142_GPIO_SETUP_OUTPUT_ACTIVE_HIGH    0x03

#define AD7142_GPIO_INPUT_CONFIG_TRIG_NEG_LVL   0x00
#define AD7142_GPIO_INPUT_CONFIG_TRIG_POS_EDGE  0x01
#define AD7142_GPIO_INPUT_CONFIG_TRIG_NEG_EDGE  0x02
#define AD7142_GPIO_INPUT_CONFIG_TRIG_POS_LVL   0x03

// STAGE_HIGH_INT_EN bit options, datasheet Table 26
#define AD7142_STAGEX_HIGH_INT_DISABLE    0x00
#define AD7142_STAGEX_HIGH_INT_ENABLE     0x01 // INT asserted if STAGEX high threshold is exceeded

// STAGE_COMPLETE_INT_EN bit options, datasheet Table 27
#define AD7142_STAGEX_COMPLETE_DISABLE    0x00
#define AD7142_STAGEX_COMPLETE_ENABLE     0x01 // INT asserted at completion of STAGEX conversion

#define AD7142_GPIO_INT_ENABLE            0x00
#define AD7142_GPIO_INT_DISABLE           0x01

// STAGE_HIGH_LIMIT_INT bit options, datasheet Table 28
#define AD7142_STAGEX_LOW_LIMIT_FALSE     0x00
#define AD7142_STAGEX_LOW_LIMIT_TRUE      0x01

// STAGE_HIGH_LIMIT_INT bit options, datasheet Table 29
#define AD7142_STAGEX_HIGH_LIMIT_FALSE     0x00
#define AD7142_STAGEX_HIGH_LIMIT_TRUE      0x01

// STAGE_COMPLETE_LIMIT_INT bit options, datasheet Table 30
#define AD7142_STAGEX_COMPLETE_FALSE       0x00
#define AD7142_STAGEX_COMPLETE_TRUE        0x01

// CDC 16-Bit Conversion Data Registers, datasheet Table 31
// Use corresponding registers in Bank3 instead

// datasheet Table 32
#define AD7142_REVISION_CODE_DEVID        0xE622

// Proximity Status Register, datasheet Table 33
#define AD7142_STAGEX_PROXIMITY_FALSE     0x00
#define AD7142_STAGEX_PROXIMITY_TRUE      0x01

// ***********
//  Settings
//   Bank 2
// ***********

// CIN connection setup, datasheet Table 46, 47, 48
#define AD7142_CIN_NOT_CONNECTED          0x00
#define AD7142_CIN_CONNECTED_NEG          0x01
#define AD7142_CIN_CONNECTED_POS          0x02
#define AD7142_CIN_CONNECTED_BIAS         0x03

#define AD7142_AFE_OFFSET_ENABLE          0x00
#define AD7142_AFE_OFFSET_DISABLE         0x01

#define AD7142_AFE_OFFSET_SWAP_FALSE      0x00
#define AD7142_AFE_OFFSET_SWAP_TRUE       0x01

// TODO table 49 sensitivity settings

// ***********
//  Settings
//   Bank 3
// ***********

  // TODO

class AD7142 {
 public:

    AD7142(uint8_t add0, uint8_t add1); 
    ~AD7142(); 

    bool init(void);                                    
    bool checkDeviceId(void);

    // Power control
    bool setPwrControl( uint8_t pwrCtrl[8] ); // writes the entire register, fnctns below write specific bits
    bool setOperatingMode(uint8_t mode);
    bool setLowPowerConvDelay(uint8_t delay_val);
    bool setSequenceStageNum(uint8_t num);
    bool setDecimation(uint8_t rate);
    bool swReset(void);
    bool setInterruptPolarityControl(uint8_t ctrl);
    bool setSrcControl(uint8_t ctrl);
    bool setInvSrcControl(uint8_t ctrl);
    bool setCdcBiasCurrentControl(uint8_t ctrl);

	// Environmental calibration control
	bool resetConversionSequence();

    // Interrupt control
    bool setInterrupt(uint8_t interruptCtrl[12][3]);
	bool isStageComplete(uint8_t stage);
    
    // CIN connection setup
    bool setConnectionSetup(uint8_t cin0_6[12][7], uint8_t cin7_13[12][7]);
    bool enableAfeOffsets(uint8_t afeOffsetDisable[12][2]);
    bool setAfeOffsets(uint8_t offsets[12][4]);

    // Getters & setters
    uint16_t* getResultsRaw() { return _resultsRaw; } 
    float* getResultsPf() { return _resultsPf; } 
    uint8_t getNumStages() { return _numStages; } 

    // Actual functionality
    void readResults();
    void formatResults();
    void printResults();

    uint16_t readRegister(const uint16_t register_addr);
    bool writeRegister(const uint16_t register_addr, const uint16_t value);
	
 private:
    uint8_t _address;  // I2C address
    uint16_t* _resultsRaw;
    float* _resultsPf;
    uint8_t _numStages; // number of conversion stages

    bool writeRegisterBits(const uint16_t register_addr, uint16_t start_bit, uint16_t end_bit, const uint16_t value);
    
    bool checkTransmissionResult(byte result);
};


// Logger wrapper, set to 0 to prevent any serial printing
#define AD7142_LOG_LEVEL 4

#if AD7142_LOG_LEVEL >= 5
#define AD7142_LOG_DEBUG(x) Serial.println(String("DEBUG: ") + String(x))
#else
#define AD7142_LOG_DEBUG(x) // nothing
#endif

#if AD7142_LOG_LEVEL >= 4
#define AD7142_LOG_INFO(x) Serial.println(String("INFO: ") + String(x))
#else
#define AD7142_LOG_INFO(x) // nothing
#endif

#if AD7142_LOG_LEVEL >= 3
#define AD7142_LOG_WARN(x) Serial.println(String("WARNING: ") + String(x))
#else
#define AD7142_LOG_WARN(x) // nothing
#endif

#if AD7142_LOG_LEVEL >= 2
#define AD7142_LOG_ERROR(x) Serial.println(String("ERROR: ") + String(x))
#else
#define AD7142_LOG_ERROR(x) // nothing
#endif

#if AD7142_LOG_LEVEL >= 1
#define AD7142_LOG_ANY(x) Serial.print(x)
#else
#define AD7142_LOG_ANY(x) // nothing
#endif

#endif /* _AD7142_H_ */
