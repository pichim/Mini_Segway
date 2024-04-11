#ifndef MPU6000_SPI_H_
#define MPU6000_SPI_H_

#include "mbed.h"
#include "mpu6500_registermap.h"
 
class mpu6500_spi
{
    SPI& spi;
    DigitalOut cs;
    
  public:

    /*-----------------------------------------------------------------------------------------------
                                    Instantiation
    _spi is the buss to be selected
    _cs is the chip select pin
    -----------------------------------------------------------------------------------------------*/
    mpu6500_spi(SPI& _spi, PinName _cs);

    /*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
    usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
    low pass filter value; suitable values are:
    BITS_DLPF_CFG_256HZ_NOLPF2
    BITS_DLPF_CFG_188HZ
    BITS_DLPF_CFG_98HZ
    BITS_DLPF_CFG_42HZ
    BITS_DLPF_CFG_20HZ
    BITS_DLPF_CFG_10HZ 
    BITS_DLPF_CFG_5HZ 
    BITS_DLPF_CFG_2100HZ_NOLPF
    returns 1 if an error occurred
    -----------------------------------------------------------------------------------------------*/
    bool init(int sample_rate_div,int low_pass_filter);
		bool init_inav(void);


    /*-----------------------------------------------------------------------------------------------
                        CONFIGURATION ACCORDING TO YOUR APPLICATION
    -----------------------------------------------------------------------------------------------*/
    bool configuration();


    /*-----------------------------------------------------------------------------------------------
                                    TEST SPI CONNECTION 
    -----------------------------------------------------------------------------------------------*/
    bool testConnection();

    /*-----------------------------------------------------------------------------------------------
                                    ENABLE INTERRUPT
    -----------------------------------------------------------------------------------------------*/
    int enableInterrupt();

    /*-----------------------------------------------------------------------------------------------
                                    READ ACCELEROMETER
    usage: call this function to read accelerometer data. Axis represents selected axis:
    0 -> X axis
    1 -> Y axis
    2 -> Z axis
    returns the value in Gs
    -----------------------------------------------------------------------------------------------*/
    //float readAcc(int axis);
	float readAcc(int axis);
    int16_t readAcc_raw(int axis);
    void readAcc(void);
    
    /*-----------------------------------------------------------------------------------------------
                                    READ GYROSCOPE
    usage: call this function to read gyroscope data. Axis represents selected axis:
    0 -> X axis
    1 -> Y axis
    2 -> Z axis
    returns the value in Degrees per second
    -----------------------------------------------------------------------------------------------*/
    float readGyro(int axis);
    void readGyro(void);
    float readGyro_raw(int axis);
	void readAccTempGyro(void);

    /*-----------------------------------------------------------------------------------------------
                                    GYROSCOPE SCALE
    usage: call this function at startup, after initialization, to set the right range for the
    gyroscopes. Suitable ranges are:
    BITS_FS_250DPS
    BITS_FS_500DPS
    BITS_FS_1000DPS
    BITS_FS_2000DPS
    returns the range set (250,500,1000 or 2000)
    -----------------------------------------------------------------------------------------------*/
    unsigned int set_gyro_scale(int scale);

    /*-----------------------------------------------------------------------------------------------
                                    ACCELEROMETER SCALE
    usage: call this function at startup, after initialization, to set the right range for the
    accelerometers. Suitable ranges are:
    BITS_FS_2G
    BITS_FS_4G
    BITS_FS_8G
    BITS_FS_16G
    returns the range set (2,4,8 or 16)
    -----------------------------------------------------------------------------------------------*/
    unsigned int set_acc_scale(int scale);

    /*-----------------------------------------------------------------------------------------------
                                    READ ACCELEROMETER CALIBRATION
    usage: call this function to read accelerometer data. Axis represents selected axis:
    0 -> X axis
    1 -> Y axis
    2 -> Z axis
    returns Factory Trim value
    -----------------------------------------------------------------------------------------------*/
    int calib_acc(int axis);

    /*-----------------------------------------------------------------------------------------------
                                    READ TEMPERATURE
    usage: call this function to read temperature data. 
    returns the value in °C
    -----------------------------------------------------------------------------------------------*/
    //float read_temp();
		int16_t read_temp();

    /*-----------------------------------------------------------------------------------------------
                                    SPI SELECT
    usage: enable mpu6000 communication bus
    -----------------------------------------------------------------------------------------------*/
    void select();

    /*-----------------------------------------------------------------------------------------------
                                    SPI DESELECT
    usage: disable mpu6000 communication bus
    -----------------------------------------------------------------------------------------------*/
    void deselect();
    
    /*-----------------------------------------------------------------------------------------------
                                    WHO AM I?
    usage: call this function to know if SPI is working correctly. It checks the I2C address of the
    mpu6000 which should be 104 when in SPI mode.
    returns the I2C address (104)
    -----------------------------------------------------------------------------------------------*/
    unsigned int whoami();
    
    float acc_divider;
    float gyro_divider;
    float gyroX, gyroY, gyroZ; // x, y, and z axis readings of the gyroscope (float value)
    float accX, accY, accZ;    // x, y, and z axis readings of the accelerometer (float value)
    uint16_t accX_raw,accY_raw,accZ_raw;
    
  private:
    PinName _CS_pin;
    PinName _SO_pin;
    PinName _SCK_pin;
    float _error;
		void write2spi(uint8_t,uint8_t);
};
 
#endif //#ifndef MPU6000_SPI_H_