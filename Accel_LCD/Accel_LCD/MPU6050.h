#ifndef MPU6050_H
#define MPU6050_H

#include "I2C_Device.h"

//buffer size
#define MPU6050_I2C_BUFFER_SIZE 14

//physical bus address
#define MPU6050_ADDRESS 0xD0
#define MPU6050_ID 0x68

//different registers on the MPU6050 (there are a lot that I didn't include for simplicity's sake)
//config registers
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_WHO_AM_I 0x75

//output data registers
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

//define some powerstates
#define POWER_STATE_NORMAL	0x02
#define POWER_STATE_SLEEP	0x42
#define POWER_STATE_RESET	0x80

//these are for the sample rates
#define	SAMPLERATE_1kHz		0x00
#define	SAMPLERATE_500Hz	0x01
#define	SAMPLERATE_333Hz	0x02
#define	SAMPLERATE_250Hz	0x03
#define	SAMPLERATE_200Hz	0x04
#define	SAMPLERATE_167Hz	0x05
#define	SAMPLERATE_143Hz	0x06
#define	SAMPLERATE_125Hz	0x07
#define	SAMPLERATE_111Hz	0x08
#define	SAMPLERATE_100Hz	0x09
#define	SAMPLERATE_91Hz		0x0A
#define	SAMPLERATE_83Hz		0x0B
#define	SAMPLERATE_77Hz		0x0C
#define	SAMPLERATE_71Hz		0x0D
#define	SAMPLERATE_67Hz		0x0E
#define	SAMPLERATE_63Hz		0x0F
#define	SAMPLERATE_59Hz		0x10
#define	SAMPLERATE_56Hz		0x11
#define	SAMPLERATE_53Hz		0x12
#define	SAMPLERATE_50Hz		0x13
#define	SAMPLERATE_48Hz		0x14
#define	SAMPLERATE_45Hz		0x15
#define	SAMPLERATE_43Hz		0x16
#define	SAMPLERATE_42Hz		0x17
#define	SAMPLERATE_40Hz		0x18
#define	SAMPLERATE_38Hz		0x19
#define	SAMPLERATE_37Hz		0x1A
#define	SAMPLERATE_36Hz		0x1B
#define	SAMPLERATE_34Hz		0x1C
#define	SAMPLERATE_33Hz		0x1D
#define	SAMPLERATE_32Hz		0x1E
#define	SAMPLERATE_31Hz		0x1F
#define	SAMPLERATE_30Hz		0x20

// DLPF_Bandwidth
#define	BW_184Hz_Delay_2p0ms		0x01
#define	BW_94Hz_Delay_3p0ms			0x02
#define	BW_44Hz_Delay_4p9ms			0x03
#define	BW_21Hz_Delay_8p5ms			0x04
#define	BW_10Hz_Delay_13p8ms		0x05
#define	BW_5Hz_Delay_19p0ms			0x06

// Gryo full-scale ranges
#define GYRORANGE_plus_minus_250deg_sec		0b00000000
#define GYRORANGE_plus_minus_500deg_sec		0b00001000
#define GYRORANGE_plus_minus_1000deg_sec	0b00010000
#define GYRORANGE_plus_minus_2000deg_sec	0b00011000

// Accelerometer full-scale ranges
#define	ACCELRANGE_plus_minus_2g		0b00000000
#define	ACCELRANGE_plus_minus_4g		0b00001000
#define	ACCELRANGE_plus_minus_8g		0b00010000
#define	ACCELRANGE_plus_minus_16g		0b00011000

template<int buffer_size> class MPU6050 : public I2C_Device<buffer_size>
{
private:
	int accelerationX;
	int accelerationY;
	int accelerationZ;
	int gyroX;
	int gyroY;
	int gyroZ;
	int temperature;
	
	unsigned char power_state;
	int sampleRate;
	unsigned char sampleRate_byte;
	int dlpf_bandwidth;
	float dlpf_delay;
	unsigned char dlpf_bandwidth_byte;
	int gyro_range;
	unsigned char gyro_range_byte;
	int accel_range;
	unsigned char accel_range_byte;	
public:
	MPU6050(int bus, int address, char power_state, char sample_rate, char filter_bandwidth, char gyro_range, char accel_range);
	
	int readFullSensorState();
	
	int setPowerState(char powerState);
	int setOnboardSampleRate(char sampleRate);
	int setLowPassBandwidth(char bandwidth);
	int setGyroFullScaleRange(char gyroRange);
	int setAccelFullScaleRange(char accelRange);
	
	float getTemperature_degC();
	float getTemperature_degF();
	
	float getAccelerationX_g();
	float getAccelerationY_g();
	float getAccelerationZ_g();
	float getAccelerationX_m_s2();
	float getAccelerationY_m_s2();
	float getAccelerationZ_m_s2();
	float getAccelerationX_ft_s2();
	float getAccelerationY_ft_s2();
	float getAccelerationZ_ft_s2();
	
	float getGyroRateX_rad_s();
	float getGyroRateY_rad_s();
	float getGyroRateZ_rad_s();
	float getGyroRateX_deg_s();
	float getGyroRateY_deg_s();
	float getGyroRateZ_deg_s();
	
	virtual ~MPU6050() {};
};

#endif