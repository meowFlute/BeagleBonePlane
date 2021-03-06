#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
#include <iostream>

#include "MPU6050.h"
#include "I2C_Device.h"

//these numbers represent where they'll end up in our dataBuffer
#define AccelX_H	0
#define AccelX_L	1
#define AccelY_H	2
#define AccelY_L	3
#define AccelZ_H	4
#define AccelZ_L	5
#define Temp_H		6
#define Temp_L		7
#define GyroX_H		8
#define GyroX_L		9
#define GyroY_H		10
#define GyroY_L		11
#define GyroZ_H		12
#define GyroZ_L		13
#define PI			3.14159265359	

using namespace std;

MPU6050::MPU6050(int bus, int address, char power_state, char sample_rate, char filter_bandwidth, char gyro_range, char accel_range)
{
	this->I2CBus = bus;
	this->I2CAddress = address;
	
	//wake up from sleep mode (which is set by default upon startup
	setPowerState(power_state);
	
	//initialize sensor ranges and such
	setOnboardSampleRate(sample_rate);
	setLowPassBandwidth(filter_bandwidth);
	setGyroFullScaleRange(gyro_range);
	setAccelFullScaleRange(accel_range);
	
	//read the full state
	readFullSensorState();
}
	
int MPU6050::readFullSensorState()
{
	//read the data into the data buffer using our handy burst-read function
	int status = this->readI2CDeviceBytes(MPU6050_ACCEL_XOUT_H, MPU6050_I2C_BUFFER_SIZE);
	if (status > 0) //see readI2CDeviceBytes below for error codes
	{
		cout << "error " << status << " during readI2CDeviceBytes in readFullSensorState" << endl;
		return (1);
	}
	
	//the resulting values need only be converted to their appropriate ints
	//accels
	this->accelerationX = convertTwosComplementToInt(this->dataBuffer[AccelX_H], this->dataBuffer[AccelX_L]);
	this->accelerationY = convertTwosComplementToInt(this->dataBuffer[AccelY_H], this->dataBuffer[AccelY_L]);
	this->accelerationZ = convertTwosComplementToInt(this->dataBuffer[AccelZ_H], this->dataBuffer[AccelZ_L]);
	
	//temp
	this->temperature = convert16bitSignedValueToInt(this->dataBuffer[Temp_H], this->dataBuffer[Temp_L]);
	
	//gyros
	this->gyroX = convertTwosComplementToInt(this->dataBuffer[GyroX_H], this->dataBuffer[GyroX_L]);
	this->gyroY = convertTwosComplementToInt(this->dataBuffer[GyroY_H], this->dataBuffer[GyroY_L]);
	this->gyroZ = convertTwosComplementToInt(this->dataBuffer[GyroZ_H], this->dataBuffer[GyroZ_L]);
	
	return (0);
}
	
//this function could serve as a really good example of how to write a byte to a location. Pretty easy once you get the hang of it
int MPU6050::setPowerState(char powerState)
{
	
	int status = this->writeI2CDeviceByte(MPU6050_PWR_MGMT_1, powerState);
	if (status > 0)
	{
		cout << "error " << status << " during writeI2CDeviceByte in setPowerState" << endl;
		return (1);
	}

	string tempstr;
	switch (powerState)
	{
	case 0x02:
		tempstr = "Normal Power, Gyro-based Clock";
	case 0x42:
		tempstr = "Sleep Power (no sampling), Gyro-based Clock";
	case 0x80:
		tempstr = "Reset -> Sleep Power, 8MHz internal oscilator clock";
	default:
		tempstr = "unkown type";
	}
	
	this->power_state = powerState;
	
	cout << "Powerstate successfully set to: " << tempstr << endl;
	
	return (0);
}

int MPU6050::setOnboardSampleRate(char sampleRate)
{
	int status = this->writeI2CDeviceByte(MPU6050_SMPLRT_DIV, sampleRate);
	if (status > 0)
	{
		cout << "error " << status << " during writeI2CDeviceByte in setOnboardSampleRate" << endl;
		return (1);
	}
	
	int tempSampleRate = 0;
	
	switch (sampleRate)
	{
	case 0x00 : 
		tempSampleRate = 1000;	
		break;
	case 0x01 : 
		tempSampleRate = 500;
		break;
	case 0x02 : 
		tempSampleRate = 333;
		break;
	case 0x03 : 
		tempSampleRate = 250;
		break;
	case 0x04 : 
		tempSampleRate = 200;
		break;
	case 0x05 : 
		tempSampleRate = 167;
		break;
	case 0x06 : 
		tempSampleRate = 143;
		break;
	case 0x07 : 
		tempSampleRate = 125;
		break;
	case 0x08 : 
		tempSampleRate = 111;
		break;
	case 0x09 : 
		tempSampleRate = 100;
		break;
	case 0x0A : 
		tempSampleRate = 91;
		break;
	case 0x0B : 
		tempSampleRate = 83;
		break;
	case 0x0C : 
		tempSampleRate = 77;
		break;
	case 0x0D : 
		tempSampleRate = 71;
		break;
	case 0x0E : 
		tempSampleRate = 67;
		break;
	case 0x0F : 
		tempSampleRate = 63;
		break;
	case 0x10 : 
		tempSampleRate = 59;
		break;
	case 0x11 : 
		tempSampleRate = 56;
		break;
	case 0x12 : 
		tempSampleRate = 53;
		break;
	case 0x13 : 
		tempSampleRate = 50;
		break;
	case 0x14 : 
		tempSampleRate = 48;
		break;
	case 0x15 : 
		tempSampleRate = 45;
		break;
	case 0x16 : 
		tempSampleRate = 43;
		break;
	case 0x17 : 
		tempSampleRate = 42;
		break;
	case 0x18 : 
		tempSampleRate = 40;
		break;
	case 0x19 :
		tempSampleRate = 38;
		break;
	case 0x1A : 
		tempSampleRate = 37;
		break;
	case 0x1B : 
		tempSampleRate = 36;
		break;
	case 0x1C : 
		tempSampleRate = 34;
		break;
	case 0x1D :
		tempSampleRate = 33;
		break;
	case 0x1E : 
		tempSampleRate = 32;
		break;
	case 0x1F : 
		tempSampleRate = 31;
		break;
	case 0x20 : 
		tempSampleRate = 30;
		break;
	default:
		tempSampleRate = -1;
		break;
	}	
	
	this->sampleRate = tempSampleRate;
	this->sampleRate_byte = sampleRate;
	
	cout << "Sample rate successfully set to " << tempSampleRate << endl;
	
	return (0);
}

int MPU6050::setLowPassBandwidth(char bandwidth)
{	
	int status = this->writeI2CDeviceByte(MPU6050_CONFIG, bandwidth);
	if (status > 0)
	{
		cout << "error " << status << " during writeI2CDeviceByte in setLowPassBandwidth" << endl;
		return (1);
	}

	int tempBandwidth;
	float tempDelay;
	switch (bandwidth)
	{
	case 0x01:
		tempBandwidth = 184;
		tempDelay = 2.0f;
		break;
	case 0x02:
		tempBandwidth = 94;
		tempDelay = 3.0f;
		break;
	case 0x03:
		tempBandwidth = 44;
		tempDelay = 4.9f;
		break;
	case 0x04:
		tempBandwidth = 21;
		tempDelay = 8.5f;
		break;
	case 0x05:
		tempBandwidth = 10;
		tempDelay = 13.8f;
		break;
	case 0x06:
		tempBandwidth = 5;
		tempDelay = 19.0f;
		break;
	default:
		tempBandwidth = -1;
		tempDelay = -1.0f;
		break;
	}		               
	
	this->dlpf_bandwidth = tempBandwidth;
	this->dlpf_bandwidth_byte = bandwidth;
	this->dlpf_delay = tempDelay;
	
	cout << "Bandwidth successfully set to " << tempBandwidth << ", \ncorresponding delay is " << tempDelay << endl;
	
	return (0);
}

int MPU6050::setGyroFullScaleRange(char gyroRange)
{
	int status = this->writeI2CDeviceByte(MPU6050_GYRO_CONFIG, gyroRange);
	if (status > 0)
	{
		cout << "error " << status << " during writeI2CDeviceByte in setGyroFullScaleRange" << endl;
		return (1);
	}
	
	int tempGyroRange;
	switch (gyroRange)
	{
	case 0b00000000:
		tempGyroRange = 250;
		break;
	case 0b00001000:
		tempGyroRange = 500;
		break;
	case 0b00010000:
		tempGyroRange = 1000;
		break;
	case 0b00011000:
		tempGyroRange = 2000;
		break;
	default:
		tempGyroRange = -1;
		break;
	}
	
	this->gyro_range = tempGyroRange;
	this->gyro_range_byte = gyroRange;
	
	cout << "gyroRange successfully set to plus or minus " << tempGyroRange << endl;
	
	return (0);
}

int MPU6050::setAccelFullScaleRange(char accelRange)
{
	int status = this->writeI2CDeviceByte(MPU6050_ACCEL_CONFIG, accelRange);
	if (status > 0)
	{
		cout << "error " << status << " during writeI2CDeviceByte in setAccelFullScaleRange" << endl;
		return (1);
	}
	
	int tempAccelRange;
	switch (accelRange)
	{
	case 0b00000000:
		tempAccelRange = 2;
		break;
	case 0b00001000:
		tempAccelRange = 4;
		break;
	case 0b00010000:
		tempAccelRange = 8;
		break;
	case 0b00011000:
		tempAccelRange = 16;
		break;
	default:
		tempAccelRange = -1;
		break;
	}
	
	this->accel_range = tempAccelRange;
	this->accel_range_byte = accelRange;
	
	cout << "accelRange successfully set to plus or minus " << tempAccelRange << endl;
	
	return (0);
}
	
float MPU6050::getTemperature_degC()
{
	return (this->temperature / 340.0f) + 36.53f;
}

float MPU6050::getTemperature_degF()
{
	return (getTemperature_degC() * 1.8f) + 32;
}
	
float MPU6050::getAccelerationX_g()
{
	int accelX = this->accelerationX;
	double fullIntRange = 32767.0; 
	int accelRange = this->accel_range;
	
	float answer = accelX / fullIntRange * accelRange;
	return answer;
}

float MPU6050::getAccelerationY_g()
{
	return (this->accelerationY) / 32767.0f * (this->accel_range);
}

float MPU6050::getAccelerationZ_g()
{
	return (this->accelerationZ) / 32767.0f * (this->accel_range);
}

float MPU6050::getAccelerationX_m_s2()
{
	float answer = getAccelerationX_g() * 9.81;
	
	return answer;
}

float MPU6050::getAccelerationY_m_s2()
{
	return getAccelerationY_g() * 9.81;
}

float MPU6050::getAccelerationZ_m_s2()
{
	return getAccelerationZ_g() * 9.81;
}

float MPU6050::getAccelerationX_ft_s2()
{
	float answer = getAccelerationX_g() * 32.174;	
	
	return answer;
}

float MPU6050::getAccelerationY_ft_s2()
{
	return getAccelerationY_g() * 32.174;
}

float MPU6050::getAccelerationZ_ft_s2()
{
	return getAccelerationZ_g() * 32.174;
}
	
float MPU6050::getGyroRateX_rad_s()
{
	return getGyroRateX_deg_s() * PI / 180.0;
}

float MPU6050::getGyroRateY_rad_s()
{
	return getGyroRateY_deg_s() * PI / 180.0;
}

float MPU6050::getGyroRateZ_rad_s()
{
	return getGyroRateZ_deg_s() * PI / 180.0;
}

float MPU6050::getGyroRateX_deg_s()
{
	return (this->gyroX) / 32767.0f * ((float)this->gyro_range);
}

float MPU6050::getGyroRateY_deg_s()
{
	return (this->gyroY) / 32767.0f * ((float)this->gyro_range);
}

float MPU6050::getGyroRateZ_deg_s()
{
	return (this->gyroZ) / 32767.0f * ((float)this->gyro_range);
}