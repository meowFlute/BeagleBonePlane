#ifndef I2C_DEVICE_H
#define I2C_DEVICE_H

template<int buffer_size> class I2C_Device
{
private:
	int I2CBus, I2CAddress;	
	
	unsigned char dataBuffer[buffer_size];
	
	int convertTwosComplementToInt(unsigned char msb, unsigned char lsb);
	int convert16bitSignedValueToInt(unsigned char msb, unsigned char lsb);
	int writeI2CDeviceByte(char address, char value);
	int readI2CDeviceBytes(char startAddress, int numBytesToRead);
public:
	I2C_Device();
	~I2C_Device(){};
};

#endif // !I2C_DEVICE_H