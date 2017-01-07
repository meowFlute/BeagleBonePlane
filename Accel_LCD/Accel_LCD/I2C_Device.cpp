#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "I2C_Device.h"

using namespace std;

//private functions
//the accelerometer and gyro registers represent a 16-bit 2's compliment value
template<int buffer_size> int I2C_Device<buffer_size>::convertTwosComplementToInt(unsigned char msb, unsigned char lsb)
{
	long t = msb * 0x100L + lsb;
	if (t >= 32768)
		t -= 65536;
	return (int)t;

}

//I think the temp is just a short split into two bytes, but we'll see if that makes sense...
template<int buffer_size> int I2C_Device<buffer_size>::convert16bitSignedValueToInt(unsigned char msb, unsigned char lsb)
{
	return (short)(((msb & 0xFF) << 8) | (lsb & 0xFF));
}

//this is the generalized i2c write function for the MPU6050
template<int buffer_size> int I2C_Device<buffer_size>::writeI2CDeviceByte(char address, char value)
{
	//again, just like when we read bytes, treat it like a text file:
	
	char namebuf[64];
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus - 1); //for some reason it is zero based on the beaglebone I'm using
	int file;
	//first, just try and open the "text file" representing the I2C bus in question
	if ((file = open(namebuf, O_RDWR)) < 0)
	{
		cout << "Failed to open I2C bus " << namebuf << " in function writeI2CDeviceByte" << endl;
		close(file);
		return (1);
	}
	//assuming that worked out, try and open the a connection to the MPU6050 at "I2CAddress"
	if (ioctl(file, I2C_SLAVE, I2CAddress) < 0)
	{
		cout << "Failed to contact I2C Slave address " << I2CAddress << endl;
		close(file);
		return (2);
	}
	
	//Send a byte for where you want to write and a byte for the data you want to set there
	char buf[2] = { address, value }; //0x3B is the first data point we're interested in
	if (write(file, buf, 2) != 2)
	{
		cout << "Failed to write byte " << value << " to location " << address << " of MPU6050" << endl;
		close(file);
		return (3);
	}
	
	//close the stream
	close(file);
	return (0);
}

//max number of bytes that can be read is set by MPU6050_I2C_BUFFER_SIZE
//bytes read to this->dataBuffer
//if you burst-read like below you are guaranteed to get data from the same sample instant
template<int buffer_size> int I2C_Device<buffer_size>::readI2CDeviceBytes(char startAddress, int numBytesToRead)
{
	//just pretend that the I2C bus is a file that we are modifying... ;)
	
	char namebuf[64]; //name of i2c bus "file"
	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus - 1); //for some reason it is zero based on the beaglebone I'm using
	int file;
	//first, just try and open the "text file" representing the I2C bus in question
	if ((file = open(namebuf, O_RDWR)) < 0)
	{
		cout << "Failed to open I2C bus " << namebuf << " in function readI2CDeviceBytes" << endl;
		close(file);
		return (1);
	}
	//assuming that worked out, try and open the a connection to the MPU6050 at "I2CAddress"
	if (ioctl(file, I2C_SLAVE, I2CAddress) < 0)
	{
		cout << "Failed to contact I2C Slave address " << I2CAddress << endl;
		close(file);
		return (2);
	}
	
	//the MPU6050 wants you to tell it where to start from when you ask it to read, so we write a single byte of that address
	char buf[1] = { startAddress }; //0x3B is the first data point we're interested in
	if (write(file, buf, 1) != 1)
	{
		cout << "Failed to write starting address to MPU6050" << endl;
		close(file);
		return (3);
	}
	
	//read from the "file" and make sure you got enough bytes
	int bytesRead = read(file, this->dataBuffer, numBytesToRead);
	if (bytesRead < numBytesToRead)		
	{
		cout << "Failure to read byte stream: wrong number of bytes read" << endl;
		close(file);
		return (4);
	}
	
	//not sure how to verify that we go the right data, so we'll just trust it
	//the data is stored in the MPU6050 object's data buffer
	close(file);
	return (0);
}