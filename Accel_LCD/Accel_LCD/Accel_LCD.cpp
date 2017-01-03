#include <iostream>
#include <unistd.h>
#include "MPU6050.h"

using namespace std;

int main(int argc, char *argv[])
{
	//create and initialize the mpu6050
	MPU6050 mpu6050(2, MPU6050_ID, POWER_STATE_NORMAL, SAMPLERATE_200Hz, BW_44Hz_Delay_4p9ms, GYRORANGE_plus_minus_500deg_sec, ACCELRANGE_plus_minus_4g);
	
	while (true)
	{
		//refreshed the values
		mpu6050.readFullSensorState();
		
		//prints the values to the console
		cout << "Acceleration X in G's: " << mpu6050.getAccelerationX_g() << endl;
		cout << "Acceleration Y in G's: " << mpu6050.getAccelerationY_g() << endl;
		cout << "Acceleration Z in G's: " << mpu6050.getAccelerationZ_g() << endl;
		cout << endl;
		
		cout << "Acceleration X in m_s2: " << mpu6050.getAccelerationX_m_s2() << endl;
		cout << "Acceleration Y in m_s2: " << mpu6050.getAccelerationY_m_s2() << endl;
		cout << "Acceleration Z in m_s2: " << mpu6050.getAccelerationZ_m_s2() << endl;
		cout << endl;
		
		cout << "Acceleration X in ft_s2: " << mpu6050.getAccelerationX_ft_s2() << endl;
		cout << "Acceleration Y in ft_s2: " << mpu6050.getAccelerationY_ft_s2() << endl;
		cout << "Acceleration Z in ft_s2: " << mpu6050.getAccelerationZ_ft_s2() << endl;  
		cout << endl << endl << endl;
		
		cout << "Gyro X in rad_s: " << mpu6050.getGyroRateX_rad_s() << endl;
		cout << "Gyro Y in rad_s: " << mpu6050.getGyroRateY_rad_s() << endl;
		cout << "Gyro Z in rad_s: " << mpu6050.getGyroRateZ_rad_s() << endl;
		cout << endl;
		
		cout << "Gryo X in deg_s: " << mpu6050.getGyroRateX_deg_s() << endl;
		cout << "Gryo Y in deg_s: " << mpu6050.getGyroRateY_deg_s() << endl;
		cout << "Gryo Z in deg_s: " << mpu6050.getGyroRateZ_deg_s() << endl;  
		cout << endl << endl << endl;
		
		cout << "Temperature in degrees C: " << mpu6050.getTemperature_degC() << endl;
		cout << "Temperature in degrees F: " << mpu6050.getTemperature_degF() << endl;
		cout << "------------------------END---------------------" << endl;
		cout << endl;
		
		//sleep for a 5 seconds
		//usleep(1000 * 5000);
	}
	
	return 0;
}