////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//
//  03/11/2016 Jeff Loughlin
//  This module adds support for an IMU containing an HMC5883L magnetometer, ADXL345
//  accelerometer, and L3G4200D gyro.
//
#include "RTIMUGY85.h"
#include "RTIMUSettings.h"


RTIMUGY85::RTIMUGY85(RTIMUSettings *settings) : RTIMU(settings)
{
    m_sampleRate = 10;
    m_compassSlaveAddr = HMC5883L_ADDRESS;
    m_gyroSlaveAddr = ITG3205_ADDRESS;
    m_accelSlaveAddr = ADXL345_ADDRESS;
    m_compassScale = 1.0;
    m_gyroScale = 0.00875;
    m_accelScale = 0.0384;
}

RTIMUGY85::~RTIMUGY85()
{
}


bool RTIMUGY85::IMUInit()
{
    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;
    

    setCalibrationData();

    if (!m_settings->HALOpen())
	return false;

    if (!setGyroSampleRate())
	return false;

    gyroBiasInit();
        

    // Configure the accelerometer
    if (!m_settings->HALWrite(m_accelSlaveAddr, 0x2d, 0x18, "Error Setting up ADXL345"))
    {
	return false;
    }


    // Set Accelerometer sensitivity:  0x09 = 4g , 0x0A = 8g, 0x0B = 16g
    if (!m_settings->HALWrite(m_accelSlaveAddr, 0x31, 0x0b, "Error Setting up ADXL345"))
    {
	return false;
    }


    // Configure the gyro
    // Enable x, y, z and turn off power down:
      /*****************************************
   * ITG3205
   * G_SMPLRT_DIV：采样率 = 125Hz
   * G_DLPF_FS：+ - 2000度/秒、低通滤波器5HZ、内部采样率1kHz
   * G_INT_CFG：没有中断
   * G_PWR_MGM：电源管理设定：无复位、无睡眠模式、无待机模式、内部振荡器
   */
    if (!m_settings->HALWrite(m_gyroSlaveAddr, ITGCTRL_REG1, 0x00, "Error Setting up itg3200"))
    {
	return false;
    }
    
    // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2 (see data sheet):
    if (!m_settings->HALWrite(m_gyroSlaveAddr, ITGCTRL_REG2, 0x07, "Error Setting up ITG3200"))
    {
	return false;
    }

    // Configure CTRL_REG3 to generate data ready interrupt on INT2
    // No interrupts used on INT1, if you'd like to configure INT1
    // or INT2 otherwise, consult the datasheet:
    if (!m_settings->HALWrite(m_gyroSlaveAddr, ITGCTRL_REG3, 0x00, "Error Setting up itg3200"))
    {
	return false;
    }

    // Configure gyro full-scale range: 0x00 = 250, 0x10 = 500, 0x30 = 2000
    if (!m_settings->HALWrite(m_gyroSlaveAddr, ITGCTRL_REG4, 0x1E, "Error Setting up itg3200"))
    {
	return false;
    }

  

    // Configure the compass
    if (!m_settings->HALWrite(HMC5883L_ADDRESS, HMC5883_CONFIG_A, 0x38, "Failed to set HMC5883 config A"))
    {
	return false;
    }

    if (!m_settings->HALWrite(HMC5883L_ADDRESS, HMC5883_CONFIG_B, 0x20, "Failed to set HMC5883 config B"))
    {
	return false;
    }

    if (!m_settings->HALWrite(HMC5883L_ADDRESS, HMC5883_MODE, 0x00, "Failed to set HMC5883 mode"))
    {
	return false;
    }
    

    return true;
}

bool RTIMUGY85::setGyroSampleRate()
{
    m_sampleRate = 50;
    return true;
}



int RTIMUGY85::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}


bool RTIMUGY85::readADXL345(short &x , short &y, short &z)
{
    unsigned char buf[7];

    if (!m_settings->HALRead(m_accelSlaveAddr, 0x32, 6, buf, "Error reading from ADXL345"))
    {
	return false;
    }
    
    x = (buf[1]<<8) |  buf[0];
    y = (buf[3]<<8) |  buf[2];
    z = (buf[5]<<8) |  buf[4];

    return true;
}

bool RTIMUGY85::readITG3200(short &x , short &y, short &z)
{
    unsigned char buf[7];

  /*  unsigned char status;
    if (!m_settings->HALRead(m_gyroSlaveAddr, L3GD20H_STATUS, 1, &status, "Failed to read L3G4200D status"))
	return false;

    if ((status & 0x8) == 0)
	return false;
	*/  
    
    if (!m_settings->HALRead(m_gyroSlaveAddr, 0X1D , 6, buf, "Error reading from ITG3200"))
    {
	return false;
    }

    x = (buf[0]<<8) |  buf[1];
    y = (buf[2]<<8) |  buf[3];
    z = (buf[4]<<8) |  buf[5];

    return true;
}

bool RTIMUGY85::readHMC5883L(short &x , short &y, short &z)
{
    unsigned char buf[7];

    if (!m_settings->HALRead(m_compassSlaveAddr, 0x80 | HMC5883_DATA_X_HI, 6, buf, "Error reading from HMC5883L"))
    {
	return false;
    }

    // Note: according to the data sheet, y and z are swapped and the data is big-endian
    x = (buf[0]<<8) |  buf[1];
    z = (buf[2]<<8) |  buf[3];
    y = (buf[4]<<8) |  buf[5];

    return true;
}




bool RTIMUGY85::IMURead()
{
    // Grab a reading from the compass, accel, and gyro
    short x, y, z;
    if (!readITG3200(x, y, z))
    {
       return false;
    }
    m_imuData.gyro.setX((RTFLOAT)x * m_gyroScale);
    m_imuData.gyro.setY((RTFLOAT)y * m_gyroScale);
    m_imuData.gyro.setZ((RTFLOAT)z * m_gyroScale);

    if (!readHMC5883L(x, y, z))
    {
	return false;
    }
    m_imuData.compass.setX((RTFLOAT)x * m_compassScale);
    m_imuData.compass.setY((RTFLOAT)y * m_compassScale);
    m_imuData.compass.setZ((RTFLOAT)z * m_compassScale);

    if (!readADXL345(x, y, z))
    {
	return false;
    }
    m_imuData.accel.setX((RTFLOAT)x * m_accelScale);
    m_imuData.accel.setY((RTFLOAT)y * m_accelScale);
    m_imuData.accel.setZ((RTFLOAT)z * m_accelScale);

    // Timestamp the data
    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();


    //  Swap the axes to match the board
    //  Might have to change these depending on specific board layout
    RTFLOAT temp;
//    temp = m_imuData.gyro.x();
//    m_imuData.gyro.setX(-m_imuData.gyro.y());
//    m_imuData.gyro.setY(-temp);
    
    temp = m_imuData.accel.x();
    m_imuData.accel.setX(m_imuData.accel.y());
    m_imuData.accel.setY(temp);
    
    temp = m_imuData.compass.y();
    m_imuData.compass.setY(m_imuData.compass.x());
    m_imuData.compass.setX(temp);

    
    //  now do standard processing and update the filter
    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();
    updateFusion();

    return true;
}
