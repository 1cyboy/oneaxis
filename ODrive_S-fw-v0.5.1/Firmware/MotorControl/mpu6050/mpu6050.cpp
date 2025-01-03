#include <stdlib.h>
#include <functional>

#include "MPU6050.hpp"
#include"dmpKey.h"
#include"dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


#define I2C_Handle hi2c2
#define I2Cx_FLAG_TIMEOUT 1000

int ODrive::MPU6050_DMP::I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len, unsigned char *data_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Write(&I2C_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, I2Cx_FLAG_TIMEOUT);
    if (status != HAL_OK) {
        // 处理错误
    }
    while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY) {
        // 等待I2C准备好
    }
    while (HAL_I2C_IsDeviceReady(&I2C_Handle, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
    while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY) {
        // 等待I2C准备好
    }
    return status;
}

int ODrive::MPU6050_DMP::I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len, unsigned char *data_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
    status = HAL_I2C_Mem_Read(&I2C_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, I2Cx_FLAG_TIMEOUT);
    if (status != HAL_OK) {
        // 处理错误
    }
    while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY) {
        // 等待I2C准备好
    }
    while (HAL_I2C_IsDeviceReady(&I2C_Handle, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
    while (HAL_I2C_GetState(&I2C_Handle) != HAL_I2C_STATE_READY) {
        // 等待I2C准备好
    }
    return status;
}

void ODrive::MPU6050_DMP::MPU6050ReadTemp(short *tempData)
{
	uint8_t buf[2];
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2); 
	*tempData = (buf[0] << 8) | buf[1];
}
void ODrive::MPU6050_DMP::MPU6050ReadGyro(short *gyroData) 
{
    uint8_t buf[6];
	MPU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
	gyroData[0] = (buf[0] << 8) | buf[1];
	gyroData[1] = (buf[2] << 8) | buf[3];
	gyroData[2] = (buf[4] << 8) | buf[5];
}

void ODrive::MPU6050_DMP::MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
	MPU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
	accData[0] = (buf[0] << 8) | buf[1];
	accData[1] = (buf[2] << 8) | buf[3];
	accData[2] = (buf[4] << 8) | buf[5];
}

void ODrive::MPU6050_DMP::MPU6050_ReturnTemp(float*Temperature)
{
    	short temp3;
	
	uint8_t buf[2];
	MPU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);
	temp3= (buf[0] << 8) | buf[1];
	*Temperature=((double) (temp3 /340.0))+36.53;
}

void ODrive::MPU6050_DMP::MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x80);
	HAL_Delay(100);
	MPU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);
	MPU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);
	MPU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);
	MPU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);
	MPU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);
	HAL_Delay(200);
}

void ODrive::MPU6050_DMP::MPU6050_ReadData(uint8_t reg_add,unsigned char*Read,uint8_t num)
{
   I2C_ReadRegister(MPU6050_ADDRESS,reg_add,num,Read);
}
void ODrive::MPU6050_DMP::MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat)
{
    I2C_WriteRegister(MPU6050_ADDRESS,reg_add,1,&reg_dat);
}

uint8_t ODrive::MPU6050_DMP::mpu_get_data(float *pitch,float *roll,float *yaw)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 
	if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more))return 1; 
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if(sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] /q30;	
		q1 = quat[1] /q30 ;
		q2 = quat[2] /q30;
		q3 = quat[3] /q30; 
		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	}else return 2;
	return 0;
}
}
 

