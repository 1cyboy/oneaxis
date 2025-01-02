#include <stdlib.h>
#include <functional>
#include "i2c.h"
#include "odrive_main.h"
#include "MPU6050.hpp"


#define I2C_Handle hi2c2
#define I2Cx_FLAG_TIMEOUT 1000

int MPU6050::I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len, unsigned char *data_ptr)
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

/*int MPU6050::I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len, unsigned char *data_ptr)
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

/*void MPU6050::MPU6050ReadTemp(short *tempData)
{

}
void MPU6050::MPU6050ReadGyro(short *gyroData) {
}

void MPU6050::MPU6050ReadAcc(short *accData);
{}
void MPU6050::MPU6050_ReturnTemp(float*Temperature);
{}
void MPU6050::MPU6050_Init(void);
{}
uint8_t MPU6050::MPU6050ReadID(void);
{}
void MPU6050::MPU6050_ReadData(uint8_t reg_add,unsigned char*Read,uint8_t num);
{}
void MPU6050::MPU6050_WriteReg(uint8_t reg_add,uint8_t reg_dat);
{}
*/
