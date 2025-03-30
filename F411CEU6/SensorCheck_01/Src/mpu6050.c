/*
 * mpu6050.c
 *
 *  Created on: Mar 28, 2025
 *      Author: seojoon
 */

#include "mpu6050.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

volatile uint8_t mpu6050_interrupt_flag = 0;

// 센서 초기화 함수
void MPU6050_Init(I2C_HandleTypeDef* hi2c)
{
    uint8_t check, data;

    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x75, 1, &check, 1, HAL_MAX_DELAY);
    if (check != 0x68) {
        printf("MPU6050 연결 실패: ID=0x%X\r\n", check);
        return;
    }

    // 슬립모드 해제
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, &data, 1, HAL_MAX_DELAY);

    // 가속도 센서 범위 ±4g (0x08)
    data = 0x08;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &data, 1, HAL_MAX_DELAY);

    // 자이로 센서 범위 ±500°/s (0x08)
    data = 0x08;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &data, 1, HAL_MAX_DELAY);

    // 밑에 하드웨어 인터럽트 활성화 (충격 감지)
    MPU6050_Set_Interrupt(hi2c);

    printf("MPU6050 초기화 완료\r\n");
}

// 하드웨어 인터럽트
void MPU6050_Set_Interrupt(I2C_HandleTypeDef* hi2c)
{
    uint8_t data;

    // 인터럽트 핀 설정: 데이터 레디 시 INT 발생
    data = 0x01; // 2g
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x38, 1, &data, 1, HAL_MAX_DELAY);

    // 1. 모션 감지 임계값 (낮을수록 민감) -> MOT_THR
    data = 10; // 약 0.625g
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1F, 1, &data, 1, HAL_MAX_DELAY);

    // 2. 모션 지속시간 (ms 단위) → MOT_DUR
    data = 1;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x20, 1, &data, 1, HAL_MAX_DELAY);

    // 3. 저역통과필터 설정
    data = 0x03; // 44Hz
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1D, 1, &data, 1, HAL_MAX_DELAY);

    // 4. 모션 인터럽트 활성화
    data = 0x40; // Motion interrupt enable
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x38, 1, &data, 1, HAL_MAX_DELAY);

    // (선택) INT 핀 설정: 활성화 조건, 레벨 등 → INT_PIN_CFG(0x37)
    //data = 0x10; // Active high, push-pull
    //HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x37, 1, &data, 1, HAL_MAX_DELAY);
}

// 폴링 전용 센서 읽기 함수 (원본, 머신러닝 학습용)
void MPU6050_ReadAll(I2C_HandleTypeDef* hi2c, MPU6050_t* data)
{
    uint8_t buffer[14];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, 1, buffer, 14, HAL_MAX_DELAY);

    data->aX = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->aY = (int16_t)(buffer[2] << 8 | buffer[3]);
    data->aZ = (int16_t)(buffer[4] << 8 | buffer[5]);

    data->gX = (int16_t)(buffer[8] << 8 | buffer[9]);
    data->gY = (int16_t)(buffer[10] << 8 | buffer[11]);
    data->gZ = (int16_t)(buffer[12] << 8 | buffer[13]);
}

// 인터럽트 이벤트 발생 함수
void MPU6050_ProcessEvent(I2C_HandleTypeDef* hi2c, MPU6050_t* data) // hi2c 인자 추가
{
	// 1) INT_STATUS 확인
	uint8_t status;
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3A, 1, &status, 1, HAL_MAX_DELAY);

	// 충격인지 확인
	if (status & 0x40)
	{
		printf("EVENT : 충격 감지!\r\n");
	}
}

