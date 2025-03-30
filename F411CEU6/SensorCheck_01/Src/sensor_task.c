/*
 * sensor_task.c
 *
 *  Created on: Mar 28, 2025
 *      Author: seojoon
 */

#include "sensor_task.h"
#include "mpu6050.h"
#include "dht22.h"
#include "mq135.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "tim.h"

extern I2C_HandleTypeDef hi2c1;
extern volatile uint8_t mpu6050_interrupt_flag;
extern float temperature, humidity; // 온습도 센서 변수 선언
MPU6050_t mpu_data;

uint32_t lastPrintTick; // 출력 타이밍 기준점

void StartSensorTask(void const * argument)
{

	//osDelay(500);

	DHT22_Init();

    MPU6050_Init(&hi2c1);
    MPU6050_Set_Interrupt(&hi2c1);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // R (PA5)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // G (PA6)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // B (PA7)

    printf("센서 데이터 출력 시작! \r\n");

    for (;;) {
        // 충격 발생 시 즉시 이벤트 출력 (인터럽트)
        if (mpu6050_interrupt_flag) { // 인터럽트가 발생했을 때만 실행
            mpu6050_interrupt_flag = 0; // 플래그 초기화
            MPU6050_ProcessEvent(&hi2c1, &mpu_data);  // &hi2c1 인자 추가
        }

        // 주기적으로 센서 데이터 출력 (폴링)
        if ((HAL_GetTick() - lastPrintTick) >= 2000) {
        	lastPrintTick = HAL_GetTick();

        	// 온습도 출력
        	if (DHT22_Read(&temperature, &humidity) == 0) {
        		printf("온도: %.1f°C, 습도: %.1f%%\r\n", temperature, humidity);
        	} else {
        		printf("DHT22 실패\r\n");
        	}

        	//osDelay(1);  // RTOS 스케줄러 정리 (짧은 안정화)

        	// 각속도, 기울기 출력
            MPU6050_ReadAll(&hi2c1, &mpu_data);
            printf("aX: %d, aY: %d, aZ: %d | gX: %d, gY: %d, gZ: %d\r\n",
                   mpu_data.aX, mpu_data.aY, mpu_data.aZ,
                   mpu_data.gX, mpu_data.gY, mpu_data.gZ);

            float ppm = MQ135_ReadPPM();
            printf("공기질(PPM): %.1f\r\n", ppm);
            Update_Air_LED_RGB(ppm);
        }
        osDelay(10);  // CPU 점유율 방지 + 이벤트 반응 빠르게 유지
    }
}

