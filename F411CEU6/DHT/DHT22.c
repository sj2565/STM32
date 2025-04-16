/*
 * dht22.c
 *
 *  Created on: Mar 23, 2025
 *      Author: seojoon
 */
#include <stdio.h>
#include "dht22.h"
#include "stm32f4xx_hal.h"

#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_1

float temperature;
float humidity;

// 딜레이용 DWT
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
    	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
    printf("✔️ DWT 초기화 완료\n");
}
__STATIC_INLINE void DWT_Delay_us(volatile uint32_t microseconds) {
    uint32_t clk_cycle_start = DWT->CYCCNT;
    microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

// 핀 제어 함수
void DHT22_Set_Pin_Output(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);
    printf("📤 DHT22 핀: 출력모드 설정\n");
}

void DHT22_Set_Pin_Input(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);
    printf("📥 DHT22 핀: 입력모드 설정\n");
}

// 센서 초기화
void DHT22_Init(void) {
    DWT_Init();
    DHT22_Set_Pin_Output();
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
}

// 센서 데이터 읽기
uint8_t DHT22_Read(float *temperature, float *humidity) {
    uint8_t data[5] = {0};
    uint32_t timeout = 0;
    printf("📡 DHT22 센서 읽기 시작\n");

    // 1. MCU -> Start signal
    DHT22_Set_Pin_Output();
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET);
    DWT_Delay_us(1200); // 최소 1ms 이상
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
    DWT_Delay_us(30); // Pull-up 유지 시간
    DHT22_Set_Pin_Input();

    // 2. DHT22 -> 응답
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
        if (++timeout > 10000) {
        	printf("⛔ DHT22 응답 없음 - 초기 LOW 실패\n");
        	return 1;
        }
    }
    printf("✅ 응답1: LOW 감지됨\n");

    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_RESET) {
        if (++timeout > 10000) {
        	printf("⛔ DHT22 응답 없음 - HIGH 전환 실패\n");
        	return 2;
        }
    }
    printf("✅ 응답2: HIGH 감지됨\n");

    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
        if (++timeout > 10000) {
        	printf("⛔ DHT22 응답 없음 - 센서 전송 준비 실패\n");
        	return 3;
        }
    }
    printf("✅ 응답3: 센서 데이터 수신 준비 완료\n");

    // 3. 40bit 데이터 수신
    for (int i = 0; i < 40; i++) {
        // LOW 기다림
    	timeout = 0;
        while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_RESET) {
        	if (++timeout > 10000) {
                printf("⛔ 타임아웃: LOW 대기 실패 (%d)\n", i);
                return 1;
            }
        }

        DWT_Delay_us(40);  // 40us 후 HIGH 유지 여부로 판단
        if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
            data[i/8] |= (1 << (7 - (i%8)));
        }

        // HIGH 떨어질 때까지 대기
        timeout = 0;
        while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
            if (++timeout > 10000) {
            	printf("⛔ 타임아웃: HIGH 유지 실패 (%d)\n", i);
            	break;
            }
        }
    }

    // DHT22에서 수신한 5바이트 확인 디버깅 (data[0], [1] : 습도, data[2], [3] : 온도, data[4] : 체크섬
    //printf("D0=%d D1=%d D2=%d D3=%d D4=%d\r\n", data[0], data[1], data[2], data[3], data[4]);

    // 4. 체크섬 확인
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) return 4;
    printf("CheckSum = %d, Calc = %d\r\n", data[4], checksum);;

    // 5. 온습도 계산
    *temperature = ((data[2] & 0x7F) << 8 | data[3]) * 0.1f;
    *humidity = ((data[0] << 8) | data[1]) * 0.1f;
    if (data[2] & 0x80) *temperature *= -1;

    printf("온도: %.1f°C, 습도: %.1f%%\r\n", temperature, humidity);
    return 0; // 성공
}
