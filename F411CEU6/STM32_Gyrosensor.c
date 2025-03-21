/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR       (0x68 << 1) // 7-bit 주소(0x68)를 왼쪽 시프트하여 8-bit 주소로
#define WHO_AM_I_REG       0x75
#define PWR_MGMT_1         0x6B
#define INT_ENABLE         0x38
#define INT_STATUS         0x3A
#define ACCEL_XOUT_H       0x3B 	// 가속도
#define GYRO_XOUT_H		   0x43 	// 각도

#define SHOCK_THRESHOLD    10000  	// 충격 감지 임계값
#define FALL_ANGLE_THRESHOLD 45.0 	// 넘어짐 감지 임계 각도
#define ROTATION_THRESHOLD 20000  	// 빠른 회전 감지 임계값

#define FILTER_SIZE 5  			    // 최근 5개의 데이터를 평균 내기(이동 평균 필터)

#define POLLING_DELAY_MS 50         // 폴링방식의 센서값 (원본) 출력 주기, ex: 50ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t mpu6050_interrupt_flag = 0; // 인터럽트 발생 플래그

int16_t Accel_X, Accel_Y, Accel_Z;
int16_t Gyro_X, Gyro_Y, Gyro_Z;

// 이동 평균 필터 버퍼
int16_t aX_Buffer[FILTER_SIZE] = {0};
int16_t aY_Buffer[FILTER_SIZE] = {0};
int16_t aZ_Buffer[FILTER_SIZE] = {0};
int buffer_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MPU6050_Init(void);
void MPU6050_Read_Accel_Gyro(void);
void MPU6050_Set_Interrupt(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 이동 평균 필터 함수
int16_t Moving_Average_Filter(int16_t *buffer, int16_t new_value)
{
    int sum = 0;

    // 새로운 데이터를 버퍼에 추가
    buffer[buffer_index] = new_value;

    // 버퍼 내 모든 값들의 합을 계산
    for (int i = 0; i < FILTER_SIZE; i++)
    {
        sum += buffer[i];
    }

    // 인덱스 업데이트 (순환 버퍼 방식)
    buffer_index = (buffer_index + 1) % FILTER_SIZE;

    // 평균값 반환
    return sum / FILTER_SIZE;
}

// A 폴링 전용 센서 읽기 함수 (원본, 머신러닝 학습용)
void Polling_Read_Sensor(void)
{
    uint8_t buffer[14];
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, buffer, 14, HAL_MAX_DELAY);

    Accel_X = (int16_t)(buffer[0] << 8 | buffer[1]);
    Accel_Y = (int16_t)(buffer[2] << 8 | buffer[3]);
    Accel_Z = (int16_t)(buffer[4] << 8 | buffer[5]);

    Gyro_X  = (int16_t)(buffer[8] << 8 | buffer[9]);
    Gyro_Y  = (int16_t)(buffer[10] << 8 | buffer[11]);
    Gyro_Z  = (int16_t)(buffer[12] << 8 | buffer[13]);

    // 폴링으로 읽은 원본값 바로 출력 (머신러닝 학습용)
    printf("%d,%d,%d,%d,%d,%d\r\n", Accel_X, Accel_Y, Accel_Z, Gyro_X, Gyro_Y, Gyro_Z);
}

// B 인터럽트 발생 시 센서 데이터 처리 (필터 + 기울기 계산 + 이벤트 감지)
void Interrupt_Process_Sensor(void) {

	// 1) INT_STATUS 확인
	uint8_t status;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, INT_STATUS, 1, &status, 1, HAL_MAX_DELAY);

	// Motion Detection(충격)인지 확인
	if (status & 0x40)
	{
	    printf("EVENT : 충격 감지!\r\n");
	}

	// 2) 추가로 센서 값을 읽어 넘어짐, 회전 등 소프트웨어 이벤트 감지
	// (굳이 안 읽어도 되지만, 이벤트 상황의 값만 별도로 보고 싶다면 읽을 수 있음)
	uint8_t buffer[14];
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, buffer, 14, HAL_MAX_DELAY);

	int16_t aX = (int16_t)(buffer[0] << 8 | buffer[1]);
	int16_t aY = (int16_t)(buffer[2] << 8 | buffer[3]);
	int16_t aZ = (int16_t)(buffer[4] << 8 | buffer[5]);

	int16_t gX = (int16_t)(buffer[8] << 8 | buffer[9]);
	int16_t gY = (int16_t)(buffer[10] << 8 | buffer[11]);
	int16_t gZ = (int16_t)(buffer[12] << 8 | buffer[13]);


	//static int16_t prev_Accel_X = 0, prev_Accel_Y = 0, prev_Accel_Z = 0;

	// 이동 평균 필터 적용(가속도 노이즈 제거)
	int16_t filtered_Accel_X = Moving_Average_Filter(aX_Buffer, aX);
	int16_t filtered_Accel_Y = Moving_Average_Filter(aY_Buffer, aY);
	int16_t filtered_Accel_Z = Moving_Average_Filter(aZ_Buffer, aZ);

	// 이벤트 시점의 가속도/자이로로 넘어짐 판단
	float ax2 = filtered_Accel_X / 16384.0f;
	float ay2 = filtered_Accel_Y / 16384.0f;
	float az2 = filtered_Accel_Z / 16384.0f;

	float angle_X = atan2f(ax2, az2) * 180.0f / M_PI;
	float angle_Y = atan2f(ay2, az2) * 180.0f / M_PI;

	if (fabs(angle_X) > FALL_ANGLE_THRESHOLD || fabs(angle_Y) > FALL_ANGLE_THRESHOLD)
	{
	    printf("EVENT : 넘어짐 감지! X=%.2f°, Y=%.2f°\r\n", angle_X, angle_Y);
	}

	// 각도에는 따로 필터 적용 x
	if (abs(gX) > ROTATION_THRESHOLD || abs(gY) > ROTATION_THRESHOLD || abs(gZ) > ROTATION_THRESHOLD)
	{
	    printf("EVENT : 빠른 회전 감지! Gyro X=%d, Y=%d, Z=%d\r\n", gX, gY, gZ);
	}

    // 센서 데이터 지속적으로 수집
    //printf("%d,%d,%d,%d,%d,%d\r\n", filtered_Accel_X, filtered_Accel_Y, filtered_Accel_Z, Gyro_X, Gyro_Y, Gyro_Z);

    // 충격감지는 하드웨어 인터럽트 방식으로 가능
    /** 충격 감지 (급격한 가속도 변화)
    int16_t delta_X = abs(Accel_X - prev_Accel_X);
    int16_t delta_Y = abs(Accel_Y - prev_Accel_Y);
    int16_t delta_Z = abs(Accel_Z - prev_Accel_Z);

    if (delta_X > SHOCK_THRESHOLD || delta_Y > SHOCK_THRESHOLD || delta_Z > SHOCK_THRESHOLD) {
        printf("충격 감지! ΔX=%d, ΔY=%d, ΔZ=%d\r\n", delta_X, delta_Y, delta_Z);
    } **/

    // 다음 비교를 위해 현재 값을 저장
    //prev_Accel_X = filtered_Accel_X;
    //prev_Accel_Y = filtered_Accel_Y;
    //prev_Accel_Z = filtered_Accel_Z;
}


// 인터럽트 콜백 함수 (MPU6050 데이터 준비 완료 신호 수신 시 실행)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

{
  if (GPIO_Pin == GPIO_PIN_10) // PA10
  {
	mpu6050_interrupt_flag = 1; // 인터럽트 발생 플래그 설정
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // MPU6050 초기화
  MPU6050_Init();
  printf("MPU6050 준비 완료\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // 폴링 방식으로 주기적으로 센서를 읽어 원본 출력
	  Polling_Read_Sensor();

	  if (mpu6050_interrupt_flag) // 인터럽트가 발생했을 때만 실행
	  {
		  mpu6050_interrupt_flag = 0;  // 플래그 초기화
		  Interrupt_Process_Sensor(); // 이벤트 발생 시점의 데이터로 충격/넘어짐/회전 감지
	  }
	  HAL_Delay(POLLING_DELAY_MS); // 너무 빠른 출력 방지용
    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

void MPU6050_Set_Interrupt(void)
{
    uint8_t data;

    // 모션 감지 활성화 (USER_CTRL) -> 0x6A
    //data = 0x20;
    //HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6A, 1, &data, 1, HAL_MAX_DELAY);

    // 가속도 충격 감지 설정 (2g 이상 감지)
    data = 0x10; // 2g
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1F, 1, &data, 1, HAL_MAX_DELAY);

    // 충격 지속 시간 설정 (20ms)
    data = 0x01; // 최소 감지 시간
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x20, 1, &data, 1, HAL_MAX_DELAY);

    // 저역 필터 설정 (필터 적용하여 노이즈 줄이기)
    data = 0x03; // 44Hz 대역 통과 필터 적용
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1D, 1, &data, 1, HAL_MAX_DELAY);

    // 충격 감지 인터럽트 활성화
    data = 0x40;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_ENABLE, 1, &data, 1, HAL_MAX_DELAY);
}

// 센서 초기화
void MPU6050_Init(void)
{
  uint8_t check, data;

  // WHO_AM_I 레지스터 읽기 (0x68이 정상)
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_MAX_DELAY);
  if (check != 0x68)
  {
    printf("MPU6050 연결 안됨! (WHO_AM_I=%02X)\r\n", check);
    return;
  }

  // PWR_MGMT_1 레지스터: 0으로 설정 (슬립 모드 해제)
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);

  // INT_ENABLE 레지스터: 데이터 준비 완료(D0 bit) 인터럽트 활성화
  data = 0x01;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, INT_ENABLE, 1, &data, 1, HAL_MAX_DELAY);

  // 추가된 부분: 충격 감지 인터럽트 활성화
  MPU6050_Set_Interrupt();

  printf("MPU6050 성공\r\n");
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// _write() 함수를 재정의하여 printf를 UART로 출력
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
