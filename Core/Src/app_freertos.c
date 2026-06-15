/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "communicationStateMachine.h"
#include "lcd_hd44780_i2c.h"
#include "pid.h"
#include "motorEncoder.h"
#include "lineSensors_v2.h"
#include <stdio.h>
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float fVbat   = 0.0f;
float fVelDir = 0.0f;
float fVelEsq = 0.0f;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
RoboData_t roboData;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern osMutexId_t Mutex_SensoresHandle;
pid_data_type xPidMotorDir;
pid_data_type xPidMotorEsq;
uint16_t g_bufIR2[1] = {0};
#define TRACK_WIDTH_CM  13.0f
#define V_BASE_CM_S     15.0f
uint8_t ucIniciado = 0U;
pid_data_type xPidMotorDir;
pid_data_type xPidMotorEsq;

#define TRACK_WIDTH_CM  13.0f
#define V_BASE_CM_S     15.0f


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern osMutexId_t Mutex_SensoresHandle;
// Caso o compilador reclame da variável "roboData" (se ela foi instanciada na main.c):
// extern RoboData roboData;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Função chamada automaticamente pelo hardware (Interrupção) quando o Bumper é pressionado
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Switch_Fr_Pin) {
        HAL_GPIO_WritePin(LED_R_PWM_GPIO_Port, LED_R_PWM_Pin, GPIO_PIN_SET);
        roboData.flag_colisao = 1;
    }
}

void task_Controle(void *argument) {
    static uint16_t contador_buzzer = 0;
    static uint8_t  repeticao_buzzer = 0;

    for (;;) {
        uint8_t estado_emergencia = 0;
        osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
        estado_emergencia = roboData.flag_colisao;
        osMutexRelease(Mutex_SensoresHandle);

        if (estado_emergencia == 1) {
            vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT,
                MOTORENCODER_DIRECTION_STOP, 0.0f);
            vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT,
                MOTORENCODER_DIRECTION_STOP, 0.0f);

            if (repeticao_buzzer < 8) {
                contador_buzzer++;
                if (contador_buzzer < 25) {
                    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 500);
                } else if (contador_buzzer < 50) {
                    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
                } else {
                    contador_buzzer = 0;
                    repeticao_buzzer++;
                }
            }
            osDelay(10);

        } else {
            repeticao_buzzer = 0;
            contador_buzzer  = 0;
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);

            if (ucIniciado == 0U) {
                vMotorEncoderInitMotors(
                    Motor_Dir_IN1_GPIO_Port, Motor_Dir_IN1_Pin,
                    Motor_Dir_IN2_GPIO_Port, Motor_Dir_IN2_Pin,
                    Motor_Esq_IN3_GPIO_Port, Motor_Esq_IN3_Pin,
                    Motor_Esq_IN4_GPIO_Port, Motor_Esq_IN4_Pin,
                    &htim1, TIM_CHANNEL_2,
                    &htim1, TIM_CHANNEL_1
                );
                vMotorEncoderInitEncoders(
                    &htim17, TIM_CHANNEL_1,
                    &htim16, TIM_CHANNEL_1
                );
                vPidInit(&xPidMotorDir, 0.05f, 0.1f, 0.0f, 1.0f, 1.0f);
                vPidInit(&xPidMotorEsq, 0.05f, 0.1f, 0.0f, 1.0f, 1.0f);
                ucIniciado = 1U;
            }

            float fPosicao = 0.0f;
            float fOmega   = 1.0f * fPosicao;
            float fVdirRef = V_BASE_CM_S + fOmega * (TRACK_WIDTH_CM / 2.0f);
            float fVesqRef = V_BASE_CM_S - fOmega * (TRACK_WIDTH_CM / 2.0f);

            float fVdirMed = fMotorEncoderReadVelocity(MOTORENCODER_MOTOR_RIGHT);
            float fVesqMed = fMotorEncoderReadVelocity(MOTORENCODER_MOTOR_LEFT);

            float fPwmDir = fPidUpdateData(&xPidMotorDir, fVdirRef, fVdirMed);
            float fPwmEsq = fPidUpdateData(&xPidMotorEsq, fVesqRef, fVesqMed);

            vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT,
                MOTORENCODER_DIRECTION_BACKWARD, fPwmDir);
            vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT,
                MOTORENCODER_DIRECTION_FORWARD, fPwmEsq);

            // Atualiza velocidades para o display
            osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
            fVelDir = fVdirMed;
            fVelEsq = fVesqMed;
            osMutexRelease(Mutex_SensoresHandle);

            osDelay(10);
        }
    }
}

void task_Sensores(void *argument) {
    for (;;) {
        if (HAL_GPIO_ReadPin(BT_Enter_GPIO_Port, BT_Enter_Pin) == GPIO_PIN_RESET) {
            osDelay(30);
            if (HAL_GPIO_ReadPin(BT_Enter_GPIO_Port, BT_Enter_Pin) == GPIO_PIN_RESET) {
                osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
                if (roboData.flag_colisao == 1) {
                    roboData.flag_colisao = 0;
                    HAL_GPIO_WritePin(LED_R_PWM_GPIO_Port, LED_R_PWM_Pin, GPIO_PIN_RESET);
                }
                osMutexRelease(Mutex_SensoresHandle);
                while (HAL_GPIO_ReadPin(BT_Enter_GPIO_Port, BT_Enter_Pin) == GPIO_PIN_RESET) {
                    osDelay(10);
                }
            }
        }
        osDelay(20);
    }
}

void task_Bluetooth(void *argument) {
    for (;;) {
        osDelay(100);
    }
}

void task_Odometria(void *argument) {
    for (;;) {
        osDelay(50);
    }
}

void task_LvBateria(void *argument) {
    extern ADC_HandleTypeDef hadc2;

    for (;;) {
        HAL_ADC_Stop_DMA(&hadc2);

        HAL_ADC_Start(&hadc2);
        if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
            uint32_t raw = HAL_ADC_GetValue(&hadc2);
            float fVpa7    = ((float)raw / 4095.0f) * 3.3f;
            float fVbatNova = fVpa7 * 2.0f;

            osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
            fVbat = fVbatNova;
            osMutexRelease(Mutex_SensoresHandle);
        }
        HAL_ADC_Stop(&hadc2);

        // Reinicia DMA do IR2
        HAL_ADC_Start_DMA(&hadc2, (uint32_t*)g_bufIR2, 1U);

        osDelay(1000);
    }
}

void floatToStr(char *buf, float val, int decimais) {
    int inteiro = (int)val;
    int frac    = (int)((val - inteiro) * (decimais == 1 ? 10.0f : 100.0f));
    if (frac < 0) frac = -frac;
    if (decimais == 1)
        sprintf(buf, "%d.%01d", inteiro, frac);
    else
        sprintf(buf, "%d.%02d", inteiro, frac);
}

void task_display(void *argument) {
    extern I2C_HandleTypeDef hi2c2;
    char buf[17];
    char sBat[8], sDir[7], sEsq[7];

    lcdInit(&hi2c2, 0x27, 2, 16);
    lcdDisplayOn();
    lcdBacklightOn();
    lcdDisplayClear();

    for (;;) {
        float vbat, vdir, vesq;

    }
}
/* USER CODE END Application */

