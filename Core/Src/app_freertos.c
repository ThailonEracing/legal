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
#include <stdio.h>
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
RoboData_t roboData; // A variável global com os "sinais vitais" do robô
extern I2C_HandleTypeDef hi2c2; // Para podermos usar o Display no I2C2

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

        // Acende o LED (RED) da placa para provar que a interrupção disparou!
        HAL_GPIO_WritePin(LED_R_PWM_GPIO_Port, LED_R_PWM_Pin, GPIO_PIN_SET);

        // TRAVA o robô em estado de emergência
        roboData.flag_colisao = 1;
    }
}

void task_Controle(void *argument) {
  static uint16_t contador_buzzer = 0;
  static uint8_t repeticao_buzzer = 0;

  for(;;) {
    // Leitura segura da flag de colisão usando o Mutex
    uint8_t estado_emergencia = 0;
    osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
    estado_emergencia = roboData.flag_colisao;
    osMutexRelease(Mutex_SensoresHandle);

    // MÁQUINA DE ESTADOS: Verifica se o robô bateu
    if (estado_emergencia == 1) {
        // --- ESTADO DE EMERGÊNCIA (OBSTÁCULO) ---

        // 1. Para os motores imediatamente (0% de Duty Cycle)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

        // 2. Aciona o Buzzer por um tempo limitado a um numero de repetições (apita a cada ~250ms)
        if (repeticao_buzzer < 8) {
        	// O período do TIM8 está em 999. Vamos usar um duty de 500 (50%).
        	contador_buzzer++;
        	if (contador_buzzer < 25) { // 25 * 10ms = 250ms ligado
        		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 500);
        	} else if (contador_buzzer < 50) { // 250ms desligado
        		__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
        	} else {
        		contador_buzzer = 0; // Reinicia o ciclo
        		repeticao_buzzer ++;
        	}
        }


    } else {
        // --- ESTADO NORMAL ---
    	// Quando voltar ao estado normal, zerar a variavel de repeticao buzzer
    	repeticao_buzzer = 0;
        // Zera o contador do buzzer e garante que ele fique mudo
        contador_buzzer = 0;
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);

        // Liga os motores (80% de Duty Cycle)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 80);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 80);

        // Sentido de rotação: Para frente
        HAL_GPIO_WritePin(Motor_Dir_IN1_GPIO_Port, Motor_Dir_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_Dir_IN2_GPIO_Port, Motor_Dir_IN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Motor_Esq_IN3_GPIO_Port, Motor_Esq_IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(Motor_Esq_IN4_GPIO_Port, Motor_Esq_IN4_Pin, GPIO_PIN_RESET);
    }

    osDelay(10); // Atraso de 10ms (100Hz)
  }
}
void task_Sensores(void *argument) {
  for(;;) {

    // Verifica se o botão Enter (PC5) foi pressionado
    // (Configurado como Pull-Up no STM32CubeMX, então Pressionado = RESET)
    if (HAL_GPIO_ReadPin(BT_Enter_GPIO_Port, BT_Enter_Pin) == GPIO_PIN_RESET) {

        osDelay(30); // Debounce simples

        // Confirma se o botão continua pressionado
        if (HAL_GPIO_ReadPin(BT_Enter_GPIO_Port, BT_Enter_Pin) == GPIO_PIN_RESET) {

            // Acesso protegido para alterar a flag
            osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
            if (roboData.flag_colisao == 1) {
                roboData.flag_colisao = 0; // Limpa a flag (Destrava o robô)

                // Apaga o LED Vermelho (LED_R_PWM) de indicação de falha
                HAL_GPIO_WritePin(LED_R_PWM_GPIO_Port, LED_R_PWM_Pin, GPIO_PIN_RESET);
            }
            osMutexRelease(Mutex_SensoresHandle);

            // Aguarda o botão ser solto para evitar disparos múltiplos
            while (HAL_GPIO_ReadPin(BT_Enter_GPIO_Port, BT_Enter_Pin) == GPIO_PIN_RESET) {
                osDelay(10);
            }
        }
    }

    osDelay(20); // Executa a 50Hz
  }
}

void task_Bluetooth(void *argument) {
  /* Loop infinito da task de Bluetooth */
  for(;;) {
    // Aqui vai entrar o envio e receção de dados via UART (HC-05)

    osDelay(100); // Atraso de 100ms (10Hz)
  }
}

void task_Odometria(void *argument) {
  /* Loop infinito da task de Odometria */
  for(;;) {
    osDelay(50);
  }
}

/* USER CODE END Application */

