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
float fVbat = 0.0f;
float fVelDir = 0.0f;
float fVelEsq = 0.0f;
uint8_t ucDisplayIniciado = 0U;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
RoboData_t roboData; // A variável global com os "sinais vitais" do robô
extern I2C_HandleTypeDef hi2c2; // Para podermos usar o Display no I2C2
RoboData_t roboData;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;  // �? adiciona
extern TIM_HandleTypeDef htim17;
extern osMutexId_t Mutex_SensoresHandle;

pid_data_type xPidMotorDir;
pid_data_type xPidMotorEsq;

#define TRACK_WIDTH_CM  13.0f
#define V_BASE_CM_S     15.0f

uint8_t ucIniciado = 0U;

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

    // M�?QUINA DE ESTADOS: Verifica se o robô bateu
    if (estado_emergencia == 1) {
        // Para os motores via biblioteca
        vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT,
            MOTORENCODER_DIRECTION_STOP, 0.0f);
        vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT,
            MOTORENCODER_DIRECTION_STOP, 0.0f);

        // Buzzer — mantém como estava
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
        contador_buzzer = 0;
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);

        // Init único antes do loop
        if (ucIniciado == 0U) {

            vMotorEncoderInitMotors(
                Motor_Dir_IN1_GPIO_Port, Motor_Dir_IN1_Pin,
                Motor_Dir_IN2_GPIO_Port, Motor_Dir_IN2_Pin,
                Motor_Esq_IN3_GPIO_Port, Motor_Esq_IN3_Pin,
                Motor_Esq_IN4_GPIO_Port, Motor_Esq_IN4_Pin,
                &htim1, TIM_CHANNEL_2,   // PWM direito
                &htim1, TIM_CHANNEL_1    // PWM esquerdo
            );

            vMotorEncoderInitEncoders(
                &htim17, TIM_CHANNEL_1,  // encoder direito
                &htim16, TIM_CHANNEL_1   // encoder esquerdo
            );

//            vLineSensors_v2_Init(
//                LINESENSORS_ADC_1, LINESENSORS_RANK_1,
//                LINESENSORS_ADC_2, LINESENSORS_RANK_1,
//                LINESENSORS_ADC_3, LINESENSORS_RANK_1,
//                LINESENSORS_ADC_4, LINESENSORS_RANK_1,
//                LINESENSORS_ADC_5, LINESENSORS_RANK_1
//            );

            // PID motor direito: erro em cm/s → PWM [0.0, 1.0]
            vPidInit(&xPidMotorDir,  0.05f, 0.1f, 0.0f, 1.0f, 1.0f);
            // PID motor esquerdo
            vPidInit(&xPidMotorEsq,  0.05f, 0.1f, 0.0f, 1.0f, 1.0f);

            ucIniciado = 1U;
        }

        // 1. Posição da linha: -1.0 (esq) a +1.0 (dir)
        float fPosicao = 0;

        // 2. Velocidade angular proporcional ao erro de posição
        //    Kp_dir = 5.0 → ajustar experimentalmente
        float fOmega = 1.0f * fPosicao;  // rad/s

        // 3. Modelo cinemático diferencial → setpoints de velocidade
        float fVdirRef = V_BASE_CM_S + fOmega * (TRACK_WIDTH_CM / 2.0f);
        float fVesqRef = V_BASE_CM_S - fOmega * (TRACK_WIDTH_CM / 2.0f);

        // 4. Lê velocidades reais dos encoders (cm/s)
        float fVdirMed = fMotorEncoderReadVelocity(MOTORENCODER_MOTOR_RIGHT);
        float fVesqMed = fMotorEncoderReadVelocity(MOTORENCODER_MOTOR_LEFT);

        // 5. PIDs de velocidade → PWM [0.0, 1.0]
        float fPwmDir = fPidUpdateData(&xPidMotorDir, fVdirRef, fVdirMed);
        float fPwmEsq = fPidUpdateData(&xPidMotorEsq, fVesqRef, fVesqMed);

        // 6. Aplica nos motores
        vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT,
            MOTORENCODER_DIRECTION_BACKWARD, fPwmDir);
        vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT,
            MOTORENCODER_DIRECTION_FORWARD, fPwmEsq);
    }
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
void task_LvBateria(void *argument) {
    extern ADC_HandleTypeDef hadc2;

    for (;;) {
        // Para o DMA do IR2 temporariamente
        HAL_ADC_Stop_DMA(&hadc2);

        // Lê PA7 (bateria) por polling — ADC2 canal IN4
        HAL_ADC_Start(&hadc2);
        if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK) {
            uint32_t raw = HAL_ADC_GetValue(&hadc2);

            // Oversampling ratio=128, right shift=3 → divisor efetivo = 128/8 = 16
            // Mas o HAL já aplica o shift, então raw já é o valor corrigido
            // 12 bits → Vpa7 = raw / 4095.0 * 3.3
            float fVpa7 = ((float)raw / 4095.0f) * 3.3f;
            float fVbatNova = fVpa7 * 2.0f;  // divisor R1=R2=3M3

            osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
            fVbat = fVbatNova;
            osMutexRelease(Mutex_SensoresHandle);
        }
        HAL_ADC_Stop(&hadc2);



        osDelay(1000);  // lê bateria a cada 1s
    }
}


void floatToStr(char *buf, float val, int decimais) {
    int inteiro = (int)val;
    int frac    = (int)((val - inteiro) * (decimais == 1 ? 10 : 100));
    if (frac < 0) frac = -frac;
    if (decimais == 1)
        sprintf(buf, "%d.%01d", inteiro, frac);
    else
        sprintf(buf, "%d.%02d", inteiro, frac);
}

void task_display(void *argument) {
    extern I2C_HandleTypeDef hi2c2;
    char buf[17];

    // Init LCD — endereço 0x27 ou 0x3F dependendo do módulo
    lcdInit(&hi2c2, 0x27, 2, 16);
    lcdDisplayOn();
    lcdBacklightOn();
    lcdDisplayClear();

    for (;;) {
        float vbat, vdir, vesq;

        // Lê dados compartilhados
        osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
        vbat = fVbat;
        vdir = fVelDir;
        vesq = fVelEsq;
        osMutexRelease(Mutex_SensoresHandle);

        // Linha 0: tensão da bateria
        // Ex: "Bat:  7.40V     "
        lcdSetCursorPosition(0, 0);
        char sBat[8], sDir[7], sEsq[7];

        floatToStr(sBat, vbat, 2);   // "7.40"
        floatToStr(sDir, vdir, 1);   // "15.2"
        floatToStr(sEsq, vesq, 1);   // "14.8"

        lcdSetCursorPosition(0, 0);
        snprintf(buf, sizeof(buf), "Bat: %sV        ", sBat);
        lcdPrintStr((uint8_t*)buf, 16);

        lcdSetCursorPosition(1, 0);
        snprintf(buf, sizeof(buf), "D:%s E:%s    ", sDir, sEsq);
        lcdPrintStr((uint8_t*)buf, 16);

        osDelay(1000);  // atualiza a cada 1s
    }
}
/* USER CODE END Application */

