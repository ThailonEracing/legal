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
#include "battery.h"
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
RoboData_t roboData;            // Variável global com os "sinais vitais" do robô
extern I2C_HandleTypeDef hi2c2; // Para podermos usar o Display no I2C2
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern osMutexId_t Mutex_SensoresHandle;
extern osMutexId_t Mutex_BatteryHandle;

pid_data_type xPidMotorDir;
pid_data_type xPidMotorEsq;

#define TRACK_WIDTH_CM  13.0f
#define V_BASE_CM_S     15.0f
uint8_t ucIniciado = 0U;

// --- VARI�?VEIS GLOBAIS DA BATERIA E ODOMETRIA ---
uint16_t gu16BateriaPorcentagem = 0U;
float gfVdirReal = 0.0f;
float gfVesqReal = 0.0f;
float gfDistanciaDireitaCM = 0.0f;
float gfDistanciaEsquerdaCM = 0.0f;

float fPosicaoLinha = 0.0f;   // �? posição da linha: -1.0 a +1.0
uint8_t ucSensoresIniciado = 0U;

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
        // --- ESTADO DE EMERGÊNCIA (OBST�?CULO) ---

        // 1. Para os motores imediatamente (0% de Duty Cycle)
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

        // 2. Aciona o Buzzer por um tempo limitado a um numero de repetições (apita a cada ~250ms)
        if (repeticao_buzzer < 8) {
            contador_buzzer++;
            if (contador_buzzer < 25) { // 25 * 10ms = 250ms ligado
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 1000);
            } else if (contador_buzzer < 50) { // 250ms desligado
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
            } else {
                contador_buzzer = 0; // Reinicia o ciclo
                repeticao_buzzer ++;
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

            // PID motor direito: erro em cm/s → PWM [0.0, 1.0]
            vPidInit(&xPidMotorDir,  0.05f, 0.1f, 0.0f, 1.0f, 1.0f);
            // PID motor esquerdo
            vPidInit(&xPidMotorEsq,  0.05f, 0.1f, 0.0f, 1.0f, 1.0f);

            ucIniciado = 1U;
        }

        float fPosicao = 0.0f;
        osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
        fPosicao = fPosicaoLinha;
        osMutexRelease(Mutex_SensoresHandle);

        // 2. Velocidade angular proporcional ao erro de posição
        float fOmega = 2.0f * fPosicao;  // rad/s

        // 3. Modelo cinemático diferencial → setpoints de velocidade
        float fVdirRef = V_BASE_CM_S + fOmega * (TRACK_WIDTH_CM / 2.0f);
        float fVesqRef = V_BASE_CM_S - fOmega * (TRACK_WIDTH_CM / 2.0f);

        // 4. Lê velocidades reais dos encoders (cm/s)
        float fVdirMed = fMotorEncoderReadVelocity(MOTORENCODER_MOTOR_RIGHT);
        float fVesqMed = fMotorEncoderReadVelocity(MOTORENCODER_MOTOR_LEFT);

        // 5. PIDs de velocidade → PWM [0.0, 1.0]
        float fPwmDir = fPidUpdateData(&xPidMotorDir, 2, fVdirMed);
        float fPwmEsq = fPidUpdateData(&xPidMotorEsq, 2, fVesqMed);

        // 6. Aplica nos motores
        vMotorEncoderControlMotor(MOTORENCODER_MOTOR_RIGHT,
            MOTORENCODER_DIRECTION_BACKWARD, fPwmDir);
        vMotorEncoderControlMotor(MOTORENCODER_MOTOR_LEFT,
            MOTORENCODER_DIRECTION_FORWARD, fPwmEsq);
        osDelay(30);
    }
  }
}

void task_Sensores(void *argument) {
  for(;;) {
    // Verifica se o botão Enter (PC5) foi pressionado (Pressionado = RESET)
    if (HAL_GPIO_ReadPin(BT_Enter_GPIO_Port, BT_Enter_Pin) == GPIO_PIN_RESET) {
        osDelay(30); // Debounce simples

        if (HAL_GPIO_ReadPin(BT_Enter_GPIO_Port, BT_Enter_Pin) == GPIO_PIN_RESET) {
            // Acesso protegido para alterar a flag
            osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
            if (roboData.flag_colisao == 1) {
                roboData.flag_colisao = 0; // Limpa a flag (Destrava o robô)

                // Apaga o LED Vermelho de indicação de falha
                HAL_GPIO_WritePin(LED_R_PWM_GPIO_Port, LED_R_PWM_Pin, GPIO_PIN_RESET);
            }
            osMutexRelease(Mutex_SensoresHandle);

            // Aguarda o botão ser solto
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
    osDelay(100); // Atraso de 100ms (10Hz)
  }
}

void task_Odometria(void *argument) {
  const float fDeltaT = 0.05f;

  // Fator de suavização (Alfa): quanto menor, mais estável fica o valor,
  // mas demora um pouquinho mais para subir. 0.3f é o equilíbrio perfeito.
  const float fAlfa = 0.3f;

  for(;;) {
    // 1. Coleta a velocidade instantânea bruta da biblioteca
    float fVdirBruta = fMotorEncoderReadVelocity(MOTORENCODER_MOTOR_RIGHT);
    float fVesqBruta = fMotorEncoderReadVelocity(MOTORENCODER_MOTOR_LEFT);

    // 2. Filtro de pulo (Ignora zeros falsos da biblioteca)
    if (fVdirBruta == 0.0f && gfVdirReal > 2.0f) fVdirBruta = gfVdirReal;
    if (fVesqBruta == 0.0f && gfVesqReal > 2.0f) fVesqBruta = gfVesqReal;

    // 3. FILTRO DIGITAL PASSA-BAIXAS (Suaviza os picos de 100 cm/s)
    // Novo Valor = (Alfa * Valor Bruto Atual) + ((1 - Alfa) * Valor Antigo)
    gfVdirReal = (fAlfa * fVdirBruta) + ((1.0f - fAlfa) * gfVdirReal);
    gfVesqReal = (fAlfa * fVesqBruta) + ((1.0f - fAlfa) * gfVesqReal);

    // 4. Integração numérica para distância (CM) usa o valor já filtrado
    gfDistanciaDireitaCM  += (gfVdirReal * fDeltaT);
    gfDistanciaEsquerdaCM += (gfVesqReal * fDeltaT);

    osDelay(50);
  }
}

void task_LvBateria(void *argument) {
  /* Inicializa o ADC com DMA, AmpOp e os limites da bateria */
  vBatteryInit();

  for(;;) {
    // 1. Pega o valor calculado (em porcentagem) pela sua biblioteca
    uint16_t capBateria = usBatteryGetCharge();

    // 2. Guarda na variável global de forma protegida pelo Mutex dedicado
    if (Mutex_BatteryHandle != NULL) {
      osMutexAcquire(Mutex_BatteryHandle, osWaitForever);
      gu16BateriaPorcentagem = capBateria;
      osMutexRelease(Mutex_BatteryHandle);
    }

    osDelay(500); // Executa a cada 500ms
  }
}

void task_display(void *argument)
{
  /* USER CODE BEGIN testLCDTaskFunction */
  unsigned char ucLCDLine1Buff[17], ucLCDLine2Buff[17];
  uint16_t bat_local = 0;

  // Variáveis para converter o float em inteiro multiplicado por 10
  int32_t i32VesqMultiplicado = 0;
  int32_t i32VdirMultiplicado = 0;

  // Inicializa o display LCD
  lcdInit(&hi2c2, 0x27, 2, 16);

  /* Infinite loop */
  for(;;)
  {
    osDelay(500); // Atualiza a tela a cada 500ms

    // ---- 1. LEITURA PROTEGIDA DA BATERIA ----
    if (Mutex_BatteryHandle != NULL) {
      osMutexAcquire(Mutex_BatteryHandle, osWaitForever);
      bat_local = gu16BateriaPorcentagem;
      osMutexRelease(Mutex_BatteryHandle);
    }

    // ---- 2. TRATAMENTO MANUAL DO FLOAT (PONTO FIXO) ----
    // Exemplo: se gfVesqReal for 15.42f, v_multiplicado vira 154
    i32VesqMultiplicado = (int32_t)(gfVesqReal * 10.0f);
    i32VdirMultiplicado = (int32_t)(gfVdirReal * 10.0f);

    // Proteção contra valores negativos (caso a roda gire para trás)
    if (i32VesqMultiplicado < 0) i32VesqMultiplicado = 0;
    if (i32VdirMultiplicado < 0) i32VdirMultiplicado = 0;

    // ---- 3. ATUALIZA A BATERIA (Linha 0, Coluna 11) ----
    lcdSetCursorPosition(11, 0);
    sprintf((char*)ucLCDLine1Buff, "%3d%%", bat_local);
    lcdPrintStr(ucLCDLine1Buff, 4);

    // ---- 4. ATUALIZA OS ENCODERS USANDO APENAS INTEIROS (%d) ----
    // i32VesqMultiplicado / 10  -> Pega a parte inteira (ex: 154 / 10 = 15)
    // i32VesqMultiplicado % 10  -> Pega o resto / décimo (ex: 154 % 10 = 4)
    lcdSetCursorPosition(0, 1);
    sprintf((char*)ucLCDLine2Buff, "E:%2d.%1d  D:%2d.%1d ",
            (int)(i32VesqMultiplicado / 10), (int)(i32VesqMultiplicado % 10),
            (int)(i32VdirMultiplicado / 10), (int)(i32VdirMultiplicado % 10));

    lcdPrintStr(ucLCDLine2Buff, 16);
  }
  /* USER CODE END testLCDTaskFunction */
}
void task_Linhas(void *argument) {

    // Init da lib — uma única vez antes do loop
    vLineSensors_v2_Init(
        LINESENSORS_ADC_1, LINESENSORS_RANK_1,   // IR1 → LEFT
        LINESENSORS_ADC_2, LINESENSORS_RANK_1,   // IR2 → CENTER LEFT
        LINESENSORS_ADC_3, LINESENSORS_RANK_1,   // IR3 → CENTER
        LINESENSORS_ADC_4, LINESENSORS_RANK_1,   // IR4 → CENTER RIGHT
        LINESENSORS_ADC_5, LINESENSORS_RANK_1    // IR5 → RIGHT
    );


    for (;;) {
        // Lê posição interpolada da linha: -1.0 (esq) a +1.0 (dir)
        float fPos = fLineSensors_v2_GetInterpolatedValue();

        // Armazena no mutex para a task_Controle consumir
        osMutexAcquire(Mutex_SensoresHandle, osWaitForever);
        fPosicaoLinha = fPos;
        osMutexRelease(Mutex_SensoresHandle);

        osDelay(50);
    }
}

/* USER CODE END Application */

