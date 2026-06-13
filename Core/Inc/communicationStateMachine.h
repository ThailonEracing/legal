/**
 ******************************************************************************
 * @file    : communicationStateMachine.h
 * @brief   : Cabeçalho da máquina de estados de comunicação. Contém as enumerações dos estados da comunicação, constantes de limite e os protótipos das funções de processamento serial.
 * @author  : Diego Xavier e Thailon Mendes
 * @date    : 25-Mar-2026
 * @version : 1.12
 ******************************************************************************
 */

/* communicationStateMachine.h */

#ifndef COMMUNICATION_STATE_MACHINE_H
#define COMMUNICATION_STATE_MACHINE_H

#include "main.h"

// Enumeração dos Estados do Robô
typedef enum {
    ESTADO_PARADO = 0,         // Robô aguarda comando para iniciar
    ESTADO_SEGUE_LINHA,        // Malha fechada (PID) a funcionar
    ESTADO_OBSTACULO,          // Bumper bateu ou Ultrassónico detetou obstáculo
    ESTADO_FIM_PERCURSO,       // Sensores detetaram a marca de fim (cruzamento total)
    ESTADO_MANUAL              // Controlo direto via Bluetooth (App)
} RoboState_t;

// Estrutura para partilhar as variáveis do robô (Substitui as variáveis da máquina de café)
typedef struct {
    RoboState_t estado_atual;
    uint16_t adc_sensores_linha[5]; // Vetor do DMA com os 5 sensores
    float velocidade_esq;
    float velocidade_dir;
    float erro_pid;
    uint8_t flag_colisao;
} RoboData_t;

// Protótipos das Funções
void StateMachine_Init(RoboData_t *roboData);
void StateMachine_Update(RoboData_t *roboData);

#endif /* COMMUNICATION_STATE_MACHINE_H */
