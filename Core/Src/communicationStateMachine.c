/**
 ******************************************************************************
 * @file    : communicationStateMachine.c
 * @brief   : IImplementação da máquina de estados para processamento do protocolo de comunicação serial. Interpreta os comandos recebidos via UART para consultar ou alterar dados de vendas e estoque.
 * @author  : Diego Xavier e Thailon Mendes
 * @date    : 25-Mar-2026
 * @version : 1.12
 ******************************************************************************
 */

/* communicationStateMachine.c */

#include "communicationStateMachine.h"
#include <stdio.h>

void StateMachine_Init(RoboData_t *roboData) {
    // Inicializa a estrutura do robô no estado seguro
    roboData->estado_atual = ESTADO_PARADO;
    roboData->velocidade_esq = 0.0f;
    roboData->velocidade_dir = 0.0f;
    roboData->erro_pid = 0.0f;
    roboData->flag_colisao = 0;

    for(int i = 0; i < 5; i++) {
        roboData->adc_sensores_linha[i] = 0;
    }
}

void StateMachine_Update(RoboData_t *roboData) {

    switch (roboData->estado_atual) {

        case ESTADO_PARADO:
            // O robô está parado, à espera de um comando de início.
            // Os motores devem estar com Duty Cycle a 0%.

            // Exemplo de Transição futura:
            // Se (Comando == INICIAR) -> roboData->estado_atual = ESTADO_SEGUE_LINHA;
            break;

        case ESTADO_SEGUE_LINHA:
            // 1. A Task do PID vai usar o erro calculado para ajustar as velocidades
            // roboData->velocidade_esq = Velocidade_Base + Ajuste_PID;
            // roboData->velocidade_dir = Velocidade_Base - Ajuste_PID;

            // 2. Transições de Segurança (Cumprindo os Requisitos):
            if (roboData->flag_colisao == 1) {
                roboData->estado_atual = ESTADO_OBSTACULO;
            }
            // else se (Sensores detetarem a marca de fim) -> ESTADO_FIM_PERCURSO
            break;

        case ESTADO_OBSTACULO:
            // Requisito de Segurança: O Bumper ou Ultrassónico detetou algo!
            // 1. Cortar motores (Velocidade = 0)
            // 2. Ligar o Buzzer

            // Transição:
            if (roboData->flag_colisao == 0) {
                roboData->estado_atual = ESTADO_SEGUE_LINHA; // Retoma o percurso se a pista ficar livre
                // Desligar o buzzer aqui
            }
            break;

        case ESTADO_FIM_PERCURSO:
            // O robô cruzou a linha de chegada.
            // 1. Parar os motores
            // 2. Atualizar o LCD com o tempo final
            break;

        case ESTADO_MANUAL:
            // Controlo direto via App do telemóvel (Bluetooth)
            // Ignora a leitura dos sensores de linha

            // Transição:
            // Se (Comando == MODO_AUTOMATICO) -> roboData->estado_atual = ESTADO_PARADO;
            break;

        default:
            roboData->estado_atual = ESTADO_PARADO;
            break;
    }
}
