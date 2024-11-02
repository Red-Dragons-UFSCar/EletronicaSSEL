/*
 * NRF24L01.c
 *
 *  Created on: May 31, 2024
 *      Author: Mancooo
 */

//Neste código desligarei o auto acknologment
#include "NRF24L01.h"

// Bits de configuração
#define CFG_BIT_EN_CRC  0x3 //Posicção do Bit de CRC no registro de power do NRF
#define CFG_BIT_PWR_UP  0x1 //Posição do Bit de Power up no registro de power do NRF
#define CFG_BIT_PRIM_RX 0x0 //Posição do Bit de Rx/Tx no registro de power do NRF (0 = Tx, 1 = Rx) <- VALOR NO REGISTRO NAO POSIÇÃO

// Status bits
#define STATUS_BIT_RX_DR   0x6
#define STATUS_BIT_TX_DS   0x5
#define STATUS_BIT_MAX_RT  0x4
#define STATUS_BIT_RX_P_NO 0x1 // consists of bits 1-3
#define STATUS_BIT_TX_FULL 0x0

// FIFO status bits
#define FIFO_STATUS_BIT_TX_REUSE 0x6
#define FIFO_STATUS_BIT_TX_FULL  0x5
#define FIFO_STATUS_BIT_TX_EMPTY 0x4
#define FIFO_STATUS_BIT_RX_FULL  0x1
#define FIFO_STATUS_BIT_RX_EMPTY 0x0

/* Local globals */
SPI_HandleTypeDef *HSPI; //SPI a ser utilizada
GPIO_TypeDef *NRF_CSN_Port; //Porta do CS do NRF
uint16_t NRF_CSN_Pin; //Pino do CS do NRF
GPIO_TypeDef *NRF_CE_Port; //Porta do CE do NRF
uint16_t NRF_CE_Pin;//Pino do CE do NRF
uint32_t CPU_Freq = 0x00; //Variável para salvar a frequência da CPU
int current_mode = NRF_MODE_POWERDOWN; // Modo atual do NRF

/* Funções de Pinagem do NRF*/

//Pino CS em Alto (Dispositivo não está conversando com o uC)
void csn_set() {
  HAL_GPIO_WritePin(NRF_CSN_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

//Pino CS em Baixo (Dispositivo  está conversando com o uC)
void csn_reset() {
  HAL_GPIO_WritePin(NRF_CSN_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

//Pino CE em Alto (Dispositivo está em funcionamento,Tx ou Rx)
void ce_set() {
  HAL_GPIO_WritePin(NRF_CE_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

//Pino CE em Baixo (Dispositivo está em standby)
void ce_reset() {
  HAL_GPIO_WritePin(NRF_CE_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

//Ler pino CSN
uint8_t read_csn() {
  return HAL_GPIO_ReadPin(NRF_CSN_Port, NRF_CSN_Pin);
}

//Ler pino CE
uint8_t read_ce() {
  return HAL_GPIO_ReadPin(NRF_CE_Port, NRF_CE_Pin);
}

//Função de "Esperar" em us
void wait(uint64_t us) {
  uint32_t volatile cycles = CPU_Freq * us / 1000000; //Quantos ciclos de CPU deverão ser esperados para alcancar tal tempo
  uint32_t volatile current = 0; //Quantos se passaram
  while (current <= cycles) {
    current++;
  }
}

/* Funções Para enviar e receber do NRFL01 em "Baixo" Nível */

//Função para enviar um comando para o NRF
NRF_Status NRF_SendCommand(uint8_t cmd) {
  NRF_Status ret = NRF_OK;
  uint8_t status;

  csn_reset(); //Seleciona o dispositivo
  ret = (NRF_Status)HAL_SPI_TransmitReceive(HSPI, &cmd, &status, 1, NRF_SPI_TIMEOUT_DUR); //Envia o comando e retorna o status
  if (ret != NRF_OK) { //Verifica se o comando foi enviado corretamente
    return ret;
  }
  csn_set(); //Desseleciona o dispositivo

  return ret;
}

/* Função para enviar um comando de escrita*/
NRF_Status NRF_SendWriteCommand(uint8_t cmd, uint8_t *write, uint8_t length) {
  NRF_Status ret = NRF_OK;
  uint8_t status;

  csn_reset(); //Seleciona o dispositivo
  ret = (NRF_Status)HAL_SPI_TransmitReceive(HSPI, &cmd, &status, 1, NRF_SPI_TIMEOUT_DUR); //Manda pro NRF o comando de escrita
  if (ret != NRF_OK) { //Verifica se está ok para escrever
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    return ret;
  }
  ret = (NRF_Status)HAL_SPI_Transmit(HSPI, write, length, NRF_SPI_TIMEOUT_DUR); //Manda  o que será escrito no registro
  if (ret != NRF_OK) {//Verifica se o processo foi um sucesso
    return ret;
  }
  csn_set(); //Desseleciona o dispositivo

  return ret; //Retorna o Resultado
}

//Função para enviar um comando de leitura
NRF_Status NRF_SendReadCommand(uint8_t cmd, uint8_t *read, uint8_t length) {
  NRF_Status ret = NRF_OK;
  uint8_t status;

  csn_reset(); //Seleciona o dispositivo
  ret = (NRF_Status)HAL_SPI_TransmitReceive(HSPI, &cmd, &status, 1, NRF_SPI_TIMEOUT_DUR);//Manda pro NRF o comando de Leitura
  if(ret != NRF_OK) { //Verifica se o processo foi um sucesso
    return ret;
  }
  ret = (NRF_Status)HAL_SPI_Receive(HSPI, read, length, NRF_SPI_TIMEOUT_DUR);//Recebe o que se deseja ler do NRF
  if(ret != NRF_OK) {
    return ret;
  }
  csn_set();//Desseleciona o dispositvo

  return ret;//Retorna o resultado
}

/* Comandos de registros - Escrita */

/* Escreve no Registro do NRF
 * Param - Registro a ser escrito
 * Param - O que será escrito
 * Param - O tamanho (em bytes)
 *  							*/
NRF_Status NRF_WriteRegister(uint8_t reg, uint8_t *write, uint8_t length) {
  return NRF_SendWriteCommand(NRF_CMD_W_REGISTER | reg, write, length);
}

//A função a seguir não faz sentido existir kkkkk

/* Escreve em um byte da Memória do NRF
 * Param - Registro a ser escrito
 * Param - O que será escrito
 *  							*/
NRF_Status NRF_WriteRegisterByte(uint8_t reg, uint8_t byte) {
  uint8_t write = byte;
  return NRF_WriteRegister(reg, &write, 1);
}

/* Comandos de registros - Leitura */

/* Lê no Registro do NRF
 * Param - Registro a ser lido
 * Param - Ponteiro para variável onde será salvo
 * Param - O tamanho (em bytes)
 *  							*/
NRF_Status NRF_ReadRegister(uint8_t reg, uint8_t *read, uint8_t length) {
  return NRF_SendReadCommand(NRF_CMD_R_REGISTER | reg, read, length);
}

uint8_t NRF_ReadPacketLoss(){
	NRF_Status ret = NRF_OK;
	uint8_t Dados= 0;
	ret = NRF_SendReadCommand(NRF_REG_OBSERVE_TX, &Dados, 1);
	if(ret != NRF_OK){
		return 0;
	} else{
		Dados = Dados>>4; //10101111 -> 00001010
		return Dados;
	}
	return 0;
}



/* Lê em um byte da Memória do NRF
 * Param - Registro a ser lido
 * Param - O que será lido
 * Retorno - Dado Lido
 *  							*/
uint8_t NRF_ReadRegisterByte(uint8_t reg) {
  uint8_t read;
  NRF_SendReadCommand(NRF_CMD_R_REGISTER | reg, &read, 1);
  return read;
}

//Lê o Status do NRF
uint8_t NRF_ReadStatus() {
  uint8_t status = 0x00;
  uint8_t cmd = NRF_CMD_NOP;

  csn_reset();
  HAL_SPI_TransmitReceive(HSPI, &cmd, &status, 1, NRF_SPI_TIMEOUT_DUR);
  csn_set();

  return status;
}

/* Manipulação de Registro */

/* Seta um Bit específico de um Registro para 1
 * Param - Registro a ser escrito
 * Param - Qual bit será alterado para 1
 *  											*/
NRF_Status NRF_SetRegisterBit(uint8_t reg, uint8_t bit) {
  NRF_Status ret = NRF_OK;
  uint8_t cfg = 0x00;

  ret = NRF_ReadRegister(reg, &cfg, 1); //Pega a informação do byte do registro
  if (ret != NRF_OK) {
    return ret;
  }

  cfg = cfg | (1 << bit); //Altera o bit por meio de um OU binário
  return NRF_WriteRegister(reg, &cfg, 1); //Escreve o registro de volta
}

/* Seta um Bit específico de um Registro para 0
 * Param - Registro a ser escrito
 * Param - Qual bit será alterado para 1
 *  											*/
NRF_Status NRF_ResetRegisterBit(uint8_t reg, uint8_t bit) {
  NRF_Status ret = NRF_OK;
  uint8_t cfg = 0x00;

  ret = NRF_ReadRegister(reg, &cfg, 1);//Pega a informação do byte do registro
  if (ret != NRF_OK) {
    return ret;
  }

  cfg = cfg & ~(1 << bit);//Altera o bit por meio de um E binário
  return NRF_WriteRegister(reg, &cfg, 1);//Escreve o registro de volta
}

/* Controle do dispositivo */

/* Coloca o NRF em algum modo
 * Param - modo
 * Modos:
 * NRF_MODE_POWERDOWN - Desligado
 * NRF_MODE_STANDBY1 - Standby
 * NRF_MODE_RX - Modo de Recepção
 * NRF_MODE_Tx - Modo de Transmissão
 * Retorno: Status
 *  								*/
NRF_Status NRF_EnterMode(uint8_t mode) {
  NRF_Status ret = NRF_OK;

  switch(mode) {
    case NRF_MODE_POWERDOWN:
      csn_set();
      ce_reset();
      ret = NRF_ResetRegisterBit(NRF_REG_CONFIG, CFG_BIT_PWR_UP); //Seta o bit de Power up em 0
      break;
    case NRF_MODE_STANDBY1:
      if (current_mode == NRF_MODE_POWERDOWN) {
        ret = NRF_SetRegisterBit(NRF_REG_CONFIG, CFG_BIT_PWR_UP); //Seta o bit de Power up em 1
        wait(1500);
      } else if (current_mode == NRF_MODE_RX) {
        ret = NRF_ResetRegisterBit(NRF_REG_CONFIG, CFG_BIT_PRIM_RX);
        ce_reset();
      } else if (current_mode == NRF_MODE_TX) {
        ce_reset();
      }
      break;
    case NRF_MODE_RX:
      if (current_mode != NRF_MODE_STANDBY1) { //O dispositivo deve estar em standby para passar para o modo Rx
        return NRF_BAD_TRANSITION;
      }
      ret = NRF_SetRegisterBit(NRF_REG_CONFIG, CFG_BIT_PRIM_RX); // 1 = Rx
      ce_set();
      break;
    case NRF_MODE_TX:
      if (current_mode != NRF_MODE_STANDBY1) {
        return NRF_BAD_TRANSITION;
      }
      ret = NRF_ResetRegisterBit(NRF_REG_CONFIG, CFG_BIT_PRIM_RX); // 0 = Tx
      ce_set();
      break;
    default:
      ret = NRF_ERROR;
      break;
  }

  if (ret == NRF_OK) {
    current_mode = mode;
  }

  return ret;
}

/* Iniciar o NRF
 * Param - SPI
 * Param - Porta Chip Select
 * Param - Pino Chip Select
 * Param - Porta Chip Enable
 * Param - Pino Chip Enable
 * Retorno: Status
 *  								*/
NRF_Status NRF_Init(SPI_HandleTypeDef *handle, GPIO_TypeDef *PortCSN, uint16_t PinCSN, GPIO_TypeDef *PortCE, uint16_t PinCE) {
  HSPI = handle;
  NRF_CSN_Port = PortCSN;
  NRF_CSN_Pin = PinCSN;
  NRF_CE_Port = PortCE;
  NRF_CE_Pin = PinCE;

  CPU_Freq = HAL_RCC_GetSysClockFreq();
  if (CPU_Freq == 0x00) {
    return NRF_ERROR;
  }

  // Make sure CSN is pulled high
  csn_set();

  // Takes ~100ms from power on to start up
  HAL_Delay(100);

  return NRF_EnterMode(NRF_MODE_STANDBY1);
}


//Reseta o NRF  o mantendo em stanby
void NRF_Reset() {
  NRF_EnterMode(NRF_MODE_POWERDOWN);
  NRF_EnterMode(NRF_MODE_STANDBY1);

  // Flush FIFOs
  NRF_EnterMode(NRF_MODE_TX);
  NRF_SendCommand(NRF_CMD_FLUSH_TX);
  NRF_EnterMode(NRF_MODE_STANDBY1);
  NRF_EnterMode(NRF_MODE_RX);
  NRF_SendCommand(NRF_CMD_FLUSH_RX);
  NRF_EnterMode(NRF_MODE_STANDBY1);

  // Flush register -> LER DATASHEET!!!!!!!!!!!!!!
  NRF_WriteRegisterByte(NRF_REG_CONFIG,       0x0A);// 00001010
  NRF_WriteRegisterByte(NRF_REG_EN_AA,        0x3F);// 00000001 = AutoAcknologment ativado no primeiro PIPE
  NRF_WriteRegisterByte(NRF_REG_EN_RXADDR,    0x03);//00000011 -> Pipes 0 e 1 no Rx
  NRF_WriteRegisterByte(NRF_REG_SETUP_AW,     0x03);//00000011 -> 5 bytes no adresss
  NRF_WriteRegisterByte(NRF_REG_SETUP_RETR,   0x03);//00001111 -> re-transmit habilitado. bits 7-4 -> delay de retransmit (250us). bits 3-0 qtd de retransmit (15).
  NRF_WriteRegisterByte(NRF_REG_RF_CH,        0x02);//00000010 -> Canal 3
  NRF_WriteRegisterByte(NRF_REG_RF_SETUP,     0x0E);//00001110 -> LNA desligado, 0dBm, 2MBs
  NRF_WriteRegisterByte(NRF_REG_STATUS,       0x70); // clear flags

  uint8_t address[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
  uint8_t address2[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
  NRF_WriteRegister(NRF_REG_RX_ADDR_P0, address, 5); //Adress do pipe 0
  NRF_WriteRegister(NRF_REG_RX_ADDR_P1, address2, 5); //Adress pipe 1
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P2,   0xC3);
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P3,   0xC4);
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P4,   0xC5);
  NRF_WriteRegisterByte(NRF_REG_RX_ADDR_P5,   0xC6);
  NRF_WriteRegister(NRF_REG_TX_ADDR, address, 5);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P0,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P1,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P2,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P3,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P4,     0x00);
  NRF_WriteRegisterByte(NRF_REG_RX_PW_P5,     0x00);

  NRF_WriteRegisterByte(NRF_REG_FIFO_STATUS,  0x00);
  NRF_WriteRegisterByte(NRF_REG_DYNPD,        0x00);
  NRF_WriteRegisterByte(NRF_REG_FEATURE,      0x00);
}

NRF_Status NRF_WritePayload(uint8_t *payload, uint8_t length) {
  return NRF_SendWriteCommand(NRF_CMD_W_TX_PAYLOAD, payload, length);
}

NRF_Status NRF_WritePayloadNoAck(uint8_t *payload, uint8_t length) {
  return NRF_SendWriteCommand(NRF_CMD_W_TX_PAYLOAD_NO_ACK, payload, length);
}

NRF_Status NRF_WriteAckPayload(uint8_t pipe, uint8_t *payload, uint8_t length) {
  return NRF_SendWriteCommand(NRF_CMD_W_ACK_PAYLOAD | pipe, payload, length);
}

NRF_Status NRF_ReadPayload(uint8_t *read, uint8_t length) {
  return NRF_SendReadCommand(NRF_CMD_R_RX_PAYLOAD, read, length);
}


NRF_Status NRF_Transmit(uint8_t *payload, uint8_t length) {
  NRF_Status ret = NRF_OK;
  ret = NRF_WritePayload(payload, length);
  if (ret != NRF_OK) {
    return ret;
  }

  ce_set();
  wait(10);
  ce_reset();

  return ret;
}

void NRF_ReTransmit() {
  ce_set();
  wait(10);
  ce_reset();
}

NRF_Status NRF_TransmitAndWait(uint8_t *payload, uint8_t length) {
  NRF_Status ret = NRF_OK;

  ret = NRF_WritePayload(payload, length);
  if (ret != NRF_OK) {

    return ret;
  }

  // Transmit
  ce_set();

  // Wait for status update
  uint8_t status;
  for (;;) {
    status = NRF_ReadStatus();
    if (status & (1<<STATUS_BIT_TX_DS)) {
      // Packet transmitted
      ret = NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_BIT_TX_DS); // clear flag
      break;
    } else if (status & (1<<STATUS_BIT_MAX_RT)) {
      // Max retransmits reached
      NRF_SetRegisterBit(NRF_REG_STATUS, STATUS_BIT_MAX_RT); // clear flag
      ret = NRF_MAX_RT;
      break;
    }
  }
  ce_reset();

  return ret;
}
