/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define _ATZ_               0
#define _DADDR_             1
#define _APPKEY_            2
#define _APPSKEY_           3
#define _NWKSKEY_           4
#define _APPEUI_            5
#define _ADR_               6
#define _DR_                7
#define _DCS_               8
#define _PNM_               9
#define _RX2FQ_             10
#define _RX2DR_             11
#define _RX1DL_             12
#define _RX2DL_             13
#define _JN1DL_             14
#define _JN2DL_             15
#define _NJM_               16
#define _NWKID_             17
#define _CLASS_             18
#define _JOIN_              19
#define _NJS_               20
#define _SENDB_             21
#define _SEND_              22
#define _VER_               23
#define _CFM_               24
#define _SNR_               25
#define _RSSI_              26
#define _BAT_               27
#define _BAUDRATE_          28
#define _NBTRIALS_          29
#define _KEEPALIVE_         30
#define _TXCFM_             31
#define _CHMASK_            32
#define _ADC_               33
#define _GPIOC_             34
#define _WPIN_              35
#define _RPIN_              36
#define _AJOIN_             37
#define _DEUI_              38

#define ABP                 0
#define OTAA                1

#define CS                  0
#define TTN                 1
#define EN                  2

#define OUTPUT				1
#define OUTPUT_OPENDRAIN    2
#define OUTPUT_FA_PUSHPULL  3
#define OUTPUT_FA_OPENDRAIN 4

#define INPUT				0
#define INPUT_PULLUP		5
#define INPUT_PULLDOWN		9
#define INPUT_ADC           6
#define INTERRUPT_RISING    7
#define INTERRUPT_FALLING   8
#define INTERRUPT_CHANGE    10

#define BUFFER_SIZE   256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t feedback = 0, connected = 0;

char* AT_CMD[39] = {
      "ATZ",
      "DADDR",
      "APPKEY",
      "APPSKEY",
      "NWKSKEY",
      "APPEUI",
      "ADR",
      "DR",
      "DCS",
      "PNM",
      "RX2FQ",
      "RX2DR",
      "RX1DL",
      "RX2DL",
      "JN1DL",
      "JN2DL",
      "NJM",
      "NWKID",
      "CLASS",
      "JOIN",
      "NJS",
      "SENDB",
      "SEND",
      "VER",
      "CFM",
      "SNR",
      "RSSI",
      "BAT",
      "BAUDRATE",
      "NBTRIALS",
      "KEEPALIVE",
      "TXCFM",
      "CHMASK",
      "ADC",
      "GPIOC",
      "WPIN",
      "RPIN",
      "AJOIN",
      "DEUI"
  };

char g_payload[BUFFER_SIZE];
uint8_t array[BUFFER_SIZE];
char* payloads[5];
uint8_t port = 1, confirmado = 0, retries = 0;
int periodicidade = 0;
uint8_t buffer_err = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void uart_communication(char *val);
void uart_read(void);
void UART_WriteString(UART_HandleTypeDef *huart, char *val);
char* UART_ReadString(UART_HandleTypeDef *huart);


void deserializeAT(uint8_t cmd);
void ATZ(void);
void ConfigNetwork(uint8_t njm, uint8_t net, char* appkey, char* appeui, char* nwkskey, char* daddr);
void printParameters(void);
void LoRaWAN_Begin(uint8_t _feedback);

char* separator(char* val);
char* feedbackSerial(char* val, uint8_t exception);
char* commandAT(uint8_t cmd, char* val, uint8_t exception);
char* bool_to_intString(uint8_t val);
char* DADDR(char* val);
char* APPKEY(char* val);
char* APPSKEY(char* val);
char* NWKSKEY(char* val);
char* APPEUI(char* val);
char* DEUI(void);
char* CHMASK(char* val);
char* NWKID(void);
char* VER(void);
char* uint32_tTocharPointer(uint32_t val);
//uint32_t chartPointerTouint32_t(char* val);

uint8_t ADR(uint8_t val);
uint8_t DR(uint8_t val);
uint8_t DCS(uint8_t val);
uint8_t PNM(uint8_t val);
uint8_t NJM(uint8_t val);
uint8_t CLASS(uint8_t val);
uint8_t JOIN(void);
uint8_t AJOIN(uint8_t val);
uint8_t NJS(void);
uint8_t NBTRIALS(uint8_t val);
uint8_t TXCFM(uint8_t _port, uint8_t _confirmado, uint8_t _retries, char* payload);
uint8_t KEEPALIVE(uint8_t habilitado, uint8_t _port, uint8_t _confirmado, int _periodicidade);
uint8_t pinMode(uint8_t pin, uint8_t modo);
uint8_t digitalRead(uint8_t pin);
uint8_t digitalWrite(uint8_t pin, uint8_t val);
uint8_t CFM(uint8_t val);
uint8_t SNR(void);
uint8_t JoinNetwork(uint8_t njm, uint8_t net,  uint8_t autoconfig, uint8_t automatic, char* appkey, char* appeui, char* nwkskey, char* daddr);
uint8_t SendString(char* string, uint8_t _port);
uint8_t SendRaw(char* payload, uint8_t _port);

uint16_t RX2DR(uint16_t val);
uint16_t RX1DL(uint16_t val);
uint16_t RX2DL(uint16_t val);
uint16_t JN1DL(uint16_t val);
uint16_t JN2DL(uint16_t val);
uint16_t GPIO(uint8_t cmd, uint8_t pin, uint8_t val);
uint16_t BAUDRATE(uint16_t val);
uint16_t analogRead(uint8_t pin);

uint32_t RX2FQ(uint32_t val);

int RSSI(void);
int indexOf(char* val, char search);

float BAT(void);

void ClearUARTBuffer(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ClearUARTBuffer(UART_HandleTypeDef *huart) {
	UART_WriteString(huart, "\r\n");
    uint8_t dummyData;
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
        HAL_UART_Receive(huart, &dummyData, 1, HAL_MAX_DELAY);
    }
}

void UART_WriteString(UART_HandleTypeDef *huart, char *val){
	uint32_t len_val = strlen(val);
	HAL_UART_Transmit(huart, (uint8_t*)val, len_val, HAL_MAX_DELAY);
}

char* UART_ReadString(UART_HandleTypeDef *huart){
	uint8_t received_data = 0;
	uint8_t buffer_pData[1024];
	uint32_t count = 0;

	while(1){
		int response = HAL_UART_Receive(huart, &received_data, 1, 1000);
		if(HAL_OK == response){
			if(received_data == '\n'){
				buffer_pData[count] = received_data;
				++count;
				break;
			}
			else{
				if(received_data != '\r'){
					buffer_pData[count] = received_data;
					++count;
				}
			}
		}
		else if(HAL_ERROR == response){
			buffer_err = HAL_ERROR;
			return "HAL_ERROR";
		}
		else if(HAL_BUSY == response){
			buffer_err = HAL_BUSY;
			return "HAL_BUSY";
		}
		else if(HAL_TIMEOUT == response){
			buffer_err = HAL_TIMEOUT;
			return "HAL_TIMEOUT";
		}
	}

	char* val_string = (char*)malloc(count + 1);
	if (val_string != NULL) {
		for (uint32_t i = 0; i < count; ++i)
			if(buffer_pData[i] != '\n' || buffer_pData[i] != '\r')
				val_string[i] = (char)buffer_pData[i];
	    val_string[count] = '\0';
	}

	char* buffer_val = val_string;
	free(val_string);
	buffer_err = HAL_OK;
	return buffer_val;
}

char* feedbackSerial(char* val, uint8_t exception){
  if(feedback == 1){
	  UART_WriteString(&huart2, "TX: ");
	  UART_WriteString(&huart2, val);
  }

  UART_WriteString(&huart1, val);
  char* buff;
  uint8_t count = 8;
  static uint8_t count_err = 0;

  while(1){
	  buff = UART_ReadString(&huart1);

	  if(count_err >= 3)
		  NVIC_SystemReset();
	  else if(buffer_err != HAL_OK)
		  ++count_err;
	  else
		  count_err = 0;

	  if(exception == 0){
		  if(indexOf(buff, 'E') == indexOf(buff, 'R') - 1)
			  return "";
	      break;
	  }
	  else{
		  if(indexOf(buff, 'E') > 0 && indexOf(buff, 'D') < 0 && count > 0)
			count -= 1;
	      else if(count <= 0)
	        break;
	      else if(indexOf(buff, 'K') > 0 || indexOf(buff, 'D') == indexOf(buff, 'N') + 2 || indexOf(buff, 'Y') > 0){
	    	  connected = 1;
	    	  break;
	      }
	  }
  }

  if(feedback == 1){
	  if(buff != ""){
		  UART_WriteString(&huart2, "RX: ");
	  	  UART_WriteString(&huart2, buff);
	  }

	  UART_WriteString(&huart2, "\r\n");
  }
  return buff;
}


char* commandAT(uint8_t cmd, char* val, uint8_t exception){
  char* AT = "AT+";
  char* buff;

  char command[BUFFER_SIZE];

  if(exception == 0 && val == "")
	  sprintf(command, "%s%s=?\r\n\0", AT, AT_CMD[cmd]);
  else if(exception == 1)
	  sprintf(command, "%s%s\r\n\0", AT, AT_CMD[cmd]);
  else
	  sprintf(command, "%s%s=%s\r\n\0", AT, AT_CMD[cmd], val);

  HAL_Delay(50);
  return feedbackSerial(command, exception);
}

char* uint32_tTocharPointer(uint32_t val){
	char buffer[1024];
	snprintf(buffer, sizeof(buffer), "%lu", (unsigned long)val);
	return buffer;
}

int indexOf(char* val, char search){
	for(uint8_t i = 0; i < strlen(val); ++i){
		if(val[i] == search)
			return i;
	}
	return -1;
}

char* bool_to_intString(uint8_t val){
	if(val == 1)
		return "1";
	return "0";
}

void deserializeAT(uint8_t cmd) {
    char* val_char = commandAT(cmd, "", 0);
    uint8_t count = 0;

    for (uint8_t i = 0; i < strlen(val_char); ++i) {
        if (val_char[i] != ':') {
            if (count < strlen(val_char) - 1) {
                payloads[count] += val_char[i];
            }
        } else {
            ++count;
        }
    }
}

char* separator(char* val) {
    size_t val_size = strlen(val);

    if (val_size % 2 == 0 && indexOf(val, ':') < 1) {
        size_t result_size = val_size * 2 + 1;
        char* result = (char*)malloc(result_size);

        if (result != NULL) {
            uint8_t count = 0;
            uint8_t pair_count = 0;

            for (uint8_t i = 0; i < val_size; ++i) {
                result[count] = val[i];
                ++count;

                if (pair_count > 0 && i + 1 < val_size) {
                    result[count] = ':';
                    ++count;
                    pair_count = 0;
                }

                if(i % 2 == 0)
                	++pair_count;

            }
            result[count] = '\0';

            val = result;
            free(result);
            return val;
        }
    }

    return val;
}

uint16_t GPIO(uint8_t cmd, uint8_t pin, uint8_t val){
  char* buff = "";

  if(val != 2){
	sprintf(g_payload, "%d:%d\0", pin, val);
	buff = commandAT(cmd, g_payload, 0);
  }
  else{
	buff = commandAT(cmd, uint32_tTocharPointer(pin), 0);
  }

  return (uint16_t)strtoul(buff, NULL, 10);
}

void printParameters(void){
  uint8_t buff = feedback;
  feedback = 0;
  char* version = VER();

  UART_WriteString(&huart2, "---------------------------------------------------\r\n");
  UART_WriteString(&huart2, "                  LoRaWAN Radioenge\r\n");
  UART_WriteString(&huart2, " Version        = "); UART_WriteString(&huart2, version);  UART_WriteString(&huart2, "\r"); char* _DEUI = DEUI();
  UART_WriteString(&huart2, " DevEui         = "); UART_WriteString(&huart2, _DEUI); UART_WriteString(&huart2, "\r"); char* _DADDR = DADDR("");
  UART_WriteString(&huart2, " DevAddr        = "); UART_WriteString(&huart2, _DADDR); UART_WriteString(&huart2, "\r"); char* _APPKEY = APPKEY("");
  UART_WriteString(&huart2, " AppKey         = "); UART_WriteString(&huart2, _APPKEY); UART_WriteString(&huart2, "\r"); char* _APPSKEY = APPSKEY("");
  UART_WriteString(&huart2, " AppSKey        = "); UART_WriteString(&huart2, _APPSKEY); UART_WriteString(&huart2, "\r"); char* _NWKSKEY = NWKSKEY("");
  UART_WriteString(&huart2, " NwkSKey        = "); UART_WriteString(&huart2, _NWKSKEY); UART_WriteString(&huart2, "\r"); char* _APPEUI = APPEUI("");
  UART_WriteString(&huart2, " AppEui/JoinEui = "); UART_WriteString(&huart2, _APPEUI); UART_WriteString(&huart2, "\r");
  UART_WriteString(&huart2, "                    elcereza.com\r\n");
  UART_WriteString(&huart2, "---------------------------------------------------\r\n");

  feedback = buff;
}

void LoRaWAN_Begin(uint8_t _feedback){
  feedback = _feedback;
  printParameters();
}

char* DADDR(char* val){
  if(val != "") commandAT(_DADDR_, separator(val), 0);
  return commandAT(_DADDR_, "", 0);;
}

char* APPKEY(char* val){
  if(val != "") commandAT(_APPKEY_, separator(val), 0);
  return commandAT(_APPKEY_, "", 0);
}

char* APPSKEY(char* val){
  if(val != "") commandAT(_APPSKEY_, separator(val), 0);
  return commandAT(_APPSKEY_, "", 0);
}

char* NWKSKEY(char* val){
  if(val != "") commandAT(_NWKSKEY_, separator(val), 0);
  return commandAT(_NWKSKEY_, "", 0);
}

char* APPEUI(char* val){
  if(val != "") commandAT(_APPEUI_, separator(val), 0);
  return commandAT(_APPEUI_, "", 0);
}

char* DEUI(void){
  return commandAT(_DEUI_, "", 0);
}

char* CHMASK(char* val){
  if(val != "") commandAT(_CHMASK_, separator(val), 0);
  return commandAT(_CHMASK_, "", 0);
}

void ATZ(void){
	commandAT(_ATZ_, "", 0);
}

uint8_t ADR(uint8_t val){
  if(val != NULL) commandAT(_ADR_, bool_to_intString(val), 0);
  return (uint8_t)strtoul(commandAT(_ADR_, "", 0), NULL, 10);
}

uint8_t DR(uint8_t val){
  if(val < 14) (uint8_t)commandAT(_DR_, uint32_tTocharPointer(val), 0);
  return (uint8_t)strtoul(commandAT(_DR_, "", 0), NULL, 10);
}

uint8_t DCS(uint8_t val){
  if(val != NULL) commandAT(_DCS_, bool_to_intString(val), 0);
  return (uint8_t)strtoul(commandAT(_DCS_, "", 0), NULL, 10);
}

uint8_t PNM(uint8_t val){
  if(val != NULL) commandAT(_PNM_, bool_to_intString(val), 0);
  return (uint8_t)strtoul(commandAT(_PNM_, "", 0), NULL, 10);
}

uint32_t RX2FQ(uint32_t val){
  if(val != NULL) commandAT(_RX2FQ_, uint32_tTocharPointer(val), 0);
  return (uint32_t)commandAT(_RX2FQ_, "", 0);
}

uint16_t RX2DR(uint16_t val){
  if(val != NULL) commandAT(_RX2DR_, uint32_tTocharPointer(val), 0);
  return (uint16_t)strtoul(commandAT(_RX2DR_, "", 0), NULL, 10);
}

uint16_t RX1DL(uint16_t val){
  if(val != NULL) commandAT(_RX1DL_, uint32_tTocharPointer(val), 0);
  return (uint16_t)strtoul(commandAT(_RX1DL_, "", 0), NULL, 10);
}

uint16_t RX2DL(uint16_t val){
  if(val != NULL) commandAT(_RX2DL_, uint32_tTocharPointer(val), 0);
  return (uint16_t)strtoul(commandAT(_RX2DL_, "", 0), NULL, 10);
}

uint16_t JN1DL(uint16_t val){
  if(val != NULL) commandAT(_JN1DL_, uint32_tTocharPointer(val), 0);
  return (uint16_t)strtoul(commandAT(_JN1DL_, "", 0), NULL, 10);
}

uint16_t JN2DL(uint16_t val){
  if(val != NULL) commandAT(_JN2DL_, uint32_tTocharPointer(val), 0);
  return (uint16_t)strtoul(commandAT(_JN2DL_, "", 0), NULL, 10);
}

uint8_t NJM(uint8_t val){
  if(val != NULL) commandAT(_NJM_, bool_to_intString(val), 0);
  return (uint8_t)strtoul(commandAT(_NJM_, "", 0), NULL, 10);
}

char* NWKID(void){
  return commandAT(_NWKID_, "", 0);
}

uint8_t CLASS(uint8_t val){
  if(val == 0) commandAT(_CLASS_, "A", 0);
  else if(val == 1) commandAT(_CLASS_, "C", 0);
  else if(commandAT(_CLASS_, "", 0) == "C") return 1;
  return 0;
}

uint8_t JOIN(void){
  char* buff = commandAT(_JOIN_, "", 1);
  return connected;
}

uint8_t AJOIN(uint8_t val){
  if(val != "") commandAT(_AJOIN_, bool_to_intString(val), 0);
  return (uint8_t)strtoul(commandAT(_AJOIN_, "", 0), NULL, 10);
}

uint8_t NJS(){
  return commandAT(_NJS_, "", 0);
}

char* VER(void){
  return commandAT(_VER_, "", 0);
}

uint8_t CFM(uint8_t val){
  if(val != NULL) commandAT(_CFM_, bool_to_intString(val), 0);
  return (uint8_t)strtoul(commandAT(_CFM_, "", 0), NULL, 10);
}

uint8_t SNR(void){
  return (uint8_t)strtoul(commandAT(_SNR_, "", 0), NULL, 10);
}

int RSSI(void){
  return (int)strtoul(commandAT(_RSSI_, "", 0), NULL, 10);
}

float BAT(void){
  return (uint16_t)strtoul(commandAT(_BAT_, "", 0), NULL, 10) * 100 / 253;
}

uint16_t BAUDRATE(uint16_t val){
  if(val != NULL) commandAT(_BAUDRATE_, uint32_tTocharPointer(val), 0);
  return (uint16_t)strtoul(commandAT(_BAUDRATE_, "", 0), NULL, 10);
}

uint8_t NBTRIALS(uint8_t val){
  if(val != NULL) commandAT(_NBTRIALS_, uint32_tTocharPointer(val), 0);
  return (uint8_t)strtoul(commandAT(_NBTRIALS_, "", 0), NULL, 10);
}

uint8_t TXCFM(uint8_t _port, uint8_t _confirmado, uint8_t _retries, char* payload){
  uint8_t index = 0;

  memset(array, 0, BUFFER_SIZE);

  strcpy((char*)&array[index], payload);
  index += strlen(payload);

  if(index > BUFFER_SIZE)
    return 0;

  char* _payload = "";
  for(int i = 0; i < index; ++i)
    _payload += array[index];

  sprintf(g_payload, "%d:%d:%d:%s\0", _port, _retries, _confirmado, _payload);
  commandAT(_SENDB_, g_payload, 0);
  return 1;
}

uint8_t KEEPALIVE(uint8_t habilitado, uint8_t _port, uint8_t _confirmado, int _periodicidade){
  if(habilitado != NULL && _port != NULL && _confirmado != NULL, _periodicidade != NULL){
    sprintf(g_payload, "%d:%d:%d:%d\0", habilitado, _port, _confirmado, _periodicidade);
	commandAT(_KEEPALIVE_, g_payload, 0);
  }

  deserializeAT(_KEEPALIVE_);
  port          = uint32_tTocharPointer(payloads[1]);
  confirmado    = uint32_tTocharPointer(payloads[2]);
  periodicidade = uint32_tTocharPointer(payloads[3]);

  return uint32_tTocharPointer(payloads[0]);
}

uint8_t pinMode(uint8_t pin, uint8_t modo){
  uint8_t pull = 0;

  if(pin > 9 || modo > 10) return 0;
  else if((modo == OUTPUT_FA_PUSHPULL || modo == OUTPUT_FA_OPENDRAIN) && pin != 0 && pin != 1) return 0;
  else if(modo == INPUT_ADC && pin != 0 && pin != 1 && pin != 7 && pin != 8) return 0;
  else if((modo == INTERRUPT_RISING || modo == INTERRUPT_FALLING || modo == INTERRUPT_CHANGE) && pin == 0 && pin == 3 && pin == 7 && pin == 8) return 0;

  if(modo == INPUT)
    modo = 0;
  else if(modo == OUTPUT)
    modo = 1;
  if(modo == INPUT_PULLUP){
    modo = 0;
    pull = 1;
  }
  else if(pull == INPUT_PULLDOWN){
    modo = 0;
    pull = 2;
  }

  deserializeAT(_GPIOC_);
  uint8_t _modo = uint32_tTocharPointer(payloads[1]);
  uint8_t _pull = uint32_tTocharPointer(payloads[2]);

  char command[BUFFER_SIZE];
  sprintf(command, "%d:%d:%d\0", pin, modo, pull);

  if(_modo != modo || _pull != pull)
	  commandAT(_GPIOC_, command, 0);
  return 1;
}

uint8_t digitalRead(uint8_t pin){
  return (uint8_t)GPIO(_RPIN_, pin, "");
}

uint8_t digitalWrite(uint8_t pin, uint8_t val){
  return (uint8_t)GPIO(_WPIN_, pin, val);
}

uint16_t analogRead(uint8_t pin){
  return GPIO(_ADC_, pin, "");
}

void ConfigNetwork(uint8_t njm, uint8_t net, char* appkey, char* appeui, char* nwkskey, char* daddr){
  if(NJM(NULL) != njm) NJM(njm);
  if(njm == OTAA && CLASS(2) == 1) CLASS(0);

  if(njm == ABP) if(appkey != "" && indexOf(appkey, ':') > 0) APPSKEY(appkey);
  else if(appkey != "" && indexOf(appkey, ':') > 0) APPKEY(appkey);
  if(appeui != "" && indexOf(appeui, ':') > 0) APPEUI(appeui);
  if(nwkskey != "" && indexOf(nwkskey, ':') > 0) NWKSKEY(nwkskey);
  if(daddr != "" && indexOf(daddr, ':') > 0) DADDR(daddr);

  uint16_t buff_uint16;
  buff_uint16 = RX1DL(""); if((CS == net || TTN == net) && buff_uint16 != 1000) RX1DL(1000); else if(EN == net && buff_uint16 != 5000) RX1DL(5000);
  buff_uint16 = RX2DL(""); if((CS == net || TTN == net) && buff_uint16 != 2000) RX2DL(2000); else if(EN == net && buff_uint16 != 6000) RX2DL(6000);
  buff_uint16 = JN1DL(""); if((CS == net || TTN == net) && buff_uint16 != 5000) JN1DL(5000); else if(EN == net && buff_uint16 != 5000) JN1DL(5000);
  buff_uint16 = JN2DL(""); if((CS == net || TTN == net) && buff_uint16 != 6000) JN2DL(6000); else if(EN == net && buff_uint16 != 6000) JN2DL(5000);

  uint8_t buff_chmask = indexOf(CHMASK(""), '1');
  if(net == EN && buff_chmask < 1) CHMASK("00ff:0000:0000:0000:0001:0000");
  else if((CS == net || TTN == net) && buff_chmask < 1) CHMASK("ff00:0000:0000:0000:0002:0000");
}

uint8_t JoinNetwork(uint8_t njm, uint8_t net,  uint8_t autoconfig, uint8_t automatic, char* appkey, char* appeui, char* nwkskey, char* daddr){
  if(autoconfig > 0)
    ConfigNetwork(njm, net, appkey, appeui, nwkskey, daddr);
  if(automatic != AJOIN("")) AJOIN(automatic);
  if(!NJS() < 1)
    return JOIN();
  else
    return 1;
  return 0;
}

uint8_t SendString(char* string, uint8_t _port)
{
  if(string == NULL || strnlen(string, BUFFER_SIZE) >= BUFFER_SIZE)
    return 0;
  if(connected > 0){
	sprintf(g_payload, "%d:%s\0", _port, string);
	commandAT(_SEND_, g_payload, 0);
	return 1;
  }

  return 0;
}


uint8_t SendRaw(char* payload, uint8_t _port)
{
  uint8_t index = 0;

  memset(array, 0, BUFFER_SIZE);

  strcpy((char*)&array[index], payload);
  index += strlen(payload);

  if(index > BUFFER_SIZE)
    return 0;

  char* _payload = "";
  for(int i = 0; i < index; ++i)
    _payload += array[index];

  sprintf(g_payload, "%d:%s\0", _port, _payload);
  commandAT(_SENDB_, _payload, 0);
  return 1;
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  LoRaWAN_Begin(1);
  pinMode(2, OUTPUT);
  JOIN();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  SendString("elcereza.com", 1);

	  for(uint8_t i = 0; i < 4; ++i){
		  digitalWrite(2, 1);
		  HAL_Delay(100);
		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		  digitalWrite(2, 0);
		  HAL_Delay(100);
	  }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
