/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CEC_HandleTypeDef hcec;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

volatile uint8_t respbuf[128];
volatile uint8_t intbuf[128];
volatile uint16_t usb_inttxlen=0;


uint8_t rxcmdbuf[128];
volatile uint8_t cmdbuf[128];
volatile uint8_t bufrpos=0;
volatile uint8_t bufwpos=0;
uint8_t rxcmdpos=0;
uint8_t repall=0;
volatile uint16_t usb_txlen=0;
#define CEC_MAX_PAYLOAD		16
#define CEC_ADDRESS		0xFF
uint32_t ErrorCode       = 0x0;
uint8_t  Tab_Rx[CEC_MAX_PAYLOAD];   /* Received data buffer. Max size = 16 bytes
                                     * header + opcode followed by up to 14 operands */
uint8_t  Tab_Tx[CEC_MAX_PAYLOAD-1]; /* Transmitted data buffer.
                                     * header is not included in Tab_Tx.
                                     *  Max size = 15 bytes.
                                     *  one opcode followed by up to 14 operands.
                                     *  When payload size = 0, only the header is sent
                                     *  (ping operation) */
uint8_t ReceivedFrame       = 0x0;  /* Set when a reception occurs */
uint16_t NbOfReceivedBytes  = 0x0;  /* Number of received bytes in addition to the header.
                                     * when a ping message has been received (header
                                     * only), NbOfReceivedBytes = 0 */
uint8_t StartSending        = 0x0;  /* Set when a transmission is triggered by the user */
uint32_t TxSize             = 0x0;  /* Number of bytes to transmit in addition to the header.
                                     * In case of ping operation (only the header sent),
                                     * TxSize = 0 */
uint8_t DestinationAddress;         /* Destination logical address */
uint8_t LogicalAddress;             /* CEC IP Initiator logical address */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_HDMI_CEC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
static void CEC_FlushRxBuffer(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


static uint8_t fromasc(uint8_t asc){
	if(asc>='A' && asc <= 'F'){
			return 0xa+asc-'A';
		}
	if(asc>='a' && asc <= 'f'){
			return 0xa+asc-'a';
		}
	if(asc>='0' && asc <= '9'){
			return asc-'0';
		}
	return 0xF0;
}

static uint8_t parsebyte(uint8_t asc1, uint8_t asc2, uint8_t *target){
	uint8_t p1=fromasc(asc1);
	uint8_t p2=fromasc(asc2);
	if(p1!=0xf0 && p2!=0xf0){
		*target=(p1<<4)|p2;
		return 0;
	}
	return 1;
}


static uint8_t toasc(uint8_t dig){
	if(dig<0xa)return '0'+dig;
	else if(dig<0x10)return 'A'+dig-0xa;
	else return 'X';
}

static int toascstr(int len, uint8_t *bufsrc, uint8_t *bufdst){
	int dstcount=0;
	int cnt=0;
	int tlen=len;
	if(tlen>16)tlen=16;
	while(dstcount<len){
		bufdst[dstcount*3]=toasc((bufsrc[dstcount]>>4)&0xf);
		bufdst[dstcount*3+1]=toasc(bufsrc[dstcount]&0xf);
		if(tlen-dstcount>1){
			cnt++;
		bufdst[dstcount*3+2]=':';
		}
		dstcount++;
		cnt+=2;
	}
	return cnt;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_HDMI_CEC_Init();
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */
  //HAL_CEC_Transmit(hcec,0xf,0,0,0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(usb_inttxlen){
	  		  	CDC_Transmit_FS(intbuf,usb_inttxlen);
	  		  	usb_inttxlen=0;
	  		  }
	  HAL_CEC_Receive_IT(&hcec,(uint8_t *)&Tab_Rx);
	  if(ReceivedFrame==1){
	  		  respbuf[0]='R';
	  		  int offset=1;

	  		  				  if(NbOfReceivedBytes){
	  		  					  respbuf[offset]=':';
	  		  					  offset++;
	  		  					  offset+=toascstr(NbOfReceivedBytes,Tab_Rx,respbuf+offset);
	  		  				  }
	  		  				  respbuf[offset]='\r';
	  		  				  respbuf[offset+1]='\n';
	  		  				  if(repall || (Tab_Rx[0]&0xf)==0xf || hcec.Init.OwnAddress==(Tab_Rx[0]&0xf)){
	  		  					  CDC_Transmit_FS(respbuf,offset+2);
	  		  				  }
	  		  					  		  	usb_txlen=0;
	  		  					  		ReceivedFrame = 0;
	  	  }
	  if(ReceivedFrame==2){
		  /*#define HAL_CEC_ERROR_NONE    (uint32_t) 0x0    // !< no error
#define HAL_CEC_ERROR_RXOVR   CEC_ISR_RXOVR          //!< CEC Rx-Overrun
#define HAL_CEC_ERROR_BRE     CEC_ISR_BRE            //!< CEC Rx Bit Rising Error
#define HAL_CEC_ERROR_SBPE    CEC_ISR_SBPE           //!< CEC Rx Short Bit period Error
#define HAL_CEC_ERROR_LBPE    CEC_ISR_LBPE           //!< CEC Rx Long Bit period Error
#define HAL_CEC_ERROR_RXACKE  CEC_ISR_RXACKE         //!< CEC Rx Missing Acknowledge
#define HAL_CEC_ERROR_ARBLST  CEC_ISR_ARBLST         //!< CEC Arbitration Lost
#define HAL_CEC_ERROR_TXUDR   CEC_ISR_TXUDR          //!< CEC Tx-Buffer Underrun
#define HAL_CEC_ERROR_TXERR   CEC_ISR_TXERR          //!< CEC Tx-Error
#define HAL_CEC_ERROR_TXACKE  CEC_ISR_TXACKE         //!< CEC Tx Missing Acknowledge
*/
	  		  respbuf[0]='F';
	  		  int offset=1;

	  		  				  respbuf[offset]='\r';
	  		  				  respbuf[offset+1]='\n';
	  		  				CDC_Transmit_FS(respbuf,offset+2);

	  		  					  		  	usb_txlen=0;
	  		  					  		ReceivedFrame = 0;
	  	  }

  /* USER CODE END WHILE */
	  uint8_t rxchar;
  /* USER CODE BEGIN 3 */
	  while(bufrpos!=bufwpos){
		  rxchar=cmdbuf[bufrpos];
		  if(rxchar!='\n' && rxchar!='\r' && rxcmdpos<96){
			  rxcmdbuf[rxcmdpos]=rxchar;
			  rxcmdpos++;
		  }else{
			  rxcmdbuf[rxcmdpos]=0;
			  rxcmdbuf[rxcmdpos+1]=0;
			  rxcmdbuf[rxcmdpos+2]=0;
			  if(rxcmdpos>95){

			  }
			  //parsing command
			  if(rxcmdbuf[0]=='L'||rxcmdbuf[0]=='l'){
				uint8_t c=fromasc(rxcmdbuf[1]);
				if(c==0xf0 && rxcmdpos>1) c=fromasc(rxcmdbuf[2]);
				if(c!=0xf0) {
					hcec.Init.OwnAddress=c;
					HAL_CEC_DeInit(&hcec);
					HAL_CEC_Init(&hcec);
					respbuf[0]='L';
					respbuf[1]=toasc(c);
					respbuf[3]='\r';
					respbuf[4]='\n';
					usb_txlen=4;
				}
			  }
			  if(rxcmdbuf[0]=='P'||rxcmdbuf[0]=='p'){
			  				uint8_t c=rxcmdbuf[1];
			  				if(c=='0'||c=='1'){
			  					if(c=='1')repall=1;
			  					else repall=0;
			  					respbuf[0]='L';
			  					respbuf[1]=c;
			  					respbuf[3]='\r';
			  					respbuf[4]='\n';
			  					usb_txlen=4;
			  				}else{
			  					respbuf[0]='F';
			  					respbuf[0]='P';
			   									  					respbuf[2]='\r';
			  									  					respbuf[3]='\n';
			  									  					usb_txlen=4;
			  				}
			  }
			  if(rxcmdbuf[0]=='E'||rxcmdbuf[0]=='e'){
				  respbuf[0]='E';
				  respbuf[1]='\r';
				  respbuf[2]='\n';
				  usb_txlen=3;

			  }
			  if(rxcmdbuf[0]=='T'||rxcmdbuf[0]=='t'){
				  int offset=1;
				  while(rxcmdbuf[offset]==' '){
					  offset++;
				  }
				  uint8_t sourceaddr=fromasc(rxcmdbuf[offset]);
				  if(sourceaddr==0xf0){
					respbuf[1]='S';
					goto fail;
				  }
				  offset++;
				  uint8_t destaddr=fromasc(rxcmdbuf[offset]);
				  if(destaddr==0xf0){
				  					respbuf[1]='D';
				  					goto fail;
				  				  }
				  offset++;
				  int txlen=0;
				  uint8_t nextbyte;
				  while(rxcmdbuf[offset]!=0 && txlen<16){
					  if(rxcmdbuf[offset]!=':'){
							respbuf[1]='C';
							goto fail;
					  }
					  if(parsebyte(rxcmdbuf[offset+1],rxcmdbuf[offset+2],&nextbyte)){
						  respbuf[1]='B';
						  goto fail;
					  }
					  Tab_Tx[txlen]=nextbyte;
					  txlen++;
					  offset+=3;
				  }
				  				  hcec.Init.InitiatorAddress=sourceaddr;
				  if(HAL_CEC_Transmit(&hcec,destaddr,Tab_Tx,txlen,200)){
					  respbuf[0]='F';
					  if(hcec.ErrorCode&HAL_CEC_ERROR_TXACKE)
						  respbuf[1]='N';
					  else
						  respbuf[1]='X';
					  respbuf[2]='\r';
					  respbuf[3]='\n';
					  usb_txlen=4;;
				  }else{
					  respbuf[0]='T';
					  				  respbuf[1]=toasc(sourceaddr);
					  				  respbuf[2]=toasc(destaddr);
					  				  offset=3;

					  				  if(txlen){
					  					  respbuf[offset]=':';
					  					  offset++;
					  					  offset+=toascstr(txlen,Tab_Tx,respbuf+offset);
					  				  }
					  				  respbuf[offset]='\r';
					  				  respbuf[offset+1]='\n';
					  				  usb_txlen=offset+2;
				  }

				  goto end;
				  fail:

				  					respbuf[0]='F';
				  					respbuf[2]='\r';
				  					respbuf[3]='\n';
				  					usb_txlen=4;
				  end:
				  	  ;
			  }

			  rxcmdpos=0;
		  }
		  bufrpos=(bufrpos+1)&0x7f;
		  if(usb_txlen){
		  	CDC_Transmit_FS(respbuf,usb_txlen);
		  	usb_txlen=0;
		  }
	  }


  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_CEC;
  PeriphClkInit.CecClockSelection = RCC_CECCLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* HDMI_CEC init function */
static void MX_HDMI_CEC_Init(void)
{

  hcec.Instance = CEC;
  hcec.Init.SignalFreeTime = CEC_DEFAULT_SFT;
  hcec.Init.Tolerance = CEC_STANDARD_TOLERANCE;
  hcec.Init.BRERxStop = CEC_RX_STOP_ON_BRE;
  hcec.Init.BREErrorBitGen = CEC_BRE_ERRORBIT_NO_GENERATION;
  hcec.Init.LBPEErrorBitGen = CEC_LBPE_ERRORBIT_NO_GENERATION;
  hcec.Init.BroadcastMsgNoErrorBitGen = CEC_BROADCASTERROR_ERRORBIT_GENERATION;
  hcec.Init.SignalFreeTimeOption = CEC_SFT_START_ON_TXSOM;
  hcec.Init.OwnAddress = CEC_ADDRESS;
  hcec.Init.ListenMode = CEC_FULL_LISTENING_MODE;
  hcec.Init.InitiatorAddress = 7;
  if (HAL_CEC_Init(&hcec) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */


void USB_CDC_Recv_CB (uint8_t* Buf, uint32_t Len){
	int i;

	for(i=0;i<Len;i++){
		cmdbuf[bufwpos]=Buf[i];
		bufwpos=(bufwpos+1)&0x7f;
	}

}

/**
  * @brief Tx Transfer completed callback
  * @param hcec: CEC handle
  * @retval None
  */
void HAL_CEC_TxCpltCallback(CEC_HandleTypeDef *hcec)
{

  /* after transmission, return to stand-by mode */
  hcec->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
  * @brief Rx Transfer completed callback
  * @param hcec: CEC handle
  * @retval None
  */
void HAL_CEC_RxCpltCallback(CEC_HandleTypeDef *hcec)
{
    ReceivedFrame = 1;
    /* Reminder: hcec->RxXferSize is the sum of opcodes + operands
     * (0 to 14 operands max).
     * If only a header is received, hcec->RxXferSize = 0 */
    NbOfReceivedBytes = hcec->RxXferSize;
    hcec->RxXferSize = 0;
    hcec->pRxBuffPtr = Tab_Rx;
    hcec->ErrorCode = HAL_CEC_ERROR_NONE;
    //intbuf[0]='A';
    //	intbuf[1]='\r';
    //	intbuf[2]='\n';
    //	usb_inttxlen=3;
    /* return to stand-by mode */
    hcec->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
  * @brief CEC error callbacks
  * @param hcec: CEC handle
  * @retval None
  */
void HAL_CEC_ErrorCallback(CEC_HandleTypeDef *hcec)
{
  ReceivedFrame = 2;
  ErrorCode=hcec->ErrorCode;
  hcec->RxXferSize = 0;
  hcec->pRxBuffPtr = Tab_Rx;
  hcec->ErrorCode = HAL_CEC_ERROR_NONE;
  hcec->State = HAL_CEC_STATE_STANDBY_RX;
}

/**
  * @brief  Reset CEC reception buffer
  * @param  None
  * @retval None
  */
static void CEC_FlushRxBuffer(void)
{
  uint32_t cpt;

  for (cpt = CEC_MAX_PAYLOAD; cpt > 0; cpt--)
  {
    Tab_Rx[cpt-1] = 0;
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
