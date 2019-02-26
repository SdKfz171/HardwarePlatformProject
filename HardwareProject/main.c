#include "stm32f10x_it.h"
#include "stm32f10x_lib.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dac.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_flash.h"
#include "stm32f10x_exti.h"
#include <stdio.h>   
#include "main.h"


#include "lcd12864.h"

#include "at24c02.h"


#define  ADC1_DR_Address    ((u32)0x4001244C)
u16 ADCConvertedValue[1];
extern u16 adc1_buffer[1];


void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Configuration(void);
void ADC_Configuration(void);
void DAC_Configuration(void);

void Timer2_Init(void);
void Timer3_Init(void);
void EXTI_Config(void);

void DMA1_Configuration(void);
void NVIC_Configuration(void);

void USART1_SEND(u8 i); 
void putstr(char *str);

u8* ConvertNumber(u16 Number);

extern u8 Buffer_USART1[50],  USART1_rx_end, USART1_len;

	
u16 A_V  = 0;
u8 Num;

u8 Servo_Zero=0;
u8 Set_Time = 0;
/******************************************************************************
  FLASH Memory ���� ������
*******************************************************************************/

void Flash_Data_Write(void);
void Flash_Data_Read(void);

void Servo_zero(void);
void Servo_90(void);
void Servo_R90(void);

#define StartAddr ((u32)0x803F800)           // 2KByte memory allocation
#define EndAddr   ((u32)0x803FFFF) 

u16 Flash_opt=0;
u8 Flash_Buffer[110];
u8 flag = 0;
u8 Timer_flag = 0;

/*******************************************************************************/

void main(void)
{
     u8 i, j;
     u16 i16, temp16;
     u16 AD_value;
	u32 i32;
     
     RCC_Configuration();
     
     RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA |  RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                             RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |  RCC_APB2Periph_AFIO, ENABLE);
     
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);


     
     GPIO_Configuration();
     USART1_Configuration();
     ADC_Configuration();
	DAC_Configuration();
     DMA1_Configuration();
     NVIC_Configuration();   

     
     putstr("Start Program. \n");
     
    lcd12864_init();
    Set_Draw();
    clear_screen();


    Timer2_Init();
        //LCD12864_Num(0,0,8);
    
     int beforeData = 0;
        
     u8 count = 1;
     while(1){  

               // A_V = ADC_GetConversionValue(ADC1);          
                A_V = adc1_buffer[0];
                
                
             
//                DAC_SetChannel1Data(DAC_Align_12b_R, A_V);
//               
//		DAC_SetChannel1Data(DAC_Align_12b_R, 0x7FF);
//		//DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//		delay_ms(500);		
//		DAC_SetChannel1Data(DAC_Align_12b_R, 0x8FF);
		//DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//		delay_ms(500);
//		DAC_SetChannel1Data(DAC_Align_12b_R, 0x9FF);
//		//DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//  		delay_ms(500);
//		DAC_SetChannel1Data(DAC_Align_12b_R, 0xAFF);
//		//DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//  		delay_ms(500);
//		DAC_SetChannel1Data(DAC_Align_12b_R, 0xBFF);
//		//DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//  		delay_ms(500);
//		DAC_SetChannel1Data(DAC_Align_12b_R, 0xCFF);
//		//DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//  		delay_ms(500);
//		DAC_SetChannel1Data(DAC_Align_12b_R, 0xFFF);
//		//DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
//		delay_ms(500);
                

                
                char * STR;
                
                
                // 적외선 감지 센서
                if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)){
                    GPIO_SetBits(GPIOE,GPIO_Pin_2);
                    GPIO_ResetBits(GPIOE,GPIO_Pin_3);
                    
                      
                    flag = 1;
                    //delay_ms(5000);
                                        
                }
                else if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)){
                    GPIO_SetBits(GPIOE,GPIO_Pin_3);
                    GPIO_ResetBits(GPIOE,GPIO_Pin_2);
                }
                
                if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1) && flag){
                    Servo_R90();
                    
                    delay_ms(3600 * 1000 * count);
                    flag = 0;
                    Servo_90();
                    count = 1;
                }
                
                if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2) && flag){
                    count++;
                }
               
                
                
//                if(!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)){
//                  putstr("toched!\n");
//                  
//                  if(!flag){
//                    Set_Time = 10;
//                    
//                    Servo_R90();
//                    
//                    flag = 1;
//                    delay_ms(1000);
//                    
////                    Timer_flag = 1;
////                    flag = 1;
////                    u16 Number = A_V;
////                    int i = 0;
////                    for (i = 0; i < 4; i++) {
////                      USART1_SEND( (u8)(Number % 10 + 0x30));
////                      Number = Number / 10;
//                   }
//                   if(flag){
//                     
//                     putstr("success\n");
//                     delay_ms(500);
//                    Servo_90();
//                  }       
//                }
                
                //USART1_SEND((u8)("\n"));
                //printf("\n");
                
                putstr("\n");
     }
}

void Servo_zero(void)
{
      Servo_Zero=15;
}
void Servo_90(void)
{
      Servo_Zero=20;
}
void Servo_R90(void)
{
      Servo_Zero=10;
}

u8* ConvertNumber(u16 Number){
  u8 u8Array[4];
  
  int i = 0;
  for (i = 0; i < 4; i++) {
    u8Array[3 - i] = Number % 10;
    Number = Number / 10;
  }

	// for (i = 0; i < 3; i++) {
	// 	printf("%d\n", NumberArray[i]);
	// }
  for(i=0;i<4;i++){
    u8Array[i] = u8Array[i] + 0x30;
  }
  
  return u8Array;
}


void EXTI_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;      
	EXTI_InitTypeDef EXTI_InitStructure;
	     
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
     GPIO_Init(GPIOC, &GPIO_InitStructure);  
        
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
     GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
     GPIO_Init(GPIOC, &GPIO_InitStructure);
     
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
     GPIO_Init(GPIOC, &GPIO_InitStructure);
     
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
     GPIO_Init(GPIOC, &GPIO_InitStructure);
	
      GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);     
     GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
     GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);
     GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
	
     EXTI_InitStructure.EXTI_Line = EXTI_Line0;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  ;  
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);  
        
     EXTI_InitStructure.EXTI_Line = EXTI_Line1;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  ;  
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line =  EXTI_Line2;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  ;  
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);
     
     EXTI_InitStructure.EXTI_Line =  EXTI_Line3;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  ;  
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);
     
     EXTI_InitStructure.EXTI_Line =  EXTI_Line4;
     EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
     EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  ;  
     EXTI_InitStructure.EXTI_LineCmd = ENABLE;
     EXTI_Init(&EXTI_InitStructure);
	
        EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearITPendingBit(EXTI_Line1);
	EXTI_ClearITPendingBit(EXTI_Line2);
        EXTI_ClearITPendingBit(EXTI_Line3);
        EXTI_ClearITPendingBit(EXTI_Line4);
        
     NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQChannel;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
     
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQChannel;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQChannel;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
     
     NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQChannel;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
     
     NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQChannel;
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&NVIC_InitStructure);
}


void Timer2_Init(void)
{

    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
    
    /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 10-1;       // 0.1msec
    TIM_TimeBaseStructure.TIM_Prescaler = 720;      // 1초에 100,000 -> 100usec
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

   // TIM_PrescalerConfig(TIM2, 4, TIM_PSCReloadMode_Immediate);
    
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE); 


     /* TIM IT enable */
    TIM_ITConfig(TIM2, TIM_IT_Update , ENABLE);     


}

void Timer3_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    /* TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
    
    /* Enable the TIM3 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 100-1;       // 0.1msec
    TIM_TimeBaseStructure.TIM_Prescaler = 7200;      // 1초에 100,000 -> 100usec
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

   // TIM_PrescalerConfig(TIM3, 4, TIM_PSCReloadMode_Immediate);
    
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE); 


     /* TIM IT enable */
    TIM_ITConfig(TIM3, TIM_IT_Update , ENABLE);     
}



void Flash_Data_Write(void)
{
  	u8 i;
	u16 i16;
	
	FLASH_Unlock();	
	
	for(i16=0; i16<255; i16+=2){        // Flash ���� ������ ��� �б� 
		Flash_Buffer[i16]= *(u16 *) (StartAddr+i16);  
	}
	
	FLASH_ErasePage(StartAddr);
	
		
	for(i=0; i<50; i++){           //  data Modify
		Flash_Buffer[(i*2)]= 'A';
	}
	
	for(i16=0; i16<255; i16+=2){    // Flash  ������ ��� ����
		FLASH_ProgramHalfWord(StartAddr+i16, Flash_Buffer[i16]);
	}
			
	FLASH_Lock(); 
}


void Flash_Data_Read(void)
{
	u8 i;
	u16 j=0;

	
	FLASH_Unlock();
	
	for(i=0; i<100; i+=2){
		Flash_Buffer[j]= *(u16 *)(StartAddr+i);  
		j++;
	}
	FLASH_Lock();		
}

void USART1_SEND(u8 i)
{
    USART_SendData(USART1, i);     
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);      
}

void putstr(char *str) 
{ 
     char ch;     
     while((ch=*str)!= '\0') { 
	USART_SendData(USART1, *str);     
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);    
	str++; 
     } 
 }



void GPIO_Configuration(void)
{
     GPIO_InitTypeDef GPIO_InitStructure;      
     
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 

     
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;          //USART1, TX
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_Init(GPIOA, &GPIO_InitStructure);   

     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          //USART1, RX
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
     GPIO_Init(GPIOA, &GPIO_InitStructure); 

 
       // KEY
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7 |
                                   GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_13 | GPIO_Pin_15;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
     GPIO_Init(GPIOC, &GPIO_InitStructure);

       // LED
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4; 
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
     GPIO_Init(GPIOE, &GPIO_InitStructure);      
     
       // Analog Input
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
     GPIO_Init(GPIOC, &GPIO_InitStructure); 
     
       // Analog Input
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; 
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
     GPIO_Init(GPIOB, &GPIO_InitStructure); 
     
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   
     GPIO_Init(GPIOD, &GPIO_InitStructure);
     
}  


void USART1_Configuration(void)
{
     USART_InitTypeDef USART_InitStructure;
     USART_ClockInitTypeDef  USART_ClockInitStructure;     
     NVIC_InitTypeDef NVIC_InitStructure;      
     
     USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
     USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
     USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
     USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;
     USART_ClockInit(USART1, &USART_ClockInitStructure);

     USART_InitStructure.USART_BaudRate = 115200;
     USART_InitStructure.USART_WordLength = USART_WordLength_8b;
     USART_InitStructure.USART_StopBits = USART_StopBits_1;
     USART_InitStructure.USART_Parity = USART_Parity_No ;
     USART_InitStructure.USART_HardwareFlowControl =   USART_HardwareFlowControl_None;

     USART_InitStructure.USART_Mode = USART_Mode_Rx |  USART_Mode_Tx;     
     USART_Init(USART1, &USART_InitStructure);         
     USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);              
     NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel; 
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             
     NVIC_Init(&NVIC_InitStructure);     
     USART_Cmd(USART1, ENABLE);
}

void DAC_Configuration(void)
{
	DAC_InitTypeDef DAC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/* DAC channel1 Configuration */
     DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	// DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
  	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Noise;
  	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits11_0;
  	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	//DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is 
     	automatically connected to the DAC converter. */
  	DAC_Cmd(DAC_Channel_1, ENABLE);
}

void ADC_Configuration(void)
{
     ADC_InitTypeDef ADC_InitStructure;

     ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
     ADC_InitStructure.ADC_ScanConvMode = ENABLE;
     ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
     ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
     ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
     ADC_InitStructure.ADC_NbrOfChannel = 1;
     ADC_Init(ADC1, &ADC_InitStructure);

     ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_55Cycles5);
   
     ADC_DMACmd(ADC1, ENABLE);
     ADC_Cmd(ADC1, ENABLE);
     ADC_ResetCalibration(ADC1); 
     
     while(ADC_GetResetCalibrationStatus(ADC1));
        ADC_StartCalibration(ADC1);     
     while(ADC_GetCalibrationStatus(ADC1));
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);    
}

void DMA1_Configuration(void)
 {
 
 	DMA_InitTypeDef DMA_InitStructure; 	

  	DMA_DeInit(DMA1_Channel1);
  	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;         	   // DMA �ֺ���ġ ������ ���� ��巹��
  	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADCConvertedValue;          // DMA �޸� ������ ���� ��巹��
  	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                      // �ֺ���ġ �����͸� ���� �����ͷ�
  	DMA_InitStructure.DMA_BufferSize = 1;                                   // ���� ������ ũ��(byte����)
  	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                // �ڵ����� �ֺ���ġ ������ ��巹�� ���� ����

     DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;  
  	//DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     // �ڵ����� �޸� ������ ��巹�� ���� ����  
  
  	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;      // �ֺ���ġ���� �ѹ��� 16bit
  	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;           // �޸𸮿��� �ѹ��� 16bit
  	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                               // ȯ�� ť ���, ���� ������ ũ�Ⱑ �������� �ٽ� reload
  	DMA_InitStructure.DMA_Priority = DMA_Priority_High;                           // DMA �켱���� ����
  	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                        // �޸� to �޸� ��� �ƴ�
  	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);  //TC:Transfer complete
  
  	/* Enable DMA1 channel1 */
  	DMA_Cmd(DMA1_Channel1, ENABLE); 
}



void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	#ifdef  VECT_TAB_RAM  
  	/* Set the Vector Table base location at 0x20000000 */ 
  	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
	#else  /* VECT_TAB_FLASH  */
  	/* Set the Vector Table base location at 0x08000000 */ 
  	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);   
	#endif
  
 	NVIC_InitStructure.NVIC_IRQChannel=DMA1_Channel1_IRQChannel;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
  	NVIC_Init(&NVIC_InitStructure);  
  
}

void RCC_Configuration(void)
{   
     ErrorStatus HSEStartUpStatus;
     /* RCC system reset(for debug purpose) */
     RCC_DeInit();

     /* Enable HSE */
     RCC_HSEConfig(RCC_HSE_ON);       

     /* Wait till HSE is ready */
     HSEStartUpStatus = RCC_WaitForHSEStartUp();

     if(HSEStartUpStatus == SUCCESS)
     {
          /* Enable Prefetch Buffer */
          FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

          /* Flash 2 wait state */
          FLASH_SetLatency(FLASH_Latency_2);
 	
          /* HCLK = SYSCLK */
          RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
          /* PCLK2 = HCLK */
          RCC_PCLK2Config(RCC_HCLK_Div1); 

          /* PCLK1 = HCLK/2 */
          RCC_PCLK1Config(RCC_HCLK_Div2);

          /* PLLCLK = 8MHz * 9 = 72 MHz */
          RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

          /* Enable PLL */ 
          RCC_PLLCmd(ENABLE);

          /* Wait till PLL is ready */
          while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
          {
          }

          /* Select PLL as system clock source */
          RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

          /* Wait till PLL is used as system clock source */
          while(RCC_GetSYSCLKSource() != 0x08)
          {
          }
     }
}
