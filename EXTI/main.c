/**
  ******************************************************************************
  * @file    EXTI/EXTI_Config/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup EXTI_Config
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -----------------------  ,.--------------------------------------*/
/* Private variables ---------------------------------------------------------*/
EXTI_InitTypeDef   EXTI_InitStructure;
GPIO_InitTypeDef   GPIO_InitStructure;
NVIC_InitTypeDef   NVIC_InitStructure;
int a =1;
int i =0;
int b =0;

/* Private function prototypes -----------------------------------------------*/
void EXTI0_Config(void);
void EXTI0_IRQHandler(void);
void EXTI1_Config(void);
void EXTI1_IRQHandler(void);
void GPIO_Config(void);
void GPIOA_Config(void);

void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/*user define Func*/
static inline void Delay_1us(uint32_t nCnt_1us);
static inline void Delay(uint32_t nCnt_1us);


int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
  
  /* Initialize LED1 and Key Button mounted on STM3210X-EVAL board */       
  // STM_EVAL_LEDInit(LED1);
  // STM_EVAL_LEDInit(LED2);
  
  /* Configure PA.00 in interrupt mode */
  GPIO_Config();
  EXTI0_Config();
  GPIOA_Config();
  EXTI1_Config();
  /* Configure PB.09 or PG.08 in interrupt mode */
  /* Generate software interrupt: simulate a falling edge applied on EXTI0 line */
  // EXTI_GenerateSWInterrupt(EXTI_Line0);
          
  while (1)
  {
  
  }
}

//Config
void GPIO_Config(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

}
void GPIOA_Config(void)
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void EXTI0_Config(void)
{
  /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_SetPriority(EXTI0_IRQn , NVIC_EncodePriority(4,2,0));
}


void EXTI1_Config(void)
{
  /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Configure PA.00 pin as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

  /* Configure EXTI0 line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_SetPriority(EXTI1_IRQn , NVIC_EncodePriority(4,1,0));
}

/*User define Func*/
void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIOx->ODR ^= GPIO_Pin;
}

static inline void Delay_1us(uint32_t nCnt_1us)
{
  volatile uint32_t nCnt;

  for (; nCnt_1us != 0; nCnt_1us--)
    for (nCnt = 13; nCnt != 0; nCnt--);
}
static inline void Delay(uint32_t nCnt_1us)
{

      while(nCnt_1us--);
}

/*Interrupt Handler*/
void EXTI0_IRQHandler(void)
{
  if(EXTI_GetFlagStatus(EXTI_Line0)!=RESET)
  {
        if (a==0)
        {
          GPIO_Write(GPIOA,0x0081);
          Delay_1us(1000);
        }
        else if (a==1)
        {
          GPIO_Write(GPIOA,0x00F3);
          Delay_1us(1000);
        }
        else if (a==2)
        {
          GPIO_Write(GPIOA,0x0049);
          Delay_1us(1000);
          a++;
        }
        else if (a==3)
        {
          GPIO_Write(GPIOA,0x0061);
          Delay_1us(1000);
        }
        else if (a==4)
        {
          GPIO_Write(GPIOA,0x0033);
          Delay_1us(1000);
        }
        else if(a==5)
        {
          GPIO_Write(GPIOA,0x0025);
          Delay_1us(1000);
        }
        else if (a==6)
        {
          GPIO_Write(GPIOA,0x0005);
          Delay_1us(1000);
        }
        else if (a==7)
        {
          GPIO_Write(GPIOA,0x00F1);
          Delay_1us(1000);
        }
        else if (a==8)
        {
          GPIO_Write(GPIOA,0x0001);
          Delay_1us(1000);
        }
        else if (a==9)
        {
          GPIO_Write(GPIOA,0x0021);
          Delay_1us(1000);
        } 
        a++;
        if (a>9)
        {
          a=0;
        }     
      }
      EXTI_ClearITPendingBit(EXTI_Line0);
} 

void EXTI1_IRQHandler(void)
{  
  if(EXTI_GetFlagStatus(EXTI_Line1)!=RESET)
  {
      if(b==0)
      {
        GPIO_Write(GPIOB,0x0081);
        Delay_1us(1000);
      }
  }
  EXTI_ClearITPendingBit(EXTI_Line1);
} 
