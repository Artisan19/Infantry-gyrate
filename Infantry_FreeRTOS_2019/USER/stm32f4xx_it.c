/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "BasicPeripherals.h"
#include "freertostask.h"
#include "RemotDbus.h"
#include "string.h"
#include "Auto_attackTask.h"
#include "RemotDbus.h"
#include "MotorCAN.h"
#include "detect_task.h"
#include "Auto_attackTask.h"
 

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
// 
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


void  EXTI2_IRQHandler()
{	
  if(EXTI_GetITStatus(EXTI_Line2)!=RESET)
  {
  //  LED_RED = !LED_RED;

  }
     EXTI_ClearITPendingBit(EXTI_Line2);
}




//串口中断
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART1);

        if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);
            DMA2_Stream2->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                SBUS_TO_RC(sbus_rx_buffer[0], &rc_ctrl);
                //记录数据接收时间
                DetectHook(DBUSTOE);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                SBUS_TO_RC(sbus_rx_buffer[1], &rc_ctrl);
                //记录数据接收时间
                DetectHook(DBUSTOE);
            }

        }
    }
}



void CAN1_RX0_IRQHandler(void)
{
	 CanRxMsg rx_message; 
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{  
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN1, CAN_FLAG_FF0);			
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);		
	    CAN_hook(CAN1,&rx_message);	    		
	}
}



struct IMU_EXT Imu_ext;
void CAN2_RX0_IRQHandler(void)
{  
	CanRxMsg rx_message; 
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);		
		CAN_ClearFlag(CAN2, CAN_FLAG_FF0);		
		CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
		CAN_hook(CAN2,&rx_message);
	}		

}


void CAN2_TX_IRQHandler()
{
		if(CAN_GetITStatus(CAN2,CAN_IT_TME) !=RESET)
		{
			 CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
		}  
}

void CAN1_TX_IRQHandler()
{
  if(CAN_GetITStatus(CAN1,CAN_IT_TME) !=RESET)
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}

void TIM2_IRQHandler(void)
{
	  if (TIM_GetITStatus(TIM2,TIM_IT_Update)!= RESET) 
		{
		
			  TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
              TIM_ClearFlag(TIM2, TIM_FLAG_Update);   
		}
} 

uint16_t failcnt = 0;
void USART3_IRQHandler()
{
   if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART3);
    }
    else if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {  
		
		static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART3);
		
		
       if(DMA_GetCurrentMemoryTarget(DMA1_Stream1) == 0)
        {
			 //重新设置DMA
            DMA_Cmd(DMA1_Stream1, DISABLE);
            this_time_rx_len = 100 - DMA_GetCurrDataCounter(DMA1_Stream1);
			DMA_SetCurrDataCounter(DMA1_Stream1, 100);
            DMA1_Stream1->CR |= DMA_SxCR_CT;//当前目标存储器为存储器1
		    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
            DMA_Cmd(DMA1_Stream1, ENABLE);
			
			if(this_time_rx_len == 8)
            {
                 //处理裁判系统数据
			   receiveMainFocusData((uint8_t *)mainfocus_rx_buffer[0]);	//read data 
            }
			else
			{
			   failcnt++;
			}
		}
		else
		{
			 //重新设置DMA
            DMA_Cmd(DMA1_Stream1, DISABLE);
            this_time_rx_len = 100 - DMA_GetCurrDataCounter(DMA1_Stream1);
			DMA_SetCurrDataCounter(DMA1_Stream1, 100);
            DMA1_Stream1->CR |= DMA_SxCR_CT;//当前目标存储器为存储器1
		    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
            DMA_Cmd(DMA1_Stream1, ENABLE);
			
			if(this_time_rx_len == 8)
            {
                 //处理裁判系统数据
			   receiveMainFocusData((uint8_t *)mainfocus_rx_buffer[1]);	//read data 
            }
			else
			{
			   failcnt++;
			}		   
		}		     		
    }  
    __nop(); 
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
