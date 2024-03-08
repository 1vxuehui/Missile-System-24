	/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "usart.h"


#include "CRC8_CRC16.h"
#include "protocol.h"
#include "referee.h"
#include "usart.h"
#include "dma.h"

extern DMA_HandleTypeDef hdma_uart8_rx;



/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

 
//extern UART_HandleTypeDef huart8;

uint8_t uart8_buf[2][USART_RX_BUF_LENGHT];

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;

void huart8_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart8.Instance->CR3, USART_CR3_DMAR);
//    SET_BIT(huart8.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart8_rx);
    
    while(hdma_uart8_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart8_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_LISR_TCIF1);

    hdma_uart8_rx.Instance->PAR = (uint32_t) & (UART8->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_uart8_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_uart8_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_uart8_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_uart8_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart8_rx);


    //disable DMA
    //失效DMA
//    __HAL_DMA_DISABLE(&hdma_usart6_tx);

//    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(&hdma_usart6_tx);
//    }

//    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}

/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void referee_usart_task(void const * argument)
{
    init_referee_struct_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
		huart8_init(uart8_buf[0], uart8_buf[1], USART_RX_BUF_LENGHT);

    while(1)
    {
        referee_unpack_fifo_data();
        osDelay(10);
    }
}


/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &referee_unpack_obj;

  while ( fifo_s_used(&referee_fifo) )
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_obj->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            referee_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}


//void USART6_IRQHandler(void)
//{
//    static volatile uint8_t res;
//    if(UART8->SR & UART_FLAG_IDLE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart8);

//        static uint16_t this_time_rx_len = 0;

//        if ((huart8.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(huart8.hdmarx);
//            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart8.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart8.hdmarx, USART_RX_BUF_LENGHT);
//            huart8.hdmarx->Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(huart8.hdmarx);
//            fifo_s_puts(&referee_fifo, (char*)uart8_buf[0], this_time_rx_len);
//            //detect_hook(REFEREE_TOE);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(huart8.hdmarx);
//            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart8.hdmarx);
//            __HAL_DMA_SET_COUNTER(huart8.hdmarx, USART_RX_BUF_LENGHT);
//            huart8.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(huart8.hdmarx);
//            fifo_s_puts(&referee_fifo, (char*)uart8_buf[1], this_time_rx_len);
//            //detect_hook(REFEREE_TOE);
//        }
//    }
//}





