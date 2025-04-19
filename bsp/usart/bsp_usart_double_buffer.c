#include "bsp_usart_double_buffer.h"
#include "usart.h"
#include "bsp_log.h"
#include "stdlib.h"
#include "memory.h"

static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/**
  * @brief  Init the multi_buffer DMA Transfer with interrupt enabled.
  * @param  huart       pointer to a UART_HandleTypeDef structure that contains
  *                     the configuration information for the specified USART Stream.  
  * @param  SrcAddress pointer to The source memory Buffer address
  * @param  DstAddress pointer to The destination memory Buffer address
  * @param  SecondMemAddress pointer to The second memory Buffer address in case of multi buffer Transfer  
  * @param  DataLength The length of data to be transferred from source to destination
  * @retval none
  */
 static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, unsigned *DstAddress, unsigned *SecondMemAddress, unsigned DataLength){

    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
   
    huart->RxXferSize    = DataLength * 2;
   
    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
   
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
           
     do{
         __HAL_DMA_DISABLE(huart->hdmarx);
     }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);
   
     /* Configure the source memory Buffer address  */
     ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;
   
     /* Configure the destination memory Buffer address */
     ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
   
     /* Configure DMA Stream destination address */
     ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;
   
     /* Configure the length of data to be transferred from source to destination */
     ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;
   
     /* Enable double memory buffer */
     SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);
   
     /* Enable DMA */
     __HAL_DMA_ENABLE(huart->hdmarx);	
       
       
   }

void USARTMulServiceInit(USARTInstance *instance)
{
    USART_RxDMA_MultiBuffer_Init(instance->usart_handle, (unsigned)instance->recv_buff[0], (unsigned)instance->recv_buff[1], instance->recv_buff_size);
}


USARTInstance *USARTMulRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    USARTInstance *instance = (USARTInstance *)malloc(sizeof(USARTInstance));
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usart_instance[idx++] = instance;
    USARTServiceInit(instance);
    return instance;
}

