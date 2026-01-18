#include "stm32f10x.h"

// 使用SPI1（常用引脚）
// PA5: SCK, PA6: MISO, PA7: MOSI, PA4: NSS

void SPI1_Master_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    
    // 1. 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);
    
    // 2. 配置SPI引脚
    // SCK, MOSI, NSS配置为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // MISO配置为浮空输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 配置SPI1参数
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // 全双工
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;     // 主机模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // 8位数据
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;        // 时钟极性：低电平空闲
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;      // 时钟相位：第一个边沿采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;         // 软件NSS控制
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 分频系数
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // 高位在前
    SPI_InitStructure.SPI_CRCPolynomial = 7;          // CRC多项式
    
    SPI_Init(SPI1, &SPI_InitStructure);
    
    // 4. 使能SPI1
    SPI_Cmd(SPI1, ENABLE);
}

/**
  * @brief  SPI主机发送一个字节（同时接收）
  * @param  data: 要发送的数据
  * @retval 接收到的数据
  */
uint8_t SPI1_Master_SendByte(uint8_t data)
{
    // 等待发送缓冲区空
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    
    // 发送数据
    SPI_I2S_SendData(SPI1, data);
    
    // 等待接收缓冲区非空
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    
    // 返回接收到的数据
    return SPI_I2S_ReceiveData(SPI1);
}

/**
  * @brief  SPI主机发送数据块
  * @param  tx_buffer: 发送缓冲区
  * @param  rx_buffer: 接收缓冲区（可为NULL，如果不关心接收）
  * @param  size: 数据大小
  * @retval 无
  */
void SPI1_Master_Transmit(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t size)
{
    uint16_t i;
    
    // 拉低NSS（选中从机）
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    
    for(i = 0; i < size; i++)
    {
        uint8_t received = SPI1_Master_SendByte(tx_buffer[i]);
        
        // 只有在rx_buffer不为空时才保存接收数据
        if(rx_buffer != 0)
        {
            rx_buffer[i] = received;
        }
    }
    
    // 拉高NSS（取消选中）
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
}
