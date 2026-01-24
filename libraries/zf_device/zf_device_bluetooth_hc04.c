/*********************************************************************************************************************
* MM32F327X-G8P Opensourec Library 即（MM32F327X-G8P 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
* 
* 本文件是 MM32F327X-G8P 开源库的一部分
* 
* MM32F327X-G8P 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
* 
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
* 
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
* 
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
* 
* 文件名称          zf_device_bluetooth_hc04
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 8.32.4 or MDK 5.37
* 适用平台          MM32F327X_G8P
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2024-XX-XX        [Your Name]         新增 HC-04 蓝牙模块驱动
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RX                  查看 zf_device_bluetooth_hc04.h 中 BLUETOOTH_HC04_RX_PIN 宏定义
*                   TX                  查看 zf_device_bluetooth_hc04.h 中 BLUETOOTH_HC04_TX_PIN 宏定义
*                   VCC                 3.3V电源
*                   GND                 电源地
*                   其余引脚悬空
*                   ------------------------------------
* 注意：HC-04蓝牙模块不支持硬件流控（RTS/CTS），因此不需要连接RTS引脚
********************************************************************************************************************/

#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_gpio.h"
#include "zf_driver_uart.h"
#include "zf_driver_delay.h"
#include "zf_device_type.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "zf_device_bluetooth_hc04.h"

//-------------------------------------------------------------------------------------------------------------------
// 全局变量
//-------------------------------------------------------------------------------------------------------------------
static  fifo_struct                             bluetooth_hc04_fifo;
static  uint8                                   bluetooth_hc04_buffer[BLUETOOTH_HC04_BUFFER_SIZE];
static  uint8                                   bluetooth_hc04_data = 0;

// 新增：接收相关变量
static  bluetooth_hc04_joystick_string_data_t   string_data;          // 字符串格式的接收数据
static  bluetooth_hc04_joystick_numeric_data_t  numeric_data;         // 数值格式的接收数据
static  bluetooth_hc04_rx_callback_t            rx_callback = NULL;   // 字符串回调函数
static  bluetooth_hc04_numeric_callback_t       numeric_callback = NULL; // 数值回调函数
static  char                                    frame_buffer[BLUETOOTH_HC04_FRAME_BUFFER_SIZE]; // 帧缓冲区
static  uint16                                  frame_index = 0;      // 帧缓冲区索引
static  uint8                                   frame_start_char = '['; // 帧起始字符
static  uint8                                   frame_end_char = ']';   // 帧结束字符
static  uint8                                   parsing_frame = 0;    // 是否正在解析帧

// 摇杆命令映射表
static const joystick_command_string_t joystick_commands[] = {
    {"STOP", 0},
    {"FORWARD", 1},
    {"BACKWARD", 2},
    {"LEFT", 3},
    {"RIGHT", 4},
    {"BUTTON1", 5},
    {"BUTTON2", 6},
    {"BUTTON3", 7},
    {"BUTTON4", 8},
    {"UP", 9},
    {"DOWN", 10},
    {"CENTER", 11},
    {NULL, 0}  // 结束标志
};

//-------------------------------------------------------------------------------------------------------------------
// 私有函数声明
//-------------------------------------------------------------------------------------------------------------------
static uint8 parse_joystick_string_frame(const char *frame);
static void  convert_string_to_numeric(void);
static void  trim_string(char *str);
static uint8 extract_string_between_quotes(const char *src, char *dest, uint8 dest_size);
static uint8 find_next_quote(const char *str, uint16 start_pos);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送数据
// 参数说明     data            8bit 数据
// 返回参数     uint32          剩余发送长度
// 使用示例     bluetooth_hc04_send_byte(0x5A);
// 备注信息     HC-04不支持硬件流控，直接发送
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_hc04_send_byte (const uint8 data)
{
    uart_write_byte(BLUETOOTH_HC04_INDEX, data);    // 直接发送数据
    return 0;                                       // 返回0表示发送成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送函数
// 参数说明     buff             需要发送的数据地址
// 参数说明     len              发送长度
// 返回参数     uint32           剩余未发送的字节数
// 使用示例     bluetooth_hc04_send_buffer(buff, 16);
// 备注信息     HC-04不支持硬件流控，直接发送
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_hc04_send_buffer (const uint8 *buff, uint32 len)
{
    zf_assert(NULL != buff);
    
    // 直接发送整个缓冲区
    uart_write_buffer(BLUETOOTH_HC04_INDEX, buff, len);
    return 0;                                       // 返回0表示全部发送成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送字符串
// 参数说明     *str            要发送的字符串地址
// 返回参数     uint32          剩余发送长度
// 使用示例     bluetooth_hc04_send_string("Trust yourself.");
// 备注信息     HC-04不支持硬件流控，直接发送
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_hc04_send_string (const char *str)
{
    zf_assert(NULL != str);
    
    // 直接发送字符串
    uart_write_string(BLUETOOTH_HC04_INDEX, str);
    return 0;                                       // 返回0表示发送成功
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送摄像头图像至上位机查看图像
// 参数说明     *image_addr     需要发送的图像地址
// 参数说明     image_size      图像的大小
// 返回参数     void
// 使用示例     bluetooth_hc04_send_image(&mt9v03x_image[0][0], MT9V03X_IMAGE_SIZE);
// 备注信息     由于HC-04波特率较低，发送大图像可能需要较长时间
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_image (const uint8 *image_addr, uint32 image_size)
{
    zf_assert(NULL != image_addr);
    
    extern uint8 camera_send_image_frame_header[4];
    
    // 先发送帧头
    bluetooth_hc04_send_buffer(camera_send_image_frame_header, 4);
    
    // 发送图像数据（分块发送以避免阻塞）
    uint32 sent = 0;
    uint32 block_size = 64;  // 每次发送64字节
    
    while (sent < image_size) {
        uint32 to_send = (image_size - sent > block_size) ? block_size : (image_size - sent);
        bluetooth_hc04_send_buffer(image_addr + sent, to_send);
        sent += to_send;
        
        // 短暂延时，让蓝牙模块有时间处理
        system_delay_ms(1);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 读取函数
// 参数说明     buff            存储的数据地址
// 参数说明     len             长度
// 返回参数     uint32          实际读取字节数
// 使用示例     bluetooth_hc04_read_buffer(buff, 16);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_hc04_read_buffer (uint8 *buff, uint32 len)
{
    zf_assert(NULL != buff);
    uint32 data_len = len;
    fifo_read_buffer(&bluetooth_hc04_fifo, buff, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 串口中断回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     
// 备注信息     该函数在 ISR 文件的串口中断程序被调用
//              由串口中断服务函数调用 wireless_module_uart_handler() 函数
//              再由 wireless_module_uart_handler() 函数调用本函数
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_uart_callback (void)
{
    uart_query_byte(BLUETOOTH_HC04_INDEX, &bluetooth_hc04_data);  // 读取串口数据
    
    // 将数据存入FIFO
    fifo_write_buffer(&bluetooth_hc04_fifo, &bluetooth_hc04_data, 1);
    
    // 帧解析：检查是否是帧起始字符
    if (bluetooth_hc04_data == frame_start_char) {
        parsing_frame = 1;
        frame_index = 0;
        frame_buffer[frame_index++] = bluetooth_hc04_data;
    }
    // 如果正在解析帧，则存储数据
    else if (parsing_frame && frame_index < BLUETOOTH_HC04_FRAME_BUFFER_SIZE - 1) {
        frame_buffer[frame_index++] = bluetooth_hc04_data;
        
        // 检查是否是帧结束字符
        if (bluetooth_hc04_data == frame_end_char) {
            frame_buffer[frame_index] = '\0';  // 添加字符串结束符
            parsing_frame = 0;
            
            // 解析帧数据（字符串格式）
            if (parse_joystick_string_frame(frame_buffer)) {
                // 设置时间戳
                //string_data.timestamp = systick_get_ms();
                
                // 转换为数值格式
                convert_string_to_numeric();
                numeric_data.timestamp = string_data.timestamp;
                
                // 如果设置了回调函数，则调用回调函数
                if (rx_callback != NULL) {
                    rx_callback(&string_data);
                }
                
                if (numeric_callback != NULL) {
                    numeric_callback(&numeric_data);
                }
                
                #ifdef BLUETOOTH_HC04_DEBUG
                bluetooth_hc04_printf("[HC-04] Received: [\"%s\",\"%s\",\"%s\",\"%s\",\"%s\"]\r\n",
                                     string_data.joystick, string_data.data1,
                                     string_data.data2, string_data.data3,
                                     string_data.data4);
                #endif
            } else {
                #ifdef BLUETOOTH_HC04_DEBUG
                bluetooth_hc04_printf("[HC-04] Parse failed: %s\r\n", frame_buffer);
                #endif
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 初始化
// 参数说明     void
// 返回参数     uint8           初始化状态 0-成功 1-失败
// 使用示例     bluetooth_hc04_init();
// 备注信息     默认波特率为9600，如需115200请使用AT命令设置
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_init (void)
{
    uint8 return_state = 0;
    
    // 注意：需要在zf_device_type.h中添加BLUETOOTH_HC04枚举值
    set_wireless_type(BLUETOOTH_HC04, bluetooth_hc04_uart_callback);

    // 初始化FIFO缓冲区
    fifo_init(&bluetooth_hc04_fifo, FIFO_DATA_8BIT, bluetooth_hc04_buffer, BLUETOOTH_HC04_BUFFER_SIZE);
    
    // 初始化接收数据结构
    memset(&string_data, 0, sizeof(string_data));
    memset(&numeric_data, 0, sizeof(numeric_data));
    memset(frame_buffer, 0, sizeof(frame_buffer));
    frame_index = 0;
    parsing_frame = 0;
    
    // HC-04默认波特率为9600或115200，根据模块设置
    // 初始化串口（HC-04没有硬件流控引脚，不需要初始化RTS）
    // 注意：TX和RX引脚在uart_init函数中的参数顺序是：TX引脚，RX引脚
    uart_init(BLUETOOTH_HC04_INDEX, BLUETOOTH_HC04_BAUD_RATE, 
              BLUETOOTH_HC04_TX_PIN, BLUETOOTH_HC04_RX_PIN);
    uart_rx_interrupt(BLUETOOTH_HC04_INDEX, 1);

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 查询是否有数据可读
// 参数说明     void
// 返回参数     uint32          可读取的字节数
// 使用示例     if (bluetooth_hc04_available() > 0) { ... }
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint32 bluetooth_hc04_available (void)
{
    // 使用fifo可用数据长度函数
    return fifo_used(&bluetooth_hc04_fifo);
}

//-------------------------------------------------------------------------------------------------------------------
// 私有函数：字符串去空格
// 参数说明     str             要处理的字符串
// 返回参数     void
// 使用示例     trim_string(buffer);
// 备注信息     去除字符串首尾的空格
//-------------------------------------------------------------------------------------------------------------------
static void trim_string(char *str)
{
    int i, start = 0, end = strlen(str) - 1;
    
    // 找到第一个非空格字符
    while (str[start] == ' ' || str[start] == '\t' || str[start] == '\n' || str[start] == '\r')
        start++;
    
    // 找到最后一个非空格字符
    while (end >= start && (str[end] == ' ' || str[end] == '\t' || str[end] == '\n' || str[end] == '\r'))
        end--;
    
    // 移动字符串
    for (i = start; i <= end; i++)
        str[i - start] = str[i];
    str[end - start + 1] = '\0';
}

//-------------------------------------------------------------------------------------------------------------------
// 私有函数：查找下一个引号位置
// 参数说明     str             要搜索的字符串
// 参数说明     start_pos       开始搜索的位置
// 返回参数     uint8           1-找到 0-未找到
// 使用示例     find_next_quote(buffer, 0);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 find_next_quote(const char *str, uint16 start_pos)
{
    uint16 len = strlen(str);
    for (uint16 i = start_pos; i < len; i++) {
        if (str[i] == '"' || str[i] == '\'') {
            return i;
        }
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 私有函数：提取引号之间的字符串
// 参数说明     src             源字符串
// 参数说明     dest            目标缓冲区
// 参数说明     dest_size       目标缓冲区大小
// 返回参数     uint8           1-成功 0-失败
// 使用示例     extract_string_between_quotes("\"hello\"", buffer, 32);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static uint8 extract_string_between_quotes(const char *src, char *dest, uint8 dest_size)
{
    uint16 len = strlen(src);
    uint16 start = 0, end = 0;
    
    // 查找第一个引号
    for (uint16 i = 0; i < len; i++) {
        if (src[i] == '"' || src[i] == '\'') {
            start = i + 1;
            break;
        }
    }
    
    if (start == 0) return 0;  // 未找到引号
    
    // 查找第二个引号
    for (uint16 i = start; i < len; i++) {
        if (src[i] == '"' || src[i] == '\'') {
            end = i;
            break;
        }
    }
    
    if (end <= start) return 0;  // 未找到匹配的引号
    
    // 计算字符串长度
    uint16 str_len = end - start;
    if (str_len >= dest_size) {
        str_len = dest_size - 1;
    }
    
    // 复制字符串
    strncpy(dest, src + start, str_len);
    dest[str_len] = '\0';
    
    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 私有函数：解析字符串格式的摇杆数据帧（格式：["joystick","0","0","0","0"]）
// 参数说明     frame           包含完整帧的字符串
// 返回参数     uint8           1-解析成功 0-解析失败
// 使用示例     parse_joystick_string_frame("[\"FORWARD\",\"100\",\"200\",\"300\",\"400\"]");
// 备注信息     支持格式：["string1","string2","string3","string4","string5"]
//-------------------------------------------------------------------------------------------------------------------
static uint8 parse_joystick_string_frame(const char *frame)
{
    char temp_buffer[BLUETOOTH_HC04_FRAME_BUFFER_SIZE];
    char *token, *next_token;
    uint8 count = 0;
    
    // 验证帧格式
    if (frame[0] != frame_start_char || frame[strlen(frame)-1] != frame_end_char) {
        return 0;
    }
    
    // 复制帧数据到临时缓冲区
    strncpy(temp_buffer, frame, sizeof(temp_buffer));
    temp_buffer[sizeof(temp_buffer) - 1] = '\0';
    
    // 重置接收数据结构
    memset(&string_data, 0, sizeof(string_data));
    string_data.valid = 0;
    
    // 手动解析字符串，处理引号内的内容
    uint16 pos = 0;
    uint16 len = strlen(temp_buffer);
    
    // 跳过起始括号
    if (temp_buffer[pos] == '[') pos++;
    
    while (pos < len && count < 5) {
        // 跳过逗号和空格
        while (pos < len && (temp_buffer[pos] == ',' || temp_buffer[pos] == ' ')) pos++;
        
        // 检查是否到达结尾
        if (pos >= len || temp_buffer[pos] == ']') break;
        
        // 检查是否有引号
        if (temp_buffer[pos] == '"' || temp_buffer[pos] == '\'') {
            uint16 start = pos + 1;  // 跳过引号
            uint16 end = start;
            
            // 查找结束引号
            while (end < len && temp_buffer[end] != temp_buffer[pos]) {
                end++;
            }
            
            if (end < len) {
                // 提取引号内的内容
                uint16 str_len = end - start;
                if (str_len >= BLUETOOTH_HC04_MAX_STRING_LEN) {
                    str_len = BLUETOOTH_HC04_MAX_STRING_LEN - 1;
                }
                
                switch (count) {
                    case 0:  // joystick
                        strncpy(string_data.joystick, temp_buffer + start, str_len);
                        string_data.joystick[str_len] = '\0';
                        break;
                    case 1:  // data1
                        strncpy(string_data.data1, temp_buffer + start, str_len);
                        string_data.data1[str_len] = '\0';
                        break;
                    case 2:  // data2
                        strncpy(string_data.data2, temp_buffer + start, str_len);
                        string_data.data2[str_len] = '\0';
                        break;
                    case 3:  // data3
                        strncpy(string_data.data3, temp_buffer + start, str_len);
                        string_data.data3[str_len] = '\0';
                        break;
                    case 4:  // data4
                        strncpy(string_data.data4, temp_buffer + start, str_len);
                        string_data.data4[str_len] = '\0';
                        break;
                }
                
                count++;
                pos = end + 1;  // 移动到引号之后
            } else {
                // 引号不匹配
                return 0;
            }
        } else {
            // 没有引号，直接读取到逗号或结束符
            uint16 start = pos;
            uint16 end = start;
            
            while (end < len && temp_buffer[end] != ',' && temp_buffer[end] != ']') {
                end++;
            }
            
            uint16 str_len = end - start;
            if (str_len >= BLUETOOTH_HC04_MAX_STRING_LEN) {
                str_len = BLUETOOTH_HC04_MAX_STRING_LEN - 1;
            }
            
            switch (count) {
                case 0:  // joystick
                    strncpy(string_data.joystick, temp_buffer + start, str_len);
                    string_data.joystick[str_len] = '\0';
                    break;
                case 1:  // data1
                    strncpy(string_data.data1, temp_buffer + start, str_len);
                    string_data.data1[str_len] = '\0';
                    break;
                case 2:  // data2
                    strncpy(string_data.data2, temp_buffer + start, str_len);
                    string_data.data2[str_len] = '\0';
                    break;
                case 3:  // data3
                    strncpy(string_data.data3, temp_buffer + start, str_len);
                    string_data.data3[str_len] = '\0';
                    break;
                case 4:  // data4
                    strncpy(string_data.data4, temp_buffer + start, str_len);
                    string_data.data4[str_len] = '\0';
                    break;
            }
            
            count++;
            pos = end;
        }
    }
    
    // 验证是否成功解析了5个数据
    if (count == 5) {
        string_data.valid = 1;
        
        // 去除所有字符串的空格
        trim_string(string_data.joystick);
        trim_string(string_data.data1);
        trim_string(string_data.data2);
        trim_string(string_data.data3);
        trim_string(string_data.data4);
        
        return 1;
    }
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 私有函数：将字符串数据转换为数值数据
// 参数说明     void
// 返回参数     void
// 使用示例     convert_string_to_numeric();
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
static void convert_string_to_numeric(void)
{
    memset(&numeric_data, 0, sizeof(numeric_data));
    
    if (string_data.valid) {
        numeric_data.valid = 1;
        
        // 转换joystick字符串为数值（如果是数字）
        if (isdigit(string_data.joystick[0]) || 
            (string_data.joystick[0] == '-' && isdigit(string_data.joystick[1]))) {
            numeric_data.joystick_value = atoi(string_data.joystick);
        } else {
            // 如果是字符串命令，查找对应的ID
            numeric_data.joystick_value = -1;  // 默认值
            for (uint8 i = 0; joystick_commands[i].command != NULL; i++) {
                if (strcasecmp(string_data.joystick, joystick_commands[i].command) == 0) {
                    numeric_data.joystick_value = joystick_commands[i].id;
                    break;
                }
            }
        }
        
        // 转换其他字符串为数值
        numeric_data.data1_value = atoi(string_data.data1);
        numeric_data.data2_value = atoi(string_data.data2);
        numeric_data.data3_value = atoi(string_data.data3);
        numeric_data.data4_value = atoi(string_data.data4);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置字符串接收回调函数
// 参数说明     callback        回调函数指针
// 返回参数     uint8           1-设置成功 0-设置失败
// 使用示例     bluetooth_hc04_set_rx_callback(my_callback);
// 备注信息     当接收到完整帧时，会自动调用回调函数
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_set_rx_callback(bluetooth_hc04_rx_callback_t callback)
{
    if (callback != NULL) {
        rx_callback = callback;
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置数值接收回调函数
// 参数说明     callback        回调函数指针
// 返回参数     uint8           1-设置成功 0-设置失败
// 使用示例     bluetooth_hc04_set_numeric_callback(my_callback);
// 备注信息     当接收到完整帧并转换为数值后，会自动调用回调函数
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_set_numeric_callback(bluetooth_hc04_numeric_callback_t callback)
{
    if (callback != NULL) {
        numeric_callback = callback;
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     手动解析数据帧
// 参数说明     void
// 返回参数     uint8           1-解析到有效数据 0-无有效数据
// 使用示例     if (bluetooth_hc04_parse_data_frame()) { ... }
// 备注信息     用于在主循环中手动解析接收到的数据
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_parse_data_frame(void)
{
    static uint8 rx_buffer[BLUETOOTH_HC04_BUFFER_SIZE];
    static uint16 buffer_index = 0;
    uint32 available_bytes;
    uint8 result = 0;
    
    // 检查FIFO中是否有数据
    available_bytes = bluetooth_hc04_available();
    if (available_bytes == 0) {
        return 0;
    }
    
    // 读取所有可用数据
    uint32 read_bytes = bluetooth_hc04_read_buffer(rx_buffer + buffer_index, 
                                                   sizeof(rx_buffer) - buffer_index - 1);
    buffer_index += read_bytes;
    rx_buffer[buffer_index] = '\0';  // 确保字符串结束
    
    // 在缓冲区中查找帧
    for (uint16 i = 0; i < buffer_index; i++) {
        // 查找帧起始字符
        if (rx_buffer[i] == frame_start_char) {
            // 查找帧结束字符
            for (uint16 j = i + 1; j < buffer_index; j++) {
                if (rx_buffer[j] == frame_end_char) {
                    // 找到完整帧
                    uint16 frame_length = j - i + 1;
                    
                    // 确保帧缓冲区足够大
                    if (frame_length < BLUETOOTH_HC04_FRAME_BUFFER_SIZE) {
                        memcpy(frame_buffer, rx_buffer + i, frame_length);
                        frame_buffer[frame_length] = '\0';
                        
                        // 解析帧（字符串格式）
                        if (parse_joystick_string_frame(frame_buffer)) {
                            // 设置时间戳
//                            string_data.timestamp = systick_get_ms();
                            
                            // 转换为数值格式
                            convert_string_to_numeric();
                            numeric_data.timestamp = string_data.timestamp;
                            
                            result = 1;
                            
                            #ifdef BLUETOOTH_HC04_DEBUG
                            bluetooth_hc04_printf("[HC-04] Parsed: [\"%s\",\"%s\",\"%s\",\"%s\",\"%s\"]\r\n",
                                                 string_data.joystick, string_data.data1,
                                                 string_data.data2, string_data.data3,
                                                 string_data.data4);
                            #endif
                        }
                        
                        // 移除已处理的数据
                        uint16 remaining = buffer_index - j - 1;
                        if (remaining > 0) {
                            memmove(rx_buffer, rx_buffer + j + 1, remaining);
                        }
                        buffer_index = remaining;
                        
                        // 返回成功
                        return result;
                    }
                }
            }
        }
    }
    
    // 如果缓冲区已满但没有找到完整帧，清空缓冲区
    if (buffer_index >= sizeof(rx_buffer) - 1) {
        buffer_index = 0;
    }
    
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取字符串格式的摇杆数据
// 参数说明     data            存储数据的结构体指针
// 返回参数     uint8           1-数据有效 0-数据无效
// 使用示例     bluetooth_hc04_joystick_string_data_t data;
//              if (bluetooth_hc04_get_joystick_string_data(&data)) { ... }
// 备注信息     获取最近一次解析的摇杆数据（字符串格式）
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_get_joystick_string_data(bluetooth_hc04_joystick_string_data_t *data)
{
    if (data != NULL && string_data.valid) {
        memcpy(data, &string_data, sizeof(bluetooth_hc04_joystick_string_data_t));
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取数值格式的摇杆数据
// 参数说明     data            存储数据的结构体指针
// 返回参数     uint8           1-数据有效 0-数据无效
// 使用示例     bluetooth_hc04_joystick_numeric_data_t data;
//              if (bluetooth_hc04_get_joystick_numeric_data(&data)) { ... }
// 备注信息     获取最近一次解析的摇杆数据（数值格式）
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_get_joystick_numeric_data(bluetooth_hc04_joystick_numeric_data_t *data)
{
    if (data != NULL && numeric_data.valid) {
        memcpy(data, &numeric_data, sizeof(bluetooth_hc04_joystick_numeric_data_t));
        return 1;
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查数据是否有效
// 参数说明     void
// 返回参数     uint8           1-数据有效 0-数据无效
// 使用示例     if (bluetooth_hc04_is_data_valid()) { ... }
// 备注信息     检查最近接收的数据是否有效
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_is_data_valid(void)
{
    return string_data.valid;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取所有字符串数据
// 参数说明     joystick        摇杆字符串缓冲区
// 参数说明     data1           第一个数据缓冲区
// 参数说明     data2           第二个数据缓冲区
// 参数说明     data3           第三个数据缓冲区
// 参数说明     data4           第四个数据缓冲区
// 返回参数     void
// 使用示例     char js[32], d1[32], d2[32], d3[32], d4[32];
//              bluetooth_hc04_get_all_strings(js, d1, d2, d3, d4);
// 备注信息     获取所有五个字符串数据
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_get_all_strings(char *joystick, char *data1, char *data2, char *data3, char *data4)
{
    if (joystick != NULL) strcpy(joystick, string_data.joystick);
    if (data1 != NULL) strcpy(data1, string_data.data1);
    if (data2 != NULL) strcpy(data2, string_data.data2);
    if (data3 != NULL) strcpy(data3, string_data.data3);
    if (data4 != NULL) strcpy(data4, string_data.data4);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取所有数值数据
// 参数说明     joystick_val    摇杆数值指针
// 参数说明     data1_val       第一个数值指针
// 参数说明     data2_val       第二个数值指针
// 参数说明     data3_val       第三个数值指针
// 参数说明     data4_val       第四个数值指针
// 返回参数     void
// 使用示例     int32 js, d1, d2, d3, d4;
//              bluetooth_hc04_get_all_numeric(&js, &d1, &d2, &d3, &d4);
// 备注信息     获取所有五个数值数据
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_get_all_numeric(int32 *joystick_val, int32 *data1_val, int32 *data2_val, 
                                    int32 *data3_val, int32 *data4_val)
{
    if (joystick_val != NULL) *joystick_val = numeric_data.joystick_value;
    if (data1_val != NULL) *data1_val = numeric_data.data1_value;
    if (data2_val != NULL) *data2_val = numeric_data.data2_value;
    if (data3_val != NULL) *data3_val = numeric_data.data3_value;
    if (data4_val != NULL) *data4_val = numeric_data.data4_value;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     字符串转换为整数
// 参数说明     str             要转换的字符串
// 返回参数     int32           转换后的整数
// 使用示例     int32 value = bluetooth_hc04_string_to_int("123");
// 备注信息     安全的字符串转整数函数
//-------------------------------------------------------------------------------------------------------------------
int32 bluetooth_hc04_string_to_int(const char *str)
{
    if (str == NULL || strlen(str) == 0) return 0;
    return atoi(str);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     比较两个字符串（不区分大小写）
// 参数说明     str1            第一个字符串
// 参数说明     str2            第二个字符串
// 返回参数     uint8           1-相等 0-不相等
// 使用示例     if (bluetooth_hc04_compare_string("FORWARD", joystick)) { ... }
// 备注信息     不区分大小写的字符串比较
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_compare_string(const char *str1, const char *str2)
{
    if (str1 == NULL || str2 == NULL) return 0;
    
    // 简单的不区分大小写比较
    uint16 i = 0;
    while (str1[i] && str2[i]) {
        char c1 = tolower(str1[i]);
        char c2 = tolower(str2[i]);
        if (c1 != c2) return 0;
        i++;
    }
    
    return (str1[i] == '\0' && str2[i] == '\0');
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查是否为指定命令
// 参数说明     command         接收到的命令字符串
// 参数说明     target          目标命令字符串
// 返回参数     uint8           1-是目标命令 0-不是目标命令
// 使用示例     if (bluetooth_hc04_is_command(string_data.joystick, "FORWARD")) { ... }
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
uint8 bluetooth_hc04_is_command(const char *command, const char *target)
{
    return bluetooth_hc04_compare_string(command, target);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置帧格式
// 参数说明     start_char      帧起始字符
// 参数说明     end_char        帧结束字符
// 返回参数     void
// 使用示例     bluetooth_hc04_set_frame_format('<', '>');
// 备注信息     默认格式为'['和']'
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_set_frame_format(uint8 start_char, uint8 end_char)
{
    frame_start_char = start_char;
    frame_end_char = end_char;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清除接收缓冲区
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_hc04_clear_rx_buffer();
// 备注信息     清除FIFO缓冲区和帧解析状态
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_clear_rx_buffer(void)
{
    uint8 dummy;
    uint32 available = bluetooth_hc04_available();
    
    // 清空FIFO
    while (available > 0) {
        bluetooth_hc04_read_buffer(&dummy, 1);
        available--;
    }
    
    // 重置帧解析状态
    parsing_frame = 0;
    frame_index = 0;
    memset(frame_buffer, 0, sizeof(frame_buffer));
    
    // 重置接收数据
    memset(&string_data, 0, sizeof(string_data));
    memset(&numeric_data, 0, sizeof(numeric_data));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 AT命令模式
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_hc04_enter_at_mode();
// 备注信息     进入AT命令模式前需要将KEY引脚拉高（如果支持）
//              此函数为可选功能，具体实现取决于硬件连接
//-------------------------------------------------------------------------------------------------------------------
#ifdef BLUETOOTH_HC04_KEY_PIN
void bluetooth_hc04_enter_at_mode (void)
{
    // 如果模块支持KEY引脚进入AT模式
    gpio_init(BLUETOOTH_HC04_KEY_PIN, GPO, 0, GPO_PUSH_PULL);
    gpio_set_level(BLUETOOTH_HC04_KEY_PIN, 1);      // 拉高KEY引脚进入AT模式
    system_delay_ms(100);
    
    // 发送AT命令测试
    bluetooth_hc04_send_string("AT\r\n");
    system_delay_ms(100);
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送浮点数
// 参数说明     value           浮点数值
// 参数说明     decimal_places  小数位数
// 返回参数     void
// 使用示例     bluetooth_hc04_send_float(3.14159, 3); // 输出: 3.142
// 备注信息     使用格式化字符串发送浮点数
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_float (float value, uint8 decimal_places)
{
    char buffer[32];
    
    // 根据指定的小数位数格式化浮点数
    switch (decimal_places) {
        case 0:
            sprintf(buffer, "%d", (int)value);
            break;
        case 1:
            sprintf(buffer, "%.1f", value);
            break;
        case 2:
            sprintf(buffer, "%.2f", value);
            break;
        case 3:
            sprintf(buffer, "%.3f", value);
            break;
        case 4:
            sprintf(buffer, "%.4f", value);
            break;
        case 5:
            sprintf(buffer, "%.5f", value);
            break;
        case 6:
            sprintf(buffer, "%.6f", value);
            break;
        default:
            sprintf(buffer, "%f", value);
            break;
    }
    
    bluetooth_hc04_send_string(buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送带标签的浮点数
// 参数说明     value           浮点数值
// 参数说明     label           标签字符串
// 参数说明     decimal_places  小数位数
// 返回参数     void
// 使用示例     bluetooth_hc04_send_float_with_label(45.67, "Angle", 2);
//              输出: Angle: 45.67
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_float_with_label (float value, const char* label, uint8 decimal_places)
{
    char buffer[64];
    
    // 根据指定的小数位数格式化浮点数
    switch (decimal_places) {
        case 0:
            sprintf(buffer, "%s: %d", label, (int)value);
            break;
        case 1:
            sprintf(buffer, "%s: %.1f", label, value);
            break;
        case 2:
            sprintf(buffer, "%s: %.2f", label, value);
            break;
        case 3:
            sprintf(buffer, "%s: %.3f", label, value);
            break;
        case 4:
            sprintf(buffer, "%s: %.4f", label, value);
            break;
        case 5:
            sprintf(buffer, "%s: %.5f", label, value);
            break;
        case 6:
            sprintf(buffer, "%s: %.6f", label, value);
            break;
        default:
            sprintf(buffer, "%s: %f", label, value);
            break;
    }
    
    bluetooth_hc04_send_string(buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 printf格式化输出（类似printf功能）
// 参数说明     format          格式化字符串
// 参数说明     ...             可变参数
// 返回参数     int             实际发送的字符数
// 使用示例     bluetooth_hc04_printf("Angle: %.2f, Speed: %d", angle, speed);
// 备注信息     需要启用BLUETOOTH_HC04_USE_PRINTF宏定义
//-------------------------------------------------------------------------------------------------------------------
int bluetooth_hc04_printf (const char* format, ...)
{
    char buffer[128];
    va_list args;
    int length;
    
    va_start(args, format);
    length = vsprintf(buffer, format, args);
    va_end(args);
    
    if (length > 0) {
        bluetooth_hc04_send_buffer((uint8*)buffer, length);
    }
    
    return length;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送整数
// 参数说明     value           整数值
// 返回参数     void
// 使用示例     bluetooth_hc04_send_int(12345);
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_int (int32 value)
{
    char buffer[16];
    sprintf(buffer, "%ld", value);
    bluetooth_hc04_send_string(buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送无符号整数
// 参数说明     value           无符号整数值
// 返回参数     void
// 使用示例     bluetooth_hc04_send_uint(65535);
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_uint (uint32 value)
{
    char buffer[16];
    sprintf(buffer, "%lu", value);
    bluetooth_hc04_send_string(buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送换行符
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_hc04_send_string("Line 1");
//              bluetooth_hc04_send_newline();
//              bluetooth_hc04_send_string("Line 2");
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_newline (void)
{
    bluetooth_hc04_send_string("\r\n");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送分隔符
// 参数说明     void
// 返回参数     void
// 使用示例     bluetooth_hc04_send_float(angle, 2);
//              bluetooth_hc04_send_separator();
//              bluetooth_hc04_send_float(speed, 1);
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_separator (void)
{
    bluetooth_hc04_send_string(", ");
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送角度值
// 参数说明     angle           角度值（浮点数）
// 返回参数     void
// 使用示例     bluetooth_hc04_send_angle(45.5); // 输出: Angle: 45.50
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_angle (float angle)
{
    bluetooth_hc04_send_float_with_label(angle, "Angle", 2);
    bluetooth_hc04_send_newline();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     HC-04蓝牙模块 发送带单位的格式化角度值
// 参数说明     angle           角度值
// 参数说明     unit            单位字符串（如"deg"、"rad"等）
// 返回参数     void
// 使用示例     bluetooth_hc04_send_angle_formatted(45.67, "deg");
//              输出: Angle: 45.67 deg
//-------------------------------------------------------------------------------------------------------------------
void bluetooth_hc04_send_angle_formatted (float angle, const char* unit)
{
    char buffer[64];
    sprintf(buffer, "Angle: %.2f %s\r\n", angle, unit);
    bluetooth_hc04_send_string(buffer);
}
