#ifndef __ZF_DEVICE_BLUETOOTH_HC04_H__
#define __ZF_DEVICE_BLUETOOTH_HC04_H__

#include "zf_common_typedef.h"

// HC-04蓝牙模块默认配置
#define BLUETOOTH_HC04_INDEX             UART_6          // 使用的串口
#define BLUETOOTH_HC04_BAUD_RATE         (9600)        // 波特率
#define BLUETOOTH_HC04_BUFFER_SIZE       (512)           // 缓冲区大小
#define BLUETOOTH_HC04_FRAME_BUFFER_SIZE (128)           // 帧缓冲区大小（增加以适应字符串数据）
#define BLUETOOTH_HC04_MAX_STRING_LEN    (32)            // 最大字符串长度

// 引脚定义（请根据实际硬件连接修改）
#define BLUETOOTH_HC04_TX_PIN            UART6_TX_C6    // TX引脚
#define BLUETOOTH_HC04_RX_PIN            UART6_RX_C7    // RX引脚

// 可选：如果支持AT模式，定义KEY引脚
// #define BLUETOOTH_HC04_KEY_PIN          Xxx

// 启用printf功能
#define BLUETOOTH_HC04_USE_PRINTF        1

// 启用调试模式
// #define BLUETOOTH_HC04_DEBUG            1

//-------------------------------------------------------------------------------------------------------------------
// 结构体定义
//-------------------------------------------------------------------------------------------------------------------

// 字符串摇杆数据结构体（格式：["joystick","0","0","0","0"]）
typedef struct
{
    char joystick[BLUETOOTH_HC04_MAX_STRING_LEN];     // 摇杆状态字符串
    char data1[BLUETOOTH_HC04_MAX_STRING_LEN];        // 第一个字符串
    char data2[BLUETOOTH_HC04_MAX_STRING_LEN];        // 第二个字符串
    char data3[BLUETOOTH_HC04_MAX_STRING_LEN];        // 第三个字符串
    char data4[BLUETOOTH_HC04_MAX_STRING_LEN];        // 第四个字符串
    uint8 valid;                                      // 数据是否有效
    uint32 timestamp;                                 // 时间戳
} bluetooth_hc04_joystick_string_data_t;

// 数值摇杆数据结构体（转换为数值后）
typedef struct
{
    int32 joystick_value;     // 摇杆数值
    int32 data1_value;        // 第一个数值
    int32 data2_value;        // 第二个数值
    int32 data3_value;        // 第三个数值
    int32 data4_value;        // 第四个数值
    uint8 valid;              // 数据是否有效
    uint32 timestamp;         // 时间戳
} bluetooth_hc04_joystick_numeric_data_t;

// 接收回调函数类型定义
typedef void (*bluetooth_hc04_rx_callback_t)(bluetooth_hc04_joystick_string_data_t *data);
typedef void (*bluetooth_hc04_numeric_callback_t)(bluetooth_hc04_joystick_numeric_data_t *data);

// 摇杆指令字符串定义
typedef struct
{
    const char* command;      // 命令字符串
    uint8 id;                 // 命令ID
} joystick_command_string_t;

//-------------------------------------------------------------------------------------------------------------------
// 函数声明
//-------------------------------------------------------------------------------------------------------------------
uint8   bluetooth_hc04_init                (void);
uint32  bluetooth_hc04_send_byte           (const uint8 data);
uint32  bluetooth_hc04_send_buffer         (const uint8 *buff, uint32 len);
uint32  bluetooth_hc04_send_string         (const char *str);
void    bluetooth_hc04_send_image          (const uint8 *image_addr, uint32 image_size);
uint32  bluetooth_hc04_read_buffer         (uint8 *buff, uint32 len);
void    bluetooth_hc04_uart_callback       (void);
uint32  bluetooth_hc04_available           (void);

// 浮点数发送函数
void    bluetooth_hc04_send_float          (float value, uint8 decimal_places);
void    bluetooth_hc04_send_float_with_label (float value, const char* label, uint8 decimal_places);


int     bluetooth_hc04_printf              (const char* format, ...);


// 便捷函数
void    bluetooth_hc04_send_int            (int32 value);
void    bluetooth_hc04_send_uint           (uint32 value);
void    bluetooth_hc04_send_newline        (void);
void    bluetooth_hc04_send_separator      (void);

// 角度上传函数
void    bluetooth_hc04_send_angle          (float angle);
void    bluetooth_hc04_send_angle_formatted (float angle, const char* unit);

// 新增：接收功能函数
uint8   bluetooth_hc04_set_rx_callback     (bluetooth_hc04_rx_callback_t callback);
uint8   bluetooth_hc04_set_numeric_callback(bluetooth_hc04_numeric_callback_t callback);
uint8   bluetooth_hc04_parse_data_frame    (void);
uint8   bluetooth_hc04_get_joystick_string_data(bluetooth_hc04_joystick_string_data_t *data);
uint8   bluetooth_hc04_get_joystick_numeric_data(bluetooth_hc04_joystick_numeric_data_t *data);
void    bluetooth_hc04_set_frame_format    (uint8 start_char, uint8 end_char);
void    bluetooth_hc04_clear_rx_buffer     (void);

// 新增：数据解析辅助函数
uint8   bluetooth_hc04_is_data_valid       (void);
void    bluetooth_hc04_get_all_strings     (char *joystick, char *data1, char *data2, char *data3, char *data4);
void    bluetooth_hc04_get_all_numeric     (int32 *joystick_val, int32 *data1_val, int32 *data2_val, 
                                            int32 *data3_val, int32 *data4_val);

// 新增：字符串处理函数
int32   bluetooth_hc04_string_to_int       (const char *str);
uint8   bluetooth_hc04_compare_string      (const char *str1, const char *str2);
uint8   bluetooth_hc04_is_command          (const char *command, const char *target);

#ifdef BLUETOOTH_HC04_KEY_PIN
void    bluetooth_hc04_enter_at_mode       (void);
#endif

#endif
