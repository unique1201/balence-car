#include "path_record.h"
#include "encoder.h"
#include "zf_driver_flash.h"
#include "zf_driver_timer.h"
#include "zf_driver_encoder.h"
#include <string.h>

// 全局变量定义
PathState path_state = PATH_IDLE;
PathData path_data;
uint32_t path_current_index = 0;
uint32_t path_start_time = 0;
uint32_t path_last_record_time = 0;

// 静态变量
static int16_t last_encoder_left = 0;
static int16_t last_encoder_right = 0;
static uint32_t replay_start_time = 0;

// 初始化路径模块
void path_init(void)
{
    // 清空路径数据
    memset(&path_data, 0, sizeof(PathData));
    path_current_index = 0;
    
    // 尝试从Flash加载已保存的路径
    path_load_from_flash();
}

// 开始记录路径
void path_record_start(void)
{
    if (path_state == PATH_RECORDING) return;
    
    // 初始化记录状态
    path_state = PATH_RECORDING;
    path_current_index = 0;
    path_start_time = timer_get(TIM_2);  // 使用TIM2作为时间基准
    path_last_record_time = path_start_time;
    
    // 获取当前编码器值作为基准
    last_encoder_left = encoder_get_count(TIM3_ENCODER);
    last_encoder_right = encoder_get_count(TIM4_ENCODER);
    
    // 清空路径数据头
    memset(&path_data.header, 0, sizeof(PathHeader));
    path_data.header.magic = PATH_MAGIC;
    
    printf("Path recording started...\r\n");
}

// 停止记录路径
void path_record_stop(void)
{
    if (path_state != PATH_RECORDING) return;
    
    // 计算总时间
    uint32_t current_time = timer_get(TIM_2);
    path_data.header.total_time = current_time - path_start_time;
    path_data.header.point_count = path_current_index;
    
    // 计算校验和
    uint8_t *data_ptr = (uint8_t*)&path_data;
    path_data.header.checksum = calculate_checksum(data_ptr + 4, 
        sizeof(PathData) - 4);  // 跳过magic字段
    
    path_state = PATH_SAVED;
    
    printf("Path recording stopped. Points: %d, Total time: %dms\r\n", 
           path_data.header.point_count, path_data.header.total_time);
}

// 记录一个路径点（在定时中断中调用，建议20ms调用一次）
void path_record_step(void)
{
    if (path_state != PATH_RECORDING) return;
    
    if (path_current_index >= MAX_PATH_POINTS) {
        printf("Path buffer full!\r\n");
        path_record_stop();
        return;
    }
    
    // 获取当前时间
    uint32_t current_time = timer_get(TIM_2);
    uint16_t time_diff = current_time - path_last_record_time;
    
    if (time_diff < 10) return;  // 最少10ms间隔
    
    // 获取当前编码器值
    int16_t current_left = encoder_get_count(TIM3_ENCODER);
    int16_t current_right = encoder_get_count(TIM4_ENCODER);
    
    // 计算编码器差值（速度）
    int16_t left_diff = current_left - last_encoder_left;
    int16_t right_diff = current_right - last_encoder_right;
    
    // 记录路径点
    path_data.points[path_current_index].left_speed = left_diff;
    path_data.points[path_current_index].right_speed = right_diff;
    path_data.points[path_current_index].time_interval = time_diff;
    
    // 更新状态
    last_encoder_left = current_left;
    last_encoder_right = current_right;
    path_last_record_time = current_time;
    path_current_index++;
    
    // 更新总时间
    path_data.header.total_time = current_time - path_start_time;
}

// 保存路径到Flash
void path_save_to_flash(void)
{
    if (path_state != PATH_SAVED) {
        printf("No valid path to save!\r\n");
        return;
    }
    
    printf("Saving path to Flash...\r\n");
    
    // 清空Flash缓冲区
    flash_buffer_clear();
    
    // 将路径数据复制到Flash缓冲区
    uint32_t *src = (uint32_t*)&path_data;
    uint32_t word_count = sizeof(PathData) / 4;
    
    for (uint32_t i = 0; i < word_count && i < FLASH_DATA_BUFFER_SIZE; i++) {
        flash_union_buffer[i].uint32_type = src[i];
    }
    
    // 写入Flash
    if (flash_write_page_from_buffer(PATH_FLASH_SECTOR, PATH_FLASH_PAGE) == 0) {
        printf("Path saved to Flash successfully!\r\n");
    } else {
        printf("Failed to save path to Flash!\r\n");
    }
}

// 从Flash加载路径
void path_load_from_flash(void)
{
    printf("Loading path from Flash...\r\n");
    
    // 从Flash读取数据
    flash_read_page_to_buffer(PATH_FLASH_SECTOR, PATH_FLASH_PAGE);
    
    // 复制到路径数据结构
    uint32_t *dest = (uint32_t*)&path_data;
    uint32_t word_count = sizeof(PathData) / 4;
    
    for (uint32_t i = 0; i < word_count && i < FLASH_DATA_BUFFER_SIZE; i++) {
        dest[i] = flash_union_buffer[i].uint32_type;
    }
    
    // 验证数据有效性
    if (path_data.header.magic != PATH_MAGIC) {
        printf("No valid path in Flash.\r\n");
        memset(&path_data, 0, sizeof(PathData));
        path_state = PATH_IDLE;
        return;
    }
    
    // 验证校验和
    uint8_t *data_ptr = (uint8_t*)&path_data;
    uint32_t calculated_checksum = calculate_checksum(data_ptr + 4, sizeof(PathData) - 4);
    
    if (calculated_checksum != path_data.header.checksum) {
        printf("Path checksum error! Data may be corrupted.\r\n");
        path_state = PATH_IDLE;
        return;
    }
    
    printf("Path loaded from Flash. Points: %d, Total time: %dms\r\n",
           path_data.header.point_count, path_data.header.total_time);
    
    path_state = PATH_LOADED;
}

// 开始路径复现
void path_replay_start(void)
{
    if (path_state != PATH_LOADED && path_state != PATH_SAVED) {
        printf("No valid path to replay!\r\n");
        return;
    }
    
    if (path_data.header.point_count == 0) {
        printf("Path is empty!\r\n");
        return;
    }
    
    path_state = PATH_REPLAYING;
    path_current_index = 0;
    replay_start_time = timer_get(TIM_2);
    
    // 清空编码器计数
    encoder_clear_count(TIM3_ENCODER);
    encoder_clear_count(TIM4_ENCODER);
    
    printf("Path replay started. Total points: %d\r\n", path_data.header.point_count);
}

// 停止路径复现
void path_replay_stop(void)
{
    if (path_state != PATH_REPLAYING) return;
    
    path_state = (path_data.header.point_count > 0) ? PATH_LOADED : PATH_IDLE;
    printf("Path replay stopped.\r\n");
}

// 路径复现步进（在定时中断中调用）
// 返回1表示仍在复现中，0表示复现结束
uint8_t path_replay_step(float *target_left, float *target_right)
{
    if (path_state != PATH_REPLAYING) return 0;
    
    if (path_current_index >= path_data.header.point_count) {
        // 复现完成
        path_replay_stop();
        *target_left = 0;
        *target_right = 0;
        return 0;
    }
    
    // 获取当前时间
    uint32_t current_time = timer_get(TIM_2);
    uint32_t elapsed_time = current_time - replay_start_time;
    
    // 查找当前时间对应的路径点
    static uint32_t accumulated_time = 0;
    
    while (path_current_index < path_data.header.point_count) {
        uint16_t interval = path_data.points[path_current_index].time_interval;
        
        if (accumulated_time + interval > elapsed_time) {
            // 还未到下一个点的时间，保持当前点
            break;
        }
        
        // 切换到下一个点
        accumulated_time += interval;
        path_current_index++;
    }
    
    // 获取目标速度（转换为速度单位，需要根据实际调整系数）
    if (path_current_index < path_data.header.point_count) {
        *target_left = (float)path_data.points[path_current_index].left_speed * 0.1f;
        *target_right = (float)path_data.points[path_current_index].right_speed * 0.1f;
    } else {
        // 到达终点，速度归零
        *target_left = 0;
        *target_right = 0;
    }
    
    return 1;
}

// 清空路径数据
void path_clear(void)
{
    memset(&path_data, 0, sizeof(PathData));
    path_current_index = 0;
    path_state = PATH_IDLE;
    printf("Path data cleared.\r\n");
}

// 获取路径点数量
uint32_t path_get_point_count(void)
{
    return path_data.header.point_count;
}

// 获取当前状态
PathState path_get_state(void)
{
    return path_state;
}

// 计算校验和（简单的累加和）
uint32_t calculate_checksum(uint8_t *data, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}
