#ifndef __PATH_RECORD_H
#define __PATH_RECORD_H

#include "zf_common_typedef.h"

// 路径点最大数量（根据Flash容量调整）
#define MAX_PATH_POINTS 500
#define PATH_MAGIC 0xAA55AA55

// Flash存储配置（使用最后几个扇区）
#define PATH_FLASH_SECTOR 126      // 使用第126扇区
#define PATH_FLASH_PAGE   0        // 使用该扇区的第0页

// 路径运行状态
typedef enum {
    PATH_IDLE = 0,
    PATH_RECORDING,
    PATH_REPLAYING,
    PATH_SAVED,
    PATH_LOADED
} PathState;

// 路径点数据结构
typedef struct {
    int16_t left_speed;      // 左轮速度（编码器差值）
    int16_t right_speed;     // 右轮速度（编码器差值）
    uint16_t time_interval;  // 时间间隔（ms）
} PathPoint;

// 路径头信息
typedef struct {
    uint32_t magic;          // 魔术字，用于验证数据有效性
    uint32_t point_count;    // 路径点数量
    uint32_t total_time;     // 总时间（ms）
    uint32_t checksum;       // 校验和
} PathHeader;

// 全局路径数据
typedef struct {
    PathHeader header;
    PathPoint points[MAX_PATH_POINTS];
} PathData;

// 外部变量声明
extern PathState path_state;
extern PathData path_data;
extern uint32_t path_current_index;
extern uint32_t path_start_time;

// 函数声明
void path_init(void);
void path_record_start(void);
void path_record_stop(void);
void path_save_to_flash(void);
void path_load_from_flash(void);
void path_replay_start(void);
void path_replay_stop(void);
void path_clear(void);

// 路径记录步进（在定时中断中调用）
void path_record_step(void);

// 路径复现步进（在定时中断中调用，返回是否结束）
uint8_t path_replay_step(float *target_left, float *target_right);

// 获取路径信息
uint32_t path_get_point_count(void);
PathState path_get_state(void);

// 工具函数
uint32_t calculate_checksum(uint8_t *data, uint32_t len);

#endif
