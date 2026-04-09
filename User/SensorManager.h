#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "MLX90393.h"

// 定义单个传感器的完整状态
typedef struct {
    // 传感器基本句柄，存放传感器地址以及传感器的DRDY引脚是否可用等信息

    MLX90393_Handle handle;
    
    // 传感器I2C总线ID
    uint8_t i2c_id;
    
    // 处理数据
    MLX90393_RawData raw;      // 原始数据
    MLX90393_Data data;        // 转换后数据
    MLX90393_Data cal;         // 标定基准值
    MLX90393_Data delta;       // 相对于基准的变化量
    MLX90393_Data force;       // 映射后的力值
    MLX90393_Data smoothForce; // 滤波后的力值
    
    // 传感器配置参数
    float kx, ky, kz;          // 力映射系数
    float deadBandX, deadBandY, deadBandZ; // 死区阈值
    float smoothingFactor;     // 平滑系数
    float stabilityThreshold;   // 静态阈值，用于过滤微小波动
} Sensor_t;

// 定义最大传感器数量
#define MAX_SENSORS 8

// 传感器管理器结构体
typedef struct {
    Sensor_t sensors[MAX_SENSORS];  // 传感器数组，数组中的每个元素都是一个Sensor_t结构体。
    uint8_t count;                   // 已添加的传感器数量
    uint8_t flags;                   // 需要读取的轴标志（共用）
} SensorManager_t;

// 全局传感器管理器
extern SensorManager_t g_sensorManager;

// 初始化传感器管理器
void SensorManager_Init(uint8_t flags);

// 添加传感器，返回传感器索引
uint8_t SensorManager_AddSensor(
    uint8_t addr,
    uint8_t A1, 
    uint8_t A0,
    int DRDY_pin,
    uint8_t i2c_id,
    float kx, float ky, float kz,
    float deadBandX, float deadBandY, float deadBandZ,
    float smoothingFactor,
    float stabilityThreshold
);

// 标定所有传感器
void SensorManager_CalibrateAll(uint8_t samples);

// 读取并处理所有传感器
void SensorManager_ReadAll(void);

// 获取传感器力值（滤波后）的结果
MLX90393_Data* SensorManager_GetSensorForce(uint8_t index);
    
MLX90393_Data stabilityFilter(MLX90393_Data prev, MLX90393_Data current, float threshold);
// 打印所有传感器的结果
void SensorManager_PrintAll(void);
    
// 打印所有传感器的原始测量值（滤波前数值）
void SensorManager_PrintRawData(void);

#endif // SENSOR_MANAGER_H
