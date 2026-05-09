#include "SensorManager.h"
#include "Serial.h"
#include "Delay.h"
#include "MyI2C.h"
#include <math.h>

// 全局传感器管理器实例
SensorManager_t g_sensorManager;

// 初始化传感器管理器
void SensorManager_Init(uint8_t flags)//flags表示测量的轴
{
    // 清空传感器计数
    g_sensorManager.count = 0;
    g_sensorManager.flags = flags;
    
    // 初始化底层接口
    MyI2C_Init();
    Serial_Init();
}

// 添加传感器，返回传感器索引
uint8_t SensorManager_AddSensor(
    uint8_t addr,
    uint8_t A1, //A1,A0地址扩展引脚
    uint8_t A0,
    int DRDY_pin,//决定是否使用传感器
    uint8_t i2c_id,
    float kx, float ky, float kz,
    float deadBandX, float deadBandY, float deadBandZ,
    float smoothingFactor,
    float stabilityThreshold
)
{
    // 检查是否超出最大传感器数量
    if (g_sensorManager.count >= MAX_SENSORS) {
        return 0xFF; // 错误：传感器数量已达上限
    }
    
    // 获取当前传感器索引
    uint8_t idx = g_sensorManager.count;
    Sensor_t* sensor = &g_sensorManager.sensors[idx];//操作当前传感器结构体指针
    
    // 初始化传感器基本参数
    MLX90393_begin(addr, &sensor->handle, A1, A0, DRDY_pin, i2c_id);
    sensor->i2c_id = i2c_id;
    
    // 初始化映射系数
    sensor->kx = kx;
    sensor->ky = ky;
    sensor->kz = kz;
    
    // 初始化死区阈值
    sensor->deadBandX = deadBandX;
    sensor->deadBandY = deadBandY;
    sensor->deadBandZ = deadBandZ;
    
    // 初始化平滑系数
    sensor->smoothingFactor = smoothingFactor;
    
    // 初始化静态阈值
    sensor->stabilityThreshold = stabilityThreshold;
    
    // 初始化平滑输出为0
    sensor->smoothForce.x = 0;
    sensor->smoothForce.y = 0;
    sensor->smoothForce.z = 0;
    
    // 开启突发模式
    MLX90393_startBurst(&sensor->handle, i2c_id, g_sensorManager.flags);
    
		
    // 先读一次数据，为后续转换预热(过滤异常)
    MLX90393_readMeasurement(&sensor->handle, i2c_id, g_sensorManager.flags, &sensor->raw);
    
    // 增加传感器计数
    g_sensorManager.count++;//传感器计数加1，下一次调用时，idx会指向下一个传感器结构体
    
    return idx;
}

// 映射和过滤磁场力值
MLX90393_Data mapAndFilterMagneticForce(MLX90393_Data deltaMag,
                                       float kx, float ky, float kz,
                                       float deadBandX, float deadBandY, float deadBandZ)
{
    MLX90393_Data force;
    // 对每个轴应用死区过滤
    float dx = (fabs(deltaMag.x) < deadBandX) ? 0.0f : deltaMag.x;//如果x轴的绝对值小于deadBandX，则dx为0；否则dx等于deltaMag.x，fab(deltaMag.x)取绝对值
    float dy = (fabs(deltaMag.y) < deadBandY) ? 0.0f : deltaMag.y;
    float dz = (fabs(deltaMag.z) < deadBandZ) ? 0.0f : deltaMag.z;
    
    // 非线性映射：F = k · sign(ΔB) · (|ΔB|²)
    force.x = kx * copysign(dx * dx, dx);//返回dx的符号，dx*dx是x轴的平方
    force.y = ky * copysign(dy * dy, dy);
    force.z = kz * copysign(dz * dz, dz);
    return force;
}

// 指数平滑滤波
MLX90393_Data smoothForceValue(MLX90393_Data prev, MLX90393_Data current, float smoothingFactor)
{
    MLX90393_Data result;
    result.x = smoothingFactor * current.x + (1.0f - smoothingFactor) * prev.x;
    result.y = smoothingFactor * current.y + (1.0f - smoothingFactor) * prev.y;
    result.z = smoothingFactor * current.z + (1.0f - smoothingFactor) * prev.z;
    return result;
}

// 静态阈值滤波，消除微小波动
MLX90393_Data stabilityFilter(MLX90393_Data prev, MLX90393_Data current, float threshold)
{
    MLX90393_Data result = current;
    
    // 如果变化小于阈值，保持原值
    if (fabs(current.x - prev.x) < threshold) {
        result.x = prev.x;
    }
    if (fabs(current.y - prev.y) < threshold) {
        result.y = prev.y;
    }
    if (fabs(current.z - prev.z) < threshold) {
        result.z = prev.z;
    }
    
    return result;
}

// 标定所有传感器
void SensorManager_CalibrateAll(uint8_t samples)
{
    // 遍历所有传感器
    for (uint8_t i = 0; i < g_sensorManager.count; i++) {
        Sensor_t* sensor = &g_sensorManager.sensors[i];
        
        // 初始化校准值为0
        sensor->cal.x = 0;
        sensor->cal.y = 0;
        sensor->cal.z = 0;
        
        // 多次采样取平均
        for (uint8_t j = 0; j < samples; j++) {
            // 延时等待测量完成
            Delay_ms(200);
            // 读取原始测量值
            MLX90393_readMeasurement(&sensor->handle, sensor->i2c_id, g_sensorManager.flags, &sensor->raw);
            // 转换为物理量
            sensor->data = MLX90393_convertRaw(sensor->raw);
            // 累加测量值
            sensor->cal.x += sensor->data.x;
            sensor->cal.y += sensor->data.y;
            sensor->cal.z += sensor->data.z;
            
            // 输出调试信息
            Serial_Printf("base%d_sample%d: X=%.4f, Y=%.4f, Z=%.4f\r\n", 
                          i+1, j+1, sensor->data.x, sensor->data.y, sensor->data.z);
        }
        
        // 计算平均值
        sensor->cal.x /= (float)samples;
        sensor->cal.y /= (float)samples;
        sensor->cal.z /= (float)samples;
        
        // 输出标定结果
        Serial_Printf("base%d: X=%.4f, Y=%.4f, Z=%.4f\r\n", 
                      i+1, sensor->cal.x, sensor->cal.y, sensor->cal.z);
    }
}

// 读取并处理所有传感器
void SensorManager_ReadAll(void)
{
    // 遍历所有传感器
    for (uint8_t i = 0; i < g_sensorManager.count; i++) {
        Sensor_t* sensor = &g_sensorManager.sensors[i];
        
        // 读取原始测量值
        uint8_t status = MLX90393_readMeasurement(&sensor->handle, sensor->i2c_id, g_sensorManager.flags, &sensor->raw);
				//Serial_Printf("measure status%02X\r\n",status);
        // 转换为物理量
        sensor->data = MLX90393_convertRaw(sensor->raw);//将ADC值转换为物理量（磁感应强度）
        
        // 计算磁场变化量
        sensor->delta.x = sensor->data.x - sensor->cal.x;
        sensor->delta.y = sensor->data.y - sensor->cal.y;
        sensor->delta.z = sensor->data.z - sensor->cal.z;
        
        // 非线性映射及死区过滤
        sensor->force = mapAndFilterMagneticForce(
            sensor->delta, 
            sensor->kx, sensor->ky, sensor->kz,
            sensor->deadBandX, sensor->deadBandY, sensor->deadBandZ
        );
        
        // 指数平滑滤波
        MLX90393_Data tempSmooth = smoothForceValue(
            sensor->smoothForce, //pre
            sensor->force,       //current
            sensor->smoothingFactor
        );
        
        // 静态阈值滤波，消除微小波动
        sensor->smoothForce = stabilityFilter(
            sensor->smoothForce,
            tempSmooth,
            sensor->stabilityThreshold
        );
    }
}

// 获取传感器力值（滤波后）的结果
MLX90393_Data* SensorManager_GetSensorForce(uint8_t index)
{
    if (index < g_sensorManager.count) {
        return &g_sensorManager.sensors[index].smoothForce;
    }
    return NULL;
}

// 打印所有传感器的结果
void SensorManager_PrintAll(void)
{
    for (uint8_t i = 0; i < g_sensorManager.count; i++) {
        Sensor_t* sensor = &g_sensorManager.sensors[i];
        Serial_Printf("sensor%d: %.4f, %.4f, %.4f\r\n", 
                     i+1, sensor->smoothForce.x, sensor->smoothForce.y, sensor->smoothForce.z);
    }
}
// 打印所有传感器的原始测量值（滤波前数值）
void SensorManager_PrintRawData(void)
{
    for (uint8_t i = 0; i < g_sensorManager.count; i++) {
        Sensor_t* sensor = &g_sensorManager.sensors[i];
        Serial_Printf("sensor%d: %.4f, %.4f, %.4f\r\n", 
                     i+1, sensor->data.x, sensor->data.y, sensor->data.z);
    }
}

// 打印所有传感器的数值变化
void SensorManager_PrintdeltaData(void)
{
    for (uint8_t i = 0; i < g_sensorManager.count; i++) {
        Sensor_t* sensor = &g_sensorManager.sensors[i];
        Serial_Printf("sensor%d: %.4f, %.4f, %.4f\r\n", 
                     i+1, sensor->delta.x, sensor->delta.y, sensor->delta.z);
    }
}
