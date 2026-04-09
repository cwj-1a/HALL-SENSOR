#include "MLX90393.h"
#include "Delay.h"
#include "SensorManager.h"
#include "Timer.h"

// 定义需要读取的轴
uint8_t flags = MLX90393_FLAG_X | MLX90393_FLAG_Y | MLX90393_FLAG_Z;

int main(void)
{
    // 初始化传感器管理器
    SensorManager_Init(flags);
    
    // 添加传感器1
    SensorManager_AddSensor(
        0x0c,       // 地址
        0, 0,       // A1, A0
        -1,         // DRDY_pin (未使用)
        0,          // I2C ID
        0.0002f, 0.0002f, 0.0001f,  // kx, ky, kz
        20.0f, 20.0f, 25.0f,        // 死区阈值
        0.3f,        // 平滑系数
        0.001       //静态阈值滤波系数
    );
    // 添加传感器2（参考传感器）
    SensorManager_AddSensor(
        0x0c,       // 地址
        0, 0,       // A1, A0
        -1,         // DRDY_pin (未使用)
        1,          // I2C ID
        0.0002f, 0.0002f, 0.0001f,  // kx, ky, kz
        20.0f, 20.0f, 25.0f,        // 死区阈值
        0.3f,        // 平滑系数
        0.001       //静态阈值滤波系数
    );
        
    // 标定所有传感器（采集3个样本）
    SensorManager_CalibrateAll(3);
    
    // 主循环
    while(1)
    {
        // 延时
        mydelay_ms(200);
        
        // 读取并处理所有传感器数据
        SensorManager_ReadAll();
        
        // 打印所有传感器数据
        //SensorManager_PrintAll();
        SensorManager_PrintRawData();
    }
}
