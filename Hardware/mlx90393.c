#include "MLX90393.h"
#include "MyI2C.h"
#include "Delay.h"
#include "Serial.h"
#include "Timer.h"

//———————————————————————————————————————————————
// 初始化函数：根据 A1、A0 设置 I2C 地址，DRDY_pin 传入 -1 表示不使用
//———————————————————————————————————————————————
void MLX90393_begin(uint8_t addr,MLX90393_Handle *handle, uint8_t A1, uint8_t A0, int DRDY_pin,uint8_t id)
{
    if(handle!=NULL)
    {
        // I2C 地址 = BASE_ADDR | (A1 ? 2 : 0) | (A0 ? 1 : 0)
        handle->I2C_Address = addr | ((A1 ? 2 : 0) | (A0 ? 1 : 0));
        handle->DRDY_pin = DRDY_pin;
    }
    // 修改1.（根据芯片手册）最好加一个MLX90393_reset(handle, id);并且Delay_ms(1);
    uint8_t status0 = MLX90393_exitBurst(handle,id);
    mydelay_ms(1);
    uint8_t status1 = MLX90393_reset(handle, id);

    mydelay_ms(1.5);

    // 配置各参数：设置增益、分辨率、过采样、数字滤波和温度补偿
    MLX90393_exitBurst(handle,id);
    uint8_t status2 = MLX90393_setGainSel(handle, id, 7);
    uint8_t status3 = MLX90393_setResolution(handle, id, 0, 0, 0);
    uint8_t status4 = MLX90393_setOverSampling(handle, id, 3);
    uint8_t status5 = MLX90393_setDigitalFiltering(handle, id, 7);
    uint8_t status6 = MLX90393_setTemperatureCompensation(handle, id, 0);

    Serial_Printf("Init statuses: %02X %02X %02X %02X %02X %02X\r\n",
                    status1, status2, status3, status4, status5, status6);
}

//———————————————————————————————————————————————
// 发送命令：发送单字节命令并读取响应状态字节
//———————————————————————————————————————————————
uint8_t MLX90393_sendCommand(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t cmd)
{
    uint8_t resp;//响应状态字节
    MyI2C_Start(i2c_id);
    MyI2C_SendByte(i2c_id, handle->I2C_Address << 1);//写模式
    MyI2C_ReceiveAck(i2c_id);
    MyI2C_SendByte(i2c_id, cmd);
    MyI2C_ReceiveAck(i2c_id);
    //修改2.按照芯片手册时序
    //MyI2C_Stop(i2c_id);
    
    MyI2C_Start(i2c_id);
    MyI2C_SendByte(i2c_id, (handle->I2C_Address << 1) | 0x01);
    MyI2C_ReceiveAck(i2c_id);
    resp = MyI2C_ReceiveByte(i2c_id);
    MyI2C_SendAck(i2c_id, 1);
    MyI2C_Stop(i2c_id);
    
    return resp;
}

//———————————————————————————————————————————————
// 读寄存器：发送 CMD_READ_REGISTER 并读取 3 字节（状态，高字节，低字节）一个状态字节+两个数据字节，返回状态字节
//———————————————————————————————————————————————
//1.先发送设备地址加读写位，再发送命令，然后发送寄存器地址的低6位并左移两位
uint8_t MLX90393_readRegister(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t reg, uint16_t *data)//指定地址读
{
    uint8_t buffer[3];
    uint8_t i;
    
    MyI2C_Start(i2c_id);
    MyI2C_SendByte(i2c_id, handle->I2C_Address << 1);
    MyI2C_ReceiveAck(i2c_id);
    MyI2C_SendByte(i2c_id, MLX90393_CMD_READ_REGISTER);
    MyI2C_ReceiveAck(i2c_id);
    MyI2C_SendByte(i2c_id, (reg & 0x3F) << 2);
    MyI2C_ReceiveAck(i2c_id);
    //修改3.按照芯片手册时序
    //MyI2C_Stop(i2c_id);
    
    MyI2C_Start(i2c_id);
    MyI2C_SendByte(i2c_id, (handle->I2C_Address << 1) | 0x01);
    MyI2C_ReceiveAck(i2c_id);
    for(i = 0; i < 3; i++){
         buffer[i] = MyI2C_ReceiveByte(i2c_id);
         if(i < 2)
             MyI2C_SendAck(i2c_id, 0);
         else
             MyI2C_SendAck(i2c_id, 1);
    }
    MyI2C_Stop(i2c_id);
    
    *data = (buffer[1] << 8) | buffer[2];
    return buffer[0]; // 状态字节
}

//———————————————————————————————————————————————
// 写寄存器：发送 CMD_WRITE_REGISTER，写入 2 字节数据及寄存器地址
//———————————————————————————————————————————————
uint8_t MLX90393_writeRegister(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t reg, uint16_t data)//指定地址写
//1.先发送设备地址加读写位，再发送命令，然后发送数据的高8位和低8位，最后发送寄存器地址的低6位并左移两位
{
    uint8_t resp;
    uint8_t MSB = data >> 8;
    uint8_t LSB = data & 0xFF;
    
    MyI2C_Start(i2c_id);
    MyI2C_SendByte(i2c_id, handle->I2C_Address << 1);
    MyI2C_ReceiveAck(i2c_id);
    MyI2C_SendByte(i2c_id, MLX90393_CMD_WRITE_REGISTER);
    MyI2C_ReceiveAck(i2c_id);
    MyI2C_SendByte(i2c_id, MSB);
    MyI2C_ReceiveAck(i2c_id);
    MyI2C_SendByte(i2c_id, LSB);
    MyI2C_ReceiveAck(i2c_id);
    MyI2C_SendByte(i2c_id, (reg & 0x3F) << 2);
    MyI2C_ReceiveAck(i2c_id);
    //修改4.按照芯片手册时序
    //MyI2C_Stop(i2c_id);
    
    MyI2C_Start(i2c_id);
    MyI2C_SendByte(i2c_id, (handle->I2C_Address << 1) | 0x01);
    MyI2C_ReceiveAck(i2c_id);
    resp = MyI2C_ReceiveByte(i2c_id);
    MyI2C_SendAck(i2c_id, 1);
    MyI2C_Stop(i2c_id);
    
    return resp;
}

//———————————————————————————————————————————————
// 复位：发送 RESET 命令并延时 2ms
//———————————————————————————————————————————————
uint8_t MLX90393_reset(MLX90393_Handle *handle, uint8_t i2c_id)
{
    //执行完RT命令等待1.5ms直至启动序列完成
    uint8_t status = MLX90393_sendCommand(handle, i2c_id, MLX90393_CMD_RESET);
    mydelay_ms(2);
    return status;
}

//———————————————————————————————————————————————
// 启动单次测量：发送 CMD_START_MEASUREMENT，flags 控制 T, X, Y, Z 的选择
//———————————————————————————————————————————————

// 修改前的 MLX90393_startMeasurement 仅发送命令，不读取状态
// uint8_t MLX90393_startMeasurement(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t flags)
// {
//     uint8_t cmd = MLX90393_CMD_START_MEASUREMENT | (flags & 0x0F);
//     MyI2C_Start(i2c_id);
//     MyI2C_SendByte(i2c_id, handle->I2C_Address << 1);
//     MyI2C_ReceiveAck(i2c_id);
//     MyI2C_SendByte(i2c_id, cmd);
//     MyI2C_ReceiveAck(i2c_id);
//     MyI2C_Stop(i2c_id);
//     return 0;
// }

// 修改后的 MLX90393_startMeasurement 通过 MLX90393_sendCommand 发送命令并读取状态字节
uint8_t MLX90393_startMeasurement(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t flags)//flags 控制 T, X, Y, Z 的选择
{
    uint8_t cmd = MLX90393_CMD_START_MEASUREMENT | (flags & 0x0F);
    // 调用 sendCommand 发送命令并返回状态字节
    return MLX90393_sendCommand(handle, i2c_id, cmd);
}
//———————————————————————————————————————————————
// 读取测量数据：发送 CMD_READ_MEASUREMENT 后，读取连续数据（T,X,Y,Z数据全部按照两字节返回）
// 数据格式：Byte0 状态，后续依次为 T, X, Y, Z（各占 2 字节，若不测量则不返回）
//———————————————————————————————————————————————
uint8_t MLX90393_readMeasurement(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t flags, MLX90393_RawData *raw)//flags 控制 T, X, Y, Z 的选择，raw记录原始数据
{
    uint8_t cmd = MLX90393_CMD_READ_MEASUREMENT | (flags & 0x0F);
    uint8_t buffer[9];
    uint8_t count = 1;
    if(flags & MLX90393_FLAG_T) count += 2;
    if(flags & MLX90393_FLAG_X) count += 2;
    if(flags & MLX90393_FLAG_Y) count += 2;
    if(flags & MLX90393_FLAG_Z) count += 2;
    
    MyI2C_Start(i2c_id);
    MyI2C_SendByte(i2c_id, handle->I2C_Address << 1);//最低位为0发送写命令
    MyI2C_ReceiveAck(i2c_id);
    MyI2C_SendByte(i2c_id, cmd);
    MyI2C_ReceiveAck(i2c_id);
   //MyI2C_Stop(i2c_id);
    
    MyI2C_Start(i2c_id);
    MyI2C_SendByte(i2c_id, (handle->I2C_Address << 1) | 0x01);//最低位为1发送读命令
    MyI2C_ReceiveAck(i2c_id);
    for(uint8_t i = 0; i < count; i++){
         buffer[i] = MyI2C_ReceiveByte(i2c_id);
         if(i < (count - 1))
             MyI2C_SendAck(i2c_id, 0);
         else
             MyI2C_SendAck(i2c_id, 1);
    }
    MyI2C_Stop(i2c_id);
    
    uint8_t index = 1;
    if(flags & MLX90393_FLAG_T){
         raw->t = (buffer[index] << 8) | buffer[index+1];
         index += 2;
    } else {
         raw->t = 0;
    }
    if(flags & MLX90393_FLAG_X){
         raw->x = (buffer[index] << 8) | buffer[index+1];
         index += 2;
    } else {
         raw->x = 0;
    }
    if(flags & MLX90393_FLAG_Y){
         raw->y = (buffer[index] << 8) | buffer[index+1];
         index += 2;
    } else {
         raw->y = 0;
    }
    if(flags & MLX90393_FLAG_Z){
         raw->z = (buffer[index] << 8) | buffer[index+1];
         index += 2;
    } else {
         raw->z = 0;
    }
    //buffer[0] 即状态字节，返回给调用者
    return buffer[0];
}

//———————————————————————————————————————————————
// 数据转换函数：将原始数据转换为物理量
// 默认配置：增益 7（gain multiplier = 1.0）、hallconf = 0x0C（xy = 0.150, z = 0.242），温度公式：T = 25 + (raw.t - 46244) / 45.2
//———————————————————————————————————————————————
MLX90393_Data MLX90393_convertRaw(MLX90393_RawData raw)
{
    MLX90393_Data data;
    float gain_factor = 1.0f;
    float xy_sens = 0.150f;//xy 灵敏度
    float z_sens = 0.242f;//z 灵敏度
    
    data.x = ((int16_t)raw.x) * xy_sens * gain_factor;
    data.y = ((int16_t)raw.y) * xy_sens * gain_factor;
    data.z = ((int16_t)raw.z) * z_sens * gain_factor;
    data.t = 25.0f + (((float)raw.t - 46244.0f) / 45.2f);
    
    return data;
}

//———————————————————————————————————————————————
// 以下为复现 Arduino 初始化时的配置接口
//———————————————————————————————————————————————
uint8_t MLX90393_setGainSel(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t gain)
{
    // 写入寄存器 GAIN_SEL_REG：gain 值左移 GAIN_SEL_SHIFT
    uint16_t value = ((uint16_t)gain << GAIN_SEL_SHIFT);
    return MLX90393_writeRegister(handle, i2c_id, GAIN_SEL_REG, value);
}

uint8_t MLX90393_setResolution(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t res_x, uint8_t res_y, uint8_t res_z)
{
    // 分辨率：组合 X/Y/Z，每项 2 位：res_xyz = (res_z << 4) | (res_y << 2) | res_x，
    // 再左移 RES_XYZ_SHIFT 写入寄存器
    uint16_t res_xyz = (((res_z & 0x03) << 4) | ((res_y & 0x03) << 2) | (res_x & 0x03));
    uint16_t value = res_xyz << RES_XYZ_SHIFT;
    return MLX90393_writeRegister(handle, i2c_id, RES_XYZ_REG, value);
}

uint8_t MLX90393_setOverSampling(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t osr)//osr 控制 OSR（过采样率）
{
    uint16_t value = ((uint16_t)osr << OSR_SHIFT);
    return MLX90393_writeRegister(handle, i2c_id, OSR_REG, value);
}

uint8_t MLX90393_setDigitalFiltering(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t flt)
{
    uint16_t value = ((uint16_t)flt << DIG_FLT_SHIFT);
    return MLX90393_writeRegister(handle, i2c_id, DIG_FLT_REG, value);
}

uint8_t MLX90393_setTemperatureCompensation(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t enable)
{
    uint16_t value = ((uint16_t)(enable ? 1 : 0) << TCMP_EN_SHIFT);
    return MLX90393_writeRegister(handle, i2c_id, TCMP_EN_REG, value);
}
uint8_t MLX90393_startBurst(MLX90393_Handle *handle, uint8_t i2c_id, uint8_t flags)
{
    // 构造突发模式命令
    uint8_t cmd = MLX90393_CMD_START_BURST | (flags & 0x0F);
    // 发送命令并返回状态字节
    return MLX90393_sendCommand(handle, i2c_id, cmd);
}
uint8_t MLX90393_exitBurst(MLX90393_Handle *handle, uint8_t i2c_id)
{
    // 发送退出突发模式的命令，CMD_EXIT 定义为 0x80
    return MLX90393_sendCommand(handle, i2c_id, MLX90393_CMD_EXIT);
}
