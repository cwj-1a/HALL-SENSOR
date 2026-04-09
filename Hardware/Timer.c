#include "stm32f10x.h"

volatile uint32_t TimingDelay = 0;  // 1ms 自增 1

void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 开 TIM2 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 2. 配置：72MHz → 1ms中断
    // 计数器时钟 = 72MHz / (71+1) = 1MHz
    // 计数到 999 → 1ms
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 3. 允许更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // 4. NVIC 中断配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 5. 开定时器
    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TimingDelay++;  // 每1ms +1
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

void mydelay_ms(uint32_t ms)//中断延时函数
{
    uint32_t start = TimingDelay;
    while ((TimingDelay - start) < ms);
}
