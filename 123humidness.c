/*
 * 123humidness.c
 *
 *  Created on: Jun 4, 2024
 *      Author: 86183
 */
#include "stm32wbxx_hal.h"
#include "main.h"
#include "adc.h"

//not useful


void wait_for_adc_event(void) {
    // 在这里进行ADC转换启动等操作（例如设置通道、采样时间等）
    // 轮询检查特定的ADC事件，例如转换完成事件
    while (HAL_ADC_PollForEvent(&hadc1, ADC_EOC_SINGLE_CONV,100) != HAL_OK);

}

//ADC采集值获取
void get_adc_IN1_value(uint32_t* DMARes)
{

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);//阻塞判断是否转换完成
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))
		{
		DMARes[0]= HAL_ADC_GetValue(&hadc1);
		}
	HAL_ADC_PollForConversion(&hadc1, 100);//阻塞判断是否转换完成
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))
			{
			DMARes[1]= HAL_ADC_GetValue(&hadc1);
			}
}



