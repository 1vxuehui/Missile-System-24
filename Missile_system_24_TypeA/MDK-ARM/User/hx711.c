#include "hx711.h"
#include "stm32f4xx.h"                  // Device header
#include "main.h"

 
#define     HX711_DOUT(x)          HAL_GPIO_WritePin(HX711_DOUT_GPIO_Port, HX711_DOUT_GPIO_PIN, x)
#define     HX711_SCK(x)           HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_GPIO_PIN, x)
#define     HX711_SCK_STATE     	 HAL_GPIO_ReadPin(HX711_SCK_GPIO_Port, HX711_SCK_GPIO_PIN)
#define     HX711_DOUT_STATE    	 HAL_GPIO_ReadPin(HX711_DOUT_GPIO_Port, HX711_DOUT_GPIO_PIN)
 
static STRUCT_HX711_TYPEDEF hx711;
float Pulling_force;

 
/* 传感器结构体参数初始化 */
static void hx711_struct_config(void)
{
	hx711.gain = GAIN_128;	/* 当前增益 128 */
    hx711.isTare = true;	/* 默认第一次称重为皮重 */
    hx711.k = 0.004757f;	/* 传感器转换系数 */
    hx711.b = 0.f;			/* 传感器误差校正/补偿值 */
    hx711.tare = 0.f;		/* 保存皮重数据 */
    hx711.actual = 0.f;		/* 保存净重数据 */
}
 
/* 读取传感器返回的ADC数据值（数字量） 假定为一阶线性关系 y=kx+b */
static unsigned long hx711_readCount(void)
{
    unsigned long count = 0;		/* 需要读取25-27个0/1数据 */
    unsigned char timeOut_cnt = 0; 	/* 检测超时计数值 */
    
    HX711_SCK(0);
	/* 添加超时检测 防止一直未识别到传感器导致程序卡死在这里 */
    while(HX711_DOUT_STATE) { /* 等待DOUT从高电平到低电平跳变 */
        timeOut_cnt ++;
        HAL_Delay(1);
        if(timeOut_cnt > 1) return 0;
    }
    /* 读取24位的ADC数字量数据 */
    for(unsigned char i=0; i<24; i++) {
        HX711_SCK(1);
        HAL_Delay(1);
        HX711_SCK(0);
        count |= (HX711_DOUT_STATE) << (24-i); /* 上升沿读取数据 */
        HAL_Delay(1);
    }
    /* 用于设置输出增益和输出通道 */
    for(unsigned char j=0; j<hx711.gain; j++) {
        HX711_SCK(1);
        HAL_Delay(1);
        HX711_SCK(0);
        HAL_Delay(1);
    }
    return (count ^ 0x800000);
}
 
/* ----------------------------- 接口函数 ---------------------- */
/* 传感器 初始化 */
void hx711_init(void)
{
    hx711_struct_config();
}
 
/* 传感器断电 */
void hx711_power_off(void)
{
    HX711_SCK(0);
    HAL_Delay(1);
    HX711_SCK(1);
    HAL_Delay(100);
}
 
/* 传感器上电 */
void hx711_reset(void)
{
    if(HX711_SCK_STATE == 1) {
        HX711_SCK(0);
    }
}
 
/* 设置传感器转换系数 */
void hx711_set_convert_ratio(float ratio)
{
    hx711.k = ratio;
}
 
/* 设置传感器误差校正/补偿值 */
void hx711_set_offset_value(float offset)
{
    hx711.b = offset;
}
 
/* 设置传感器增益 */
void hx711_set_gain(ENUM_HX711_GAIN_TYPEDEF gain)
{
    hx711.gain = gain;
}
 
/* 获取传感器皮重数据 */
float hx711_get_tare_weight(void)
{
    hx711.isTare = true; /* 获取皮重的时候 将该标志位置位 */
    if(hx711.isTare == true) {
        hx711.isTare = false;
        hx711.tare = (float)hx711_readCount() * hx711.k + hx711.b; /* 此时获取到的重量为皮重 */
    }
    return (hx711.tare);
}
 
float hx711_get_actual_weight(void)
{
    float weight = 0.f;
    
    if(hx711.isTare == false) { /* 当该标志位复位 代表此时应计算净重 */
        weight = (float)hx711_readCount() * hx711.k + hx711.b;
        hx711.actual = weight - hx711.tare;
    }
    return (hx711.actual);
}

void hx711_get_weight(void)
{
	hx711_get_tare_weight();
	hx711_get_actual_weight();
	Pulling_force = hx711.actual;
}
