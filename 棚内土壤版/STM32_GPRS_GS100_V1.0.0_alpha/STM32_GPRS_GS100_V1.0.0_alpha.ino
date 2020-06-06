
#define TINY_GSM_MODEM_SIM800

#include "GSM/TinyGsmClient.h"
#include <Arduino.h>
#include <libmaple/nvic.h>
#include <libmaple/pwr.h>
#include <libmaple/bkp.h>
#include "memory.h"
#include "Private_Sensor.h"
#include "data_transmit.h"
#include "private_delay.h"
#include "Periph.h"
#include "Private_RTC.h"
#include "AT24C1024.h"

TinyGsm modem(GSM_Serial);

TinyGsmClient client(modem);

unsigned int Run_Time_Out_Sec = 0; //运行超时计数
char Toggle = 0;				   //状态灯闪灭位翻转

void setup()
{
	Some_GPIO_Init(); //IO口初始化定义

	Serial.begin(115200); //USB serial port.
	ModBus_Serial.begin(9600);
	GSM_Serial.begin(9600);
	delay(3000);

	// #if (DEVICE_V2_5 || GS100_DEVICE_V1_0)
	// 	EP_Write_Enable();
	// #endif

	// EpromDb.open(EEPROM_BASE_ADDR);
	// EpromDb.clear();

	// #if (DEVICE_V2_5 || GS100_DEVICE_V1_0)
	// 	EP_Write_Disable();
	// #endif
	
	bkp_init();
	Serial.println("bkp初始化完成");

	//初始化RTC闹钟
	Init_RTC(180);
	Serial.println("RTC闹钟初始化完成");

	//如果电池电压小于6.5V，设备休眠
	if (Get_Bat_Voltage(DEFAULT_VOL_CHANGE_TIMES) < MIN_BAT_VOL)
	{
		Serial.println("电压极低，进入休眠状态");
		delay(100);
		SYS_Sleep();
	}

	System_ID_Self_Check(SysHostID); //系统ID自检
	Serial.println("系统ID自检完成");

	Key_Clear_Device_Parameter(); //按键2清除系统参数，慎用
	Key_Clear_Muti_Sensor_Data(); //按键1清除存储在EP中的传感器数据

	//初始化定时器2
	Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
	Timer2.setPeriod(1000000); // in microseconds，1S
	Timer2.setCompare1(1);	   // overflow might be small
	Timer2.attachCompare1Interrupt(Time2_Handler);
	Serial.println("定时器2初始化完成");

	//RS485模块开始采集传感器数据
	Serial.println("开始采集传感器数据...");
	Data_Acquisition();
}

void loop()
{
	if(Get_Bat_Voltage(DEFAULT_VOL_CHANGE_TIMES) < MIN_BAT_VOL)
	{
    	Serial.println("电压极低，进入休眠状态");
		delay(100);
		SYS_Sleep();
	}

  	if(!Pre_Access_Network())//注册到网络
	{
		Serial.println("注册到网络失败，进入休眠 <Pre_Access_Network>");
		delay(100);
		SYS_Sleep();
	}

  	if(!Connect_to_The_Server())//连接到服务器
	{
		Serial.println("连接到服务器失败，进入休眠 <Connect_to_The_Server>");
		delay(100);
		SYS_Sleep();
	}

	if (Send_Data_To_Server() == true)
	{
		Serial.println("发送数据至服务器完成，进入休眠 <Send_Data_To_Server>");
		delay(100);
		SYS_Sleep();
	}
	else
	{
		Serial.println("发送数据至服务器失败，重启 <Send_Data_To_Server>");
		delay(100);
		nvic_sys_reset();
	}
}

/*
 *brief   : 关闭相关电源，进入待机模式
 *para    : 无
 *return  :
 */
void SYS_Sleep(void)
{
	#if GS100_DEVICE_V1_0
		Timer2.detachCompare1Interrupt();
		LED1_OFF;
		LED2_OFF;
		GPRS_PWR_OFF;

		PWR_WakeUpPinCmd(ENABLE); //使能唤醒引脚，默认PA0
		PWR_ClearFlag(PWR_FLAG_WU);
		PWR_EnterSTANDBYMode(); //进入待机
	#else
		Timer2.detachCompare1Interrupt();
		LED1_OFF;
		LED2_OFF;
		LED3_OFF;
		LED4_OFF;
		GPRS_PWR_OFF;
		DC12V_PWR_OFF;
		USB_PORT_DIS;
		GPS_ANT_PWR_OFF;

		PWR_WakeUpPinCmd(ENABLE); //使能唤醒引脚，默认PA0
		PWR_ClearFlag(PWR_FLAG_WU);
		PWR_EnterSTANDBYMode(); //进入待机
	#endif
}

/*
 *brief   : 按键2清除储存的系统相关参数
 *para    : 无
 *return  : 无
 */
void Key_Clear_Device_Parameter(void)
{
	if (digitalRead(KEY2_INPUT) == LOW)
	{
		GSM_STATUS_LED_ON;
		Delay_ms(1000);
		if (digitalRead(KEY2_INPUT) == LOW)
		{
			Clear_HostID = true;
			Serial.println("Clear System parameter OK...");
			GSM_STATUS_LED_OFF;
		}
	}
}

/*
 *brief   : 按键1清除储存在EP中的传感器数据
 *para    : 无
 *return  : 无
 */
void Key_Clear_Muti_Sensor_Data(void)
{
	if (digitalRead(KEY1_INPUT) == LOW)
	{
		GSM_STATUS_LED_ON;
		Delay_ms(1000);
		if (digitalRead(KEY1_INPUT) == LOW)
		{
			#if (DEVICE_V2_5 || GS100_DEVICE_V1_0)
				EP_Write_Enable();
			#endif

			EpromDb.open(EEPROM_BASE_ADDR);
			EpromDb.clear();

			#if (DEVICE_V2_5 || GS100_DEVICE_V1_0)
				EP_Write_Disable();
			#endif

			Serial.println("Clear Sensor Data OK...");
			GSM_STATUS_LED_OFF;
		}
	}
}

/*
 *brief   : 定时器2中断函数
 *para    : 无
 *return  : 无
*/
void Time2_Handler(void)
{
	Toggle ^= 1;
	digitalWrite(LED1, Toggle); //状态灯闪烁

	Run_Time_Out_Sec++;
	//如果运行超时，复位
	if (Run_Time_Out_Sec >= 300)
	{
		Run_Time_Out_Sec = 0;
		noInterrupts();
		nvic_sys_reset();
	}
}
