#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "AD.h"
#include "Key.h"
#include "MCP4725.h"
#include <stdio.h>
#include <math.h>

/* ============================================================
 *                    可调参数区 / 宏定义区
 * ============================================================
 */

/* ADC参考电压，单位 mV */
#define ADC_VREF_MV                 3300U

/* 电压采样分压比例
   R11 = 15k, R10 = 5k
   则 ADC 采到的是输出电压的 1/4 */
#define VOLT_DIVIDER_NUM            4U
#define VOLT_DIVIDER_DEN            1U

/* 电流采样换算参数
   当前按：1A -> ADC_I 大约 1.55V */
#define CURRENT_ADC_MV_PER_A        1550U

/* 输出电压允许的设定范围，单位 mV */
#define TARGET_MIN_MV               0U
#define TARGET_MAX_MV               12000U

/* 两种步进长度
   STEP_SMALL_MV = 0.1V
   STEP_LARGE_MV = 1.0V */
#define STEP_SMALL_MV               100U
#define STEP_LARGE_MV               1000U

/* 保护阈值 */
#define OCP_LIMIT_MA                1250U
#define OVP_LIMIT_MV                12300U

/* ADC通道定义 */
#define ADC_CHANNEL_VOUT            ADC_Channel_0      /* PA0 */
#define ADC_CHANNEL_IOUT            ADC_Channel_1      /* PA1 */

/* ADC采样平均次数 */
#define VOLTAGE_AVG_COUNT           32U
#define CURRENT_AVG_COUNT           16U

/* 电压校准模型参数
   你拟合得到的是：
   y = x^A

   其中：
   y = 程序原始显示的 OUT 值（单位 V）
   x = 实际输出电压（单位 V）

   所以反求实际电压：
   x = y^(1/A)
*/
#define VOLT_FIT_A                  1.00565f
#define VOLT_FIT_INV_A              (1.0f / VOLT_FIT_A)

/* ============================================================
 *                    全局变量区
 * ============================================================
 */

static uint32_t g_TargetVoltage_mV = 0;     /* 目标输出电压（设定值） */
static uint32_t g_OutputVoltage_mV = 0;     /* 实际输出电压（校准后） */
static uint32_t g_OutputCurrent_mA = 0;     /* 实际输出电流 */
static uint16_t g_DacCode = 0;              /* 当前 DAC 数码值 */
static uint8_t  g_ProtectFlag = 0;          /* 保护标志 */
static uint32_t g_StepVoltage_mV = STEP_SMALL_MV;

/* ============================================================
 *                    电压校准函数
 * ============================================================
 */

/* 把“程序原始换算得到的电压值”校准成“实际输出电压”
 *
 * 输入：
 *   Raw_mV -> 原始换算电压，单位 mV
 *
 * 输出：
 *   校准后的实际电压，单位 mV
 *
 * 注意：
 *   拟合关系 y = x^A 是以“伏特 V”为单位建立的，
 *   所以这里必须先把 mV 转成 V，再做幂运算，最后再转回 mV。
 */
static uint32_t Voltage_Calibration(uint32_t Raw_mV)
{
	float y_V;            /* 原始电压值，单位 V */
	float x_V;            /* 校准后的实际电压，单位 V */
	uint32_t Real_mV;     /* 校准后的实际电压，单位 mV */

	/* 防止 0 或非常小的数参与幂运算后出现不必要误差 */
	if (Raw_mV == 0)
	{
		return 0;
	}

	/* mV -> V */
	y_V = (float)Raw_mV / 1000.0f;

	/* 根据 x = y^(1/A) 反推实际电压 */
	x_V = powf(y_V, VOLT_FIT_INV_A);

	/* V -> mV，并四舍五入 */
	Real_mV = (uint32_t)(x_V * 1000.0f + 0.5f);

	return Real_mV;
}

/* ============================================================
 *                    采样函数区
 * ============================================================
 */

/* 读取输出电压，返回值单位 mV
 *
 * 流程：
 * 1. ADC采样得到 ADC_V 节点电压
 * 2. 根据分压比换算出“原始输出电压”
 * 3. 用拟合关系做校准
 * 4. 返回“校准后的实际输出电压”
 */
static uint32_t Read_OutputVoltage_mV(void)
{
	uint16_t ADC_Value;      /* ADC 原始值 */
	uint32_t ADC_mV;         /* ADC引脚上的实际电压，单位 mV */
	uint32_t RawVout_mV;     /* 未校准前的输出电压，单位 mV */
	uint32_t CalVout_mV;     /* 校准后的输出电压，单位 mV */

	/* 对电压通道采样 32 次并求平均，降低抖动 */
	ADC_Value = AD_GetAverage(ADC_CHANNEL_VOUT, VOLTAGE_AVG_COUNT);

	/* ADC原始值 -> ADC引脚电压 */
	ADC_mV = (uint32_t)ADC_Value * ADC_VREF_MV / 4095U;

	/* 根据分压比反推原始输出电压 */
	RawVout_mV = ADC_mV * VOLT_DIVIDER_NUM / VOLT_DIVIDER_DEN;

	/* 用拟合关系做电压校准 */
	CalVout_mV = Voltage_Calibration(RawVout_mV);

	return CalVout_mV;
}

/* 读取输出电流，返回值单位 mA */
static uint32_t Read_OutputCurrent_mA(void)
{
	uint16_t ADC_Value;
	uint32_t ADC_mV;
	uint32_t Iout_mA;

	ADC_Value = AD_GetAverage(ADC_CHANNEL_IOUT, CURRENT_AVG_COUNT);
	ADC_mV = (uint32_t)ADC_Value * ADC_VREF_MV / 4095U;

	Iout_mA = ADC_mV * 1000U / CURRENT_ADC_MV_PER_A;

	/* 很小的电流直接认为是 0 */
	if (Iout_mA < 10U)
	{
		Iout_mA = 0;
	}

	return Iout_mA;
}

/* ============================================================
 *                    DAC输出控制函数区
 * ============================================================
 */

static void DAC_Output(uint16_t Code)
{
	if (Code > 4095)
	{
		Code = 4095;
	}

	g_DacCode = Code;
	MCP4725_SetValue(g_DacCode);
}

static void Output_Shutdown(void)
{
	DAC_Output(0);
}

/* ============================================================
 *                    保护逻辑函数区
 * ============================================================
 */

static void Protection_Check(void)
{
	if (g_OutputCurrent_mA > OCP_LIMIT_MA)
	{
		g_ProtectFlag = 1;
		g_TargetVoltage_mV = 0;
		Output_Shutdown();
	}

	if (g_OutputVoltage_mV > OVP_LIMIT_MV)
	{
		g_ProtectFlag = 1;
		g_TargetVoltage_mV = 0;
		Output_Shutdown();
	}
}

/* ============================================================
 *                    控制算法函数区
 * ============================================================
 */

/* 简单闭环控制
 *
 * 与原版相比：
 * 1. 使用“校准后的实际输出电压”做闭环比较
 * 2. 死区由 ±40mV 缩小到 ±20mV
 * 这样更容易把最终误差压到 0.05V 以内
 */
static void Control_Update(void)
{
	int32_t Error;
	int16_t Step = 0;
	int32_t NewCode;

	if (g_ProtectFlag)
	{
		return;
	}

	/* 误差 = 目标值 - 实际值（这里的实际值已经校准过） */
	Error = (int32_t)g_TargetVoltage_mV - (int32_t)g_OutputVoltage_mV;

	if (Error > 1200)
	{
		Step = 24;
	}
	else if (Error > 500)
	{
		Step = 10;
	}
	else if (Error > 150)
	{
		Step = 4;
	}
	else if (Error > 60)
	{
		Step = 2;
	}
	else if (Error > 40)
	{
		Step = 1;
	}
	else if (Error < -1200)
	{
		Step = -24;
	}
	else if (Error < -500)
	{
		Step = -10;
	}
	else if (Error < -150)
	{
		Step = -4;
	}
	else if (Error < -60)
	{
		Step = -2;
	}
	else if (Error < -40)
	{
		Step = -1;
	}
	else
	{
		/* 误差在 ±20mV 以内时不再调节 */
		Step = 0;
	}

	NewCode = (int32_t)g_DacCode + Step;

	if (NewCode < 0)
	{
		NewCode = 0;
	}
	if (NewCode > 4000)
	{
		NewCode = 4000;
	}

	DAC_Output((uint16_t)NewCode);
}

/* ============================================================
 *                    按键处理函数区
 * ============================================================
 */

static void Key_Process(void)
{
	uint8_t KeyNum;

	KeyNum = Key_GetNum();

	if (KeyNum == 1)
	{
		if (g_ProtectFlag)
		{
			g_ProtectFlag = 0;
			g_TargetVoltage_mV = 0;
			Output_Shutdown();
		}

		if (g_TargetVoltage_mV + g_StepVoltage_mV <= TARGET_MAX_MV)
		{
			g_TargetVoltage_mV += g_StepVoltage_mV;
		}
		else
		{
			g_TargetVoltage_mV = TARGET_MAX_MV;
		}
	}
	else if (KeyNum == 2)
	{
		if (g_ProtectFlag)
		{
			g_ProtectFlag = 0;
			g_TargetVoltage_mV = 0;
			Output_Shutdown();
		}

		if (g_TargetVoltage_mV >= g_StepVoltage_mV)
		{
			g_TargetVoltage_mV -= g_StepVoltage_mV;
		}
		else
		{
			g_TargetVoltage_mV = TARGET_MIN_MV;
		}
	}
	else if (KeyNum == 3)
	{
		if (g_StepVoltage_mV == STEP_SMALL_MV)
		{
			g_StepVoltage_mV = STEP_LARGE_MV;
		}
		else
		{
			g_StepVoltage_mV = STEP_SMALL_MV;
		}
	}
}

/* ============================================================
 *                    OLED显示辅助函数区
 * ============================================================
 */

static void OLED_ShowFixedVoltage(uint8_t Line, const char *Title, uint32_t mV)
{
	char Buf[17];
	uint32_t V_int;
	uint32_t V_dec;

	V_int = mV / 1000U;
	V_dec = (mV % 1000U) / 10U;

	sprintf(Buf, "%s:%2lu.%02luV   ", Title, V_int, V_dec);
	OLED_ShowString(Line, 1, Buf);
}

/* 显示内容：
   第1行：设定电压
   第2行：实际输出电压（校准后）
   第3行：当前步进值
   第4行：当前DAC设定值
*/
static void Display_Update(void)
{
	char Buf[17];

	OLED_ShowFixedVoltage(1, "SET", g_TargetVoltage_mV);
	OLED_ShowFixedVoltage(2, "OUT", g_OutputVoltage_mV);

	if (g_StepVoltage_mV == STEP_SMALL_MV)
	{
		sprintf(Buf, "STEP:0.1V      ");
	}
	else
	{
		sprintf(Buf, "STEP:1.0V      ");
	}
	OLED_ShowString(3, 1, Buf);

	sprintf(Buf, "DAC:%4u        ", g_DacCode);
	OLED_ShowString(4, 1, Buf);
}

/* ============================================================
 *                    主函数
 * ============================================================
 */

int main(void)
{
	OLED_Init();
	OLED_Clear();

	AD_Init();
	Key_Init();
	MCP4725_Init();

	g_TargetVoltage_mV = 0;
	g_OutputVoltage_mV = 0;
	g_OutputCurrent_mA = 0;
	g_DacCode = 0;
	g_ProtectFlag = 0;
	g_StepVoltage_mV = STEP_SMALL_MV;

	DAC_Output(0);

	while (1)
	{
		Key_Process();

		g_OutputVoltage_mV = Read_OutputVoltage_mV();
		g_OutputCurrent_mA = Read_OutputCurrent_mA();

		Protection_Check();
		Control_Update();
		Display_Update();

		Delay_ms(20);
	}
}