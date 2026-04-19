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
#define OVP_LIMIT_MV                14000U

/* ADC通道定义 */
#define ADC_CHANNEL_VOUT            ADC_Channel_0      /* PA0 */
#define ADC_CHANNEL_IOUT            ADC_Channel_1      /* PA1 */

/* ADC采样平均次数 */
#define VOLTAGE_AVG_COUNT           32U
#define CURRENT_AVG_COUNT           16U

/* 控制周期：主循环 Delay_ms(20) -> 20ms */
#define CONTROL_PERIOD_S            0.02f

/* PID参数
   你测试认为这组比较合适：
   P = 0.5
   I = 10
   D = 0
 */
#define PID_KP                      0.5f
#define PID_KI                      10.0f
#define PID_KD                      0.0f

/* PID输出到 DAC 码值增量的映射系数
   这里采用“增量式 PID”：
   先根据误差计算本周期应增加/减少多少控制量，
   再把这个控制量映射成 DAC 码值增量。

   经验上取 32 比较接近你原来分段逼近的动作量级：
   - 误差 1.0V 时，首个周期大约调整 22 个码值
   - 误差 0.5V 时，首个周期大约调整 11 个码值
 */
#define PID_OUTPUT_TO_DAC_CODE      32.0f

/* 每个控制周期允许的最大 DAC 变化量，防止瞬间跳太大 */
#define PID_MAX_DELTA_CODE          40

/* 目标接近时的静区，减小抖动 */
#define PID_DEADBAND_MV             40

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

static uint32_t g_SetVoltage_mV = 0;        /* 待确认设定值 */
static uint32_t g_TargetVoltage_mV = 0;     /* 已确认的目标输出电压 */
static uint32_t g_OutputVoltage_mV = 0;     /* 实际输出电压（校准后） */
static uint32_t g_OutputCurrent_mA = 0;     /* 实际输出电流 */
static uint16_t g_DacCode = 0;              /* 当前 DAC 数码值 */
static uint8_t  g_ProtectFlag = 0;          /* 保护标志 */
static uint8_t  g_ProtectType = 0;          /* 0:无保护 1:OCP 2:OVP */
static uint32_t g_StepVoltage_mV = STEP_SMALL_MV;

/* PID历史误差（单位：V）
   用于增量式 PID：
   du = Kp*(e[k]-e[k-1]) + Ki*e[k]*T + Kd*(e[k]-2e[k-1]+e[k-2])/T
 */
static float g_PidErr_1 = 0.0f;
static float g_PidErr_2 = 0.0f;

/* ============================================================
 *                    电压校准函数
 * ============================================================
 */

static uint32_t Voltage_Calibration(uint32_t Raw_mV)
{
	float y_V;
	float x_V;
	uint32_t Real_mV;

	if (Raw_mV == 0)
	{
		return 0;
	}

	y_V = (float)Raw_mV / 1000.0f;
	x_V = powf(y_V, VOLT_FIT_INV_A);
	Real_mV = (uint32_t)(x_V * 1000.0f + 0.5f);

	return Real_mV-100.0f;
}

/* ============================================================
 *                    采样函数区
 * ============================================================
 */

static uint32_t Read_OutputVoltage_mV(void)
{
	uint16_t ADC_Value;
	uint32_t ADC_mV;
	uint32_t RawVout_mV;
	uint32_t CalVout_mV;

	ADC_Value = AD_GetAverage(ADC_CHANNEL_VOUT, VOLTAGE_AVG_COUNT);
	ADC_mV = (uint32_t)ADC_Value * ADC_VREF_MV / 4095U;
	RawVout_mV = ADC_mV * VOLT_DIVIDER_NUM / VOLT_DIVIDER_DEN;
	CalVout_mV = Voltage_Calibration(RawVout_mV);

	return CalVout_mV;
}

static uint32_t Read_OutputCurrent_mA(void)
{
	uint16_t ADC_Value;
	uint32_t ADC_mV;
	uint32_t Iout_mA;

	ADC_Value = AD_GetAverage(ADC_CHANNEL_IOUT, CURRENT_AVG_COUNT);
	ADC_mV = (uint32_t)ADC_Value * ADC_VREF_MV / 4095U;

	Iout_mA = ADC_mV * 1000U / CURRENT_ADC_MV_PER_A;

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
 *                    PID辅助函数区
 * ============================================================
 */

static void PID_Reset(void)
{
	g_PidErr_1 = 0.0f;
	g_PidErr_2 = 0.0f;
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
		g_ProtectType = 1;    /* OCP */
		g_TargetVoltage_mV = 0;
		PID_Reset();
		Output_Shutdown();
		return;
	}

	if (g_OutputVoltage_mV > OVP_LIMIT_MV)
	{
		g_ProtectFlag = 1;
		g_ProtectType = 2;    /* OVP */
		g_TargetVoltage_mV = 0;
		PID_Reset();
		Output_Shutdown();
		return;
	}
}

/* ============================================================
 *                    控制算法函数区
 * ============================================================
 */

/* 增量式 PID 控制
 *
 * 说明：
 * 1. 只跟随“已确认目标值 g_TargetVoltage_mV”
 * 2. 用户修改 g_SetVoltage_mV 时，DAC 不会立即动作
 * 3. 只有按下确认键后，才把设定值提交给闭环控制
 * 4. 控制输出不是直接给一个绝对 DAC 值，而是每个周期修正一个 DAC 增量
 */
static void Control_Update(void)
{
	int32_t Error_mV;
	float Error_V;
	float DeltaU;
	int32_t DeltaCode;
	int32_t NewCode;

	if (g_ProtectFlag)
	{
		return;
	}

	/* 设定目标为 0V 时，直接关断输出并清空 PID 状态 */
	if (g_TargetVoltage_mV == 0U)
	{
		PID_Reset();
		if (g_DacCode != 0U)
		{
			Output_Shutdown();
		}
		return;
	}

	Error_mV = (int32_t)g_TargetVoltage_mV - (int32_t)g_OutputVoltage_mV;

	/* 小误差区不动作，减小抖动 */
	if ((Error_mV >= -PID_DEADBAND_MV) && (Error_mV <= PID_DEADBAND_MV))
	{
		g_PidErr_2 = g_PidErr_1;
		g_PidErr_1 = (float)Error_mV / 1000.0f;
		return;
	}

	Error_V = (float)Error_mV / 1000.0f;

	/* 增量式 PID */
	DeltaU = PID_KP * (Error_V - g_PidErr_1)
	       + PID_KI * Error_V * CONTROL_PERIOD_S
	       + PID_KD * (Error_V - 2.0f * g_PidErr_1 + g_PidErr_2) / CONTROL_PERIOD_S;

	/* 把控制增量映射成 DAC 码值增量 */
	DeltaCode = (int32_t)(DeltaU * PID_OUTPUT_TO_DAC_CODE);

	/* 四舍五入，避免小数截断偏差 */
	if ((DeltaU * PID_OUTPUT_TO_DAC_CODE) >= 0.0f)
	{
		DeltaCode = (int32_t)(DeltaU * PID_OUTPUT_TO_DAC_CODE + 0.5f);
	}
	else
	{
		DeltaCode = (int32_t)(DeltaU * PID_OUTPUT_TO_DAC_CODE - 0.5f);
	}

	/* 限制单周期最大调整量 */
	if (DeltaCode > PID_MAX_DELTA_CODE)
	{
		DeltaCode = PID_MAX_DELTA_CODE;
	}
	else if (DeltaCode < -PID_MAX_DELTA_CODE)
	{
		DeltaCode = -PID_MAX_DELTA_CODE;
	}

	NewCode = (int32_t)g_DacCode + DeltaCode;

	if (NewCode < 0)
	{
		NewCode = 0;
	}
	if (NewCode > 4000)
	{
		NewCode = 4000;
	}

	DAC_Output((uint16_t)NewCode);

	/* 更新误差历史 */
	g_PidErr_2 = g_PidErr_1;
	g_PidErr_1 = Error_V;
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
		if (g_SetVoltage_mV + g_StepVoltage_mV <= TARGET_MAX_MV)
		{
			g_SetVoltage_mV += g_StepVoltage_mV;
		}
		else
		{
			g_SetVoltage_mV = TARGET_MAX_MV;
		}
	}
	else if (KeyNum == 2)
	{
		if (g_SetVoltage_mV >= g_StepVoltage_mV)
		{
			g_SetVoltage_mV -= g_StepVoltage_mV;
		}
		else
		{
			g_SetVoltage_mV = TARGET_MIN_MV;
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
	else if (KeyNum == 4)
	{
		/* 确认键：清除保护并提交当前设定值 */
		if (g_ProtectFlag)
		{
			g_ProtectFlag = 0;
			g_ProtectType = 0;
			Output_Shutdown();
		}

		g_TargetVoltage_mV = g_SetVoltage_mV;
		PID_Reset();
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

static void Display_Update(void)
{
	char Buf[17];

	OLED_ShowFixedVoltage(1, "SET", g_SetVoltage_mV);
	OLED_ShowFixedVoltage(2, "OUT", g_OutputVoltage_mV);

	if (g_StepVoltage_mV == STEP_SMALL_MV)
	{
		sprintf(Buf, "STEP:0.1V %c   ", (g_SetVoltage_mV == g_TargetVoltage_mV) ? ' ' : '*');
	}
	else
	{
		sprintf(Buf, "STEP:1.0V %c   ", (g_SetVoltage_mV == g_TargetVoltage_mV) ? ' ' : '*');
	}
	OLED_ShowString(3, 1, Buf);

	if (g_ProtectFlag)
{
	if (g_ProtectType == 1)
	{
		sprintf(Buf, "PROTECT:OCP   ");
	}
	else if (g_ProtectType == 2)
	{
		sprintf(Buf, "PROTECT:OVP   ");
	}
	else
	{
		sprintf(Buf, "PROTECT       ");
	}
}
	else
	{
		sprintf(Buf, "DAC:%4u A:%2lu.%02lu", g_DacCode,
				g_TargetVoltage_mV / 1000U,
				(g_TargetVoltage_mV % 1000U) / 10U);
	}
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

	g_SetVoltage_mV = 0;
	g_TargetVoltage_mV = 0;
	g_OutputVoltage_mV = 0;
	g_OutputCurrent_mA = 0;
	g_DacCode = 0;
	g_ProtectFlag = 0;
	g_StepVoltage_mV = STEP_SMALL_MV;
	PID_Reset();

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
