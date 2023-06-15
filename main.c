#include "MDR32F9Qx_config.h"
#include "MDR32Fx.h"
#include "MDR32F9Qx_i2c.h"
#include "MDR32F9Qx_port.h"
#include "MDR32F9Qx_rst_clk.h"
#include <MDR32F9Qx_timer.h>
#include <MDR32F9Qx_it.h>

// подключение библиотеки для работы с MPU9250
#include "MPU9250.h"

// math
#include <math.h>

uint16_t j;
#define DELAY(T) for (j = T; j > 0; j--)

void init_I2C();
void readData();
void printData();
void servo_impulse();
/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef I2C_InitStruct;
PORT_InitTypeDef PortInit;


float Pitch[2] = {0, 0};
float Roll[2] = {0, 0};

float RAD_TO_DEG = 57.295779513082320876798154814105;
float Pitch_to_print;
float Roll_to_print;
float apitch; 
float aroll;
float sumax = 0;
float sumay = 0; 
float sumaz = 0; 
float sumgx = 0;
float sumgy = 0;
float sumgz = 0;
int ti = 0;
short calibration = 1;

float dax, day, daz, dgx, dgy, dgz;
int TTK;
int TTN;

int16_t accelX;
int16_t accelY;
int16_t accelZ;

	
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;



// Процедура инициализации таймера 
void Timer1Init() { 
	TIMER_CntInitTypeDef sTIM_CntInit;
	TIMER_ChnInitTypeDef sTIM_ChnInit;
	TIMER_ChnOutInitTypeDef sTIM_ChnOutInit;
	
	
	// Включение тактирования 
	RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER1, ENABLE); 
	// Заполнение структуры значениями по умолчанию 
	TIMER_CntStructInit(&sTIM_CntInit); 
	// Настройка делителя тактовой частоты 
	TIMER_BRGInit (MDR_TIMER1, TIMER_HCLKdiv1); 
	// Задание предделителя тактовой частоты 
	sTIM_CntInit.TIMER_Prescaler = 4; 
	// Задание периода срабатывания таймера 
	sTIM_CntInit.TIMER_Period = 30000; 
	// Инициализация таймера 1
	TIMER_CntInit (MDR_TIMER1,&sTIM_CntInit);
	/* Инициализация каналов таймера 1: СН1,СН1N,СН2,СН2N,СН3,СН3N */
	TIMER_ChnStructInit(&sTIM_ChnInit);
	// Режим работы канала - генерация ШИМ
	sTIM_ChnInit.TIMER_CH_Mode = TIMER_CH_MODE_PWM;
	// Формат выработки сигнала REF номер 6:
	// при счете вверх – REF = 1 если CNT<CCR, иначе REF = 0;
	// значение регистра CCR определяет длительность импульса ШИМ
	sTIM_ChnInit.TIMER_CH_REF_Format = TIMER_CH_REF_Format6;
	// Инициализируем канал 1
	sTIM_ChnInit.TIMER_CH_Number = TIMER_CHANNEL1;
	TIMER_ChnInit(MDR_TIMER1, &sTIM_ChnInit);
	
	// Инициализируем канал 2
	sTIM_ChnInit.TIMER_CH_Number = TIMER_CHANNEL2;
	TIMER_ChnInit(MDR_TIMER1, &sTIM_ChnInit);
	
	
	// Устанавливаем длительность импульсов по каждому каналу
	TIMER_SetChnCompare(MDR_TIMER1, TIMER_CHANNEL1, 2475);
	TIMER_SetChnCompare(MDR_TIMER1, TIMER_CHANNEL2, 2475);


	// Заполнение элементов структуры значениями по умолчанию
	TIMER_ChnOutStructInit(&sTIM_ChnOutInit);
	// Выбор источника сигнала для прямого выхода CHxN - сигнал REF
	sTIM_ChnOutInit.TIMER_CH_DirOut_Source = TIMER_CH_OutSrc_REF;
	// Настройна прямого выхода микроконтроллера CHxN на вывод данных
	sTIM_ChnOutInit.TIMER_CH_DirOut_Mode = TIMER_CH_OutMode_Output;
	// Выбор источника сигнала для инверсного выхода CHxN - сигнал REF
	sTIM_ChnOutInit.TIMER_CH_NegOut_Source = TIMER_CH_OutSrc_REF;
	// Настройна инверсного выхода микроконтроллера CHxN на вывод данных
	sTIM_ChnOutInit.TIMER_CH_NegOut_Mode = TIMER_CH_OutMode_Output;

	// Настраиваем выходы канала 1
	sTIM_ChnOutInit.TIMER_CH_Number = TIMER_CHANNEL1;
	TIMER_ChnOutInit(MDR_TIMER1, &sTIM_ChnOutInit);
	
	// Настраиваем выходы канала 2
	sTIM_ChnOutInit.TIMER_CH_Number = TIMER_CHANNEL2;
	TIMER_ChnOutInit(MDR_TIMER1, &sTIM_ChnOutInit);	

	/* Включаем делитель тактовой частоты таймера 1*/
	TIMER_BRGInit(MDR_TIMER1,TIMER_HCLKdiv1);
	
	// Включение прерываний 
	NVIC_EnableIRQ (TIMER1_IRQn); 
	// Установка приоритета прерываний 
	NVIC_SetPriority (TIMER1_IRQn, 0); 
	// Включение прерывания при равенстве нулю значения TIMER1 
	TIMER_ITConfig(MDR_TIMER1, TIMER_STATUS_CNT_ARR, ENABLE);

	/* После всех настроек разрешаем работу таймера 1 */
	TIMER_Cmd(MDR_TIMER1,ENABLE);
}


void init_PortA(){
	
	PORT_InitTypeDef PORT_InitStructure;
//Включение тактирования порта A
	RST_CLK_PCLKcmd((RST_CLK_PCLK_PORTA), ENABLE);

/* Конфигурация выводов таймера TIMER1: CH1, CH1N, CH2, CH2N, CH3, CH3N */
/* Настройка порта А разряды 1, 3*/
	PORT_InitStructure.PORT_Pin =(PORT_Pin_1 | PORT_Pin_3);
	PORT_InitStructure.PORT_OE = PORT_OE_OUT;
	PORT_InitStructure.PORT_FUNC = PORT_FUNC_ALTER;
	PORT_InitStructure.PORT_MODE = PORT_MODE_DIGITAL;
	PORT_InitStructure.PORT_SPEED = PORT_SPEED_FAST;
	PORT_Init(MDR_PORTA, &PORT_InitStructure);
}


// Процедура обработки прерывания, вызванного таймером 1 
void Timer1_IRQHandler() { 
	// Если таймер сброшен в ноль, вызвать процедуру чтения с датчика readData()
	// и сбросить флаг прерывания 
	if (TIMER_GetITStatus(MDR_TIMER1, TIMER_STATUS_CNT_ARR)) { 
		readData(); 

		TIMER_ClearITPendingBit(MDR_TIMER1, TIMER_STATUS_CNT_ARR); 
	} 
}


void readData() {
	init_I2C();
	uint8_t accel_regs[6] = {0, 0, 0, 0, 0, 0};
	uint8_t gyro_regs[6] = {0, 0, 0, 0, 0, 0};
	
	
	
	float Ts = 0.02; //Период опроса датчиков, с 
	float T = 1; //Постоянная КФ, с
	
	
	MPU9250_readAllGyroData(gyro_regs);
	MPU9250_readAllAccelData(accel_regs);
		
	accelX = convertTo16Bit(accel_regs[0], accel_regs[1]);
	accelY = convertTo16Bit(accel_regs[2], accel_regs[3]);
	accelZ = convertTo16Bit(accel_regs[4], accel_regs[5]);
		
	gyroX = convertTo16Bit(gyro_regs[0], gyro_regs[1]);
	gyroY = convertTo16Bit(gyro_regs[2], gyro_regs[3]);
	gyroZ = convertTo16Bit(gyro_regs[4], gyro_regs[5]);
	
	
	
	
	

		
		float gyroXFloat = gyroX / (131*RAD_TO_DEG);
		float gyroYFloat = gyroY / (131*RAD_TO_DEG); 
		float gyroZFloat = gyroZ / (131*RAD_TO_DEG);
	
	  //Угол крена и тангажа с помощью акселерометра 
		//int g = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
		float Axn =  accelX/sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2)); 
		float Ayn = accelY/sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2)); 
		apitch = asin(Axn) ; 
		aroll = -1 * asin(Ayn/cos(apitch)) ;
		
		
		float gpitch = gyroYFloat * cos(Roll[0])- gyroZFloat * sin(Roll[0]);
		float groll = gyroXFloat + tan(Pitch[0])*(gyroYFloat*sin(Roll[0])+gyroZFloat*cos(Roll[0]));
		
	
		
		Pitch[1] = Pitch[0] + (apitch - Pitch[0] + gpitch*T) * (Ts/T);
		Roll[1] = Roll[0] + (aroll - Roll[0] + groll*T) * (Ts/T); 
		Pitch[0] = Pitch[1]; 
		Roll[0] = Roll[1];
		
		if (Pitch[0] > 1) {
			Pitch[0] = 1;
		}
		
		if (Pitch[0] < -1) {
			Pitch[0] = -1;
		}
		
		if (Roll[0] > 1) {
			Roll[0] = 1;
		}
		
		if (Roll[0] < -1) {
			Roll[0] = -1;
		}
		
		Pitch_to_print = Pitch[0] * RAD_TO_DEG;
		Roll_to_print = Roll[0] * RAD_TO_DEG;
		
		
		servo_impulse();
}
	

void servo_impulse(){
	 
	int MMM = 2450;
	int TTT = 850;
	
	TTK=MMM+Pitch[0]*TTT-Roll[0]*TTT; // тангаж, PA1 - 12
	TTN=MMM+Roll[0]*TTT; 							// крен, PA3 - 10
	
	if (TTK > 3450) {
		TTK = 3450;
	}
	if (TTN > 3450) {
		TTN = 3450;
	}
	if (TTK < 900) {
		TTK = 900;
	}
	if (TTN < 900) {
		TTN = 900;
	}
	
	
	TIMER_SetChnCompare(MDR_TIMER1, TIMER_CHANNEL1, TTK); // тангаж
	TIMER_SetChnCompare(MDR_TIMER1, TIMER_CHANNEL2, TTN); // крен
	

}
	

void printData() {
	//LCD_Init();
	//LCD_ClearScreen();
	
	char strPitch[40];
	sprintf(strPitch, "pitch = %lf", Pitch_to_print);
		
	char strRoll[40];
	sprintf(strRoll, "roll = %lf", Roll_to_print);
		
	LCD_PutString(strPitch, 1);
	LCD_PutString(strRoll, 2);
		
	char strApitch[40];
	sprintf(strApitch, "Apitch = %lf", apitch * RAD_TO_DEG);
		
	char strAroll[40];
	sprintf(strAroll, "Aroll = %lf", aroll * RAD_TO_DEG);
		
	LCD_PutString(strApitch, 3);
	LCD_PutString(strAroll, 4);
}

int main (void) {
	init_I2C();
	init_PortA();
	Timer1Init();
	while (1) {
		//__NOP();

	}

}


void init_I2C() {
	/* Enables the HSI clock on PORTC */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC,ENABLE);

  /* Fill PortInit structure */
  PortInit.PORT_PULL_UP = PORT_PULL_UP_ON;
  PortInit.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
  PortInit.PORT_PD_SHM = PORT_PD_SHM_OFF;
  PortInit.PORT_PD = PORT_PD_DRIVER;
  PortInit.PORT_GFEN = PORT_GFEN_OFF;
  PortInit.PORT_FUNC = PORT_FUNC_ALTER;
  PortInit.PORT_SPEED = PORT_SPEED_MAXFAST;
  PortInit.PORT_MODE = PORT_MODE_DIGITAL;

  /* Configure PORTC pins 0,1 (I2C_SCL,I2C_SDA) */
  PortInit.PORT_Pin = PORT_Pin_0 | PORT_Pin_1;
  PORT_Init(MDR_PORTC, &PortInit);

  /* Enables the HSI clock on I2C */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_I2C,ENABLE);

  /* Enables I2C peripheral */
  I2C_Cmd(ENABLE);

  /* Initialize I2C_InitStruct */
  I2C_InitStruct.I2C_ClkDiv = 16;
  I2C_InitStruct.I2C_Speed = I2C_SPEED_UP_TO_400KHz;

  /* Configure I2C parameters */
  I2C_Init(&I2C_InitStruct);
}