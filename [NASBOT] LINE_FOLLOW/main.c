#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"

#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/qei.h"

#include "utils/uartstdio.h"

#define CHANNEL_NUMBER		8
#define ADC_TO_MV			0.806			// = 4095/3300
#define DELTA				80

#define PWM_FREQUENCY 		20000
#define MAX_QEI_POS			29000			// 1 cycle ~ 14500 pulses

#define RSAB_PIN				GPIO_PIN_4
#define RSCD_PIN				GPIO_PIN_3
#define RS_PORT_BASE			GPIO_PORTA_BASE


//* Private function prototype ----------------------------------------------*/
static void QEI0_VelocityIsr(void);
static void QEI1_VelocityIsr(void);
bool qei_getVelocity(bool Select, int32_t *Velocity);
void hbridge_enable(bool en);
void Uart_RF_config(void);
//* Private variables -------------------------------------------------------*/
static bool qei_velocity_timeout[2];
static int32_t qei_velocity[2] = {0, 0};
uint32_t raw_ADC[CHANNEL_NUMBER];
uint32_t calib_white[CHANNEL_NUMBER], calib_black[CHANNEL_NUMBER];
char sensor_state;

uint16_t i;

uint32_t qeiPositionA;
uint32_t qeiPositionB;

volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile int16_t ui16Adjust;
int32_t Velocity;

enum mode
{
	CALIB_SENSOR,
	TEST_HW,
	LINE_FOLLOW,
} mode;

enum calib_color
{
	CALIB_BLACK,
	CALIB_WHITE,
} calib_color;

int main(void)
{
	//======================================//
	// System clock
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

	//======================================//
	// H-Bridge Reset Pins - Reset @ startup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(RS_PORT_BASE, RSAB_PIN|RSCD_PIN);
	GPIOPinWrite(RS_PORT_BASE, RSAB_PIN, 0);
	GPIOPinWrite(RS_PORT_BASE, RSCD_PIN, 0);

	//======================================//
	// QEI
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	//Unlock PD7 - Like PF0 its used for NMI - Without this step it doesn't work
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

	//Set Pins to be PHA0 and PHB0
	GPIOPinConfigure(GPIO_PD6_PHA0);
	GPIOPinConfigure(GPIO_PD7_PHB0);

	GPIOPinConfigure(GPIO_PC5_PHA1);
	GPIOPinConfigure(GPIO_PC6_PHB1);

	//Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
	GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

	//DISable peripheral and int before configuration
//	QEIDisable(QEI0_BASE);
//	QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
	//	QEIDisable(QEI1_BASE);
	//	QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);


	// Configure quadrature encoder, use an arbitrary top limit of 1000
	QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP, MAX_QEI_POS);
	QEIConfigure(QEI1_BASE, QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET | QEI_CONFIG_QUADRATURE | QEI_CONFIG_SWAP, MAX_QEI_POS);

	//Set position to a middle value so we can see if things are working
	QEIPositionSet(QEI0_BASE, MAX_QEI_POS/2);
	QEIPositionSet(QEI1_BASE, MAX_QEI_POS/2);

	QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_2, 	SysCtlClockGet() * 20 / 1000);
	QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_2, 	SysCtlClockGet() * 20 / 1000);

	QEIVelocityEnable(QEI0_BASE);
	QEIVelocityEnable(QEI1_BASE);

	QEIEnable(QEI0_BASE);
	QEIEnable(QEI1_BASE);

	QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
	QEIIntEnable(QEI1_BASE, QEI_INTTIMER);

	QEIIntRegister(QEI0_BASE, &QEI0_VelocityIsr);
	QEIIntRegister(QEI1_BASE, &QEI1_VelocityIsr);

	GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD_WPU);

	//======================================//
	// PWM
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);				// PWM Clock = SystemCLK/1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		// PWM Channel: PA6,7

	// PWM Configuration
	ui32PWMClock = SysCtlClockGet() / 1;	// Calculate PWM clock
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

	// 2 PWM channels: PA6, PA7 (module 1, generator 1)
	//		GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7);

	//		GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PA6_M1PWM2);
	GPIOPinConfigure(GPIO_PA7_M1PWM3);

	//	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	//	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);


	//		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui16Adjust * ui32Load / 1000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui16Adjust * ui32Load / 1000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui16Adjust * ui32Load / 1000);
	//		PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
	//		PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);

	//======================================//
	// UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);


	UARTStdioConfig(0, 115200, SysCtlClockGet());

	IntMasterEnable(); //enable processor interrupts
	IntEnable(INT_UART0); //enable the UART interrupt
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts

	// Enable GPIO
	//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	//	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5);
	//	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_4|GPIO_PIN_5);

	//======================================//
	// ADC
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);		// ADC_IN: PD0,1,2,3 - AIN7,6,5,4
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);		// ADC_IN: PE0,1,2,3 - AIN3,2,1,0

	ADCHardwareOversampleConfigure(ADC0_BASE, 64);

	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);			//SS0: Depth 8
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH5);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH6);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH7 | ADC_CTL_IE | ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 0);

	//======================================//
	// User Buttons
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// Unlock Hardware function @PF0
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

	// Buttons: PF0 + PF4
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

	// User LEDs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	//======================================//
	// LED SENSOR
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);

	//======================================//
	// Initial State
	mode = CALIB_SENSOR;
	ui16Adjust = 500;
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
	UARTprintf("CALIB_SENSOR\r\n");
	UARTprintf("CALIB_BLACK: ");

	// Disable H-bridge
	hbridge_enable(false);

	while(1)
	{
		// SW1: MODE SELECT
		if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
		{
			switch(mode){
			case CALIB_SENSOR:
				mode = TEST_HW;
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_3);
				UARTprintf("\r\nTEST_HW\r\n");
				break;

			case TEST_HW:
				mode = LINE_FOLLOW;
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
				UARTprintf("\r\nLINE_FOLLOW\r\n");

				hbridge_enable(true);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui16Adjust * ui32Load / 1000);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui16Adjust * ui32Load / 1000);

				break;

			case LINE_FOLLOW:
//				mode = CALIB_SENSOR;
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_2);
//				UARTprintf("\r\nCALIB_SENSOR\r\n");
				break;
			}
			SysCtlDelay(5000000);	// Delay for BUTTON
		}

		switch (mode){
		case CALIB_SENSOR:
			// SW2: LOG CALIB DATA
			// ADC Process
			if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
			{
				if(calib_color == CALIB_BLACK){
					ADCIntClear(ADC0_BASE, 0);					// Reset ADC Interrupt Flag
					ADCProcessorTrigger(ADC0_BASE, 0);			// Start new ADC cycle
					while(!ADCIntStatus(ADC0_BASE, 0, false)){}	// Wait for Conversion
					ADCSequenceDataGet(ADC0_BASE, 0, calib_black);	// Update calib_black

					for(i=0;i<CHANNEL_NUMBER;i++){
						UARTprintf("%04d ",calib_black[i]);
					}
					UARTprintf("\r\n");
					calib_color = CALIB_WHITE;
					UARTprintf("CALIB_WHITE: ");
				}

				else if(calib_color == CALIB_WHITE){
					ADCIntClear(ADC0_BASE, 0);					// Reset ADC Interrupt Flag
					ADCProcessorTrigger(ADC0_BASE, 0);			// Start new ADC cycle
					while(!ADCIntStatus(ADC0_BASE, 0, false)){}	// Wait for Conversion
					ADCSequenceDataGet(ADC0_BASE, 0, calib_white);	// Update calib_black
					// Print ADC values
					for(i=0;i<CHANNEL_NUMBER;i++){
						UARTprintf("%04d ",calib_white[i]);
					}
					UARTprintf("\r\n");
					calib_color = CALIB_BLACK;
					UARTprintf("CALIB_BLACK: ");
				}
				SysCtlDelay(5000000);	// Delay for BUTTON
			}
			break;

		case TEST_HW:
			// Read ADC
			ADCIntClear(ADC0_BASE, 0);					// Reset ADC Interrupt Flag
			ADCProcessorTrigger(ADC0_BASE, 0);			// Start new ADC cycle
			while(!ADCIntStatus(ADC0_BASE, 0, false)){}	// Wait for Conversion
			ADCSequenceDataGet(ADC0_BASE, 0, raw_ADC);	// Update ADC buffer

			// Process --> Digital
			UARTprintf("ADC_BIN: ");
			for(i=0;i<CHANNEL_NUMBER;i++){
				//	UARTprintf("%04d ",raw_ADC[i]);
				if(raw_ADC[i] < (calib_white[i]+DELTA)){
					sensor_state |= (1<<i);
					UARTprintf("1");
				}
				//	else if(raw_ADC[i] > (calib_black[i]-DELTA)){
				else{
					sensor_state &= ~(1<<i);
					UARTprintf("0");
				}
			}
			//UARTprintf("	; sensor_state %x",sensor_state);
			UARTprintf("\r\n");
			SysCtlDelay(1000);
			//		UARTprintf("ADC_MV:  ");						// Print mVolt values
			//		for(i=0;i<CHANNEL_NUMBER;i++){
			//			UARTprintf("%04d ",(uint16_t)(raw_ADC[i]*ADC_TO_MV));
			//		}
			//		UARTprintf("\r\n");

			break;

		case LINE_FOLLOW:
			//================================================================//
			// Read ADC
			ADCIntClear(ADC0_BASE, 0);					// Reset ADC Interrupt Flag
			ADCProcessorTrigger(ADC0_BASE, 0);			// Start new ADC cycle
			while(!ADCIntStatus(ADC0_BASE, 0, false)){}	// Wait for Conversion
			ADCSequenceDataGet(ADC0_BASE, 0, raw_ADC);	// Update ADC buffer

			// Process --> Digital
			//UARTprintf("ADC_BIN: ");
			for(i=0;i<CHANNEL_NUMBER;i++){
				//	UARTprintf("%04d ",raw_ADC[i]);
				if(raw_ADC[i] < (calib_white[i]+DELTA)){
					sensor_state |= (1<<i);
					//UARTprintf("1");
				}
				//	else if(raw_ADC[i] > (calib_black[i]-DELTA)){
				else{
					sensor_state &= ~(1<<i);
					//UARTprintf("0");
				}
			}
			//UARTprintf("	; sensor_state %x",sensor_state);
			//UARTprintf("\r\n");

			//================================================================//
			// Motor Control
			if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4)==0x00)
			{
				ui16Adjust-=20;
				if (ui16Adjust < 50)
				{
					ui16Adjust = 50;
				}
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui16Adjust * ui32Load / 1000);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui16Adjust * ui32Load / 1000);
			}

			if(GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0)==0x00)
			{
				ui16Adjust+=20;
				if (ui16Adjust > 950)
				{
					ui16Adjust = 950;
				}
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui16Adjust * ui32Load / 1000);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ui16Adjust * ui32Load / 1000);
			}

			//================================================================//
			// QEI
			qei_getVelocity(0,&Velocity);
			qeiPositionA = QEIPositionGet(QEI0_BASE);
			UARTprintf("Motor 1: %5d, %6d \n", Velocity,qeiPositionA);

			qei_getVelocity(1,&Velocity);
			qeiPositionB = QEIPositionGet(QEI1_BASE);
			UARTprintf("Motor 2: %5d, %6d \n", Velocity,qeiPositionB);
			SysCtlDelay(5000000);
			break;
		}
	}
}

void hbridge_enable(bool en){
	if(en){
		GPIOPinWrite(RS_PORT_BASE, RSAB_PIN, RSAB_PIN);
		GPIOPinWrite(RS_PORT_BASE, RSCD_PIN, RSCD_PIN);
	}
	else{
		GPIOPinWrite(RS_PORT_BASE, RSAB_PIN, 0);
		GPIOPinWrite(RS_PORT_BASE, RSCD_PIN, 0);
	}
}

static void QEI0_VelocityIsr(void)
{
	QEIIntClear(QEI0_BASE, QEIIntStatus(QEI0_BASE, true));
	qei_velocity[0] = QEIVelocityGet(QEI0_BASE) * QEIDirectionGet(QEI0_BASE);
	qei_velocity_timeout[0] = true;
}

static void QEI1_VelocityIsr(void)
{
	QEIIntClear(QEI1_BASE, QEIIntStatus(QEI1_BASE, true));
	qei_velocity[1] = QEIVelocityGet(QEI1_BASE) * QEIDirectionGet(QEI1_BASE);
	qei_velocity_timeout[1] = true;
}

bool qei_getVelocity(bool Select, int32_t *Velocity)
{
	if (!Select)
	{
		if (qei_velocity_timeout[0])
		{
			*Velocity = qei_velocity[0];
			qei_velocity_timeout[0] = false;
			return true;
		}
		else
			return false;
	}
	else
		if (qei_velocity_timeout[1])
		{
			*Velocity = qei_velocity[1];
			qei_velocity_timeout[1] = false;
			return true;
		}
		else
			return false;
}

