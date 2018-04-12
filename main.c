/*****************************************************************************
 *
 *   EE2024 Assignment 2 Program
 *
 *   Tuesday Lab Group
 *   CHUA ZHENYU ANTON (A0161811N)
 *   KWEK JUN KAI SEAN (A0155565X)
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "led7seg.h"
#include "joystick.h"
#include "pca9532.h"
#include "light.h"
#include "acc.h"
#include "oled.h"
#include "temp.h"
#include "rgb.h"

#include <stdio.h>
#include <string.h>

volatile uint32_t msTicks;
volatile uint32_t secTicks;
volatile uint32_t redTicks; // 333ms
volatile uint32_t blueTicks; // 333ms

#define UART_PORT   (LPC_UART_TypeDef *)LPC_UART3   // Select UART3
uint8_t rev_buf[255];                               // Reception buffer
uint32_t rev_cnt = 0;                               // Reception counter
uint32_t isReceived = 0;                            // Init to be not received

typedef enum {
    Stationary,
    Launch,
    Return,
} MachineMode;

static MachineMode mode;

static int8_t countdown;
static int8_t countdownMode;
static int8_t clearOLED;
static int8_t buttonPresses = 0;

static uint32_t buttonPressSecTicks;

static uint8_t ACC_WARNING = 0;
static uint8_t TEMP_WARNING = 0;
static uint8_t OBST_WARNING = 0;
static uint8_t redLED = 0;
static uint8_t blueLED = 0;

void clearWarnings() {
	ACC_WARNING = 0;
	TEMP_WARNING = 0;
	OBST_WARNING = 0;
}

// engage Stationary mode
void stationaryMode() {
	mode = Stationary;
	OBST_WARNING = 0;
	clearOLED = 1;
}

// engage Launch mode
void launchMode() {
	mode = Launch;
	countdownMode = 0;
	countdown = 16;
	oled_clearScreen(OLED_COLOR_BLACK);
}

// engage Return mode
void returnMode() {
	mode = Return;
	clearWarnings();
	clearOLED = 1;
}

void stationaryOLEDDisplay(int32_t temp) {
	char modeString[20];
	char displayTemp[20];
	char tempWarning[20];
	char clear[20];

	sprintf(modeString, "STATIONARY");
	sprintf(displayTemp, "TEMP: %0.1fC", temp/10.0);
	sprintf(clear, "                  ");

	oled_putString(0, 0, (uint8_t *) modeString, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(0, 20, (uint8_t *) displayTemp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	if (TEMP_WARNING == 1) {
		sprintf(tempWarning, "Temp. too high");
		oled_putString(0, 40, (uint8_t *) tempWarning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	} else {
		oled_putString(0, 40, (uint8_t *) clear, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
}

void launchOLEDDisplay(int32_t temp, int8_t x, int8_t y) {
	char modeString[20];
	char displayTemp[20];
	char tempWarning[20];
	char clear[20];
	char displayAccel[20];
	char accWarning[20];

	sprintf(displayTemp, "TEMP: %0.1fC", temp/10.0);
	sprintf(clear, "                  ");

	oled_putString(0, 0, (uint8_t *) modeString, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(0, 20, (uint8_t *) displayTemp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	if (TEMP_WARNING == 1) {
		sprintf(tempWarning, "Temp. too high");
		oled_putString(0, 40, (uint8_t *) tempWarning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	} else {
		oled_putString(0, 40, (uint8_t *) clear, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}

	sprintf(modeString, "LAUNCH");
	sprintf(displayTemp, "Temp: %0.1fC", temp/10.0);
	sprintf(displayAccel, "X:%0.2f_g,Y:%0.2f_g", x/64.0, y/64.0);
	sprintf(tempWarning, "Temp. too high");
	sprintf(accWarning, "Veer off course");
	sprintf(clearMessage, "                  ");

	oled_putString(0, 0, (uint8_t *) modeString, OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(0, 10, (uint8_t *) displayTemp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(0, 20, (uint8_t *) displayAccel, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

	if (TEMP_WARNING == 1 && ACC_WARNING == 1) {
		oled_putString(0, 30, (uint8_t *) tempWarning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0, 40, (uint8_t *) accWarning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	} else if (TEMP_WARNING == 1) {
		oled_putString(0, 30, (uint8_t *) tempWarning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0, 40, (uint8_t *) clear, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	} else if (ACC_WARNING == 1) {
		oled_putString(0, 30, (uint8_t *) accWarning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0, 40, (uint8_t *) clear, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	} else {
		oled_putString(0, 30, (uint8_t *) clear, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
		oled_putString(0, 40, (uint8_t *) clear, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void returnOLEDDisplay() {
	char modeString[20];
	char obstWarning[20];
	char clear[20];

	sprintf(modeString, "RETURN");
	oled_putString(0, 0, (uint8_t *) modeString, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	if (OBST_WARNING == 1) {
		sprintf(obstWarning, "Obstacle near");
		oled_putString(0, 20, (uint8_t *) obstWarning, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	} else {
		sprintf(clear, "             ");
		oled_putString(0, 20, (uint8_t *) clear, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	}
}

void startCountdown() {
	countdownMode = 1;
	countdown = 15;
}

void abortCountdown() {
	countdownMode = 0;
	countdown = 16;
}

void EINT3_IRQHandler(void) {
	if ((LPC_GPIOINT ->IO2IntStatF >> 10) & 0x1) {
		printf("Hello P2.10\n");
		if (buttonPresses == 0) {
			buttonPressSecTicks = msTicks;
		}
		buttonPresses++;
		LPC_GPIOINT ->IO2IntClr = 1 << 10;
	}
}

void LEDlightSteps(uint32_t lux) {
	uint16_t ledInt;
	if (lux <= 228) {
		ledInt = 0b0;
	} else if (lux <= 457) {
		ledInt = 0b1;
	} else if (lux <= 684) {
		ledInt = 0b11;
	} else if (lux <= 912) {
		ledInt = 0b111;
	} else if (lux <= 1140) {
		ledInt = 0b1111;
	} else if (lux <= 1368) {
		ledInt = 0b11111;
	} else if (lux <= 1596) {
		ledInt = 0b111111;
	} else if (lux <= 1824) {
		ledInt = 0b1111111;
	} else if (lux <= 2052) {
		ledInt = 0b11111111;
	} else if (lux <= 2280) {
		ledInt = 0b111111111;
	} else if (lux <= 2508) {
		ledInt = 0b1111111111;
	} else if (lux <= 2736) {
		ledInt = 0b11111111111;
	} else if (lux <= 2964) {
		ledInt = 0b111111111111;
	} else if (lux <= 3192) {
		ledInt = 0b1111111111111;
	} else if (lux <= 3420) {
		ledInt = 0b11111111111111;
	} else if (lux <= 3648) {
		ledInt = 0b111111111111111;
	} else {
		ledInt = 0xffff;
	}
	pca9532_setLeds(ledInt, 0xffff);
}

void disableLED() {
    GPIO_ClearValue( 2, (1<<0) );
    GPIO_ClearValue( 0, (1<<26) );
}

void enableRedLED() {
    GPIO_SetValue( 2, (1<<0) );
    GPIO_ClearValue( 0, (1<<26) );
}

void enableBlueLED() {
    GPIO_ClearValue( 2, (1<<0) );
    GPIO_SetValue( 0, (1<<26) );
}

void setLED() {
	if(TEMP_WARNING == 1 && ACC_WARNING == 1) {
		if(msTicks - redTicks > 333) {
			redTicks = msTicks;
			if(redLED == 1) {
				enableBlueLED();
				redLED = 0;
			} else {
				enableRedLED();
				redLED = 1;
			}
		}
	} else if (TEMP_WARNING == 1) { //blink red
		if(msTicks - redTicks > 333) {
			redTicks = msTicks;
			if(redLED == 1) {
				disableLED();
				redLED = 0;
			} else {
				enableRedLED();
				redLED = 1;
			}
		}
	} else if (ACC_WARNING == 1) { //acc warning == 1
		if(msTicks - blueTicks > 333) {
			blueTicks = msTicks;
			if(blueLED == 1) {
				disableLED();
				blueLED = 0;
			} else {
				enableBlueLED();
				blueLED = 1;
			}
		}
	} else {
		redLED = 0;
		blueLED = 0;
		disableLED();
	}
}

void display_7seg(MachineMode mode) {
	switch(mode) {
	case Stationary:
		led7seg_setChar('F', FALSE);
		break;
	default:
		led7seg_setChar('0', FALSE);
	}
}

void countdown_7seg(int8_t countdown) {
	switch (countdown) {
	case 16: case 15:
		led7seg_setChar('F', FALSE);
		break;
	case 14:
		led7seg_setChar('E', FALSE);
		break;
	case 13:
		led7seg_setChar('D', FALSE);
		break;
	case 12:
		led7seg_setChar('C', FALSE);
		break;
	case 11:
		led7seg_setChar('B', FALSE);
		break;
	case 10:
		led7seg_setChar('A', FALSE);
		break;
	case 9:
		led7seg_setChar('9', FALSE);
		break;
	case 8:
		led7seg_setChar('8', FALSE);
		break;
	case 7:
		led7seg_setChar('7', FALSE);
		break;
	case 6:
		led7seg_setChar('6', FALSE);
		break;
	case 5:
		led7seg_setChar('5', FALSE);
		break;
	case 4:
		led7seg_setChar('4', FALSE);
		break;
	case 3:
		led7seg_setChar('3', FALSE);
		break;
	case 2:
		led7seg_setChar('2', FALSE);
		break;
	case 1:
		led7seg_setChar('1', FALSE);
		break;
	default:
		led7seg_setChar('0', FALSE);
	}
}

void SysTick_Handler(void) {
	msTicks++;
	if (buttonPresses > 0 && msTicks - buttonPressSecTicks > 1000) {
		if (buttonPresses == 2 && mode == Launch) {
			returnMode();
		} else if (buttonPresses == 1 && mode == Stationary) {
			startCountdown();
		} else if (buttonPresses == 1 && mode == Return) {
			stationaryMode();
		}
		buttonPresses = 0;
	}
	if(TEMP_WARNING == 1 || ACC_WARNING == 1) {
		setLED();
	}
}

uint32_t getTicks() {
	return msTicks;
}

static void init_ssp(void) {
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_GPIO(void) {
	// Initialize button
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1 << 31, 0);

}

void init() {
	init_i2c();
	init_ssp();
	init_GPIO();

	pca9532_init();
	acc_init();
	joystick_init();
	temp_init(&getTicks);
	rgb_init();
	led7seg_init();
	oled_init();

	light_enable();
	light_setRange(LIGHT_RANGE_4000);

	LPC_GPIOINT ->IO2IntEnF |= 1 << 10;
	NVIC_SetPriorityGrouping(5);
	NVIC_SetPriority(EINT3_IRQn, 0x18);
	NVIC_EnableIRQ(EINT3_IRQn);
}

int main(void) {

	mode = Stationary;
	countdownMode = 0;
	countdown = 16;

	int32_t TEMP_HIGH_THRESHOLD = 28; // deg C
	int32_t OBSTACLE_NEAR_THRESHOLD = 3000; // 3000 lux
	float ACC_THRESHOLD = 0.4; // 0.4G

	int8_t xoff = 0;
	int8_t yoff = 0;

	int8_t x = 0;
	int8_t y = 0;
	int8_t z = 0;


	uint8_t btn1 = 1;

	int32_t lightVal;
	int32_t temp;

	initAll();


	// Setup Systick Timer to interrupt at 1 msec intervals

	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1)
			;  // Capture error
	}

	/*
	 * Assume base board in zero-g position when reading first value.
	 */
	acc_read(&x, &y, &z);
	xoff = 0 - x;
	yoff = 0 - y;

	/* ---- Speaker ------> */

	GPIO_SetDir(2, 1 << 0, 1);
	GPIO_SetDir(2, 1 << 1, 1);

	GPIO_SetDir(0, 1 << 27, 1);
	GPIO_SetDir(0, 1 << 28, 1);
	GPIO_SetDir(2, 1 << 13, 1);
	GPIO_SetDir(0, 1 << 26, 1);

	GPIO_ClearValue(0, 1 << 27); //LM4811-clk
	GPIO_ClearValue(0, 1 << 28); //LM4811-up/dn
	GPIO_ClearValue(2, 1 << 13); //LM4811-shutdn

	/* <---- Speaker ------ */

	oled_clearScreen(OLED_COLOR_BLACK);
	pca9532_setLeds(0b0, 0xffff);

	while (1) {

		/* ####### Accelerometer and LEDs  ###### */
		/* # */




		/* # */
		/* ############################################# */

		/* ####### Joystick and OLED  ###### */
		/* # */

//		state = joystick_read();
//		if (state != 0)
//			drawOled(state);

		/* # */
		/* ############################################# */

		/* ############ Trimpot and RGB LED  ########### */
		/* # */

		btn1 = (GPIO_ReadValue(1) >> 31) & 0x01;

		if (btn1 == 0)
		{
			clearWarnings();
		}

		Timer0_Wait(1);

		switch(mode) {
		case Stationary:
			if(clearOLED == 1) {
				oled_clearScreen(OLED_COLOR_BLACK);
				clearOLED = 0;
			}
			temp = temp_read();
			if (temp > TEMP_HIGH_THRESHOLD * 10) {
				TEMP_WARNING = 1;
			}

			display_Stationary_Mode_Oled(temp);
			if(TEMP_WARNING == 0) {
				disableLED();
			}
			countdown_7seg(countdown);

			if(countdownMode == 1) {
				if(TEMP_WARNING == 1) {
					abortCountdown();
				}
				if(msTicks - secTicks > 1000) {
					secTicks = msTicks;
					countdownTimer--;
				}
				if(countdown < 0) {
					launchMode();
				}
			}
			break;

		case Launch:
			temp = temp_read();
			acc_read(&x, &y, &z);
			x = x + xoff;
			y = y + yoff;
			if (temp > TEMP_HIGH_THRESHOLD * 10) {
				TEMP_WARNING = 1;
			}

			if (x > ACC_THRESHOLD * 64 || y > ACC_THRESHOLD * 64) {
				ACC_WARNING = 1;
			}

			launchOLEDDisplay(temp, x, y);
			if(TEMP_WARNING == 0 && ACC_WARNING == 0) {
				disableLED();
			}
			display_7seg(mode);

			break;

		case RETURN_MODE:
			if(clearOLED == 1) {
				oled_clearScreen(OLED_COLOR_BLACK);
				clearOLED = 0;
			}
			lightVal = light_read();

			LEDlightSteps(lightVal);

			if (lightVal < OBSTACLE_NEAR_THRESHOLD) {
				OBST_WARNING = 1;
			} else if (OBST_WARNING == 1) {
				OBST_WARNING = 0;
			}
			returnOLEDDisplay();
			display_7seg(mode);
			break;
		default:
			mode = STATIONARY_MODE; //attempt a reset
		}

		//the modules below are to be put in the if branches, for reference only, remove after use
/*
		temp = temp_read();
		// printf("The temp is %f\n", temp/10.0);
		if (temp > TEMP_HIGH_THRESHOLD * 10) {
			display_temp_high_OLED(temp);

		}

		lightVal = light_read();
		//printf("The light intensity is %d\n", lightVal);
		LEDlightSteps(lightVal);

		if (lightVal > OBSTACLE_NEAR_THRESHOLD) {
			//should go to OBST_WARNING state
			display_obstacle_near_OLED();
			OBST_WARNING = 1;
		} else if (OBST_WARNING == 1) {
			OBST_WARNING = 0;
			//oled_clearScreen(OLED_COLOR_BLACK);
		}

        float xValue, yValue;
        xValue = (float) x;
        yValue = (float) y;
        printf("X = %f, Y = %f\n", xValue, yValue);
        if (xValue > ACC_THRESHOLD && yValue > ACC_THRESHOLD) {
        	printf("both x and y are too high\n");
        } else if (xValue > ACC_THRESHOLD) {
        	printf("x is too high\n");
        } else if (yValue > ACC_THRESHOLD) {
        	printf("y is too high\n");
        } else {
        	printf("acc is ok\n");
        }*/

	}
}

void check_failed(uint8_t *file, uint32_t line) {
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
		;
}
