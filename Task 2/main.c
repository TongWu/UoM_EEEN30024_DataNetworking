/*
This program allows the user to read the tilt register value from the accelerometer sensor,
write it to the EEPROM, and read it back again.

The following joystick presses perform the listed functions, after performing a function it displays
a success message on the LCD:
Centre: Read tilt register value from the sensor
Right: Write tilt register value to the EEPROM
Left: Read tilt register value from the EEPROM
Down: Display the first of two copies read back from the EEPROM
Up: Display the second of two copies read back from the EEPROM

There should be two versions of functions that write to and read from the EEPROM:
one version for single-byte mode writing&reading, and another forpage-mode writing&reading.
*/

#include "main.h"

#include "Time_Delays.h"
#include "Clk_Config.h"
#include "LCD_Display.h"

#include <stdio.h>
#include <string.h>

#include "Small_7.h"
#include "Arial_9.h"
#include "Arial_12.h"
#include "Arial_24.h"

//Accelerometer Sensor I2C Address
#define ACCELADR 0x98

//EEPROM I2C Address
#define EEPROMADR 0xA0

//Function prototypes defined as needed:
uint32_t joystick_centre (void);        //JoyStick centre
uint32_t joystick_up (void);            //JoyStick up
uint32_t joystick_down (void);          //JoyStick down
uint32_t joystick_left (void);          //JoyStick left
uint32_t joystick_right (void);         //JoyStick right
void configure_GPIO(void);              //GPIO
void configure_JoyStick(void);          //JoyStick configuration
void configure_I2C(void);               //I2C
void configure_Accelerometer(void);     //Accelerometer
char read_tilt(void);                   //Read from tile register(sensor)

//Global variables -if needed- defined below:
char outputString[18]; //Buffer to store text in for LCD																				 //a separate variable.

int main(void){
	//Init
        SystemClock_Config();/* Configure the system clock to 84.0 MHz */
	SysTick_Config_MCE2(us);

	//Configure LCD
	Configure_LCD_Pins();
	Configure_SPI1();
	Activate_SPI1();
	Clear_Screen();
	Initialise_LCD_Controller();
	set_font((unsigned char*) Arial_12);

	//Configure GPIO and Joystick
	configure_GPIO();
  configure_JoyStick();

	//Configure I2C
	configure_I2C();

	//Configure Accelerometer
	configure_Accelerometer();

	//Main Loop
  while (1){
		if(joystick_centre()){			
			//Read and display tilt register value
			
			
		} else if(joystick_right()){
			//Write tilt register value to EEPROM and report if successful
			

		} else if(joystick_left()){
			//Read tilt register value from EEPROM and report if successful
			

		} else if(joystick_up()){
			//Display first copy read back from EEPROM
			

		} else if(joystick_down()){
			//Display second copy read back from EEPROM
			
		}
  }
}

//Define other functions as needed
uint32_t joystick_centre (void) {
        // returns 1 if the joystick is pressed in the centre , or 0 otherwise
        return (LL_GPIO_IsInputPinSet (GPIOB , LL_GPIO_PIN_5));
}

uint32_t joystick_up (void) {
        // returns 1 if the joystick is pressed in the centre , or 0 otherwise
        return (LL_GPIO_IsInputPinSet (GPIOA , LL_GPIO_PIN_4));
}

uint32_t joystick_down (void) {
        // returns 1 if the joystick is pressed in the centre , or 0 otherwise
        return (LL_GPIO_IsInputPinSet (GPIOB , LL_GPIO_PIN_0));
}

uint32_t joystick_left (void) {
        // returns 1 if the joystick is pressed in the centre , or 0 otherwise
        return (LL_GPIO_IsInputPinSet (GPIOC , LL_GPIO_PIN_1));
}

uint32_t joystick_right (void) {
        // returns 1 if the joystick is pressed in the centre , or 0 otherwise
        return (LL_GPIO_IsInputPinSet (GPIOC , LL_GPIO_PIN_0));
}

void configure_GPIO(void){
    //GPIO
		LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_GPIOB);               //Enable peripheral clock
    //LL_GPIO_SetPinMode (GPIOA , LL_GPIO_PIN_5 , LL_GPIO_MODE_INPUT);    //Set A5 as input
    //LL_GPIO_SetPinPull (GPIOA , LL_GPIO_PIN_5 , LL_GPIO_PULL_NO);       //Set A5 as NO pull
}

void configure_JoyStick(void){
    //Joystick
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 	//set PB0 as Input
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); 	//set PB0 as NO pull
  LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 	//set PC0 as Input  
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); 	//set PC0 as NO pull
  LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 	//set PA0 as Input
	LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); 	//set PA0 as NO pull
}

void configure_I2C(void){
    //Configure SCL and SDA

    // Configure SCL as: Alternate function, High Speed, Open Drain, Pull Up
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);

    // Configure SDA as: Alternate, High Speed, Open Drain, Pull Up
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);

    //Enable I2C1 Clock
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    LL_I2C_Disable(I2C1);
    LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
    LL_I2C_ConfigSpeed(I2C1, 84000000, 100000, LL_I2C_DUTYCYCLE_2); //set speed to 100 kHz
    LL_I2C_Enable(I2C1);
}

void configure_Accelerometer(void){
    //ACCELADR is a macro that equates 0x98, the address of the sensor
    //set the accelerometer to active mode
    LL_I2C_GenerateStartCondition(I2C1); //START
    while (!LL_I2C_IsActiveFlag_SB(I2C1))
        ;

    LL_I2C_TransmitData8(I2C1, ACCELADR); // ADDRESS + WRITE
    while (!LL_I2C_IsActiveFlag_ADDR(I2C1))
        ;
    LL_I2C_ClearFlag_ADDR(I2C1);

    LL_I2C_TransmitData8(I2C1, 0x07); //Set pointer register to mode register
    while (!LL_I2C_IsActiveFlag_TXE(I2C1))
        ;

    LL_I2C_TransmitData8(I2C1, 0x01); //Set set to active mode
    while (!LL_I2C_IsActiveFlag_TXE(I2C1))
        ;

    LL_I2C_GenerateStopCondition(I2C1); //STOP
}

 char read_tilt(void){
 LL_I2C_GenerateStartCondition (I2C1); //START
	 uint8_t sample = 0;
            while (! LL_I2C_IsActiveFlag_SB (I2C1));

            LL_I2C_TransmitData8 (I2C1 , ACCELADR); // ADDRESS + WRITE
            while(! LL_I2C_IsActiveFlag_ADDR (I2C1));
            LL_I2C_ClearFlag_ADDR (I2C1);

            LL_I2C_TransmitData8 (I2C1 , 0x03); //Set pointer register to tilt register
            while (! LL_I2C_IsActiveFlag_TXE (I2C1));

            LL_I2C_GenerateStartCondition (I2C1); //RE-START
            while (! LL_I2C_IsActiveFlag_SB (I2C1));

            LL_I2C_TransmitData8 (I2C1 , ACCELADR +1); // ADDRESS + READ
            while (! LL_I2C_IsActiveFlag_ADDR (I2C1));
            LL_I2C_ClearFlag_ADDR (I2C1);

            LL_I2C_AcknowledgeNextData (I2C1 , LL_I2C_NACK); //ACK INCOMING DATA
            while (! LL_I2C_IsActiveFlag_RXNE (I2C1));
            sample = LL_I2C_ReceiveData8 (I2C1); //DATA BYTE

            LL_I2C_GenerateStopCondition (I2C1); //STOP
						return sample;
 }
