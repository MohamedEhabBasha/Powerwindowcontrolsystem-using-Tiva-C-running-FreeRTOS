/*
Hardware Connections:
//Inputs 
PA2 - Manual Driver  Up
PA3 - Manual Driver  Down 
PA6 - Manual Passenger Up 
PA7 - Manual Passenger Down

PB2 - Auto Driver Up
PB3 - Auto Driver Down
PB6 - Auto Passenger Up
PB7 - Auto Passenger Down

PA4 - Jamming
PA5 - Lock

PC4 - Window down Limit switch 
PC5 - Window up Limit switch

//Outputs
PE1 - MOTOR A1
PE2 - MOTOR A2
*/

//Includes
#define PART_TM4C123GH6PM		
#include "TM4C123GH6PM.h"
#include <FreeRTOS.h>
#include "task.h"
#include <semphr.h>
#include <driverlib/gpio.c>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"


//Function declarations
void init(); // Initializations
void GPIOA_Handler(); // GPIOA ISR
void GPIOB_Handler(); // GPIOB ISR

		/*Delay function in milliseconds*/
void delayMs(uint32_t n)
{
  int i,j;
  for(i=0;i<n;i++)
  {
    for(j=0;j<3180;j++);
  }
}

				/*Task declarations */

void manualDriverUp(void *params);
void manualDriverDown(void *params);

void manualPassengerUp (void *params);
void manualPassengerDown (void *params);

void autoDriverUp (void *params);
void autoDriverDown (void *params);

void autoPassengerUp (void *params);
void autoPassengerDown (void *params);

void jamming (void* params);
void lock (void* params);


					/*Semaphores declarations*/

SemaphoreHandle_t S_manualDriverUp;
SemaphoreHandle_t S_manualDriverDown;
SemaphoreHandle_t S_autoDriverUp;
SemaphoreHandle_t S_autoDriverDown;
SemaphoreHandle_t S_manualPassengerUp;
SemaphoreHandle_t S_manualPassengerDown;
SemaphoreHandle_t S_autoPassengerUp;
SemaphoreHandle_t S_autoPassengerDown;
SemaphoreHandle_t S_jamming;
SemaphoreHandle_t S_lock;
							
							/*Mutex Declarations*/
							
SemaphoreHandle_t MotorMutex;
							
							/*Queue Declarations*/
							
xQueueHandle xAutoUpQueue;

int main()
{
	//Initializations
	
	init();
	
	//Create the Binary semaphores
	
	S_manualDriverUp = xSemaphoreCreateBinary();
	S_manualDriverDown = xSemaphoreCreateBinary();
	S_autoDriverUp = xSemaphoreCreateBinary();
	S_autoDriverDown = xSemaphoreCreateBinary();
	S_manualPassengerUp = xSemaphoreCreateBinary();
	S_manualPassengerDown = xSemaphoreCreateBinary();
	S_autoPassengerUp = xSemaphoreCreateBinary();
	S_autoPassengerDown = xSemaphoreCreateBinary();
	S_jamming = xSemaphoreCreateBinary();
	S_lock = xSemaphoreCreateBinary();

	//Create Mutex semaphore
	
	MotorMutex = xSemaphoreCreateMutex();
	
	//Create the Queue
						
	xAutoUpQueue = xQueueCreate( 1, sizeof(int));
	
	//Create the tasks

	xTaskCreate(manualDriverUp,"manualDriverUp",80,NULL,2,NULL);
	xTaskCreate(manualDriverDown,"manualDriverDown",80,NULL,2,NULL);
	xTaskCreate(autoDriverUp,"autoDriverUp",80,NULL,2,NULL);
	xTaskCreate(autoDriverDown,"autoDriverDown",80,NULL,2,NULL);
	xTaskCreate(manualPassengerUp,"manualPassengerUp",80,NULL,1,NULL);
	xTaskCreate(manualPassengerDown,"manualPassengerDown",80,NULL,1,NULL);
	xTaskCreate(autoPassengerUp,"autoPassengerUp",80,NULL,1,NULL);
	xTaskCreate(autoPassengerDown,"autoPassengerDown",80,NULL,1,NULL);
	xTaskCreate(jamming,"jamming",80,NULL,2,NULL);
	xTaskCreate(lock,"lock_passenger",80,NULL,2,NULL);
	

	//Start the scheduler
	vTaskStartScheduler();
	// This loop should never be reached	
	for(;;);
}

									/*Functions*/
// Initializations of GPIOs
void init() 
{
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
			;
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);	
		while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
			;
		// Configure pins of PORTA
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4|GPIO_PIN_5);
		GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4  , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
		//All buttons, limit switches, and lock switch are set to work on falling edge detection
		GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4| GPIO_PIN_5, GPIO_FALLING_EDGE);
		GPIOIntRegister(GPIO_PORTA_BASE, GPIOA_Handler);
		IntPrioritySet(INT_GPIOA, 0XE0);
		
		// Configure pins of PORTB
		GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3);
		GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3);
		GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7| GPIO_PIN_2 | GPIO_PIN_3, GPIO_FALLING_EDGE);
		GPIOIntRegister(GPIO_PORTB_BASE, GPIOB_Handler);
		IntPrioritySet(INT_GPIOB, 0XE0);
		
		//Configure pins of PORTC
		GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
		GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 , GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
		
		//Configure pins of PORTE
		GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2); 		// PORTE output pins for the motor
		GPIOUnlockPin(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2);
			
}

				/*Interrupt Service Routines*/
//PORT A ISR
void GPIOA_Handler() 
{
	//Checks which button has been pressed
	//Highest priority flag is initialized to false , the flag is used for context switching if a higher or equal priority task is preempted from the ISR
	BaseType_t xHigherPriorityTaskWoken= pdFALSE;    
	/**Manual Driver Up**/
	if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_2) != GPIO_PIN_2)     
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_2); // Clear Interrupt
		xSemaphoreGiveFromISR(S_manualDriverUp,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}
	/**Manual Driver Down**/
	else if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_3) != GPIO_PIN_3) 
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3); // Clear Interrupt
		xSemaphoreGiveFromISR(S_manualDriverDown,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}
	
	/**Manual Passenger Up**/
	else if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_6) != GPIO_PIN_6)
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_6);
		xSemaphoreGiveFromISR(S_manualPassengerUp,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}

	/**Manual Passenger Down**/
	else if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_7) != GPIO_PIN_7) 
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_7);
		xSemaphoreGiveFromISR(S_manualPassengerDown,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	
	/** Lock **/
	 // check if the lock switch is pressed
	else if ((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5))!= GPIO_PIN_5)
	{
		delayMs(10); //Debouncing
		if ((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5))!= GPIO_PIN_5){	
			GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_5);
			xSemaphoreGiveFromISR(S_lock, &xHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}
	}

	/**Jamming**/
	
	else if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_4) != GPIO_PIN_4)
	{
 		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_4);
 		xSemaphoreGiveFromISR(S_jamming,&xHigherPriorityTaskWoken);
 		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	
	else 
	{
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_2);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_6);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_7);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_5);
		GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_4);	
	}
}
//ISR of PORT B
void GPIOB_Handler()
{
	 //Checks which button has been pressed
	//Highest priority flag is initialized to false , the flag is used for context switching if a higher or equal priority task is preempted from the ISR
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	/**Automatic Driver Up**/
	if (GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_2) != GPIO_PIN_2)     
	{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2); // Clear Interrupt
		xSemaphoreGiveFromISR(S_autoDriverUp,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1	
	}

	/**Automatic Driver Down**/
	else if (GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_3) != GPIO_PIN_3) 
	{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_3); // Clear Interrupt
		xSemaphoreGiveFromISR(S_autoDriverDown,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken); //context switching if flag == 1
	}
	
	/**Automatic Passenger Up**/
	else if (GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_6) != GPIO_PIN_6)
	{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_6);
		xSemaphoreGiveFromISR(S_autoPassengerUp,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}

	/**Automatic Passenger Down**/
	else if (GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_7) != GPIO_PIN_7) 
	{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_7);
		xSemaphoreGiveFromISR(S_autoPassengerDown,&xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	else 
		{
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2);
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_3);
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_6);
		GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_7);
	  }
}
	
					/*Tasks*/

void manualDriverUp(void *params)
{
	// An initial check to ensure that the semaphore is empty (for safety)
	xSemaphoreTake(S_manualDriverUp,0);
	for(;;)
	{
		xSemaphoreTake (S_manualDriverUp, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		//As long as the corresponding button is pressed, and the corresponding limit switch is not reached, the code will be stuck in here , and the motor will stay on
		while(GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_2) != GPIO_PIN_2 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5) ==  GPIO_PIN_5);

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);  //Turn the motor off
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		xSemaphoreGive(MotorMutex);
	} 
}

void manualDriverDown(void *params)
{
	// An initial check to ensure that the semaphore is empty (for safety)
	xSemaphoreTake(S_manualDriverDown,0);
	for(;;)
	{
		xSemaphoreTake (S_manualDriverDown, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
		//As long as the corresponding button is pressed, and the corresponding limit switch is not reached, the code will be stuck in here , and the motor will stay on
		while(GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_3) != GPIO_PIN_3 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) ==  GPIO_PIN_4); 

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0); //Turn the motor off
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0);
		xSemaphoreGive(MotorMutex);
	}
}

void manualPassengerUp (void *params)
{
	// An initial check to ensure that the semaphore is empty (for safety)
	xSemaphoreTake(S_manualPassengerUp,0);
	for(;;)
	{
		xSemaphoreTake (S_manualPassengerUp, portMAX_DELAY); 
		//xSemaphoreTake(MotorMutex, portMAX_DELAY); //Mutex is not given to the passenger's manual task so that the priority resides with the driver
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		//As long as the corresponding button is pressed, and the corresponding limit switch is not reached, the code will be stuck in here , and the motor will stay on
		while(GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_6) != GPIO_PIN_6 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5) ==  GPIO_PIN_5); 

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor off
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
	}
}

void manualPassengerDown (void *params)
{
	// An initial check to ensure that the semaphore is empty (for safety)
	xSemaphoreTake(S_manualPassengerDown,0);
	for(;;)
	{
		xSemaphoreTake (S_manualPassengerDown, portMAX_DELAY);
		//xSemaphoreTake(MotorMutex, portMAX_DELAY);//Mutex is not given to the passenger's manual task so that the priority resides with the driver
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
		//As long as the corresponding button is pressed, and the corresponding limit switch is not reached, the code will be stuck in here , and the motor will stay on
		while(GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_7) != GPIO_PIN_7 && GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) ==  GPIO_PIN_4);

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);// Turn the motor off
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);		
	}
}

void autoDriverUp (void *params)
{
	xSemaphoreTake(S_autoDriverUp,0);
	for(;;)
	{
		xSemaphoreTake (S_autoDriverUp, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);	
		while(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5) ==  GPIO_PIN_5) //Keep the motor on as long as the corresponding limit switch is not reached
		{
			int auto_pressed = 0;
			if(xQueueReceive(xAutoUpQueue, &auto_pressed,0) == pdTRUE)	//check if jamming occured
			{
				//check if the window is currently in automatic up mode
				if(auto_pressed == 1){
				// Stop the motor			
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
				delayMs(300);
				// Bring the window down for a little bit	
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); 
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
				delayMs(500);
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);
				GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
				break;	// Break from the nested if condition to check if the jamming condition is cleared
				}
			 }
		}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor off
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		xSemaphoreGive(MotorMutex);	
	}
}
	
void autoDriverDown (void *params)
{
	xSemaphoreTake(S_autoDriverDown,0);
	for(;;)
	{
		xSemaphoreTake (S_autoDriverDown, portMAX_DELAY);
		xSemaphoreTake(MotorMutex, portMAX_DELAY);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor on
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);reached
		//Keep the motor on as long as the corresponding limit switch is not 
		while(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) ==  GPIO_PIN_4);
			
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn the motor off
	    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
   	    xSemaphoreGive(MotorMutex);
	}
}
void autoPassengerUp (void *params)
{
	xSemaphoreTake(S_autoPassengerUp,0);
	for(;;)
	{
		xSemaphoreTake (S_autoPassengerUp, portMAX_DELAY);

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,GPIO_PIN_1); //Turn on the motor
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
		while(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_5) ==  GPIO_PIN_5) //Keep the motor on as long as the corresponding limit switch is not reached
		{
			int auto_pressed = 0;
			if(xQueueReceive(xAutoUpQueue, &auto_pressed,0) == pdTRUE)//Check if jamming occured
			{
				//check if the window is currently in automatic up mode
				if(auto_pressed == 1){
					// Stop the motor			
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
					delayMs(300);
					// Bring the window down for a little bit	
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); 
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
					delayMs(500);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00);
					GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
					break;	
				}
			}
		}
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); // Turn off the motor
	    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
	}
}
void autoPassengerDown (void *params)
{
	xSemaphoreTake(S_autoPassengerDown,0);
	for(;;)
	{
		xSemaphoreTake (S_autoPassengerDown, portMAX_DELAY);

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn on the motor
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,GPIO_PIN_2);
		//Keep the motor on as long as the corresponding limit switch is not reached
		while(GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_4) ==  GPIO_PIN_4);

		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1,0x00); //Turn off the motor
	    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2,0x00);
	}
}
void lock(void *params)
{
	xSemaphoreTake(S_lock, 0);
	for(;;)
	{
		xSemaphoreTake(S_lock, portMAX_DELAY);	
		while(!(GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5))) //Check if lock switch is pressed
		{
			//Disable manual control by the passenger
			GPIOIntDisable(GPIO_PORTA_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7 ); 
			//Disable automatic control by the passenger
			GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7 ); 
		}
		//Enable manual control by the passenger
		GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7 );
		//Enable automatic control by the passenger
		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_6 | GPIO_INT_PIN_7 ); 
	}
}

void jamming (void* params)
{
		 
	xSemaphoreTake(S_jamming,0); 
	for(;;)
	{
		xSemaphoreTake (S_jamming, portMAX_DELAY);
		const TickType_t xDelay = 10 / portTICK_RATE_MS; //standard delay to be used with queues
		int flag = 1;
		xQueueSend(xAutoUpQueue,&flag,xDelay);	//Queue that let's other tasks know that jamming was detected
	}
}
	
