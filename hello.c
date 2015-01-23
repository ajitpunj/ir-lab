#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "inc/tm4c123gh6pm.h"
#include "driverlib/timer.h"
#include "driverlib/systick.h"

// global integers for edge 1 and edge 2
volatile int edge1, edge2, counter, pulse, arrayVals;
volatile int iTick;
volatile int arrayValues[32];

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
// Configure the UART and its pins.  This must be called before UARTprintf().
//***************************************************************************
void
ConfigureUART(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable UART0
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); // Use the internal 16MHz oscillator as the UART clock source.
    UARTStdioConfig(0, 115200, 16000000);  // Initialize the UART for console I/O.
}

//*****************************************************************************
// Interrupt handler on rising edge
//*****************************************************************************


void SysTick_Handler (void) {
 
 //clear interrupts
  GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
 //UARTprintf("hit clear edge1=%d counter=%d\n",edge1, counter);
	counter++;
	if(iTick==0){//first hit
		//reset timer, set edge1, set iTick=1, then edge2 gets set
		SysTickPeriodSet (16777000);
		NVIC_ST_CURRENT_R=0;//reset systick
		edge1=SysTickValueGet();//get reset value
		iTick=1;
		//UARTprintf("first edge\n");
	}
	else if(iTick==1){
		//get edge2 value and store it
		edge2=SysTickValueGet();
		pulse=edge1-edge2;
		arrayValues[arrayVals]=pulse;
		arrayVals++;
		//reset systick and get edge1, on next falling edge it will get edge2 and calculate from here to next falling edge
		SysTickPeriodSet (16777000);
		NVIC_ST_CURRENT_R=0;//reset systick
		edge1=SysTickValueGet();//get reset value
		
		//UARTprintf("edge2=%d\n",edge2);
		//iTick=0;
		//UARTprintf("second edge\n");
	}
	
} 


int main(void)
{
        ConfigureUART();
    
    //FPULazyStackingEnable();//from hello
        //set clock frequency to 50 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    
        //enable ports F and B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
        //configure PB5 as input before configuring pad w/ pull up resistors
        GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5);
    //GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
    
        //enable LED outputs
        GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    
        //configure interrupt type for PB5
		GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_FALLING_EDGE);
		GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
		IntEnable(INT_GPIOB);
		IntMasterEnable();
		edge1=0;
		edge2=0;
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);//red led
    int bitValues[32];
    int pulseWidth=0;
    int arrayPos=0;
		SysTickPeriodSet (16777000);
		SysTickEnable();
		int resetInt=0;
		iTick=0;
		arrayVals=0;

		while(1)
    {
			
				if(arrayVals==32){
					GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
					GPIOIntDisable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
					int loopcount;
					for(loopcount=0;loopcount<32;loopcount++){
							UARTprintf("value %d = %d\n",loopcount,arrayValues[loopcount]);
							int tempbitval=arrayValues[loopcount];
							if(tempbitval>178000 && tempbitval<180000)
									arrayValues[loopcount]=1;
							else if(tempbitval>87000 && tempbitval<89000)
									arrayValues[loopcount]=0;
							UARTprintf("binary value %d = %d\n",loopcount,arrayValues[loopcount]);
					}
					int remoteVal=0;
					for(loopcount=24;loopcount<32;loopcount++){
						remoteVal*=10;
						remoteVal+=arrayValues[loopcount];
						
					}
					UARTprintf("remote val=%d\n",remoteVal);
                    //remoteVal is a 8 bit number represented in decimal (so beginning 0's omitted)
                    //which represents the last 8 bits of the pattern, minus the final bit (so bits 23:30)
					switch(remoteVal){
						case 1101111: //0000 1101 1111 last 12 bits of "1", so this takes 01101111 (first 0 omitted)
							UARTprintf("1\n");
							break;
						case 101111:
							UARTprintf("2\n");
							break;
						case 1001111:
							UARTprintf("3\n");
							break;
						case 1110111:
							UARTprintf("4\n");
							break;
						case 110111:
							UARTprintf("5\n");
							break;
						case 1010111:
							UARTprintf("6\n");
							break;
						case 1100111:
							UARTprintf("7\n");
							break;
						case 100111:
							UARTprintf("8\n");
							break;
						case 1000111:
							UARTprintf("9\n");
							break;
						case 111011:
							UARTprintf("0\n");
							break;
                        case 111100://0 0 1 1 1  1 0 0 1
							UARTprintf("Down\n");
							break;
                        case 1111100:
							UARTprintf("Up\n");
							break;
                        case 101100:
							UARTprintf("Left\n");
							break;
                        case 1011100:
							UARTprintf("Right\n");
							break;
                        case 111:
							UARTprintf("Mute\n");
							break;
						default:
							UARTprintf("error, try again\n");
							break;
					}
					arrayPos=0;
					arrayVals=0;
					iTick=0;
					for(loopcount=0;loopcount<32;loopcount++){
						arrayValues[loopcount]=0;
					}
					SysCtlDelay(SysCtlClockGet()/3/10);//1.5 second delay before enabling interrupt again
					GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_5);//enabling it again breaks it!
					GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_5);
				}
			
   
    }
}




