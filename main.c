#include <project.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#define ARM_MATH_ROUNDING

#define __DEBUG
#define TRUE 1
#define FALSE 0
#define DUTYCYCLENEG 0.001 // 1ms
#define DUTYCYCLEIDLE 0.0015 // 1.5ms
#define DUTYCYCLEPOS 0.002 //2ms
#define FCLK 100000 // 100 kHz
#define IDLETOLERANCE 6 // idle tolerance %
#define TROTTLECOUNTSIDLE 623
#define TFACTOR 6.235

volatile uint8 __SIGNAL_LOST = TRUE;


CY_ISR( ch1Int );
CY_ISR( risingEdgeInt );
uint8 getSignalLostFlag(void);
void SteeringWheelDriver(uint32_t ch1Value );
uint32_t trottleResponse(uint32_t channel2);


int main(void)
{
    #ifdef __DEBUG
        char uartMessage[20] = "";
    #endif
    uint32_t pulseWidthCh1 = 0;
    uint32_t pulseWidthCh2 = 0;

    #ifdef __DEBUG
        UART_1_Start();
    #endif /* DEBUG INFO */
    
    #ifdef __DEBUG
        UART_1_UartPutString("\r\nPSoC4200M Hello World!\r\n");
    #endif
    
    TimerCh1_Start();
    TimerCh2_Start();
    PinOutDrv8871IN1_Write(0);
    PinOutDrv8871IN2_Write(0);
    PWMTrottle_WriteCompare1(TROTTLECOUNTSIDLE);
    PWMTrottle_WriteCompare1(TROTTLECOUNTSIDLE);
    PWMTrottle_Start();
    isrCh1_StartEx( ch1Int );
    isrRisingEdge_StartEx( risingEdgeInt );

   
    CyGlobalIntEnable;      /* Enable global interrupts */
        
    while(TRUE)
    {
        if(getSignalLostFlag()) //KILL ALL
        {
            PinLed_Write(1);
            PinOutDrv8871IN1_Write(0);
            PinOutDrv8871IN2_Write(0);   
            PWMTrottle_Stop();
            PWMTrottle_WriteCompare1(TROTTLECOUNTSIDLE);
            PWMTrottle_WriteCompare2(TROTTLECOUNTSIDLE);
        }
        else //DO Stuff
        {
            PinLed_Write(0);
            pulseWidthCh1 = TimerCh1_ReadCapture();
            pulseWidthCh2 = TimerCh2_ReadCapture();
            SteeringWheelDriver( pulseWidthCh1 );
            PWMTrottle_Start();
            PWMTrottle_WriteCompare1(trottleResponse(pulseWidthCh2));
            PWMTrottle_WriteCompare2(trottleResponse(pulseWidthCh2));
        }

       
        #ifdef __DEBUG
            sprintf(uartMessage, "%lu", (unsigned long) pulseWidthCh1);
            UART_1_UartPutString(uartMessage); 
            strcpy(uartMessage, "");
            UART_1_UartPutString("\t");
            sprintf(uartMessage, "%lu", (unsigned long) trottleResponse(pulseWidthCh2)); 
            UART_1_UartPutString(uartMessage);
            strcpy(uartMessage, "");
            UART_1_UartPutString("\r\n");
        #endif
          
        CyDelay(50); // 50ms
        
        
    }
}
CY_ISR( ch1Int )
{
    __SIGNAL_LOST = TRUE;
    PinOutDrv8871IN1_Write(0);
    PinOutDrv8871IN2_Write(0); 
    TimerCh1_ClearInterrupt(TimerCh1_INTR_MASK_TC);
}

CY_ISR( risingEdgeInt )
{
    isrRisingEdge_ClearPending();
    __SIGNAL_LOST = FALSE;
}

uint8 getSignalLostFlag()
{
    CyGlobalIntDisable;
    uint8 temp = __SIGNAL_LOST;
    CyGlobalIntEnable;
    return temp;
}

void SteeringWheelDriver(uint32_t ch1Value )
{
    double idleTol =  (IDLETOLERANCE)*(DUTYCYCLEIDLE*FCLK)/100;
    if(ch1Value >= (DUTYCYCLEIDLE*FCLK-idleTol) && ch1Value <= (DUTYCYCLEIDLE*FCLK+idleTol)) //DEAD ZONE --> No drive
    {
        PinOutDrv8871IN1_Write(0);
        PinOutDrv8871IN2_Write(0);       
    }
    else if( ch1Value > (DUTYCYCLEIDLE*FCLK+idleTol))
    {
        PinOutDrv8871IN1_Write(0);
        PinOutDrv8871IN2_Write(1);   
    }
    else
    {
        PinOutDrv8871IN1_Write(1);
        PinOutDrv8871IN2_Write(0);   
    }
}

uint32_t trottleResponse(uint32_t channel2)
{
    double trottleValue = 0.0;
    if(channel2 > 147 && channel2 < 153) //IDLE ]147, 153[
    {
        trottleValue = 934.5;
    }
    else if(channel2 > 190 || channel2 < 110) //MAX ]190[ ]110[
    {
        trottleValue = channel2*TFACTOR;
    }
    else if( channel2 >= 110 && channel2 <= 129) // [110, 129]
    {
        trottleValue = ((channel2)*0.015789-0.63684)*623.5;
    }
    else if( channel2 >= 130 && channel2 <= 147) // [130, 147]
    {
        trottleValue = ((channel2)*0.003529+0.941176)*623.5;
    }
    else if( channel2 >= 153 && channel2 <= 170) // [153, 170]
    {
        trottleValue = ((channel2)*0.004029+0.9153)*623.5;
    }
    else if( channel2 >= 171 && channel2 <= 190) // [171, 190]
    {
        trottleValue = ((channel2)*0.015789-1.1)*623.5;
    }
    else
    {
        trottleValue = 934.5; //back up
    }
    return (uint32_t) trottleValue;
}
