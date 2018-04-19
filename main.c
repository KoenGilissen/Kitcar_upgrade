#include <project.h>
#include <stdio.h>
#include <math.h>

#define __DEBUG
#define TRUE 1
#define FALSE 0
#define DUTYCYCLENEG 0.001 // 1ms
#define DUTYCYCLEIDLE 0.0015 // 1.5ms
#define DUTYCYCLEPOS 0.002 //2ms
#define FCLK 100000 // 100 kHz
#define IDLETOLERANCE 6 // idle tolerance %
volatile uint8 __SIGNAL_LOST = TRUE;

CY_ISR( ch1Int );
CY_ISR( risingEdgeInt );
uint8 getSignalLostFlag(void);
void SteeringWheelDriver(uint32_t ch1Value );
uint32 rescaleCh1Value(uint32_t ch1Value);


int main(void)
{
    #ifdef __DEBUG
        char pulseWidth_CH1[10] = "";
        char pulseWidth_CH2[10] = "";
        char ch1RescaledVal[10] = "";
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
    PWM_STEERING_Start();
    
    isrCh1_StartEx( ch1Int );
    isrRisingEdge_StartEx( risingEdgeInt );

   
    CyGlobalIntEnable;      /* Enable global interrupts */
        
    while(TRUE)
    {
        if(getSignalLostFlag()) //KILL ALL
        {
            PinLed_Write(1);
            PWM_STEERING_Stop();
        }
        else
        {
            PinLed_Write(0);
            PWM_STEERING_Start();
        }
        pulseWidthCh1 = TimerCh1_ReadCapture();
        pulseWidthCh2 = TimerCh2_ReadCapture();
        SteeringWheelDriver( pulseWidthCh1 );
       
        #ifdef __DEBUG
            sprintf(pulseWidth_CH1, "%lu", (unsigned long) pulseWidthCh1);
            sprintf(pulseWidth_CH2, "%lu", (unsigned long) pulseWidthCh2); 
            sprintf(ch1RescaledVal, "%lu", (unsigned long) rescaleCh1Value(pulseWidthCh1)); 
            UART_1_UartPutString(pulseWidth_CH1); 
            UART_1_UartPutString("\t");
            UART_1_UartPutString(pulseWidth_CH2); 
            UART_1_UartPutString("\t");
            UART_1_UartPutString(ch1RescaledVal); 
            UART_1_UartPutString("\r\n");
        #endif
          
        CyDelay(50); // 50ms
        
        
    }
}
CY_ISR( ch1Int )
{
    __SIGNAL_LOST = TRUE;
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
    double idleTol =  (DUTYCYCLEIDLE*FCLK)+((IDLETOLERANCE/100)*(DUTYCYCLEIDLE*FCLK));
    if(ch1Value >= (DUTYCYCLEIDLE*FCLK-idleTol) && ch1Value <= (DUTYCYCLEIDLE*FCLK+idleTol)) //DEAD ZONE --> No drive
    {
        PWM_STEERING_WriteCompare1(0);
        PWM_STEERING_WriteCompare2(0);        
    }
    else if( ch1Value > (DUTYCYCLEIDLE*FCLK+idleTol))
    {
        PWM_STEERING_WriteCompare1(0); //IN1 = 0
        PWM_STEERING_WriteCompare2(rescaleCh1Value(ch1Value)); //IN2 = x
    }
    else
    {
        PWM_STEERING_WriteCompare1(rescaleCh1Value(ch1Value)); //IN1 = x
        PWM_STEERING_WriteCompare2(0); //IN2 = 0
    }
}

uint32 rescaleCh1Value(uint32_t ch1Value)
{
    double rescaledValue = 0;
    uint32_t pwmPeriod = PWM_STEERING_ReadPeriod();
    double chValueIdle = DUTYCYCLEIDLE*FCLK;
    double chValuePos = DUTYCYCLEPOS*FCLK;
    rescaledValue = ((fabs(ch1Value-chValueIdle))/(chValuePos-chValueIdle))*(double)pwmPeriod; 
    return (uint32_t) rescaledValue;
}
