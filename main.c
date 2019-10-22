/*  -------------------------------------------------------------------------------------------------------------------------
*   Author(s):      Nicholas Veltema & Corbin Bremmeyr
*   Date:           October 20, 2019
*   Class:          EGR326-901
*   Description:    Runs unipolar stepper continuously to simulate RPMs detectable using Hall Effect sensor
*
*   P6.0 - Pushbutton 3
*   P6.4 - Pushbutton 1
*   P6.5 - Pushbutton 2
*
*   P5.0 - Stepper IN1
*   P5.1 - Stepper IN2
*   P5.2 - Stepper IN3
*   P5.3 - Stepper IN4
*   -------------------------------------------------------------------------------------------------------------------------
*/

#include "msp.h"

#define MS 3000

void init_systick(void);
void delay_ms(uint16_t delay);
void btn1_debounce(void);
void btn2_debounce(void);
void btn3_debounce(void);
void init_uniStepper(void);

void init_Switches(void);
uint8_t btn_1 = 0;
uint8_t pressed_1 = 0;
uint8_t btn_2 = 0;
uint8_t pressed_2 = 0;
uint8_t btn_3 = 0;
uint8_t pressed_3 = 0;
uint8_t enable = 0;
static uint8_t last_state = 1;
uint8_t pulse_delay = 100;
/**
 * main.c
 */
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    // Init system
    __disable_irq();
    init_uniStepper();
    init_systick();
    init_Switches();
    __enable_irq();

    while(1) {

        if(btn_1)
        {
            btn1_debounce();
            if(pressed_1)
            {
                enable = 1;
                pressed_1 = 0;
                P5->OUT = last_state & 0x0F;
            }
            btn_1 = 0;
            P6->IE |= BIT4;
        }
        if(btn_2)
        {
            btn2_debounce();
            if(pressed_2)
            {
                last_state = P5->OUT & 0x0F;
                P5->OUT &=~ 0x0F;
                pressed_1 = 0;
                enable = 0;
            }
            btn_2 = 0;
            P6->IE |= BIT5;
        }
        if(btn_3)
        {
            btn3_debounce();
            if(pressed_3 && pulse_delay > 20)
            {
                pulse_delay -= 10;
            }
            pressed_3 = 0;
            btn_3 = 0;
            P6->IE |= BIT0;
        }
        if(enable)
        {

            P5->OUT = (P5->OUT << 1) & 0x0F;
            if(!(P5->OUT & 0x0F)) {
                P5->OUT |= 0x01;
            }
            delay_ms(pulse_delay);
        }
    }
}

/**
 * Blocking delay using SysTick timer
 *
 * param delay - time to delay in ms
 */
void delay_ms(uint16_t delay) {
    SysTick->LOAD = delay * MS;
    while(!(SysTick->CTRL & 0x10000));
}

/**
 * Set P10.0 - 3 as GPIO outputs, initilzed to LOW
 */
void init_uniStepper(void) {
    P5->SEL0 &=~ 0x0F;
    P5->SEL1 &=~ 0x0F;
    P5->OUT  &=~ 0x0F;
    P5->DIR  |=  0x0F;
}

/**
 *  Initialize push button switches
 *  ->System setup routine
 */
void init_Switches(void)
{
    //initialize btn_1 on P6.4 with interrupt
    P6->SEL0 &= ~0x31;
    P6->SEL1 &= ~0x31;
    P6->DIR &= ~0x31;
    P6->REN |= 0x31;
    P6->OUT |= 0x31;
    P6->IES |= 0x31;
    P6->IE |= 0x31;

    NVIC_EnableIRQ(PORT6_IRQn);     //initialize port 5 interrupt handler
    P6->IFG = 0;                    //clear port 5 interrupt flags
}
/**
 *  Port 5 Interrupt Handler
 *  -> Flags which button was pressed
 *  -> Disables corresponding interrupt bit until the button press has been processed
 */
void PORT6_IRQHandler()
{
    int flag = P6->IFG;         //store the port 5 interrupt flags
    P6->IFG = 0;                //clear port 5 interrupt flags

    if(flag & BIT0)
    {
        P6->IE &= ~BIT0;
        btn_3 = 1;
    }
    if(flag & BIT4)             //if btn_1 was pressed
    {
        P6->IE &= ~BIT4;    //turn off P5.4 interrupt until steady state is confirmed
        btn_1 = 1;          //set the btn_1 flag
    }
    if(flag & BIT5)
    {
        P6->IE &= ~BIT5;
        btn_2 = 1;
    }
}
/**
 *  Switch Bounce Routine
 *  -> Confirms button press
 *  -> BLOCKING while the button remains pressed
 */
void btn1_debounce(void)
{
    uint16_t bounce = 0xf801;   //bounce register = 0b1111 1000 0000 0001

    while(bounce != 0xffff)     //loop while switch is pressed or bouncing
    {
        bounce = (bounce<<1) | (P6->IN & BIT4)>>4 | 0xf800; //shift last switch level left and add new switch level
        if(bounce == 0xf800)    //eleven sequential zeros confirms button press
        {
            pressed_1 = 1;      //set button flag
        }
    }

}
/**
 *  Switch Bounce Routine
 *  -> Confirms button press
 *  -> BLOCKING while the button remains pressed
 */
void btn2_debounce(void)
{
    uint16_t bounce = 0xf801;   //bounce register = 0b1111 1000 0000 0001

    while(bounce != 0xffff)     //loop while switch is pressed or bouncing
    {
        bounce = (bounce<<1) | (P6->IN & BIT5)>>5 | 0xf800; //shift last switch level left and add new switch level
        if(bounce == 0xf800)    //eleven sequential zeros confirms button press
        {
            pressed_2 = 1;      //set button flag
        }
    }

}
/**
 *  Switch Bounce Routine
 *  -> Confirms button press
 *  -> BLOCKING while the button remains pressed
 */
void btn3_debounce(void)
{
    uint16_t bounce = 0xf801;   //bounce register = 0b1111 1000 0000 0001

    while(bounce != 0xffff)     //loop while switch is pressed or bouncing
    {
        bounce = (bounce<<1) | (P6->IN & BIT0) | 0xf800; //shift last switch level left and add new switch level
        if(bounce == 0xf800)    //eleven sequential zeros confirms button press
        {
            pressed_3 = 1;      //set button flag
        }
    }

}
/**
 * Setup SysTick timer
 */
void init_systick(void) {
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x05;
}
