/*  -------------------------------------------------------------------------------------------------------------------------
*   Author(s):      Corbin Bremmeyr & Nicholas Veltema
*   Date:           October 19, 2019
*   Class:          EGR326-901
*
*   ST7735 libraries and function calls are Copyright 2015 by Jonathan W. Valvano and
*   based off libraries originally written by Limor Fried/Ladyada for Adafruit
*   Industries. Reference ST7735.h and ST7735.c for full copyright details.
*   -------------------------------------------------------------------------------------------------------------------------
*/
#include "msp.h"
#include "ST7735.h"
#include "images.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SEC 48000000     // 48E^6 clock cycles = 1s
#define MS 48000         // 48000 clock cycles = 1ms
#define US 48            // 48 clock cycles = 1us
#define BOUNCE 10        // macro for debounce time delay

// Struct to hold the Time-Date Stamp (TDS)
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;
    uint8_t date;
    uint8_t month;
    uint8_t year;
} TDS_t;

TDS_t TDS = {0};            // Struct for storing current TDS

void init_Switches(void);
void init_CLK_48MHz(void);
void init_RotoryEncoder(void);
void init_timer32(void);
void delay_ms(int delay);
void init_SysTick(void);
void rotory_db_CW(void);
void rotory_db_CCW(void);
void btn_debounce(uint8_t port,uint8_t pin);
void user_input(void);
void update_LCD(void);
void print_string(uint8_t x, uint8_t y,char str1[], uint16_t f_color, uint16_t bg_color, uint8_t f_size);

// Real Time Clock (RTC) Functions
void rtc_get_TDS();
void rtc_set_TDS();
float rtc_get_temp(void);
uint8_t bcd_to_dec(uint8_t bcd);
uint8_t dec_to_bcd(uint8_t dec);

// I2C Communication Functions
void i2c_init(void);
void i2c_burst_write(uint8_t slave_addr, uint8_t mem_addr, uint32_t byte_count, uint8_t *data_buff);
void i2c_burst_read(uint8_t slave_addr, uint8_t mem_addr, uint32_t byte_count, uint8_t *data_buff);

// Proximity Sensor Functions
void init_prox(void);
void init_timerA(void);

//Global variables
//----------------------------------------------------------------------------------------------------------------------------
// User Input: Rotary encoder & pushbutton switch
uint8_t btn = 0;
int enc_count = 0;
int update = 0;
uint8_t enc_btn = 0;
uint8_t enc_flag = 0;
int last_enc_count = 0;
uint8_t time_out = 0;       // flag for menu timeout
uint8_t cw = 0;
uint8_t ccw = 0;
uint8_t emp_speed = 1;
uint8_t emp_tds = 0;
uint8_t emp_temp = 0;
uint8_t emp_flag = 0;
uint8_t emp_btn = 0;

// Proximity Sensor
uint32_t TA0_count = 0;
uint8_t prox_flag = 0;
uint8_t reset_background = 0;


uint8_t speed = 55; //TODO: set to zero once speed detection is set up
uint8_t temperature = 72;   //TODO: set to zero once temp detection is set up



/**
 * main.c
 */
void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    // System Setup
	__disable_irq();
	init_CLK_48MHz();
    init_SysTick();
    i2c_init();
    init_RotoryEncoder();
    init_Switches();
    init_timer32();
    init_prox();
    init_timerA();
    __enable_irq();

    ST7735_InitR(INITR_BLACKTAB);                   // Initialize Greentab ST7735 Display
    ST7735_FillScreen(0x0000);     // set screen to black

    rtc_get_TDS(&TDS);

    ST7735_DrawBitmap(4, 116, large_background, 120, 80);
    ST7735_DrawBitmap(4, 156, small_background, 60, 40);
    ST7735_DrawBitmap(64, 156, small_background, 60, 40);

    while(1)
    {
        rtc_get_TDS();
        update_LCD();
        if(emp_flag)
        {
            btn_debounce(2,0);
            emp_flag = 0;
            if(emp_btn)
            {
                if(emp_speed)
                {
                    emp_tds = 1;
                    emp_speed = 0;
                }
                else if(emp_tds)
                {
                    emp_tds = 0;
                    emp_temp = 1;
                }
                else if(emp_temp)
                {
                    emp_speed = 1;
                    emp_temp = 0;
                }

            }
            P2->IE |= 0x01;
        }
        if(enc_flag)
        {
            btn_debounce(5,6);
            enc_flag = 0;
            if(enc_btn)
                enc_btn = 0;
                user_input();
            P5->IE |= 0x40;
        }
    }
}
void update_LCD(void)
{
    char speed_str[7];
    char time_str[5];
    char date_str[8];
    char temp_str[5];

    if(prox_flag)
    {
        ST7735_DrawBitmap(0,160,warning_background,128,128);
        ST7735_DrawBitmap(2,133,prox3,124,101);
        reset_background = 1;
    }
    else
    {
        if(emp_btn || reset_background)
        {
            ST7735_FillScreen(0x0000);     // set screen to black
            ST7735_DrawBitmap(4, 116, large_background, 120, 80);
            ST7735_DrawBitmap(4, 156, small_background, 60, 40);
            ST7735_DrawBitmap(64, 156, small_background, 60, 40);
            emp_btn = 0;
            reset_background = 0;
        }

        if(emp_speed)
        {

            sprintf(speed_str,"%3d MPH",speed);
            print_string(20,78,speed_str,ST7735_BLACK,ST7735_WHITE,2);

            sprintf(time_str,"%2d:%02d",TDS.hours,TDS.minutes);
            print_string(20,130,time_str,ST7735_BLACK,ST7735_WHITE,1);

            sprintf(date_str,"%02d/%02d/%02d",TDS.month,TDS.date,TDS.year);
            print_string(10,140,date_str,ST7735_BLACK,ST7735_WHITE,1);

            sprintf(temp_str,"%3d F",temperature);
            print_string(78,135,temp_str,ST7735_BLACK,ST7735_WHITE,1);
        }
        if(emp_tds)
        {

            sprintf(time_str,"%2d:%02d",TDS.hours,TDS.minutes);
            print_string(32,68,time_str,ST7735_BLACK,ST7735_WHITE,2);

            sprintf(date_str,"%02d/%02d/%02d",TDS.month,TDS.date,TDS.year);
            print_string(15,88,date_str,ST7735_BLACK,ST7735_WHITE,2);

            sprintf(temp_str,"%3d F",temperature);
            print_string(20,135,temp_str,ST7735_BLACK,ST7735_WHITE,1);

            sprintf(speed_str,"%3d MPH",speed);
            print_string(75,135,speed_str,ST7735_BLACK,ST7735_WHITE,1);
        }
        if(emp_temp)
        {

            sprintf(temp_str,"%3d F",temperature);
            print_string(34,78,temp_str,ST7735_BLACK,ST7735_WHITE,2);

            sprintf(time_str,"%2d:%02d",TDS.hours,TDS.minutes);
            print_string(79,130,time_str,ST7735_BLACK,ST7735_WHITE,1);

            sprintf(date_str,"%02d/%02d/%02d",TDS.month,TDS.date,TDS.year);
            print_string(70,140,date_str,ST7735_BLACK,ST7735_WHITE,1);

            sprintf(speed_str,"%3d MPH",speed);
            print_string(14,135,speed_str,ST7735_BLACK,ST7735_WHITE,1);
        }
    }
}

/*
 * Draw string one character at a time
 *  --> First character replaced by second and so on
 *  --> Inputs: x = horizontal position
 *              y = vertical position
 *              str1 = null terminated string of characters
 */
void print_string(uint8_t x, uint8_t y,char str1[], uint16_t f_color, uint16_t bg_color, uint8_t f_size)
{
    uint16_t i = 0;

    while(str1[i] != '\0')
    {
        // write the character and
        ST7735_DrawChar( (x+(i*(f_size*6))), y, str1[i], f_color, bg_color, f_size);
        i++;
    }
}

void user_input(void)
{
    TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
    TIMER32_1->CONTROL |= 0x20;     // Enable timer32 interrupts (turn on menu timeouts)

    uint8_t x = 10;
    uint8_t y_settime = 82;
    uint8_t y_setdate = 102;
    uint8_t y_log = 122;
    uint8_t menu = 0;
    char str1[10];
    char str2[2];
    P5->IE |= 0x70;                                             // Enable encoder pin interrupts

    ST7735_DrawBitmap(0,160,menu_background,128,128);
    ST7735_FillRect(x, y_settime, 8, 8, ST7735_RED);        // Print red square for selection marker

    // reset direction flags
    cw = 0;
    ccw = 0;

    while(!time_out && !enc_btn)
    {
        if(enc_flag)
        {
            TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
            btn_debounce(5,6);
            enc_flag = 0;
            __delay_cycles(MS);     //TODO: need solution for delay
            P5->IE |= 0x40;
        }

        // Move menu selection marker according to rotary encoder
        if(menu == 0)
        {
            if(cw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                ST7735_FillRect(x, y_settime, 8, 8, ST7735_WHITE);
                ST7735_FillRect(x, y_setdate, 8, 8, ST7735_RED);
                menu = 1;
            }
            else if(ccw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                ST7735_FillRect(x, y_settime, 8, 8, ST7735_WHITE);
                ST7735_FillRect(x, y_log, 8, 8, ST7735_RED);
                menu = 2;
            }
            cw = 0;
            ccw = 0;
        }
        else if(menu == 1)
        {
            if(cw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                ST7735_FillRect(x, y_setdate, 8, 8, ST7735_WHITE);
                ST7735_FillRect(x, y_log, 8, 8, ST7735_RED);
                menu = 2;
            }
            else if(ccw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                ST7735_FillRect(x, y_setdate, 8, 8, ST7735_WHITE);
                ST7735_FillRect(x, y_settime, 8, 8, ST7735_RED);
                menu = 0;
            }
            cw = 0;
            ccw = 0;
        }
        else if(menu == 2)
        {
            if(cw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                ST7735_FillRect(x, y_log, 8, 8, ST7735_WHITE);
                ST7735_FillRect(x, y_settime, 8, 8, ST7735_RED);
                menu = 0;
            }
            else if(ccw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                ST7735_FillRect(x, y_log, 8, 8, ST7735_WHITE);
                ST7735_FillRect(x, y_setdate, 8, 8, ST7735_RED);
                menu = 1;
            }
            cw = 0;
            ccw = 0;
        }
        __delay_cycles(MS);     //TODO: fix timing issue
        P5->IE |= 0x30;
    }
    enc_btn = 0;

    // Set time
    if(menu == 0)
    {
        sprintf(str1,"%2d:%02d",TDS.hours,TDS.minutes);
        ST7735_DrawBitmap(0,160,time_background,128,128);
        ST7735_SetCursor(5,9);
        ST7735_SetTextColor(0x0000);
        ST7735_OutString(str1);
        while(!enc_btn && !time_out)
        {
            if(enc_flag)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                btn_debounce(5,6);
                enc_flag = 0;
                __delay_cycles(MS);     //TODO: need solution for delay
                P5->IE |= 0x40;
            }
            if(cw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.hours++;
                if(TDS.hours == 13)
                    TDS.hours = 1;
                cw = 0;
                sprintf(str2,"%2d",TDS.hours);
                ST7735_SetCursor(5,9);
                ST7735_OutString(str2);
            }
            else if(ccw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.hours--;
                if(TDS.hours > 13)
                    TDS.hours = 12;
                if(TDS.hours == 0)
                    TDS.hours = 12;
                ccw = 0;
                sprintf(str2,"%2d",TDS.hours);
                ST7735_SetCursor(5,9);
                ST7735_OutString(str2);
            }
            __delay_cycles(MS);     //TODO: fix timing issue
            P5->IE |= 0x30;
        }
        enc_btn = 0;
        while(!enc_btn && !time_out)
        {
            if(enc_flag)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                btn_debounce(5,6);
                enc_flag = 0;
                __delay_cycles(MS);     //TODO: need solution for delay
                P5->IE |= 0x40;
            }
            if(cw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.minutes++;
                if(TDS.minutes == 60)
                    TDS.minutes = 0;
                cw = 0;
                sprintf(str2,"%02d",TDS.minutes);
                ST7735_SetCursor(11,9);
                ST7735_OutString(str2);
            }
            else if(ccw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.minutes--;
                if(TDS.minutes > 60)
                    TDS.minutes = 59;
                ccw = 0;
                sprintf(str2,"%02d",TDS.minutes);
                ST7735_SetCursor(11,9);
                ST7735_OutString(str2);
            }
            __delay_cycles(MS);     //TODO: fix timing issue
            P5->IE |= 0x30;
        }
        enc_btn = 0;
    }

    // Set date
    else if(menu == 1)
    {
        sprintf(str1,"%02d/%02d/%02d",TDS.month,TDS.date,TDS.year);
        ST7735_SetCursor(3,9);
        ST7735_DrawBitmap(0,160,date_background,128,128);
        ST7735_OutString(str1);
        while(!enc_btn && !time_out)
        {
            if(enc_flag)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                btn_debounce(5,6);
                enc_flag = 0;
                __delay_cycles(MS);     //TODO: need solution for delay
                P5->IE |= 0x40;
            }
            if(cw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.month++;
                if(TDS.month == 13)
                    TDS.month = 1;
                cw = 0;
                sprintf(str2,"%02d",TDS.month);
                ST7735_SetCursor(3,9);
                ST7735_OutString(str2);
            }
            else if(ccw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.month--;
                if(TDS.month > 13)
                    TDS.month = 12;
                if(TDS.month == 0)
                    TDS.month = 12;
                ccw = 0;
                sprintf(str2,"%02d",TDS.month);
                ST7735_SetCursor(3,9);
                ST7735_OutString(str2);
            }
            __delay_cycles(MS);     //TODO: fix timing issue
            P5->IE |= 0x30;
        }
        enc_btn = 0;
        while(!enc_btn && !time_out)
        {
            if(enc_flag)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                btn_debounce(5,6);
                enc_flag = 0;
                __delay_cycles(MS);     //TODO: need solution for delay
                P5->IE |= 0x40;
            }
            if(cw)                              // TODO: add logic for varying # of days in months
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.date++;
                if(TDS.date == 32)
                    TDS.date = 1;
                cw = 0;
                sprintf(str2,"%02d",TDS.date);
                ST7735_SetCursor(9,9);
                ST7735_OutString(str2);
            }
            else if(ccw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.date--;
                if(TDS.date == 0)
                    TDS.date = 31;
                ccw = 0;
                sprintf(str2,"%02d",TDS.date);
                ST7735_SetCursor(9,9);
                ST7735_OutString(str2);
            }
            __delay_cycles(MS);     //TODO: fix timing issue
            P5->IE |= 0x30;
        }
        enc_btn = 0;
        while(!enc_btn && !time_out)
        {
            if(enc_flag)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                btn_debounce(5,6);
                enc_flag = 0;
                __delay_cycles(MS);     //TODO: need solution for delay
                P5->IE |= 0x40;
            }
            if(cw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.year++;
                if(TDS.year == 100)
                    TDS.year = 0;
                cw = 0;
                sprintf(str2,"%02d",TDS.year);
                ST7735_SetCursor(15,9);
                ST7735_OutString(str2);
            }
            else if(ccw)
            {
                TIMER32_1->LOAD = (SEC * 60) - 1;      // Reset timer32 count
                TDS.year--;
                if(TDS.year > 100)
                    TDS.year = 99;
                ccw = 0;
                sprintf(str2,"%02d",TDS.year);
                ST7735_SetCursor(15,9);
                ST7735_OutString(str2);
            }
            __delay_cycles(MS);     //TODO: fix timing issue
            P5->IE |= 0x30;
        }
        enc_btn = 0;
    }
    else if(menu == 2)
    {
        //rtc_get_TDS();
        // TODO: add exit debounce routine for encoder button
        // TODO: add displaying event log
    }

    rtc_set_TDS();
    P5->IE &= ~0x20;
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_DrawBitmap(4, 116, large_background, 120, 80);
    ST7735_DrawBitmap(4, 156, small_background, 60, 40);
    ST7735_DrawBitmap(64, 156, small_background, 60, 40);
    TIMER32_1->CONTROL &= ~0x20;    // Disable timer32 interrupt (turn off menu time out)
    time_out = 0;
}



//---------------------------------------------------------------------------------------------------------------- Interrupt Handlers

/**
 * Timer32_1 interrupt handler
 */
void T32_INT1_IRQHandler(void)
{
    TIMER32_1->INTCLR = 1;              //clear Timer32 interrupt flag
    time_out = 1;
}

/**
 *  Port 2 Interrupt Handler
 *  -> Flags which button was pressed
 *  -> Disables corresponding interrupt bit until the button press has been processed
 */
void PORT2_IRQHandler()
{
    int flag = P2->IFG;         //store the port 2 interrupt flags
    P2->IFG = 0;                //clear port 2 interrupt flags

    if(flag & BIT0)             //if btn_1 was pressed
        {
            P2->IE &= ~BIT0;    //turn off interrupt until steady state is confirmed
            emp_flag = 1;          //set the btn_1 flag
        }
}
/**
 *  Port 5 Interrupt Handler
 *  -> CLK 5.4
 *  -> DATA 5.5
 *  -> Switch 5.6
 */
void PORT5_IRQHandler()
{
    int flag = P5->IFG;                 // Store the port 5 interrupt flags
    P5->IFG = 0;                        // Clear port 5 interrupt flags
    //delay_ms(1);                        // Allow input pins time to change
    uint8_t DT = (P5->IN & BIT5)>>5;    // Read Data pin and store as LSB
    uint8_t CLK = (P5->IN & BIT4)>>4;   // Read CLK pin and store as LSB

    if(CLK != DT)    // Rule out accidental interrupt -> CLK and DT pins change out of phase
    {

        // If CLK interrupts direction is CCW
        if(flag & BIT4)
        {
            P5->IE &= ~0x30;    // Turn off rotation pin interrupts
            enc_count--;
            rotory_db_CCW();
            while(((P5->IN & BIT4)>>4) != ((P5->IN & BIT5)>>5)){}   // Wait until CLK and DT pins match
            P5->IES = (P5->IES ^= BIT4);                            // Toggle edge interrupt select
            ccw = 1;
        }
        // If DT interrupts direction is CW
        else if(flag & BIT5)
        {
            P5->IE &= ~0x30;    // Turn off rotation pin interrupts
            enc_count++;
            rotory_db_CW();
            while(((P5->IN & BIT4)>>4) != ((P5->IN & BIT5)>>5)){}   // Wait until CLK and DT pins match
            P5->IES = (P5->IES ^= BIT5);                            // Toggle edge interrupt select
            cw = 1;
        }
        //P5->IE |= 0x30;                                      // Enable rotation pin interrupts
    }
    if(flag & 0x40)
    {
        P5->IE &= ~0x40;
        enc_flag = 1;
    }
}
/**
 *  Timer A0 Interrupt Handler
 *  ->
 */
void TA0_N_IRQHandler(void)
{
    TIMER_A0->CCTL[1] &=~ 1;    // Clear interrupt flag

    // Reset count value on rising edge
    if(!(P2->IN & 0x10)) {
        TIMER_A0->CTL |= 0x4;
    }

    // Save high time of echo signal
    else if(P2->IN & 0x10) {
        TA0_count = TIMER_A0->R;
    }
    if(TA0_count < 3774)    // 3774 cycles / 1.5 cycles per US / 148 = 17 inches (16 + 1in offset)
            {
                prox_flag = 1;
            }
    else
    {
        prox_flag = 0;
    }
}

//---------------------------------------------------------------------------------------------------------------- System Routines
void delay_ms(int delay)
{
    SysTick->LOAD = ((delay * MS) - 1);     // Load delay time
    SysTick->VAL = 0;                       // Count down to zero
    while((SysTick->CTRL & BIT(16))==0){}   // Wait for delay to elapse
}
/**
 *  Switch Bounce Routine
 *  -> Confirms button press
 *  -> BLOCKING while the button remains pressed
 */
void btn_debounce(uint8_t port,uint8_t pin)
{
    uint64_t bounce = 0x00000001;   //bounce register = 0b0000 0000 0000 0001

    if(port == 5)
    {
        while(bounce != 0xffffffffffffffff)     //loop while switch is pressed or bouncing
        {
            bounce = (bounce<<1) | ((P5->IN & BIT(pin))>>pin); //shift last switch level left and add new switch level
            if(bounce == 0x0000)    //eleven sequential zeros confirms button press
            {
                enc_btn = 1;      //set button flag
            }
        }
        __delay_cycles(50);
    }
    if(port == 2)
    {
        while(bounce != 0xffffffff)     //loop while switch is pressed or bouncing
        {
            bounce = (bounce<<1) | ((P2->IN & BIT(pin))>>pin); //shift last switch level left and add new switch level
            if(bounce == 0x00000000)    //eleven sequential zeros confirms button press
            {
                emp_btn = 1;      //set button flag
            }
        }
        __delay_cycles(50);
    }
}
/**
 *  Switch Bounce Routine
 *  -> Confirms button press
 *  -> BLOCKING while the button remains pressed
 */
void rotory_db_CW(void)
{
    uint32_t bounce = 0x00000001;   //bounce register = 0b0000 0000 0000 0001

    while(bounce != 0xffffffff)     //loop while switch is pressed or bouncing
    {
        bounce = (bounce<<1) | (P5->IN & BIT5)>>5; //shift last switch level left and add new switch level
        __delay_cycles(50);
    }
}
/**
 *  Switch Bounce Routine
 *  -> Confirms button press
 *  -> BLOCKING while the button remains pressed
 */
void rotory_db_CCW(void)
{
    uint32_t bounce = 0x00000001;   //bounce register = 0b0000 0000 0000 0001

    while(bounce != 0xffffffff)     //loop while switch is pressed or bouncing
    {
        bounce = (bounce<<1) | (P5->IN & BIT4)>>4; //shift last switch level left and add new switch level
        __delay_cycles(50);
    }
}

//---------------------------------------------------------------------------------------------------------------- System Initialization Functions
void init_timer32(void)
{
    TIMER32_1->CONTROL = 0b11000010;    // Enabled, Periodic, IE disabled, 32-Bit counter (Interrupt enable is BIT5)
    TIMER32_1->LOAD = (SEC * 60) - 1;     // Interrupt every second to update RPMs
    NVIC_EnableIRQ(T32_INT1_IRQn);      // Enable Timer32 interrupts
}


void init_RotoryEncoder(void)
{
    // Initialize P5.4, P5.5, P5.6 as inputs with interrupts
    P5->SEL0 &= ~0x70;
    P5->SEL1 &= ~0x70;
    P5->DIR &= ~0x70;
    P5->REN |= 0x70;
    P5->OUT |= 0x70;
    P5->IES &= ~0x10;               //Interrupt P5.4 on rising edge
    P5->IES |= 0x20;                //Interrupt P5.5, P5.6 on falling edge
    P5->IE |= 0x40;                 // Initialize only P5.6 with active interrupts

    NVIC_EnableIRQ(PORT5_IRQn);     // Initialize port 5 interrupt handler
    P5->IFG = 0;                    // Clear port 5 interrupt flags
}

void init_SysTick()
{
   SysTick->CTRL = 0;
   SysTick->LOAD = (MS - 1);        // Load arbitrary count for initialization
   SysTick->VAL = 0;
   SysTick->CTRL = 5;
}

/* Initialize MSP432 to run off the 48MHz External Clock
 *      --> Run SMCLK at 12MHz
 * Author: ??
 * Reference: EGR326 Lecture Slides
 */
void init_CLK_48MHz(void)
{
    // Configure Flash wait-state to 1 for both banks 0 & 1
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) | FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) | FLCTL_BANK1_RDCTL_WAIT_1;

    //Configure HFXT to use 48MHz crystal, source to MCLK & HSMCLK*
    PJ->SEL0 |= BIT2 | BIT3; // Configure PJ.2/3 for HFXT function
    PJ->SEL1 &= ~(BIT2 | BIT3);
    CS->KEY = CS_KEY_VAL ; // Unlock CS module for register access
    CS->CTL2 |= CS_CTL2_HFXT_EN | CS_CTL2_HFXTFREQ_6 | CS_CTL2_HFXTDRIVE;
    while(CS->IFG & CS_IFG_HFXTIFG)
    CS->CLRIFG |= CS_CLRIFG_CLR_HFXTIFG;

    /* Select MCLK & HSMCLK = HFXT, no divider */
    CS->CTL1 = CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK | CS_CTL1_SELS_MASK | CS_CTL1_DIVHS_MASK) | CS_CTL1_SELM__HFXTCLK | CS_CTL1_SELS__HFXTCLK;
    CS->CTL1 |= 0x20000000; // SMCLK = HFXT / 4
    CS->KEY = 0; // Lock CS module from unintended accesses

    // 010 0000 0000    ACLK sourced by REFOCLK
    CS->KEY = 0x0695A;      // Unlock CS registers
    CS->CTL1 |= 0x200;      // 010 0000 0000
    CS->CTL1 &= ~0x500;
    CS->CLKEN |= 0x8000;

    CS->KEY = 0; // Lock CS module from unintended accesses
}
/**
 *  Initialize push button switches
 *  ->System setup routine
 */
void init_Switches(void)
{
    //initialize btn_1 on P2.0 with interrupt
    P2->SEL0 &= ~BIT0;
    P2->SEL1 &= ~BIT0;
    P2->DIR &= ~BIT0;
    P2->REN |= BIT0;
    P2->OUT |= BIT0;
    P2->IES |= BIT0;
    P2->IE |= BIT0;

    NVIC_EnableIRQ(PORT2_IRQn);     //initialize port 5 interrupt handler
    P2->IFG = 0;                    //clear port 5 interrupt flags
}

/**
 * Setup Proximity Echo & Trigger
 * -> P2.5 as TA output
 * -> P2.4 as TA capture input
 */
void init_prox(void) {

    // Trigger pin  P2.5 -> TA0.2
    // Echo pin     P2.4 -> TA0.1
    P2->SEL0 |=  0x30;
    P2->SEL1 &=~ 0x30;
    P2->DIR  |=  0x20;
    P2->DIR  &=~ 0x10;
    P2->OUT  &=~ 0x20;
}
/**
 * Setup TA0.1 for capture input
 *       TA0.2 for pulse output
 */
void init_timerA(void)
{
    TIMER_A0->CCR[1]  = 0;
    TIMER_A0->CCTL[1] = 0xC1F0;

    TIMER_A0->CCR[2]  = (10 * 2) - 1;   // 1us = 1.5 cycles after division so round up to 2
    TIMER_A0->CCTL[2] = 0xE0;

    TIMER_A0->CTL     = 0x2E0;  //0010 1110 0000       SMCLK, /8, UP Mode

    NVIC_EnableIRQ(TA0_N_IRQn);
}

//---------------------------------------------------------------------------------------------------------------- RTC Functions
/**
 * Set the time on the RTC over the I2C bus
 *
 * param *TDS - TDS_t struct holds the time-date stamp to be written to the RTC
 */
void rtc_set_TDS() {    //const TDS_t *TDS

    const uint8_t buff_size = 7;
    uint8_t buff[7] = {0};

    // Pull values from struct into buffer array
    buff[0] = dec_to_bcd(TDS.seconds);
    buff[1] = dec_to_bcd(TDS.minutes);
    buff[2] = dec_to_bcd(TDS.hours);
    buff[3] = dec_to_bcd(TDS.day);
    buff[4] = dec_to_bcd(TDS.date);
    buff[5] = dec_to_bcd(TDS.month);
    buff[6] = dec_to_bcd(TDS.year);

    // Send data to RTC
    i2c_burst_write(0x68, 0x00, buff_size, buff);
    EUSCI_B1->IFG = 0;          // Clear all interrupt flags
}

/**
 * Read time-date stamp from RTC over I2C bus
 *
 * param *TDS - TDS_t struct where time-date stamp will be stored
 */
void rtc_get_TDS() {    //TDS_t *TDS

    const uint8_t buff_size = 7;
    uint8_t buff[7] = {0};

    // Read raw date values from RTC
    i2c_burst_read(0x68, 0x00, buff_size, buff);

    // Convert BCD to normal int encoding
    TDS.seconds = bcd_to_dec(buff[0]);
    TDS.minutes = bcd_to_dec(buff[1]);
    TDS.hours   = bcd_to_dec(buff[2] & 0x1F);
    TDS.day   = bcd_to_dec(buff[3]);
    TDS.date  = bcd_to_dec(buff[4]);
    TDS.month = bcd_to_dec(buff[5] & 0x1F);
    TDS.year  = bcd_to_dec(buff[6]);
    EUSCI_B1->IFG = 0;          // Clear all interrupt flags
}

/**
 * Read the temperature from the RTC
 *
 * return the temperature in degrees celsius
 */
//void rtc_get_temp(void) {
//
//    const uint8_t buff_size = 2;
//    uint8_t buff[2] = {0};
//    int temp = 0;
//
//    // Read raw data from RTC temp sensor
//    i2c_burst_read(0x68, 0x11, buff_size, buff);
//
//    // Convert temp data into an int
//    temp = ((buff[0] << 8) | (buff[1])) >> 6;
//    if(buff[0] & 0x80) {
//        temp = temp * -1;
//    }
//
//    return temp * 0.25;
//}

/**
 * Convert a BCD encoded value to standard integer encoding
 *
 * param bcd - BCD byte to be converted
 *
 * return decimal value of the BCD encoded input
 */
uint8_t bcd_to_dec(uint8_t bcd) {
    return (((bcd & 0xF0) >> 4) * 10) + (bcd & 0x0F);
}

/**
 * Convert a standard integer encoding to BCD encoding
 *
 * param dec - integer to be converted
 *
 * return converted BCD value
 */
uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

/**
 * Sequencially write data to i2c slave's memory address. Function returns after the write is complete
 * Source: Mazidi, Naimi, Chen, & Salmanzadeh: TI MSP432 ARM Programming for Embedded systems
 *
 * param slave_addr - ID value of slave to write to
 * param mem_addr - starting memory address on slave to write to
 * param byte_count - number of bytes to write to slave
 * param *data_buff - pointer to start of data to write to slave
 */
void i2c_burst_write(uint8_t slave_addr, uint8_t mem_addr, uint32_t byte_count, uint8_t *data_buff)
{
    EUSCI_B1->I2CSA  = slave_addr;      // Set salve addr for transaction
    EUSCI_B1->CTLW0 |= 0x0010;          // Set next transaction to be a 'write' operation
    EUSCI_B1->CTLW0 |= 0x0002;          // Generate the start bit
    while(!(EUSCI_B1->IFG & 0x2));      // Wait for transmit to be ready
    EUSCI_B1->TXBUF = mem_addr;         // Send starting memeory addr

    // Send data
    do {
        while(!(EUSCI_B1->IFG & 0x2));  // Wait for transmit to be ready
        EUSCI_B1->TXBUF = *data_buff;
        data_buff++;
        byte_count--;
    } while(byte_count > 0);

    while(!(EUSCI_B1->IFG & 0x2));      // Wait for transmit to be ready
    EUSCI_B1->CTLW0 |= 0x0004;          // Send stop bit
    while(EUSCI_B1->CTLW0 & 0x4);       // Wait for stop bit to be sent
}

/**
 * Sequencial reads from a slave's memory, stored to provided buffer
 * Source: Mazidi, Naimi, Chen, & Salmanzadeh: TI MSP432 ARM Programming for Embedded systems
 *
 * param slave_addr - address of slave to read from
 * param mem_addr - starting memory address (slave's memory) to read from
 * param byte_count - number of bytes to read from slave's memory
 * param *data_buff - location to store read data, memory block needs to own `byte_count` number of bytes
 */
void i2c_burst_read(uint8_t slave_addr, uint8_t mem_addr, uint32_t byte_count, uint8_t *data_buff) {

    EUSCI_B1->I2CSA  = slave_addr;      // Set slave addr to send
    EUSCI_B1->CTLW0 |= 0x0010;          // Set transaction to perform a 'write' operation to send memory address
    EUSCI_B1->CTLW0 |= 0x0002;          // Start transaction
    while(EUSCI_B1->CTLW0 & 0x2);       // Wait for transmitter to be ready for data bytes and send starting memory addr
    EUSCI_B1->TXBUF = mem_addr;
    while(!(EUSCI_B1->IFG & 0x2));      // Wait for transmit to be done

    // Start a 'read' transmission
    EUSCI_B1->CTLW0 &=~ 0x0010;
    EUSCI_B1->CTLW0 |=  0x0002;
    while(EUSCI_B1->CTLW0 & 0x2);

    // Read desired number of bytes from slave
    do {
        // Setup stop bit if on last byte
        if(byte_count == 1) {
            EUSCI_B1->CTLW0 |= 0x04;
        }
        while(!(EUSCI_B1->IFG & 0x1));  // Wait for i2c to be ready
        *data_buff++ = EUSCI_B1->RXBUF; // Write received data into buffer
        byte_count--;
    } while(byte_count);
    while(EUSCI_B1->CTLW0 & 0x4);       // Wait for stop bit to be sent
}

/**
 * Setup P6.4 and P6.5 as an I2C bus
 * Source: Mazidi, Naimi, Chen, & Salmanzadeh: TI MSP432 ARM Programming for Embedded systems
 */
void i2c_init(void) {
    EUSCI_B1->CTLW0 |= 1;

    // ACLK
    EUSCI_B1->CTLW0  = 0x0F81;  // 0b 0000 1111 0100 0001
    EUSCI_B1->BRW    = 120;     // 12MHz/120 = 100kHz

    EUSCI_B1->IFG = 0;          // Clear all interrupt flags

    P6->SEL0 |=  0x30;
    P6->SEL1 &=~ 0x30;
    EUSCI_B1->CTLW0 &=~ 1;
}

