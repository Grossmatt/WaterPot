//
// Matthew Grossweiler

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "adc0.h"
#include <math.h>

#define MAX_CHARS 80
#define MAX_FIELDS 5
#define MEL_LEN 100
char FS[MAX_CHARS+1];

typedef struct _USER_DATA
{
char buffer[MAX_CHARS+1];
uint8_t fieldCount;
uint8_t fieldPosition[MAX_FIELDS];
char fieldType[MAX_FIELDS];
} USER_DATA;

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
#define C0_mask 128
#define PE3_AIN0_MASK 8
#define PE2_AIN1_MASK 4
#define PE1_AIN2_MASK 2
#define PB4_MASK 16
#define PB6_MASK 64
#define PD1_MASK 2

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4 | SYSCTL_RCGCGPIO_R3;
    SYSCTL_RCGCACMP_R |= 1;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2;
    SYSCTL_PPHIB_R = 1;
    _delay_cycles(3);

    //Turn on Comparator
    GPIO_PORTC_DIR_R |= C0_mask;
    GPIO_PORTC_AMSEL_R |= C0_mask;
    COMP_ACREFCTL_R |= COMP_ACREFCTL_EN | COMP_ACREFCTL_VREF_M;
    COMP_ACREFCTL_R &= ~COMP_ACREFCTL_RNG;
    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF;

    //Turn on AIN
    GPIO_PORTE_AFSEL_R |= PE3_AIN0_MASK | PE2_AIN1_MASK | PE1_AIN2_MASK;
    GPIO_PORTE_DEN_R &= ~PE3_AIN0_MASK & ~PE2_AIN1_MASK & ~PE1_AIN2_MASK;
    GPIO_PORTE_AMSEL_R |= PE3_AIN0_MASK | PE2_AIN1_MASK | PE1_AIN2_MASK;

    //Timer 1 Config
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;

    //Timer 2 Config
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER2_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN0_R |= 1 << (INT_TIMER2A-16);


    //DEINT - PB4 / Motor Control - PB6
    GPIO_PORTB_DIR_R |= PB4_MASK | PB6_MASK;
    GPIO_PORTB_DR2R_R |= PB4_MASK | PB6_MASK;
    GPIO_PORTB_DEN_R |= PB4_MASK | PB6_MASK;

    //Speaker Control - PD1
    GPIO_PORTD_DIR_R |= PD1_MASK;
    GPIO_PORTD_DR2R_R |= PD1_MASK;
    GPIO_PORTD_DEN_R |= PD1_MASK;

    //Configure RTC
    HIB_CTL_R |= 64 | 1;

    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs
}

void getsUart0(USER_DATA* data)
{
    int count = 0;
    char c;
    while (true)
    {
        c = getcUart0();

        if (((c == 8) || (c == 127)) && (count > 0))
        {
             count = count - 1;
             continue;
        }

        if (c == 13)
        {
            data->buffer[count] = '\0';
            return;
        }

        if (c >= 32)
        {
            data->buffer[count] = c;
            count = count + 1;
            if (count == MAX_CHARS+1)
            {
                data->buffer[count] = '\0';
                return;
            }
        }


    }

}

void parseField(USER_DATA* data)
{
   int ftc = 0;
   int fpc = 0;
   data->fieldCount = 0;
   while(true)
   {
       // Initial check for characters
       if (ftc == 0)
       {
           if (((data->buffer[ftc] >= 65) && (data->buffer[ftc] <= 90)) || ((data->buffer[ftc] >= 97) && (data->buffer[ftc] <= 122)))
           {
               data->fieldType[fpc] = 'a';
               data->fieldPosition[fpc] = ftc;
               fpc = fpc + 1;
               data->fieldCount = fpc;
           }
           if ((data->buffer[ftc] >= 48) && (data->buffer[ftc] <= 57))
           {
               data->fieldType[fpc] = 'n';
               data->fieldPosition[fpc] = ftc;
               fpc = fpc + 1;
               data->fieldCount = fpc;
           }
           ftc = ftc + 1;
           continue;
       }

       // Checking for new alphas
       if (((data->buffer[ftc] >= 65) && (data->buffer[ftc] <= 90)) || ((data->buffer[ftc] >= 97) && (data->buffer[ftc] <= 122)))
       {
           if (!(((data->buffer[ftc-1] >= 65) && (data->buffer[ftc-1] <= 90)) || ((data->buffer[ftc-1] >= 97) && (data->buffer[ftc-1] <= 122))) || (data->fieldType[fpc-1] == 'n'))
           {
               data->fieldType[fpc] = 'a';
               data->fieldPosition[fpc] = ftc;
               data->buffer[ftc-1] = '\0';
               fpc = fpc + 1;
               ftc = ftc + 1;
               data->fieldCount = fpc;
               continue;
           }

       }

       // Checking for new numerics
       if ((data->buffer[ftc] >= 48) && (data->buffer[ftc] <= 57))
       {
           if (!((data->buffer[ftc-1] >= 48) && (data->buffer[ftc-1] <= 57)) || (data->fieldType[fpc-1] == 'a'))
           {
               data->fieldType[fpc] = 'n';
               data->fieldPosition[fpc] = ftc;
               data->buffer[ftc-1] = '\0';
               fpc = fpc + 1;
               ftc = ftc + 1;
               data->fieldCount = fpc;
               continue;
           }

       }

       // Returning
       if ((data->fieldCount == MAX_FIELDS) || (data->buffer[ftc] == '\0'))
       {
           return;
       }
       ftc = ftc + 1;
   }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)
{

    if ((fieldNumber <= MAX_FIELDS) && ((data->fieldType[fieldNumber] == 'a') || (data->fieldType[fieldNumber] == 'n')))
    {
       uint8_t str = 0;
       while(data->buffer[data->fieldPosition[fieldNumber] + str] != '\0')
       {
           FS[str] = data->buffer[data->fieldPosition[fieldNumber] + str];
           str = str + 1;
       }
       FS[str] = '\0';
       return FS;
    }
    else
    {
        FS[0] = '\0';
        return FS;
    }
}

uint32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)
{
    uint32_t FI = 0;
    char* int_val = getFieldString(data->buffer, fieldNumber);
    int gap = strlen(int_val);
    int count_up = 0;
    int count_down = gap - 1;

    if (fieldNumber <= MAX_FIELDS)
    {
        while (gap > count_up)
            {
                FI = FI + ((data->buffer[count_up + data->fieldPosition[fieldNumber]]) - 48) * pow(10,count_down);
                count_up = count_up + 1;
                count_down = count_down - 1;
            }
        return FI;
    }

    return FI;
}

bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments)
{
    char* check = getFieldString(data->buffer, data->fieldPosition[0]);

    if ((strcmp(check, strCommand) == 0) && (minArguments >= data->fieldCount - 1))
    {
        return true;
    }
    return false;
}
uint32_t getVolume()
{

    GPIO_PORTB_DATA_R |= PB4_MASK;
    while(!COMP_ACSTAT0_R)
    {

    }
    GPIO_PORTB_DATA_R &= ~PB4_MASK;
    TIMER1_TAV_R = 0;
    while(COMP_ACSTAT0_R)
    {
    }
    int timeval = TIMER1_TAV_R;
    uint32_t volume;
    volume = (timeval - 423.64)/0.3408;
    return volume;
}
float getLightPercentage()
{
    setAdc0Ss3Mux(2);

    float light_val = readAdc0Ss3();

    float light_val_percent = light_val/13;

    return light_val_percent;
}

float getMoisturePercentage()
{
    setAdc0Ss3Mux(1);

    float moisture_val = readAdc0Ss3();

    float moisture_val_percent = moisture_val/30.658;

    return moisture_val_percent;
}

float getBatteryLevel()
{
    setAdc0Ss3Mux(0);

    float battery_val = readAdc0Ss3();

    battery_val = battery_val*485.1/192512;

    return battery_val;
}

void enablePump()
{
    GPIO_PORTB_DATA_R |= PB6_MASK;
}

void disablePump()
{
    GPIO_PORTB_DATA_R &= ~PB6_MASK;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

void playBatteryLowAlert()
{
    int bat_Low_Mel_Array[MEL_LEN];
    int x;
    int y;
    for (y = 0; y < 10; y++)
    {
        for (x = 0; x < MEL_LEN; x++)
        {
            bat_Low_Mel_Array[x] = floor(440*(pow(2,(((12)%13)/12))));
            bat_Low_Mel_Array[x+1] = floor(440*(pow(2,(((13)%13)/12))));
            x = x + 1;
        }
        for (x = 0; x < MEL_LEN; x++)
        {
            TIMER2_TAILR_R = 40000000/(2*bat_Low_Mel_Array[x]);
            TIMER2_CTL_R |= TIMER_CTL_TAEN;
            waitMicrosecond(1000000/bat_Low_Mel_Array[x]);
            TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
        }
    }

}

void playWaterLowAlert()
{
    int wat_Low_Mel_Array[MEL_LEN];
    int x;
    int y;
    for (y = 0; y < 10; y++)
    {
        for (x = 0; x < MEL_LEN; x++)
        {
            wat_Low_Mel_Array[x] = floor(440*(pow(2,(0/12))));
            wat_Low_Mel_Array[x+1] = floor(440*(pow(2,(1/12))));
            x = x + 1;
        }
        for (x = 0; x < MEL_LEN; x++)
        {
            TIMER2_TAILR_R = 40000000/(2*wat_Low_Mel_Array[x]);
            TIMER2_CTL_R |= TIMER_CTL_TAEN;
            waitMicrosecond(1000000/wat_Low_Mel_Array[x]);
            TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
        }
    }

}

void timer2Isr()
{
    GPIO_PORTD_DATA_R ^= PD1_MASK;
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
}

int getCurrentSeconds()
{
    int time_of_day_val = HIB_RTCC_R;
    return time_of_day_val;
}

bool isWateringAllowed(uint32_t time_of_day, uint32_t start_time, uint32_t end_time)
{
    if ((time_of_day > start_time) && (time_of_day < end_time))
    {
        return true;
    }
    else
        return false;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    USER_DATA data;
    initHw();
    initUart0();
    initAdc0Ss3();

    setAdc0Ss3Log2AverageCount(2);

    int volumeVal;
    float percent_Light;
    float battery_Level;
    float percent_Moisture;
    uint32_t start_time;
    uint32_t end_time;
    float moisture_level;
    int light_level;

    putsUart0("Please set time of day (H M) and watering window first.\n\rThen set the saturation percentage threshold for water pumping.\n\rFinally set the level of light needed for alerts.");
    putsUart0("\n\r");

    HIB_RTCLD_R = 0;
    start_time = 86280;
    end_time = 86340;
    moisture_level = 0;
    light_level = 1000;




    while(true)
    {



       if (kbhitUart0())
       {
           getsUart0(&data);
           putsUart0(&data);
           putsUart0("\n\r");



           parseField(&data);

           uint8_t i;

           for (i = 0; i < data.fieldCount; i++)
           {
              putcUart0(data.fieldType[i]);
              putcUart0('\t');
              putsUart0(&data.buffer[data.fieldPosition[i]]);
              putsUart0("\n\r");
           }

           bool valid = false;

           if(isCommand(&data,"status", 0))
           {
               char buff[1000];


               volumeVal = getVolume();
               percent_Light = getLightPercentage();
               percent_Moisture = getMoisturePercentage();
               battery_Level = getBatteryLevel();

               sprintf(buff, "%d mL", volumeVal);

               putsUart0(buff);
               putsUart0("\n\r");

               sprintf(buff, "%.2f %%", 12.0);

               sprintf(buff, "%.2f %% Sunlight Exposure", percent_Light);

               putsUart0(buff);
               putsUart0("\n\r");

               sprintf(buff, "%.2f %%", 12.0);

               sprintf(buff, "%.2f %% Saturated Soil", percent_Moisture);

               putsUart0(buff);
               putsUart0("\n\r");

               sprintf(buff, "%.2f %%", 12.0);

               sprintf(buff, "%.2f Volts", battery_Level);

               putsUart0(buff);
               putsUart0("\n\r");


               valid = true;
           }
           if (isCommand(&data, "alert", 1))
           {

               light_level = getFieldInteger(&data, 1);

               valid = true;
           }

           if (isCommand(&data, "pump", 1))
           {
               char* on_off = getFieldString(&data, 1);
               if(strcmp(on_off, "on") == 0)
               {
                   enablePump();
                   valid = true;
               }
               if(strcmp(on_off, "off") == 0)
               {
                   disablePump();
                   valid = true;
               }

           }

           if (isCommand(&data, "time", 2))
           {
               uint32_t hour = getFieldInteger(&data, 1);
               uint32_t minutes = getFieldInteger(&data, 2);

               uint32_t seconds = (hour*60*60) + (minutes*60);

               HIB_RTCLD_R = seconds;


               valid = true;
           }

           if (isCommand(&data, "water", 4))
           {
               uint32_t hour1 = getFieldInteger(&data, 1);
               uint32_t minute1 = getFieldInteger(&data, 2);
               uint32_t hour2 = getFieldInteger(&data, 3);
               uint32_t minute2 = getFieldInteger(&data, 4);

               start_time = (hour1*60*60) + (minute1*60);
               end_time = (hour2*60*60) + (minute2*60);


               valid = true;
           }

           if (isCommand(&data, "level" , 1))
           {
               moisture_level = getFieldInteger(&data, 1);
               valid = true;
           }

           if (!valid)
           {
               putsUart0("Invalid Command\n\r");
           }
       }

       if (!kbhitUart0())
       {
           //check moisture and time to determine if pumping can occur
           percent_Moisture = getMoisturePercentage();

           uint32_t time_of_day_val = getCurrentSeconds();
           if (isWateringAllowed(time_of_day_val, start_time, end_time))
           {
               volumeVal = getVolume();
               while ((percent_Moisture < moisture_level) && (volumeVal > 20050))
               {
                   enablePump();
                   waitMicrosecond(5000000);
                   disablePump();
                   waitMicrosecond(30000000);
                   percent_Moisture = getMoisturePercentage();
                   volumeVal = getVolume();
               }

               volumeVal = getVolume();
               percent_Light = getLightPercentage();

               if ((volumeVal < 50) && (light_level > percent_Light))
               {
                   playWaterLowAlert();
               }

               // check bat level and play tune
               battery_Level = getBatteryLevel();
               if ((battery_Level < 4) && (light_level > percent_Light))
               {
                   playBatteryLowAlert();
               }

           }

       }


    }

    return 0;
}
