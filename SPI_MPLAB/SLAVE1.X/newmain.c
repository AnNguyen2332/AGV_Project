/*
 * File:   newmain.c
 * Author: Nguyen Truong An
 *
 * Created on November 20, 2021, 7:47 PM
 */


#pragma config FOSC = XT   // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF  // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF   // Low-Voltage In-Circuit Serial Programming Enable bit
#pragma config CPD = OFF   // Data EEPROM Memory Code Protection bit 
#pragma config WRT = OFF   // Flash Program Memory Write Enable bits 
#pragma config CP = OFF    // Flash Program Memory Code Protection bit

#include <xc.h>
#include <pic16f877a.h>
#include <math.h>
#include "spi.h"

#define _XTAL_FREQ 8000000
#include "uart.h"

void ADC_Init()
{
   ADCON0 = 0b1000001;
   ADCON1 = 0b11000000;
}

unsigned int ADC_Read(unsigned char channel)
{
  ADCON0 &= 0x11000101; 
  ADCON0 |= channel<<3; 
  __delay_ms(2); 
  GO_nDONE = 1; 
  while(GO_nDONE); 
  return ((ADRESH<<8)+ADRESL); 
}

float Quadra_Al ()
{
    float x [7] = {0, 0, 0, 0, 0, 0, 0};
    float max, max1, max2 = 0;
    float a, b, d, out;
    int j, xmax;
    
    for (j = 0; j < 7; j++)
    {
        if (j < 4) {
            x [j] = ADC_Read (j);
        }

        if (j >= 4) {
            x [j] = ADC_Read (j + 1);
        }
    }

    for (j = 0; j < 7; j++)
    {
        if (x [j] > max) {
            max = x [j];
            xmax = j;
        }
    }

    max1 = x [xmax - 1];
    max2 = x [xmax + 1];

    a = (max1 + max2 - 2*max)*0.5;
    b = max - max1 - 2*a*(xmax - 1) - a;
    d = -b/(2*a);
    out = 17*(d - 3);

    for (j = 0; j < 7; j++)
    {
        UART_Write_Text ("CB");
        UART_Write (j);
        UART_Write_Text (" = ");
        UART_Write (x [j]);
        __delay_ms(90);
        UART_Write_Text ("\n\r");
    }

    UART_Write_Text ("Khoang cach = ");
    UART_Write (out);
    __delay_ms(90);
    UART_Write_Text ("\n\r");
    return out;
}

void main()
{
   TRISA = 1;
   TRISD = 0;                 //PORTD as output
   PORTD = 0;                 //All LEDs OFF
   
   GIE = 1;
   PEIE = 1;
   SSPIF = 0;
   SSPIE = 1;
   ADCON1 = 0x07;
   
   ADC_Init ();
   UART_Init (9600);
   spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);
   while(1)
   {
       if(spiDataReady())
       {   
           float data;
           if (spiRead ())
           {
               RD0 = 1;
               data = Quadra_Al ();
               spiWrite (data);
               __delay_ms (1000);
           }
           RD0 = 0;
           SSPIF = 0;
       }
       __delay_ms(5);
   }
}