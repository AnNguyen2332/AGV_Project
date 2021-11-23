/*
 * File:    newmain.c
 * Author: NguyenTruongAn
 *
 * Created on November 20, 2021, 6:00 PM
 */
#pragma config FOSC = XT   // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF  // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF   // Low-Voltage In-Circuit Serial Programming Enable bit
#pragma config CPD = OFF   // Data EEPROM Memory Code Protection bit 
#pragma config WRT = OFF   // Flash Program Memory Write Enable bits 
#pragma config CP = OFF    // Flash Program Memory Code Protection bit

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <pic16f877a.h>
#define _XTAL_FREQ 8000000
#include "uart.h"
#include "spi.h"


void main()
{
   nRBPU = 0;                   //Enable PORTB internal pull up resistor
   TRISA = 0;
   TRISB = 1;                   //PORTB as input
   TRISD = 0;                   //PORTD as output
   PORTD = 0;
   RA1 = 1;
   RA2 = 1;
   UART_Init (9600);
   
   spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);
   
   while(1)
   {    
       int s1 = 0;
       int s2 = 0;
      
       if (RB0 == 0)
       {
           RA1 = 0;       //Slave 1 Select
           __delay_ms(1);
           
           spiWrite(1);
           s1 = spiRead();
           UART_Write_Text ("s1 = ");
           UART_Write(s1);
           __delay_ms(90);
           UART_Write_Text ("\n\r");
           
           __delay_ms(1);
           RA1 = 1;       //Slave 1 Deselect 
       }
       
       if (s1 == 1)
       {
           RD0 = 1;
           RA2 = 0;       //Slave 2 Select
           __delay_ms(1);

           spiWrite(1);
           s2 = spiRead();
           UART_Write_Text ("s2 = ");
           UART_Write (s2);
           __delay_ms(90);
           UART_Write_Text ("\n\r");
           
           __delay_ms(1);
           RA2 = 1;       //Slave 2 Deselect 
       }
       
       if (s2 == 1)
       {
           RD0 = 0;
           RD1 = 1;
           __delay_ms (1000);
           RD1 = 0;
       }
   }
}