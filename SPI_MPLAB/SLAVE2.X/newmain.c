/*
 * File:   newmain.c
 * Author: Nguyen Truong An & Dang Linh Anh
 *
 * Created on November 20, 2021, 9:09 PM
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <pic16f877a.h>
#define _XTAL_FREQ 8000000

#include "spi.h"
#include "uart.h"
#include "pwm.h"

void main() {
    TRISD = 0; //PORTD as output
    PORTD = 0; //All LEDs OFF

    GIE = 1;
    PEIE = 1;
    SSPIF = 0;
    SSPIE = 1;
    ADCON1 = 0x07;
    TRISA5 = 1;
    UART_Init(9600);
    spiInit(SPI_SLAVE_SS_EN, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);
    PWM2_Init_Fre(1000); // tan so
    PWM1_Init_Fre(1000); // tan so
    PORTDbits.RD1 = 0; //0
    PORTDbits.RD2 = 1;
    PORTDbits.RD3 = 0; //0
    PORTDbits.RD4 = 1;
    while (1) {
        PWM2_Duty(255);
        PWM1_Duty(50);
        PWM2_Start();
        PWM1_Start();

        if (SSPIF == 1) {
            char uart_logs [20];
            //int v_l, v_r;
            
            int PID_Val = 0;
            PID_Val = spiRead();
            RD0 = 1;

            sprintf(uart_logs, "PID_Val = %d\n\r", PID_Val);
            __delay_ms(50);
            UART_Write_Text(uart_logs);
            /*if (out == 1)
            {
                RD0 = 1;
                __delay_ms (300);
                spiWrite (1);
                SSPBUF = 0;
                __delay_ms (5);
                v_l = spiRead ();
                SSPBUF = 0;
                __delay_ms (10);
                v_r = spiRead ();
                
                sprintf(uart_logs, "v_left = %d\n\r", v_l);
                __delay_ms (1);
                UART_Write_Text(uart_logs);
                sprintf(uart_logs, "v_right = %d\n\r", v_r);
                __delay_ms (1);
                UART_Write_Text(uart_logs);
                __delay_ms (100);
            }*/
            RD0 = 0;
            SSPIF = 0;
        }
        __delay_ms(5);
    }
}