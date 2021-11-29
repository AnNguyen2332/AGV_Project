/*
 * File:    newmain.c
 * Author: Nguyen Truong An & Dang Linh Anh
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
#include <math.h>
#include <xc.h>
#include <pic16f877a.h>
#define _XTAL_FREQ 8000000
#include "uart.h"
#include "spi.h"

int v_l, v_r;
float k1 = 0.01;
float k2 = 0.0005;
float k3 = 0.01;
float v_ref = 1800;
float omega_ref = 3.6;
int wheel_distance = 170;

void computeLyapunov(float e2, float e3) {
    float v, omega;
    v = v_ref * cos(e3);
    omega = k2 * v_ref * e2 + omega_ref + k3 * sin(e3);

    v_r = (int) (2 * v + omega * wheel_distance) / 2;
    v_l = (int) 2 * v - v_r;
}

void main() {
    nRBPU = 0;                     //Enable PORTB internal pull up resistor
    TRISA = 0;
    //TRISB = 1;                   //PORTB as input
    TRISD = 0; //PORTD as output
    PORTD = 0;
    RA1 = 1;
    RA2 = 1;
    UART_Init(9600);

    spiInit(SPI_MASTER_OSC_DIV4, SPI_DATA_SAMPLE_MIDDLE, SPI_CLOCK_IDLE_LOW, SPI_IDLE_2_ACTIVE);

    while (1) {
        int e2 = 0;
        int e3 = 0;
        char out [10];
        __delay_ms(300);

        //Receive e2 from Slave 1
        RA1 = 0;                    //Slave 1 Select
        __delay_ms(1);

        spiWrite(1);
        __delay_ms(300);
        e2 = spiRead();

        sprintf(out, "e2 = %d\n\r", e2);
        __delay_ms(1);
        UART_Write_Text(out);

        __delay_ms(1);
        RA1 = 1;                    //Slave 1 Deselect

        //Receive e3 from Slave 2
        RD0 = 1;
        RA2 = 0;                    //Slave 2 Select
        __delay_ms(1);

        spiWrite(1);
        __delay_ms(300);
        e3 = spiRead();
        sprintf(out, "e3 = %d\n\r", e3);
        __delay_ms(1);
        UART_Write_Text(out);

        //Compute Lyapunov and send v_left, v_right to Slave 2
        computeLyapunov(e2, e3);
        spiWrite(v_l);
        __delay_ms(10);
        spiWrite(v_r);

        __delay_ms(1);
        RA2 = 1;                    //Slave 2 Deselect 

        RD0 = 0;
        RD1 = 1;
        __delay_ms(500);
        RD1 = 0;
    }
}