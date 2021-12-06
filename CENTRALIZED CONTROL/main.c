/*
 * File:   main.c
 * Author: Nguyen Truong An
 *
 * Created on December 2, 2021, 9:44 PM
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
#include <string.h>
#include <xc.h>
#include <pic16f877a.h>
#define _XTAL_FREQ 8000000
#include "uart.h"
#include "pwm.h"

char e = 0, prev_e = 0;
char P = 0, I = 0, D = 0;
char PID_Val = 0;

float Kp = 0.8;
float Ki = 0.6;
float Kd = 0;
int v_ref = 100;
void ADC_Init() {
    ADCON0 = 0b1000001;
    ADCON1 = 0b11000000;
}

unsigned int ADC_Read(unsigned char channel) {
    ADCON0 &= 0x11000101;
    ADCON0 |= channel << 3;
    __delay_ms(2);
    GO_nDONE = 1;
    while (GO_nDONE);
    return ((ADRESH << 8) + ADRESL);
}

void AVG_Index() {
    unsigned char x0, x1, x2, x3, x4;
    char uart_logs[50];

    x0 = ADC_Read(0);
    x1 = ADC_Read(1);
    x2 = ADC_Read(2);
    x3 = ADC_Read(3);
    x4 = ADC_Read(4);

    e = 17 * ((-2) * x0 + (-1) * x1 + 0 * x2 + 1 * x3 + 2 * x4) / (x0 + x1 + x2 + x3 + x4);

    sprintf(uart_logs, "%d %d %d %d %d\n\r", x0, x1, x2, x3, x4);
    UART_Write_Text(uart_logs);
    __delay_ms(50);
    sprintf(uart_logs, "e = %d\n\r", e);
    UART_Write_Text(uart_logs);
    __delay_ms(50);
}

/*void Lyapunov(float e2, float e3) {
    float v, omega;
    e3 = 0;
    v = v_ref * cos(e3);
    omega = k2 * v_ref * e2 + omega_ref + k3 * sin(e3);

    v_r = (int) (2 * v + omega * wheel_distance) / 2;
    v_l = (int) 2 * v - v_r;
}*/

void PID() {
    char uart_logs[50];
    P = e;
    I = I + e;
    D = e - prev_e;
    PID_Val = Kp * P + Ki * I + Kd*D;
    prev_e = e;
    sprintf(uart_logs, "PID_Val = %d\n\r", PID_Val);
    UART_Write_Text(uart_logs);
}

void main() {
    TRISA = 1; //PORTA as input
    TRISD = 0; //PORTD as output

    PWM1_Init_Fre(1000);
    PWM2_Init_Fre(1000);

    PORTDbits.RD1 = 0;
    PORTDbits.RD2 = 1;
    PORTDbits.RD3 = 0;
    PORTDbits.RD4 = 1;

    ADC_Init();
    UART_Init(9600);

    while (1) {
        AVG_Index();
        PID();
        PWM1_Duty(v_ref - PID_Val);
        PWM2_Duty(v_ref + PID_Val);
        PWM1_Start();
        PWM2_Start();
        __delay_ms (500);
    }
}