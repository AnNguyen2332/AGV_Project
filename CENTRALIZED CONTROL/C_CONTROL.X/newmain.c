/*
 * File:   newmain.c
 * Author: Nguyen Truong An
 *
 * Created on December 3, 2021, 10:29 AM
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
#include <string.h>
#include <math.h>
#include <xc.h>
#include <pic16f877a.h>
#define _XTAL_FREQ 20000000
#include "uart.h"
#include "pwm.h"

char e = 0, prev_e = 0;
char P = 0, I = 0, D = 0;

float Kp = 0.8;
float Ki = 0.6;
float Kd = 0;

int v_l = 0, v_r = 0;
int prev_vl = 0, prev_vr = 0;
float k2 = 0.0005;
float k3 = 0.01;
float v_ref = 1800;
float omega_ref = 3.6;
int wheel_distance = 163;
char en_l = 0, en_r = 0;

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
    UART_Write_Text("Text");
}

void Lyapunov(float e2) {
    float v, omega;
    v = v_ref * cos(0);
    omega = k2 * v_ref * e2 + omega_ref + k3 * sin(0);

    v_r = (int) (2 * v + omega * wheel_distance) / 2;
    v_l = (int) 2 * v - v_r;
}
void readEncoder (){
    char uart_logs[50];
    en_l = ADC_Read (5) - en_l;
    en_r = ADC_Read (6) - en_r;
    sprintf(uart_logs, "En_left = %d\n\r", en_l);
    UART_Write_Text(uart_logs);
    __delay_ms (50);
    sprintf(uart_logs, "En_right = %d\n\r", en_r);
    UART_Write_Text(uart_logs);  
}
char PID (int v, int prev_v) {
    char PID_Val = 0;
    P = v;
    I = I + v;
    D = v - prev_v;
    PID_Val = Kp * P + Ki * I + Kd*D;
    return PID_Val;
}

void main() {
    TRISA = 1; //PORTA as input
    TRISE = 1;
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
        char PID_l, PID_r;
        AVG_Index();
        /*Lyapunov (e);
        readEncoder ();
        PID_l = PID(v_l, prev_vl);
        PID_r = PID(v_r, prev_vr);
        PWM1_Duty(v_ref - PID_l);
        PWM2_Duty(v_ref + PID_l);
        PWM1_Start();
        PWM2_Start();
        prev_vl = v_l;
        prev_vl = v_r;*/
        __delay_ms (500);
    }
}