#include <16F877A.h>
#device ADC=10
#use delay(clock=20000000)
#fuses HS, NOWDT, NOPROTECT, NOLVP
#use rs232 (baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8)
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

float e_env = 0, prev_e = 0;

int16 v_ref = 150;
char P = 0, I = 0, D = 0;
int8 PID_Val = 0;
float Kp = 8;
float Ki = 0;
float Kd = 6;

int16 Calib (INT16 x)
{
   if (x > 650) RETURN 650;
   else if (x < 50) RETURN 50;
   else RETURN x;
}

int16 AVG_Index(boolean init) 
{
   INT16 x0, x1, x2, x3, x4, x5, x6;
   INT16 e;
   set_adc_channel (0);
   delay_us (10);
   x0 = Calib (read_adc ());
   
   set_adc_channel (1);
   delay_us (10);
   x1 = Calib (read_adc ());
   
   set_adc_channel (2);
   delay_us (10);
   x2 = Calib (read_adc ());
   
   set_adc_channel (3);
   delay_us (10);
   x3 = Calib (read_adc ());
   
   set_adc_channel (4);
   delay_us (10);
   x4 = Calib (read_adc ());
   
   set_adc_channel (5);
   delay_us (10);
   x5 = Calib (read_adc ());
   
   set_adc_channel (6);
   delay_us (10);
   x6 = Calib (read_adc ());
   
   e = 17 * (( - 3) * x0 + ( - 2) * x1 + (-1) * x2 + 0 * x3 + 1 * x4 + 2 * x5 + 3 * x6) / (x0 + x1 + x2 + x3 + x4 + x5 + x6) ;
   
   IF (init) printf ("e_env = %lu\t", e);

   ELSE
   {
      printf ("CB1 = %lu", x0);
      printf ("\t");
      printf ("CB2 = %lu", x1);
      printf ("\t");
      printf ("CB3 = %lu", x2);
      printf ("\t");
      printf ("CB4 = %lu", x3);
      printf ("\t");
      printf ("CB5 = %lu", x4);
      printf ("\t");
      printf ("CB6 = %lu", x5);
      printf ("\t");
      printf ("CB7 = %lu", x6);
      printf ("\t");
      printf ("Khoang cach = %lu", e);
      printf ("\t");
   }

   
   RETURN e;
}

void PID (INT e) 
{
   P = e;
   I = I + e;
   D = e - prev_e;
   PID_Val = Kp * P + Ki * I + Kd*D;
   prev_e = e;
}

void main()
{
   setup_adc_ports (ALL_ANALOG);
   setup_adc (ADC_CLOCK_INTERNAL);
   
   setup_ccp1 (CCP_PWM);
   setup_ccp2 (CCP_PWM);
   setup_timer_2 (T2_DIV_BY_1, 249, 1) ;
   
   delay_ms (500);
   output_high (PIN_D4);
   e_env = AVG_Index (TRUE);
   delay_ms (1000);
   output_low (PIN_D4);
   output_high (PIN_D0);
   output_low (PIN_D1);
   
   output_high (PIN_D2);
   output_low (PIN_D3);
   set_pwm1_duty (v_ref);
   set_pwm2_duty (v_ref);
   WHILE (TRUE)
   {
      output_high (PIN_D5);
      INT16 e = 0;
      e = AVG_Index (FALSE);
      PID (e);
      
      set_pwm1_duty (v_ref - PID_val);
      set_pwm2_duty (v_ref + PID_val);
   }
}

