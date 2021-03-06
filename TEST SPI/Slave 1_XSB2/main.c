#include <16F877A.h>
#device ADC=10
#include <stdlib.h>
#include <stdio.h>
#use delay (clock = 20000000)
#fuses HS, NOWDT, NOPROTECT, NOLVP
#use rs232 (baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8)
int i = 0;
#INT_SSP
void rcv_spi ()
{
   if (spi_data_is_in () == 1)
   {
      i = spi_read ();
   }
}

void main()
{
   enable_interrupts (INT_SSP);
   enable_interrupts (global);
   setup_spi (spi_slave|spi_l_to_h|spi_clk_div_16);
   setup_adc_ports(ALL_ANALOG);
   setup_adc(ADC_CLOCK_INTERNAL);
   
   while(TRUE)
   {
       float x [7] = {0, 0, 0, 0, 0, 0, 0};
       float xmax, max, max1, max2 = 0;
       float a, b, d, out;
       int j;
       for (j = 0; j < 7; j++)
       {
           if (j < 4) {
               set_adc_channel (j);
               delay_us (10);
               x [j] = read_adc ();
           }
               
           if (j >= 4) {
               set_adc_channel (j + 1);
               delay_us (10);
               x [j] = read_adc ();
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
           printf ("CB%d = %f\t", j, x [j]);
       }
       
       printf ("Khoang cach = %f\t", out);
       printf ("\t\t\t\t\t\t");
       delay_ms (1000);
    }
}
