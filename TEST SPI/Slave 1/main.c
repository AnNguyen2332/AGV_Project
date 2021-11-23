#include <16F877A.h>
#use delay (clock = 20M)
#include <stdlib.h>
#include <string.h>
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

int16 calib (int16 x, int16 max_x, int16 min_x)
{
   int16 max_y = 929;
   int16 min_y = 38;
   int16 y = min_y + (max_y - min_y)*(x - min_x)/(max_x - min_x);
   return y;
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
      int16 x1, x2, x3, x4, x5, x6, x7;
      set_adc_channel (0);
      delay_us (10);
      x1 = calib (read_adc (), 890, 39);
      
      set_adc_channel (1);
      delay_us (10);
      x2 = calib (read_adc (), 919, 39);
      
      set_adc_channel (2);
      delay_us (10);
      x3 = calib (read_adc (), 929, 40);
      
      set_adc_channel (3);
      delay_us (10);
      x4 = calib (read_adc (), 929, 42);
      
      set_adc_channel (5);
      delay_us (10);
      x5 = calib (read_adc (), 929, 41);
      
      set_adc_channel (6);
      delay_us (10);
      x6 = calib (read_adc (), 929, 39);
      
      set_adc_channel (7);
      delay_us (10);
      x7 = calib (read_adc (), 890, 38);
      
      float x = ((-3)*x1 + (-2)*x2 + (-1)*x3 + 0*x4 + 1*x5 + 2*x6 + 3*x7)*17/(x1 + x2 + x3 + x4 + x5 + x6 + x7);
      
      if (i == 1)
      {
         output_high (PIN_B7);
         spi_write (x);
         delay_ms (1000);
         i = 0;
      }
      
      printf ("CB1 = %lu\t", x1);
      printf ("CB2 = %lu\t", x2);
      printf ("CB3 = %lu\t", x3);
      printf ("CB4 = %lu\t", x4);
      printf ("CB5 = %lu\t", x5);
      printf ("CB6 = %lu\t", x6);
      printf ("CB7 = %lu\t", x7);
      printf ("Khoang cach = %f\t", x);
      printf ("\t\t\t\t\t\t");
    }
}
