#include <16F877A.h>
#use delay(clock = 20M)
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
void main()
{
   enable_interrupts (INT_SSP);
   enable_interrupts (global);
   setup_spi (spi_slave|spi_l_to_h|spi_clk_div_16);
   
   while(TRUE)
   {
      if (i == 1)
      {
         output_high (PIN_B7);
         delay_ms (1000);
      }
      i = 0;
      output_low (PIN_B7);
   }

}
