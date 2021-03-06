#include <16F877A.h>
#use delay(clock = 20000000)
#include <stdlib.h>
#include <string.h>
#use I2C (master, sda = PIN_C4, scl = PIN_C3, address = 0xa0, FORCE_HW)
#use rs232 (baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8)
int i;
#INT_SSP
void rcv_i2c ()
{
   if (i2c_poll () == TRUE)
   {
      i = i2c_read ();
   }
}
void main()
{
   enable_interrupts (INT_SSP);
   enable_interrupts (global);
   while(TRUE)
   {
      if (input (PIN_B7) == 0)
      {
         i2c_start ();
         i2c_write (0xa1);
         i2c_write (1);
         i2c_stop ();
      }
      
      if (i == 1)
      {
         output_high (PIN_B0);
         delay_ms (1000);
         output_low (PIN_B0);
      }
   }

}
