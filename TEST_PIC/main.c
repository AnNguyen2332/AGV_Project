#include <16F877a.h>
#use delay(clock=20000000)

void main()
{

   while(TRUE)
   {
      output_low (PIN_D4);
      delay_ms (1000);
      output_high (PIN_D4);
      delay_ms (1000);
   }

}
