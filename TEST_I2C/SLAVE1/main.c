#include <16F877A.h>
#device ADC=10
#fuses HS, NOWDT, NOPROTECT, NOLVP
#use delay (clock = 20M)
#use i2c (SLAVE, SDA = PIN_C4, SCL = PIN_C3, address = 0xa0, force_hw)
#use rs232 (baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8)
byte i;
#INT_SSP
void rcv_i2c ()
{
   byte state, address, incoming;
   state = i2c_isr_state();
   if(state < 0x80)
   {
      incoming = i2c_read();
      if (state == 0)
      {
         address = incoming;
      }
      
      if (state == 1)
      {
         i = incoming;
      }
   }
   if(state == 0x80) i2c_write(1);
}

/*void send_i2c (int data, int8 slv_add)
{
   i2c_start();
   i2c_write(slv_add);
   i2c_write(data);
   i2c_stop();
}*/

/*int8 Rece_data(int8 slave_add)
{
   int8 value;
   i2c_start();//khoi tao lenh khi o che do master
   i2c_write(slave_add + 1);//gui dia chi slave giao tiep
   value = i2c_read(0);///doc du lieu
   i2c_stop();
   return value;
}*/
void main()
{
   ENABLE_INTERRUPTS(GLOBAL);
   ENABLE_INTERRUPTS(INT_SSP);
   while(TRUE)
   {
      if (i == 0x01)
      {
         output_high (PIN_B0);
         delay_ms (500);
         output_low (PIN_B0);
      }
      
      /*if (i == 1)
      {
         output_high (PIN_B0);
         delay_ms (1000);
         output_low (PIN_B0);
      }*/
   }
}
