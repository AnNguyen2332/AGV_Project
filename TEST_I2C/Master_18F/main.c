#include <18F4431.h>
#device ADC=10
#fuses HS, NOWDT, NOPROTECT, NOLVP
#use delay (clock = 20M)
#use i2c (MASTER, SDA = PIN_C4, SCL = PIN_C5, force_hw)
#use rs232 (baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8)
byte slv_add1 = 0xa0;
byte slv_add2 = 0xa1;
byte i;
#INT_SSP
void rcv_i2c (void)
{
   i2c_start();
   i2c_write (slv_add1+1);
   i = i2c_read (0);
   i2c_stop();
}

void send_i2c (byte data, byte slv_add)
{
   i2c_start();
   i2c_write (slv_add);
   i2c_write (data);
   i2c_stop ();
}

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
      if (input (PIN_B7) == 0)
      {
         send_i2c (0x01, slv_add1);
         delay_ms (500);
      }
      
      if (i == 1)
      {
         output_high (PIN_B0);
         delay_ms (1000);
         output_low (PIN_B0);
      }
   }
}
