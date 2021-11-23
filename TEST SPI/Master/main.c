#include <16F877a.h>
#device ADC=10
#fuses HS, NOWDT, NOPROTECT, NOLVP
#use delay (clock = 20M)
#include <stdlib.h>
#include <string.h>
#use rs232 (baud = 9600, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8)
int i = 0;

//Hàm nhận SPI viết theo sách thầy Hạnh :)))))
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
   setup_spi (spi_master|spi_l_to_h|spi_clk_div_16);
   enable_interrupts (INT_SSP);
   enable_interrupts (global);
   while(TRUE)
   {  //Gửi thông báo muốn nhận thông tin cho Slave 1
      delay_ms (1000);
      output_low (PIN_A0);     //Kết nối Slave 1
      output_high (PIN_A1);    //Đóng kết nối Slave 2
      delay_ms (500);
      spi_write (1);           //Gửi thông báo
      delay_ms (500);
      output_high (PIN_A0);    //Đóng kết nối cả 2 slave
      output_high (PIN_A1);
      
      //Nếu đã nhận được tín hiệu từ Slave 1, bắt đầu điều khiển Slave 2
      if (i == 1)
      {
         output_low (PIN_A1);  //Kết nối Slave 2
         output_high (PIN_A0); //Đóng kết nối Slave 1
         delay_ms (1000);
         spi_write (1);        //Gửi tín hiệu điều khiển cho Slave 2
         delay_ms (1000);
         output_high (PIN_A0); //Đóng kết nối cả 2 slave
         output_high (PIN_A1);
      }
      i = 0;
   }
}
