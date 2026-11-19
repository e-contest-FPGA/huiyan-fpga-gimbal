`timescale 1ns/1ps
module tb_servo_telemetry;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,send=0;reg [623:0] payload=0;wire tx,busy;wire [15:0] dropped;
 reg [7:0] received[0:79];integer i,j,raw_file;reg [15:0] crc;
 servo_telemetry #(.CLK_HZ(1000000),.BAUD(100000)) dut(.clk(clk),.rst_n(rst_n),.send(send),.payload(payload),.tx(tx),.busy(busy),.dropped(dropped));
 task receive_byte(output [7:0] value);
  integer bit_index;
  begin
   @(negedge tx);repeat(15)@(posedge clk);
   for(bit_index=0;bit_index<8;bit_index++)begin value[bit_index]=tx;repeat(10)@(posedge clk);end
   if(tx!==1)$fatal(1,"FAIL UART stop bit");
  end
 endtask
 initial begin
  for(i=0;i<78;i++)payload[623-i*8-:8]=i^8'h5a;
  payload[623-:32]=32'h53563231;
  payload[623-4*8-:8]=1;payload[623-5*8-:8]=3;
  payload[623-6*8-:8]=2;payload[623-63*8-:8]=1;
  repeat(5)@(negedge clk);rst_n=1;send=1;@(negedge clk);send=0;
  fork
   begin for(integer k=0;k<80;k++)receive_byte(received[k]);end
   begin repeat(100)@(negedge clk);send=1;@(negedge clk);send=0;end
  join
  crc=16'hffff;
  for(i=0;i<78;i++)begin
   if(received[i]!==payload[623-i*8-:8])$fatal(1,"FAIL telemetry payload byte %0d",i);
   crc=crc^{received[i],8'd0};for(j=0;j<8;j++)crc=crc[15]?(crc<<1)^16'h1021:crc<<1;
  end
  if({received[78],received[79]}!==crc || dropped!=1)$fatal(1,"FAIL telemetry CRC/drop accounting");
  raw_file=$fopen("rtl_uart_synthetic.bin","wb");
  for(i=0;i<80;i++)$fwrite(raw_file,"%c",received[i]);
  $fclose(raw_file);
  $display("PASS servo_telemetry bytes=80 CRC=%h dropped=%0d",crc,dropped);$finish;
 end
 initial begin repeat(12000)@(negedge clk);$fatal(1,"FAIL UART timeout");end
endmodule
