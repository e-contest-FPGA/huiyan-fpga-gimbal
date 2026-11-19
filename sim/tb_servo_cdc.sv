`timescale 1ns/1ps
module tb_servo_cdc;
 reg s_clk=0,d_clk=0;always #7 s_clk=~s_clk;always #11 d_clk=~d_clk;
 reg rst_n=0,send=0;reg [146:0] data=0;wire busy,valid;wire [146:0] received;
 reg [146:0] expected[0:199];integer pushed=0,popped=0,i;
 servo_cdc #(.WIDTH(147)) dut(.s_clk(s_clk),.s_rst_n(rst_n),.s_send(send),.s_data(data),.s_busy(busy),
  .d_clk(d_clk),.d_rst_n(rst_n),.d_valid(valid),.d_data(received));
 always @(posedge d_clk)if(rst_n && valid)begin
  if(popped>=pushed || received!==expected[popped])$fatal(1,"FAIL torn/duplicated/reordered mailbox index=%0d",popped);
  popped++;
 end
 initial begin
  repeat(5)@(negedge s_clk);rst_n=1;
  for(i=0;i<150;i++)begin
   while(busy)@(negedge s_clk);
   @(negedge s_clk);data={$random,$random,$random,$random,$random};expected[pushed]=data;pushed++;send=1;
   @(negedge s_clk);send=0;
   // Busy requests must neither alter held data nor create extra records.
   if(i%3==0)begin @(negedge s_clk);send=1;data=~data;@(negedge s_clk);send=0;end
  end
  repeat(25)@(negedge d_clk);
  if(popped!=150)$fatal(1,"FAIL mailbox lost data %0d",popped);
  $display("PASS servo_cdc records=%0d width=147 asynchronous_clocks=14ns/22ns",popped);$finish;
 end
endmodule
