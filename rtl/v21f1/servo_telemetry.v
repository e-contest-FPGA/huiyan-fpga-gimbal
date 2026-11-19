// Fixed 80-byte packet: 78-byte big-endian payload + CRC16-CCITT-FALSE.
// Never stalls control/video. Dropped snapshots are counted explicitly.
module servo_telemetry #(parameter CLK_HZ=75000000,BAUD=115200)(
 input wire clk,rst_n,send,input wire [623:0] payload,
 output reg tx,output wire busy,output reg [15:0] dropped
);
 localparam DIV=CLK_HZ/BAUD;
 reg [623:0] shift;
 reg [15:0] crc;
 reg [6:0] index;
 reg active;
 reg [9:0] serial;
 reg [3:0] bits_left;
 reg [$clog2(DIV+1)-1:0] divider;
 reg [7:0] next_byte;
 assign busy=active || bits_left!=0;
 function [15:0] update_crc;
  input [15:0] previous;input [7:0] data;reg [15:0] c;integer j;
  begin c=previous^{data,8'd0};for(j=0;j<8;j=j+1)c=c[15]?(c<<1)^16'h1021:c<<1;update_crc=c;end
 endfunction
 always @*begin
  next_byte=index<78?shift[623-index*8-:8]:index==78?crc[15:8]:crc[7:0];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin shift<=0;crc<=16'hffff;index<=0;active<=0;serial<=10'h3ff;bits_left<=0;divider<=0;tx<=1;dropped<=0;end
  else begin
   if(send)begin
    if(!busy)begin shift<=payload;crc<=16'hffff;index<=0;active<=1;end
    else if(dropped!=16'hffff)dropped<=dropped+1'b1;
   end
   if(bits_left!=0)begin
    if(divider==DIV-1)begin
     divider<=0;serial<={1'b1,serial[9:1]};bits_left<=bits_left-1'b1;
     tx<=bits_left==1?1'b1:serial[1];
    end else divider<=divider+1'b1;
   end else if(active)begin
    serial<={1'b1,next_byte,1'b0};tx<=0;bits_left<=10;divider<=0;
    if(index<78)crc<=update_crc(crc,next_byte);
    if(index==79)active<=0;else index<=index+1'b1;
   end else tx<=1;
  end
 end
endmodule
