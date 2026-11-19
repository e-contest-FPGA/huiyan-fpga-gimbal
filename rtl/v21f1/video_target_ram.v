module video_target_ram #(
 parameter integer WIDTH=9,DEPTH=3600,
 parameter integer AW=(DEPTH>1)?$clog2(DEPTH):1
)(
 input wire clk,read_enable,write_enable,
 input wire [AW-1:0] read_address,write_address,
 input wire [WIDTH-1:0] write_data,
 output reg [WIDTH-1:0] read_data
);
 reg [WIDTH-1:0] memory[0:DEPTH-1];
 always @(posedge clk)begin
  if(read_enable)read_data<=memory[read_address];
  if(write_enable)memory[write_address]<=write_data;
 end
endmodule
