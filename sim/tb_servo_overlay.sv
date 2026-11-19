`timescale 1ns/1ps
module tb_servo_overlay;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,rv=0,vs=0,de=0;reg [93:0] record=0;reg [15:0] frame=10;
 wire [93:0] overlay;wire matched,updated;wire [15:0] xb,yb;
 integer checks=0;
 servo_overlay_history dut(.clk(clk),.rst_n(rst_n),.record_valid(rv),.record(record),.vs(vs),.de(de),.display_frame(frame),.overlay(overlay),.matched(matched),.updated(updated));
 servo_bcd_pair bcd(.clk(clk),.rst_n(rst_n),.start(updated),.x(12'd639),.y(12'd359),.x_bcd(xb),.y_bcd(yb));
 task put(input [15:0] id,input [11:0] centre);
  begin @(negedge clk);record={id,1'b1,1'b1,12'd1,12'd2,12'd3,12'd4,centre,12'd20,4'd1};rv=1;@(negedge clk);rv=0;repeat(4)@(negedge clk);end
 endtask
 task check(input bit pass,input string name);begin checks++;if(!pass)$fatal(1,"FAIL %s",name);end endtask
 task blank(input [15:0] id);begin @(negedge clk);de=0;frame=id;vs=1;@(negedge clk);vs=0;repeat(20)@(negedge clk);end endtask
 initial begin
  repeat(4)@(negedge clk);rst_n=1;put(10,100);put(11,200);put(12,300);blank(10);
  check(matched && overlay[27:16]==100,"older frame uses its own coordinates");
  check(xb==16'h0639 && yb==16'h0359,"decimal coordinates");
  de=1;put(14,400);check(overlay[27:16]==100,"active video holds metadata");
  blank(10);check(!matched && overlay==0,"overwritten history hides stale frame");
  put(16'hffff,500);blank(16'hffff);check(matched && overlay[27:16]==500,"frame counter final value supported");
  put(0,600);blank(0);check(matched && overlay[27:16]==600,"frame counter wrap supported");
  $display("PASS servo_overlay checks=%0d",checks);$finish;
 end
endmodule
