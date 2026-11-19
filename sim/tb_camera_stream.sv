`timescale 1ns/1ps
module tb_camera_stream;
 parameter W=160,H=96;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,sof=0,valid=0,eol=0,error=0;reg [95:0] data=0;
 wire fs,fe,sv,bad;wire [23:0] rgb;wire [11:0] x,y;wire [15:0] id,bad_frames;
 integer samples=0,ended=0,checks=0,last_x=-4,last_y=0,pattern=0;
 camera_analysis_stream #(.WIDTH(W),.HEIGHT(H)) dut(
  .clk(clk),.rst_n(rst_n),.sof(sof),.valid(valid),.eol(eol),.rgb4(data),.rx_error(error),
  .frame_start(fs),.frame_end(fe),.sample_valid(sv),.frame_bad(bad),.rgb(rgb),.x(x),.y(y),.frame_id(id),.bad_frames(bad_frames));
 always @(posedge clk)if(rst_n)begin
  if(fs)begin samples=0;last_x=-4;last_y=0;end
  if(sv)begin
   samples++;
   if(pattern==0)begin
    if(x!=(last_x==W-4?0:last_x+4) || y!=(last_x==W-4?last_y+4:last_y))$fatal(1,"FAIL sample coordinates %0d,%0d",x,y);
    if(rgb!==24'h123456)$fatal(1,"FAIL RGB pixel ordering");
    last_x=x;last_y=y;
   end
  end
  if(fe)begin
   ended++;
   if(pattern==0 && (bad || samples!=(W/4)*(H/4)))$fatal(1,"FAIL complete frame samples=%0d bad=%b",samples,bad);
   if(pattern!=0 && !bad)$fatal(1,"FAIL malformed frame not rejected");
  end
 end
 task frame(input integer mode);
  integer xx,yy;
  begin
   pattern=mode;
   for(yy=0;yy<H;yy++)for(xx=0;xx<((mode==1 && yy==0)?W/2:W);xx+=4)begin
    @(negedge clk);sof=xx==0 && yy==0;valid=1;eol=(mode==1 && yy==0)?xx==W/2-4:xx==W-4;
    data={24'h123456,24'habcdef,24'h000000,24'hffffff};error=mode==2 && yy==4 && xx==4;
    if(xx%12==0)begin @(negedge clk);valid=0;sof=0;eol=0;error=0;end
   end
   @(negedge clk);valid=0;sof=0;eol=0;error=0;repeat(8)@(negedge clk);
   checks++;
  end
 endtask
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  frame(0);if(ended!=1 || id!=1 || bad_frames!=0)$fatal(1,"FAIL native stream identity");
  frame(1);if(bad_frames!=1)$fatal(1,"FAIL width error counter");
  frame(2);if(bad_frames!=2)$fatal(1,"FAIL receiver error counter");
  frame(0);if(bad_frames!=2 || id!=4)$fatal(1,"FAIL clean recovery");
  $display("PASS camera_stream W=%0d H=%0d checks=%0d sample_pixels_per_good_frame=%0d",W,H,checks,(W/4)*(H/4));$finish;
 end
endmodule
