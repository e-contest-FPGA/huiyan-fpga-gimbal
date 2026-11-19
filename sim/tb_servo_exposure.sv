`timescale 1ns/1ps
module tb_servo_exposure;
 localparam W=160,H=96;
 reg clk=0,ctrl=0;always #5 clk=~clk;always #13 ctrl=~ctrl;
 reg rst_n=0,fs=0,fe=0,bad=0,sv=0,target_priority=1,observed=1,motion=0,measure=0;
 reg [1:0] quality=2;reg [11:0] x=0,y=0;reg [23:0] rgb=0;
 wire active,limited,req,global_req;wire [15:0] ae,ag,global_ae,global_ag,drops;
 reg cfg_done=1,global_done=1;integer busy=0,gbusy=0,checks=0;
 servo_exposure #(.WIDTH(W),.HEIGHT(H),.SETTLE_FRAMES(0)) dut(
  .clk(clk),.rst_n(rst_n),.ctrl_clk(ctrl),.ctrl_rst_n(rst_n),.frame_start(fs),.frame_end(fe),.frame_bad(bad),
  .sample_valid(sv),.sample_x(x),.sample_y(y),.rgb(rgb),.target_enable(target_priority),.observed(observed),.quality(quality),
  .x0(12'd48),.y0(12'd32),.x1(12'd79),.y1(12'd63),.motion_fast(motion),.measurement(measure),
  .cam_cfg_done(1'b1),.ae_cfg_done(cfg_done),.target_active(active),.motion_limited(limited),.ae_req(req),.ae(ae),.ag(ag),.commands_dropped(drops));
 servo_exposure #(.WIDTH(W),.HEIGHT(H),.SETTLE_FRAMES(0)) reference(
  .clk(clk),.rst_n(rst_n),.ctrl_clk(ctrl),.ctrl_rst_n(rst_n),.frame_start(fs),.frame_end(fe),.frame_bad(bad),
  .sample_valid(sv),.sample_x(x),.sample_y(y),.rgb(rgb),.target_enable(1'b0),.observed(observed),.quality(quality),
  .x0(12'd48),.y0(12'd32),.x1(12'd79),.y1(12'd63),.motion_fast(1'b0),.measurement(1'b0),
  .cam_cfg_done(1'b1),.ae_cfg_done(global_done),.ae_req(global_req),.ae(global_ae),.ag(global_ag));
 always @(posedge ctrl)begin
  if(req)begin if(!cfg_done)$fatal(1,"FAIL concurrent I2C request");busy<=4;cfg_done<=0;end
  else if(busy!=0)begin busy<=busy-1;if(busy==1)cfg_done<=1;end
  if(global_req)begin gbusy<=4;global_done<=0;end
  else if(gbusy!=0)begin gbusy<=gbusy-1;if(gbusy==1)global_done<=1;end
 end
 task check(input bit ok,input string message);begin checks++;if(!ok)$fatal(1,"FAIL %s AE=%0d AG=%0d",message,ae,ag);end endtask
 task frame(input bit bright_target,input bit invalid);
  integer xx,yy;
  begin
   @(negedge clk);fs=1;@(negedge clk);fs=0;
   for(yy=0;yy<H;yy+=4)for(xx=0;xx<W;xx+=4)begin
    @(negedge clk);sv=1;x=xx;y=yy;
    rgb=(xx>=48 && xx<80 && yy>=32 && yy<64)?(bright_target?24'hececec:24'h282828):24'hf5f5f5;
   end
   @(negedge clk);sv=0;fe=1;bad=invalid;@(negedge clk);fe=0;bad=0;repeat(100)@(negedge clk);
  end
 endtask
 reg [15:0] old_ae,old_ag;
 initial begin
  repeat(5)@(negedge clk);rst_n=1;
  frame(0,0);check(!active,"ROI first frame global");frame(0,0);check(!active,"ROI second frame global");frame(0,0);check(active,"three good frames acquire ROI");
  repeat(12)frame(0,0);
  check(ae>global_ae || ag>global_ag,"target exposure survives bright background");
  $display("SYNTHETIC comparison target=%0d/%0d global=%0d/%0d",ae,ag,global_ae,global_ag);
  motion=1;frame(0,0);check(limited && ae<=600,"motion exposure cap applied");
  measure=1;old_ae=ae;old_ag=ag;frame(1,0);check(ae==old_ae && ag==old_ag,"optical test freezes sensor feedback");measure=0;
  quality=0;observed=0;old_ae=ae;old_ag=ag;frame(1,0);check(ae==old_ae && ag==old_ag,"ambiguous target freezes ROI command");
  repeat(7)frame(1,0);check(!active,"eight misses return global");
  old_ae=ae;old_ag=ag;frame(1,1);check(ae==old_ae && ag==old_ag,"malformed frame sends no exposure command");
  check(ae>=8 && ae<=1990 && ag>=16 && ag<=80 && drops==0,"sensor bounds and CDC delivery");
  $display("PASS servo_exposure checks=%0d",checks);$finish;
 end
endmodule
