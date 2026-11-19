`timescale 1ns/1ps
module tb_servo_background;
 localparam W=320,H=192;
 reg clk=0,rst_n=0,enable=1,acquire=0,fs=0,fe=0,sv=0;
 always #5 clk=~clk;
 reg [11:0] sx=0,sy=0;reg [23:0] rgb=0;
 reg [15:0] fid=0;
 wire done,tv,observed,ambiguous;wire[15:0] result_frame,overruns;
 wire[2:0] status;wire[1:0] quality;wire[11:0] x0,y0,x1,y1,cx,cy;
 wire[14:0] model;wire[3:0] color_id;
 integer checks=0,frames=0,scene=0,k;
 reg[23:0] object_rgb=24'hc02820;
 servo_stream_tracker #(.WIDTH(W),.HEIGHT(H),.HOLD_FRAMES(4)) dut(
  .clk(clk),.rst_n(rst_n),.enable(enable),.acquire(acquire),
  .acq_x0(12'd128),.acq_y0(12'd64),.acq_x1(12'd191),.acq_y1(12'd127),
  .frame_start(fs),.frame_end(fe),.frame_bad(1'b0),.sample_valid(sv),.sample_x(sx),.sample_y(sy),.rgb(rgb),.frame_id(fid),
  .result_valid(done),.result_frame(result_frame),.status(status),.track_valid(tv),.track_observed(observed),
  .quality(quality),.ambiguous(ambiguous),.x0(x0),.y0(y0),.x1(x1),.y1(y1),.cx(cx),.cy(cy),
  .model(model),.color_id(color_id),.overruns(overruns));
 task check(input bit ok,input string message);
 begin checks++;if(!ok)$fatal(1,"%s scene=%0d frame=%0d q=%0d model=%h box=%0d,%0d..%0d,%0d",message,scene,fid,quality,model,x0,y0,x1,y1);end
 endtask
 task reset;
 begin @(negedge clk);rst_n=0;acquire=0;repeat(5)@(negedge clk);rst_n=1;end
 endtask
 task frame(input integer pattern);
 integer xx,yy,waited;
 begin
  scene=pattern;frames++;@(negedge clk);fid=frames;fs=1;
  @(negedge clk);fs=0;
  for(yy=0;yy<H;yy+=4)for(xx=0;xx<W;xx+=4)begin
   @(negedge clk);sv=1;sx=xx;sy=yy;rgb=(pattern==1)?24'he9e5dc:24'hfafafa;
   if(pattern==2)begin
    rgb=24'h404040;
    if(xx>=96 && xx<224 && yy>=32 && yy<160)rgb=24'hfafafa;
   end
   if(pattern==3 || pattern==4 || pattern==5)begin
    if(xx<96)rgb=object_rgb;
    if(pattern==5 && yy==96 && xx>=96 && xx<140)rgb=object_rgb;
   end
   if(pattern==2)begin
    if(xx>=148 && xx<168 && yy>=84 && yy<104)rgb=object_rgb;
   end else if(pattern!=4 && pattern!=6)begin
    if(xx>=140 && xx<184 && yy>=76 && yy<120)rgb=object_rgb;
   end
   // Sparse matching samples must not enlarge the target rectangle.
   if(pattern==1 && xx%48==0 && yy%48==0)rgb=object_rgb;
   @(negedge clk);sv=0;
  end
  @(negedge clk);fe=1;@(negedge clk);fe=0;waited=0;
  while(!done && waited<30000)begin @(negedge clk);waited++;end
  check(done && result_frame==fid && overruns==0,"complete current frame");
  @(negedge clk);
 end
 endtask
 task learn(input integer pattern);
 begin acquire=1;frame(pattern);acquire=0;check(model!=0 && !model[14],"light background excluded from learned model");repeat(3)frame(pattern);end
 endtask
 task tight_box(input integer left,top,right,bottom);
 begin
  check(observed && quality==2,"reliable foreground observation");
  check(x0==left && y0==top && x1==right && y1==bottom,"box follows sampled foreground, not full cells/background");
 end
 endtask
 initial begin
  // Colours are varied; white and pale backgrounds must not become a target.
  for(k=0;k<4;k++)begin
   case(k)0:object_rgb=24'hc02820;1:object_rgb=24'h2860d0;2:object_rgb=24'h30b050;3:object_rgb=24'h202020;endcase
   reset();learn(k%2);tight_box(140,76,183,119);
  end
  // A small target only occupies part of the central learning patch. A white
  // wall covers a minority of the full scene but most of the target's vicinity.
  // This prevents the original global histogram from learning that white wall.
  object_rgb=24'hc02820;reset();learn(2);tight_box(148,84,167,103);
  // Large same-colour background remains a separate component.
  reset();learn(3);tight_box(140,76,183,119);
  frame(4);check(!observed && quality==0 && cx==161,"occlusion cannot jump to the large background");
  repeat(3)frame(3);tight_box(140,76,183,119);
  frame(5);check(!observed && quality==0 && x1==183,"thin connection to large background is held, not merged into green box");
  reset();acquire=1;frame(6);acquire=0;
  check(model==0 && status==5 && !observed,"featureless white-on-white scene rejects false lock");
  $display("PASS servo_background checks=%0d frames=%0d; white/pale backgrounds, four colours, tight bounds, same-colour clutter, merged region, no-feature rejection",checks,frames);
  $finish;
 end
 initial begin #1000000000;$fatal(1,"timeout");end
endmodule
