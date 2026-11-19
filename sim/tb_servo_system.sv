`timescale 1ns/1ps
module tb_servo_system;
 localparam W=320,H=192;
 reg c=0,p=0,k=0,d=0;always #7 c=~c;always #5 p=~p;always #13 k=~k;always #11 d=~d;
 reg rst_n=0,sof=0,cv=0,eol=0,rx=0,vs=0,de=0;reg [95:0] rgb4=0;
 reg [15:0] display_tag=16'hffff;reg ae_done=1;
 wire pan,tilt,led,uart,motion_en,measurement,meter,ae_req;
 wire [15:0] ae,ag,camera_id,xb,yb,errors,underflows;wire cbad,cend,tv,observed,mask,latvalid,ambig,matched,stale,limit;
 wire [2:0] status;wire [1:0] quality;wire [11:0] x0,y0,x1,y1,cx,cy;wire [3:0] colour;wire [15:0] latbcd;
 integer checks=0,frames=0,busy=0;reg early_response=0;
 servo_vision_system #(.WIDTH(W),.HEIGHT(H),.PIXEL_HZ(1000000),.OPTICAL_LATENCY(0)) dut(
  .camera_clk(c),.camera_rst_n(rst_n),.pixel_clk(p),.pixel_rst_n(rst_n),.ctrl_clk(k),.ctrl_rst_n(rst_n),.ddr_clk(d),
  .camera_sof(sof),.camera_valid(cv),.camera_eol(eol),.rx_error(rx),.camera_rgb4(rgb4),
  .vs(vs),.de(de),.raw_rgb(24'h404040),.key_n(2'b11),.system_on(1'b1),.lock(1'b1),.gimbal_on(1'b1),.target_priority(1'b1),
  .motion_valid(1'b0),.motion_observed(1'b0),.motion_x0(12'd0),.motion_y0(12'd0),.motion_x1(12'd0),.motion_y1(12'd0),
  .cam_cfg_done(1'b1),.ae_cfg_done(ae_done),.input_overflow(16'd0),.output_overflow(16'd0),.output_underflow(16'd0),.display_frame_ddr(display_tag),
  .pwm_pan(pan),.pwm_tilt(tilt),.led(led),.uart_tx(uart),.motion_enable(motion_en),.measurement(measurement),.meter_target(meter),
  .ae_req(ae_req),.ae(ae),.ag(ag),.camera_frame_id(camera_id),.camera_bad(cbad),.camera_frame_end(cend),
  .track_status(status),.track_valid(tv),.track_observed(observed),.x0(x0),.y0(y0),.x1(x1),.y1(y1),.cx(cx),.cy(cy),
  .cx_bcd(xb),.cy_bcd(yb),.color_id(colour),.track_mask(mask),.lat_valid(latvalid),.lat_bcd(latbcd),
  .quality(quality),.ambiguous(ambig),.overlay_matched(matched),.control_stale(stale),.at_limit(limit),.rx_errors(errors),.underflows(underflows));
 always @(posedge k)begin
  if(ae_req)begin ae_done<=0;busy<=4;end
  else if(busy!=0)begin busy<=busy-1;if(busy==1)ae_done<=1;end
 end
 initial begin
  wait(rst_n);
  forever begin
   @(negedge p);vs=1;de=0;repeat(30)@(negedge p);vs=0;
   repeat(100)@(negedge p);de=1;repeat(100)@(negedge p);de=0;
   repeat(29700)@(negedge p);
  end
 end
 always @(posedge p)if(rst_n && dut.u_gimbal.pan_us<1500 && dut.live_frame>=display_tag+2 && display_tag!=16'hffff)early_response<=1;
 task check(input bit pass,input string message);begin checks++;if(!pass)$fatal(1,"FAIL %s frame=%0d status=%0d q=%0d",message,frames,status,quality);end endtask
 task frame(input integer offset,input bit absent,input bit receiver_fault);
  integer xx,yy;reg [23:0] pixel;
  begin
   frames++;
   for(yy=0;yy<H;yy++)begin
    for(xx=0;xx<W;xx+=4)begin
     @(negedge c);sof=xx==0 && yy==0;cv=1;eol=xx==W-4;
     pixel=(!absent && xx>=128+offset && xx<192+offset && yy>=64 && yy<128)?24'hc02820:24'h404040;
     rgb4={pixel,pixel,pixel,pixel};rx=receiver_fault && xx==4 && yy==4;
    end
    @(negedge c);cv=0;eol=0;sof=0;rx=0;repeat(4)@(negedge c);
   end
   display_tag=frames>2?frames-2:16'hffff;
   repeat(6000)@(negedge c);
  end
 endtask
 initial begin
  repeat(30)@(negedge c);rst_n=1;
  repeat(8)frame(0,0,0);
  check(quality==2 && dut.live_cx==159,"camera-domain learned target and confirmation");
  check(dut.u_tracker.overruns==0 && dut.record_drops==0,"complete records delivered");
  // A 32px shift is intentionally inside the 48px anti-jitter start band.
  // Exercise a sustained displacement outside that band through the EMA.
  repeat(7)frame(64,0,0);check(quality==2 && dut.live_cx==223 && early_response,"control uses newer result than delayed video");
  check(meter && !led && !measurement && !latvalid && latbcd==0,"target exposure active, LED absent in normal tracking");
  frame(64,0,1);check(quality==0 && errors==1,"receiver fault gates control and is counted");
  repeat(3)frame(64,0,0);check(quality==2,"confirmed recovery after bad frame");
  repeat(3)frame(0,1,0);check(quality==0 && status==3,"occlusion holds target");
  check(dut.u_tracker.overruns==0 && underflows==0,"integration health remains valid");
  $display("PASS servo_system checks=%0d frames=%0d early_control=%b",checks,frames,early_response);$finish;
 end
 initial begin #20000000;$fatal(1,"FAIL integrated system timeout");end
endmodule
