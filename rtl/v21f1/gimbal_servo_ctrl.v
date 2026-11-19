// V19 two-axis servo gimbal: keeps the tracked target at the image centre.
// Once per frame (VS) the pixel error outside a deadband is scaled by
// 2^-KP_SHIFT, rate limited and added to the pulse width, so the pulse width
// integrates the error (no steady offset). Damping subtracts part of the
// error change. Pulse widths are clamped to safe mechanical limits.
// Disabled: both axes ramp back to centre. Lost target: hold, then return to
// centre after RETURN_FRAMES. 'moving' marks a pulse change within the last
// eight frames so fixed-camera motion detection can pause and re-learn.
// Servo signal: 50 Hz, 3.3 V, pulse 500-2500 us (MG90S/SG90 class).
module gimbal_servo_ctrl #(
 parameter integer CLK_HZ=75000000,
 parameter integer WIDTH=1280, HEIGHT=720,
 parameter integer PAN_DIR=1, TILT_DIR=1,        // flip to -1 if an axis runs away
 parameter integer CENTER_US=1500,
 parameter integer PAN_MIN_US=700, PAN_MAX_US=2300,
 parameter integer TILT_MIN_US=1000, TILT_MAX_US=2000,
 parameter integer DEADBAND=16,                  // pixels
 parameter integer KP_SHIFT=3,                   // step = error / 8 us per frame
 parameter integer KD_SHIFT=2,                   // damping = error change / 4
 parameter integer MAX_STEP_US=24,               // per frame
 parameter integer RETURN_FRAMES=300,
 parameter integer QUALITY_MODE=0
)(
 input wire clk,rst_n,enable,vs,
 input wire target_observed,
 input wire [11:0] target_cx,target_cy,
 output reg pwm_pan,pwm_tilt,
 output wire moving,
 output reg [11:0] pan_us,tilt_us,
 input wire [1:0] quality,input wire freeze,
 input wire [15:0] command_frame,
 output reg [15:0] pwm_frame,
 output reg [31:0] pwm_wait_cycles,
 output wire stale,output wire fast_motion,output wire at_limit
);
 localparam PERIOD=CLK_HZ/50, US_CYCLES=CLK_HZ/1000000;
 reg vs_prev;
 reg [20:0] count;
 reg [21:0] pan_cycles,tilt_cycles;
 reg signed [12:0] last_ex,last_ey;
 reg have_last;
 reg [8:0] lost;
 reg [7:0] change_history;
 reg [31:0] record_age;
 reg [31:0] command_age;
 reg [15:0] pending_frame;
 reg have_command;
 reg update_pending;
 reg signed [12:0] step_x_d,step_y_d;
 wire observation_tick=QUALITY_MODE?update_pending:(vs && !vs_prev);
 assign stale=QUALITY_MODE && record_age>CLK_HZ/10;
 assign fast_motion=QUALITY_MODE && (step_x>=12 || step_x<=-12 || step_y>=12 || step_y<=-12) && target_observed;
 assign at_limit=pan_us==PAN_MIN_US || pan_us==PAN_MAX_US || tilt_us==TILT_MIN_US || tilt_us==TILT_MAX_US;
 assign moving=change_history!=0;
 function signed [12:0] deadband;
  input signed [12:0] e;
  begin deadband=(e>DEADBAND || e<-DEADBAND)?e:13'sd0; end
 endfunction
 function signed [12:0] limit_step;
  input signed [12:0] s;
  begin limit_step=(s>MAX_STEP_US)?MAX_STEP_US:(s<-MAX_STEP_US)?-MAX_STEP_US:s; end
 endfunction
 function [11:0] clamp_us;
  input signed [13:0] v;input integer lo,hi;
  begin clamp_us=(v<lo)?lo:(v>hi)?hi:v[11:0]; end
 endfunction
 // Error of the target from the centre, positive right/down.
 wire signed [12:0] ex=deadband($signed({1'b0,target_cx})-WIDTH/2);
 wire signed [12:0] ey=deadband($signed({1'b0,target_cy})-HEIGHT/2);
 // Inside the deadband no step is taken at all (no derivative kick).
 wire signed [12:0] step_x=(ex==0)?13'sd0:limit_step((ex>>>KP_SHIFT)+(have_last?((ex-last_ex)>>>KD_SHIFT):13'sd0));
 wire signed [12:0] step_y=(ey==0)?13'sd0:limit_step((ey>>>KP_SHIFT)+(have_last?((ey-last_ey)>>>KD_SHIFT):13'sd0));
 // Return to centre at a gentle 8 us per frame.
 wire signed [12:0] home_x=limit_step(($signed({1'b0,pan_us})-CENTER_US)>>>3);
 wire signed [12:0] home_y=limit_step(($signed({1'b0,tilt_us})-CENTER_US)>>>3);
 wire signed [13:0] next_pan_track=$signed({2'b00,pan_us})-PAN_DIR*(QUALITY_MODE?step_x_d:step_x);
 wire signed [13:0] next_tilt_track=$signed({2'b00,tilt_us})-TILT_DIR*(QUALITY_MODE?step_y_d:step_y);
 wire [11:0] clamped_pan=clamp_us(next_pan_track,PAN_MIN_US,PAN_MAX_US);
 wire [11:0] clamped_tilt=clamp_us(next_tilt_track,TILT_MIN_US,TILT_MAX_US);
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   vs_prev<=0;count<=0;pan_us<=CENTER_US;tilt_us<=CENTER_US;
   pan_cycles<=CENTER_US*US_CYCLES;tilt_cycles<=CENTER_US*US_CYCLES;
   pwm_pan<=0;pwm_tilt<=0;last_ex<=0;last_ey<=0;have_last<=0;lost<=0;change_history<=0;
   record_age<=32'hffffffff;command_age<=0;pending_frame<=0;pwm_frame<=0;pwm_wait_cycles<=0;have_command<=0;
   update_pending<=0;step_x_d<=0;step_y_d<=0;
  end else begin
   vs_prev<=vs;
   update_pending<=vs && !vs_prev;step_x_d<=step_x;step_y_d<=step_y;
   if(record_age!=32'hffffffff)record_age<=record_age+1'b1;
   if(command_age!=32'hffffffff)command_age<=command_age+1'b1;
   if(vs && !vs_prev)record_age<=0;
   if(vs && !vs_prev && enable && target_observed && (!QUALITY_MODE || (quality==2 && !freeze)))command_age<=0;
   // 50 Hz pulse train; widths reload only at the start of a period.
   if(count==PERIOD-1) begin
    count<=0;pan_cycles<=pan_us*US_CYCLES;tilt_cycles<=tilt_us*US_CYCLES;
    if(have_command)begin pwm_frame<=pending_frame;pwm_wait_cycles<=command_age;have_command<=0;end
   end else count<=count+1'b1;
   pwm_pan<=count<pan_cycles;pwm_tilt<=count<tilt_cycles;
   if(observation_tick || (QUALITY_MODE && stale && count==PERIOD-1)) begin
    change_history<={change_history[6:0],1'b0};
    if(QUALITY_MODE && freeze)begin have_last<=0;end
    else if(enable && target_observed && (!QUALITY_MODE || quality==2) && observation_tick) begin
     lost<=0;last_ex<=ex;last_ey<=ey;have_last<=1;
     pan_us<=clamped_pan;
     tilt_us<=clamped_tilt;
     pending_frame<=command_frame;have_command<=1;
     if(clamped_pan!=pan_us || clamped_tilt!=tilt_us)change_history[0]<=1;
    end else begin
     have_last<=0;
     if(lost!=9'h1ff)lost<=lost+1'b1;
     if(!enable || lost>=RETURN_FRAMES) begin
      pan_us<=pan_us-home_x;tilt_us<=tilt_us-home_y;
      if(home_x!=0 || home_y!=0)change_history[0]<=1;
     end
    end
   end
  end
 end
endmodule

// V21 production controller. The tracker already confirms quality=2 across
// consecutive frames, so its first reliable record can command immediately.
// Subsequent corrections keep the V20S smoothing, hysteresis and slow cadence.
// All movement consumes fresh records; a time watchdog also permits SW3-off
// homing when camera records stop. Measurement mode freezes without homing.
module gimbal_servo_quality_stable #(
 parameter integer CLK_HZ=75000000,WIDTH=1280,HEIGHT=720,
 parameter integer PAN_DIR=1,TILT_DIR=-1,CENTER_US=1500,
 parameter integer PAN_MIN_US=700,PAN_MAX_US=2300,
 parameter integer TILT_MIN_US=700,TILT_MAX_US=2300,
 parameter integer STOP_PX=24,START_PX=48,KP_SHIFT=4,MAX_STEP_US=6,
 parameter integer CONTROL_FRAMES=3,RETURN_FRAMES=300
)(
 input wire clk,rst_n,enable,vs,target_observed,
 input wire [11:0] target_cx,target_cy,
 output reg pwm_pan,pwm_tilt,output wire moving,
 output reg [11:0] pan_us,tilt_us,
 input wire [1:0] quality,input wire freeze,input wire [15:0] command_frame,
 output reg [15:0] pwm_frame,output reg [31:0] pwm_wait_cycles,
 output wire stale,fast_motion,at_limit
);
 localparam PERIOD=CLK_HZ/50,US_CYCLES=CLK_HZ/1000000;
 reg vs_prev,have_filter,active_x,active_y;
 reg [20:0] count;
 reg [21:0] pan_cycles,tilt_cycles;
 reg signed [18:0] filtered_x,filtered_y;
 reg signed [12:0] command_step_x,command_step_y;
 reg command_pending,apply_pending,pending_home,apply_home,have_command;
 reg [7:0] cadence,change_history;
 reg [8:0] lost;
 reg [31:0] record_age,cycles,queued_time,pending_time;
 reg [15:0] queued_frame,pending_frame;
 wire record_tick=vs && !vs_prev;
 assign stale=record_age>CLK_HZ/10;
 wire observation_tick=record_tick || (stale && count==PERIOD-1);
 wire reliable=enable && target_observed && quality==2 && !freeze;
 assign moving=change_history!=0;
 assign at_limit=pan_us==PAN_MIN_US || pan_us==PAN_MAX_US || tilt_us==TILT_MIN_US || tilt_us==TILT_MAX_US;
 assign fast_motion=reliable && !stale && moving && !at_limit &&
                    (command_step_x>=3 || command_step_x<=-3 || command_step_y>=3 || command_step_y<=-3);
 wire signed [18:0] sample_x=($signed({1'b0,target_cx})-WIDTH/2)*16;
 wire signed [18:0] sample_y=($signed({1'b0,target_cy})-HEIGHT/2)*16;
 wire signed [18:0] next_x=have_filter?filtered_x+((sample_x-filtered_x)>>>2):sample_x;
 wire signed [18:0] next_y=have_filter?filtered_y+((sample_y-filtered_y)>>>2):sample_y;
 wire signed [12:0] error_x=next_x>>>4,error_y=next_y>>>4;
 wire [12:0] abs_x=error_x[12]?-error_x:error_x,abs_y=error_y[12]?-error_y:error_y;
 wire run_x=active_x?abs_x>STOP_PX:abs_x>=START_PX;
 wire run_y=active_y?abs_y>STOP_PX:abs_y>=START_PX;
 wire signed [12:0] filtered_error_x=filtered_x>>>4,filtered_error_y=filtered_y>>>4;
 function signed [12:0] correction;
  input signed [12:0] error;input running;reg [12:0] magnitude,step;
  begin
   magnitude=error[12]?-error:error;step=0;
   if(running && magnitude>STOP_PX)begin
    step=(magnitude-STOP_PX)>>KP_SHIFT;
    if(step==0)step=1;if(step>MAX_STEP_US)step=MAX_STEP_US;
   end
   correction=error[12]?-$signed(step):$signed(step);
  end
 endfunction
 function [11:0] clamp_us;
  input signed [13:0] value;input integer lo,hi;
  begin clamp_us=value<lo?lo:value>hi?hi:value[11:0];end
 endfunction
 function [11:0] home;
  input [11:0] current;
  begin
   if(current>CENTER_US)home=current-CENTER_US>MAX_STEP_US?current-MAX_STEP_US:CENTER_US;
   else home=CENTER_US-current>MAX_STEP_US?current+MAX_STEP_US:CENTER_US;
  end
 endfunction
 wire [11:0] next_pan=clamp_us($signed({2'b00,pan_us})-PAN_DIR*command_step_x,PAN_MIN_US,PAN_MAX_US);
 wire [11:0] next_tilt=clamp_us($signed({2'b00,tilt_us})-TILT_DIR*command_step_y,TILT_MIN_US,TILT_MAX_US);
 wire [11:0] home_pan=home(pan_us),home_tilt=home(tilt_us);
 wire control_due=cadence==CONTROL_FRAMES-1;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   vs_prev<=0;have_filter<=0;active_x<=0;active_y<=0;count<=0;
   pan_us<=CENTER_US;tilt_us<=CENTER_US;pan_cycles<=CENTER_US*US_CYCLES;tilt_cycles<=CENTER_US*US_CYCLES;
   pwm_pan<=0;pwm_tilt<=0;filtered_x<=0;filtered_y<=0;command_step_x<=0;command_step_y<=0;
   command_pending<=0;apply_pending<=0;pending_home<=0;apply_home<=0;have_command<=0;
   cadence<=0;change_history<=0;lost<=0;record_age<=32'hffffffff;cycles<=0;
   queued_time<=0;pending_time<=0;queued_frame<=0;pending_frame<=0;pwm_frame<=0;pwm_wait_cycles<=0;
  end else begin
   vs_prev<=vs;cycles<=cycles+1'b1;
   if(record_age!=32'hffffffff)record_age<=record_age+1'b1;
   if(record_tick)record_age<=0;
   command_step_x<=correction(filtered_error_x,active_x);
   command_step_y<=correction(filtered_error_y,active_y);
   command_pending<=0;apply_pending<=command_pending;apply_home<=pending_home;
   if(count==PERIOD-1)begin
    count<=0;pan_cycles<=pan_us*US_CYCLES;tilt_cycles<=tilt_us*US_CYCLES;
    if(have_command)begin pwm_frame<=pending_frame;pwm_wait_cycles<=cycles-pending_time;have_command<=0;end
   end else count<=count+1'b1;
   pwm_pan<=count<pan_cycles;pwm_tilt<=count<tilt_cycles;
   if(apply_pending && !freeze)begin
    if(apply_home)begin
     pan_us<=home_pan;tilt_us<=home_tilt;have_command<=0;
     if(home_pan!=pan_us || home_tilt!=tilt_us)change_history[0]<=1;
    end else if(reliable)begin
     pan_us<=next_pan;tilt_us<=next_tilt;
     pending_frame<=queued_frame;pending_time<=queued_time;have_command<=1;
     if(next_pan!=pan_us || next_tilt!=tilt_us)change_history[0]<=1;
    end
   end
   if(observation_tick)begin
    change_history<={change_history[6:0],1'b0};
    if(freeze)begin
     have_filter<=0;active_x<=0;active_y<=0;cadence<=0;lost<=0;
    end else if(record_tick && reliable)begin
     lost<=0;filtered_x<=next_x;filtered_y<=next_y;have_filter<=1;active_x<=run_x;active_y<=run_y;
     if(!have_filter || control_due)begin
      cadence<=0;command_pending<=1;pending_home<=0;queued_frame<=command_frame;queued_time<=cycles;
     end else cadence<=cadence+1'b1;
    end else begin
     have_filter<=0;active_x<=0;active_y<=0;
     if(lost!=9'h1ff)lost<=lost+1'b1;
     if(!enable || lost>=RETURN_FRAMES)begin
      if(control_due)begin cadence<=0;command_pending<=1;pending_home<=1;end
      else cadence<=cadence+1'b1;
     end else cadence<=0;
    end
   end
  end
 end
endmodule
