`timescale 1ns/1ps
module tb_servo_stable;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,enable=1,observed=1,vs=0,freeze=0;reg [1:0] quality=2;
 reg [11:0] cx=640,cy=360;reg [15:0] frame_id=0;
 wire pan,tilt,moving,stale,fast,limit;wire [11:0] pu,tu;wire [15:0] pwm_frame;wire [31:0] wait_cycles;
 gimbal_servo_quality_stable #(.CLK_HZ(1000000),.RETURN_FRAMES(12)) dut(
 .clk(clk),.rst_n(rst_n),.enable(enable),.vs(vs),.target_observed(observed),.target_cx(cx),.target_cy(cy),
 .quality(quality),.freeze(freeze),.command_frame(frame_id),.pwm_pan(pan),.pwm_tilt(tilt),.moving(moving),
 .pan_us(pu),.tilt_us(tu),.pwm_frame(pwm_frame),.pwm_wait_cycles(wait_cycles),.stale(stale),.fast_motion(fast),.at_limit(limit));
 integer checks=0,frames=0,oldp,oldt,i,j,last_change=-100,basep,baset,width_seen,max_error,ex,ey;
 integer xd[0:3],yd[0:3],log_file;
 real physical_pan,physical_tilt;
 task check(input bit ok,input string msg);begin checks++;if(!ok)$fatal(1,"FAIL %s pan=%0d tilt=%0d frame=%0d",msg,pu,tu,frames);end endtask
 task reset;begin
  @(negedge clk);rst_n=0;enable=1;observed=1;freeze=0;quality=2;vs=0;cx=640;cy=360;
  repeat(5)@(negedge clk);rst_n=1;repeat(4)@(negedge clk);last_change=-100;
 end endtask
 task observation;begin
  oldp=pu;oldt=tu;@(negedge clk);frame_id++;vs=1;
  repeat(3)@(negedge clk);vs=0;repeat(37)@(negedge clk);frames++;
  if($signed({1'b0,pu})-oldp>6 || oldp-$signed({1'b0,pu})>6 ||
     $signed({1'b0,tu})-oldt>6 || oldt-$signed({1'b0,tu})>6)$fatal(1,"FAIL slew");
  if(pu!=oldp || tu!=oldt)begin
   if(frames-last_change<3)$fatal(1,"FAIL cadence");last_change=frames;
  end
  if(pu<700 || pu>2300 || tu<700 || tu>2300)$fatal(1,"FAIL bounds");
 end endtask
 task pulse_check(input integer expected);begin
  @(negedge tilt);@(posedge tilt);width_seen=0;
  while(tilt)begin @(negedge clk);if(tilt)width_seen++;end
  check(width_seen>=expected-1 && width_seen<=expected+1,"physical PWM width in simulation");
 end endtask
 initial begin
  reset();quality=1;cx=1100;cy=30;repeat(10)observation();check(pu==1500 && tu==1500,"tentative cannot steer");
  quality=2;frame_id=16'hfffe;observation();check(pu==1494 && tu==1494,"first confirmed record commands without extra confirmation");
  basep=pu;baset=tu;repeat(2)observation();check(pu==basep && tu==baset,"three-record cadence");
  observation();check(pu==1488 && tu==1488,"next conservative correction");
  freeze=1;enable=0;basep=pu;baset=tu;repeat(100)observation();
  repeat(130000)@(negedge clk);check(pu==basep && tu==baset,"measurement freezes even disabled and stale");
  reset();cx=1100;cy=30;repeat(440)observation();check(pu==700 && tu==700 && limit && !moving,"wide lower limits settle");
  pulse_check(700);
  reset();cx=30;cy=700;repeat(440)observation();check(pu==2300 && tu==2300 && limit && !moving,"wide upper limits settle");
  pulse_check(2300);
  observed=0;basep=pu;baset=tu;repeat(12)observation();check(pu==basep && tu==baset,"loss holds first twelve records");
  repeat(430)observation();check(pu==1500 && tu==1500,"loss returns slowly to centre");
  reset();for(i=0;i<240;i++)begin cx=640+(i%2?30:-30);cy=360+(i%3?26:-26);observation();end
  check(pu==1500 && tu==1500,"centre noise does not drift");
  cx=800;cy=200;observation();cx=640;cy=360;repeat(30)observation();
  check(pu>=1494 && pu<=1506 && tu>=1494 && tu<=1506,"isolated centroid spike bounded");
  reset();cx=1100;cy=30;repeat(20)observation();
  repeat(110000)@(negedge clk);check(stale,"missing records time watchdog");
  enable=0;repeat(3000000)@(negedge clk);check(pu==1500 && tu==1500,"SW3-off homes without incoming camera records");
  reset();cx=1100;frame_id=16'hffff;observation();basep=frame_id;
  repeat(22000)@(negedge clk);check(pwm_frame==basep && wait_cycles>0 && wait_cycles<=20010,"PWM identity wraps and timestamp includes pipeline wait");
  // Delayed synthetic plant: no physical performance claim.
  reset();physical_pan=1500;physical_tilt=1500;max_error=0;
  for(j=0;j<4;j++)begin xd[j]=880;yd[j]=210;end
  log_file=$fopen("servo_stable_model.csv","w");$fdisplay(log_file,"frame,cx,cy,pan_us,tilt_us,model_error_x,model_error_y");
  for(i=0;i<480;i++)begin
   physical_pan=physical_pan+(pu-physical_pan)*0.25;physical_tilt=physical_tilt+(tu-physical_tilt)*0.25;
   ex=$rtoi(240+(physical_pan-1500)*3);ey=$rtoi(-150-(physical_tilt-1500)*2.8);
   for(j=3;j>0;j--)begin xd[j]=xd[j-1];yd[j]=yd[j-1];end
   xd[0]=640+ex+((i*17)%37)-18;yd[0]=360+ey-((i*17)%37)+18;
   cx=xd[3];cy=yd[3];observation();
   if(i>300)begin
    if(ex>max_error)max_error=ex;if(-ex>max_error)max_error=-ex;
    if(ey>max_error)max_error=ey;if(-ey>max_error)max_error=-ey;
   end
   $fdisplay(log_file,"%0d,%0d,%0d,%0d,%0d,%0d,%0d",i,cx,cy,pu,tu,ex,ey);
  end
  $fclose(log_file);check(max_error<=48,"delayed model settles within hysteresis start band");
  $display("PASS servo_stable checks=%0d frames=%0d model_late_max_error=%0d",checks,frames,max_error);$finish;
 end
 initial begin #100000000;$fatal(1,"FAIL stable timeout");end
endmodule
