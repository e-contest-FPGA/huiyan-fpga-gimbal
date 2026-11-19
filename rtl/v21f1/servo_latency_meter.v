// V21 optical stimulus -> video ROI threshold crossing, NOT panel photons.
// Small spots are counted against an LED-off hot-pixel baseline. A second
// adjacent ROI rejects common illumination changes. AE freezes in test mode.
module servo_latency_meter #(
 parameter CLK_HZ=75000000,PERIOD_MS=1000,ON_MS=300,SETTLE_MS=150,
 parameter X0=1200,Y0=96,SIZE=32,EXTRA_PIXELS=8,PIXEL_STEP=32,REFERENCE_STEP=16
)(
 input wire clk,rst_n,enable,vs,de,input wire [7:0] gray,
 output reg led,result_valid,lat_valid,output reg [31:0] raw_cycles,
 output reg [15:0] sample_id,timeouts,lat_bcd
);
 localparam MS_CYCLES=CLK_HZ/1000,TENTH_CYCLES=CLK_HZ/10000,AREA=SIZE*SIZE;
 localparam SHIFT=$clog2(AREA);
 reg [16:0] ms_counter;reg [10:0] phase;
 reg [31:0] since_on;
 reg [11:0] px_reg,py;reg de_prev,vs_prev;
 wire [11:0] px=de_prev?px_reg:12'd0;
 wire in_roi=de && px>=X0 && px<X0+SIZE && py>=Y0 && py<Y0+SIZE;
 wire in_ref=de && px>=X0-SIZE && px<X0 && py>=Y0 && py<Y0+SIZE;
 wire roi_first=in_roi && px==X0 && py==Y0;
 wire roi_last=in_roi && px==X0+SIZE-1 && py==Y0+SIZE-1;
 wire ref_first=in_ref && px==X0-SIZE && py==Y0;
 wire ref_last=in_ref && px==X0-1 && py==Y0+SIZE-1;
 reg [23:0] roi_sum,ref_sum,ref_complete;
 reg [15:0] hot_count,base_hot;
 reg [7:0] base_mean,base_ref;
 reg base_valid,armed,roi_armed,crossed;
 reg [31:0] crossing_time;
 wire hot={1'b0,gray}>{1'b0,base_mean}+PIXEL_STEP;
 wire [16:0] hot_next=roi_first?hot:{1'b0,hot_count}+hot;
 wire ref_stable=(ref_complete>>SHIFT)<base_ref+REFERENCE_STEP &&
                 (ref_complete>>SHIFT)+REFERENCE_STEP>base_ref;
 wire ms_tick=ms_counter==MS_CYCLES-1;
 reg [31:0] quotient;reg [31:0] remainder;
 reg [5:0] div_step;reg [2:0] conversion;
 wire [32:0] trial={remainder,quotient[31]};
 wire take=trial>=TENTH_CYCLES;
 reg [15:0] binary_shift,bcd;reg [4:0] bcd_step;
 function [15:0] add3;
  input [15:0] value;integer j;
  begin add3=value;for(j=0;j<4;j=j+1)if(value[j*4+:4]>=5)add3[j*4+:4]=value[j*4+:4]+4'd3;end
 endfunction
 wire [15:0] adjusted=add3(bcd);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   led<=0;result_valid<=0;lat_valid<=0;raw_cycles<=0;sample_id<=0;timeouts<=0;lat_bcd<=0;
   ms_counter<=0;phase<=0;since_on<=0;px_reg<=0;py<=0;de_prev<=0;vs_prev<=0;
   roi_sum<=0;ref_sum<=0;ref_complete<=0;hot_count<=0;base_hot<=0;base_mean<=0;base_ref<=0;
   base_valid<=0;armed<=0;roi_armed<=0;crossed<=0;crossing_time<=0;
   quotient<=0;remainder<=0;div_step<=0;conversion<=0;binary_shift<=0;bcd<=0;bcd_step<=0;
  end else begin
   result_valid<=0;de_prev<=de;vs_prev<=vs;
   if(de)px_reg<=px+1'b1;else begin px_reg<=0;if(de_prev)py<=py+1'b1;end
   if(vs && !vs_prev)py<=0;
   if(since_on!=32'hffffffff)since_on<=since_on+1'b1;
   if(ms_tick)begin
    ms_counter<=0;
    if(phase==PERIOD_MS-1)begin phase<=0;if(enable)begin led<=1;since_on<=0;armed<=base_valid;end end
    else phase<=phase+1'b1;
    if(phase==ON_MS-1)begin
     led<=0;
     if(armed)begin armed<=0;lat_valid<=0;if(timeouts!=16'hffff)timeouts<=timeouts+1'b1;end
    end
   end else ms_counter<=ms_counter+1'b1;
   if(ref_first)ref_sum<=gray;else if(in_ref)ref_sum<=ref_sum+gray;
   if(ref_last)ref_complete<=ref_sum+gray;
   if(roi_first)begin roi_sum<=gray;hot_count<=hot;roi_armed<=armed;crossed<=0;end
   else if(in_roi)begin roi_sum<=roi_sum+gray;hot_count<=hot_next;end
   if(in_roi && armed && (roi_first?armed:roi_armed) && !crossed && hot_next>={1'b0,base_hot}+EXTRA_PIXELS)begin
    crossing_time<=since_on;crossed<=1;
   end
   if(roi_last)begin
    if(!led && phase>=ON_MS+SETTLE_MS)begin
     base_mean<=(roi_sum+gray)>>SHIFT;base_ref<=ref_complete>>SHIFT;base_hot<=hot_next;base_valid<=1;
    end
    if(armed && roi_armed && crossed && ref_stable)begin
     armed<=0;raw_cycles<=crossing_time;sample_id<=sample_id+1'b1;result_valid<=1;lat_valid<=1;
     quotient<=crossing_time;remainder<=0;div_step<=0;conversion<=1;
    end
   end
   case(conversion)
    1:begin
     remainder<=take?trial-TENTH_CYCLES:trial;quotient<={quotient[30:0],take};
     if(div_step==31)conversion<=2;else div_step<=div_step+1'b1;
    end
    2:begin binary_shift<=quotient>9999?16'd9999:quotient[15:0];bcd<=0;bcd_step<=0;conversion<=3;end
    3:begin
     bcd<={adjusted[14:0],binary_shift[15]};binary_shift<=binary_shift<<1;
     if(bcd_step==15)conversion<=4;else bcd_step<=bcd_step+1'b1;
    end
    4:begin lat_bcd<=bcd;conversion<=0;end
    default:;
   endcase
   if(!enable)begin
    led<=0;armed<=0;base_valid<=0;roi_armed<=0;lat_valid<=0;phase<=0;ms_counter<=0;crossed<=0;
   end
  end
 end
endmodule

// Hold both debounced keys for 1.5 seconds, then commit at a frame boundary.
module servo_measure_control #(parameter CLK_HZ=75000000,HOLD_MS=1500)(
 input wire clk,rst_n,vs,input wire [1:0] key_n,output reg measurement
);
 localparam LIMIT=(CLK_HZ/1000)*HOLD_MS;
 reg [$clog2(LIMIT+1)-1:0] held;
 reg fired,pending,vs_prev;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin held<=0;fired<=0;pending<=0;measurement<=0;vs_prev<=0;end
  else begin
   vs_prev<=vs;
   if(key_n==0)begin
    if(held<LIMIT)held<=held+1'b1;
    else if(!fired)begin pending<=1;fired<=1;end
   end else begin held<=0;fired<=0;end
   if(vs && !vs_prev && pending)begin measurement<=~measurement;pending<=0;end
  end
 end
endmodule
