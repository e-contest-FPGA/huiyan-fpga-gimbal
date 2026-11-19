// Camera-domain target statistics and quality/motion-aware sensor feedback.
// Exposure caps are SC500 register units, not an asserted time in ms.
module servo_exposure #(
 parameter WIDTH=1280,HEIGHT=720,MIN_AE=8,MAX_AE=1990,FAST_MAX_AE=600,
 parameter MIN_AG=16,MAX_AG=80,SETTLE_FRAMES=4
)(
 input wire clk,rst_n,ctrl_clk,ctrl_rst_n,
 input wire frame_start,frame_end,frame_bad,sample_valid,
 input wire [11:0] sample_x,sample_y,input wire [23:0] rgb,
 input wire target_enable,observed,input wire [1:0] quality,
 input wire [11:0] x0,y0,x1,y1,input wire motion_fast,measurement,
 input wire cam_cfg_done,ae_cfg_done,
 output reg target_active,output reg motion_limited,
 output reg ae_req,output reg [15:0] ae,ag,
 output reg [15:0] commands_dropped
);
 localparam EXPECTED=(WIDTH/4)*(HEIGHT/4);
 reg [15:0] count,n,bright,clipped,color_clip,global_clip;
 reg [23:0] sum;
 reg [11:0] rx0,ry0,rx1,ry1;
 reg [1:0] good_streak;reg [3:0] misses;
 reg freeze_frame,fast_frame;
 wire candidate=observed && quality==2 && x1<WIDTH && y1<HEIGHT && x1>=x0+31 && y1>=y0+23;
 wire inside_roi=sample_x>=rx0 && sample_x<=rx1 && sample_y>=ry0 && sample_y<=ry1;
 wire [15:0] weighted=77*rgb[23:16]+150*rgb[15:8]+29*rgb[7:0]+128;
 wire [7:0] y=weighted[15:8];
 wire white=rgb[23:16]>=235 && rgb[15:8]>=235 && rgb[7:0]>=235;
 wire color_white=rgb[23:16]>=250 || rgb[15:8]>=250 || rgb[7:0]>=250;
 wire extreme=sum>{n,7'b0000000}+{n,6'b000000}+{n,4'b0000} || ({clipped,3'b000}>=n) ||
              (target_active && {global_clip,5'b0}>=EXPECTED*31);
 wire high=sum>n*152 || {bright,4'b0000}>=n || {color_clip,3'b000}>=n;
 wire highlights_rare={clipped,7'b0}<=n && {color_clip,6'b0}<=n && {bright,6'b0}<n;
 wire [2:0] action=extreme?3'd3:high?3'd1:(sum<n*64 && highlights_rare)?3'd5:
                   (sum<n*112 && highlights_rare)?3'd4:3'd0;
 wire send=frame_end && !frame_bad && count==EXPECTED && n>=4;
 wire busy,record_valid;wire [4:0] record;
 servo_cdc #(.WIDTH(5)) u_action_mailbox(
  .s_clk(clk),.s_rst_n(rst_n),.s_send(send),.s_data({freeze_frame,fast_frame,action}),.s_busy(busy),
  .d_clk(ctrl_clk),.d_rst_n(ctrl_rst_n),.d_valid(record_valid),.d_data(record));
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   count<=0;n<=0;bright<=0;clipped<=0;color_clip<=0;global_clip<=0;sum<=0;
   rx0<=0;ry0<=0;rx1<=0;ry1<=0;good_streak<=0;misses<=0;
   target_active<=0;motion_limited<=0;freeze_frame<=0;fast_frame<=0;commands_dropped<=0;
  end else begin
   if(sample_valid)begin
    count<=count+1'b1;global_clip<=global_clip+white;
    if(!target_active || inside_roi)begin
     n<=n+1'b1;sum<=sum+y;bright<=bright+(y>=225);clipped<=clipped+white;color_clip<=color_clip+color_white;
    end
   end
   if(send && busy && commands_dropped!=16'hffff)commands_dropped<=commands_dropped+1'b1;
   if(frame_start)begin
    count<=0;n<=0;bright<=0;clipped<=0;color_clip<=0;global_clip<=0;sum<=0;
    fast_frame<=motion_fast;motion_limited<=motion_fast;
    freeze_frame<=measurement || (target_enable && target_active && !candidate && misses<7);
    if(!target_enable)begin target_active<=0;good_streak<=0;misses<=0;end
    else if(candidate)begin
     rx0<=x0;ry0<=y0;rx1<=x1;ry1<=y1;misses<=0;
     if(good_streak!=3)good_streak<=good_streak+1'b1;
     if(good_streak>=2)target_active<=1;
    end else begin
     good_streak<=0;if(misses!=15)misses<=misses+1'b1;
     if(misses>=7)target_active<=0;
    end
   end
  end
 end
 reg [4:0] pending;
 reg pending_valid,inflight,seen_busy;
 reg [3:0] settle;
 integer next_ae,next_ag,delta,cap;
 always @*begin
  cap=pending[3]?FAST_MAX_AE:MAX_AE;
  if(cap<MIN_AE)cap=MIN_AE;
  next_ae=ae;next_ag=ag;delta=0;
  if(!pending[4])begin
   if(ae>cap)next_ae=cap;
   else case(pending[2:0])
    1,2,3:begin
     if(ag>MIN_AG)begin delta=pending[2:0]==3?32:4;next_ag=ag-MIN_AG<delta?MIN_AG:ag-delta;end
     else begin delta=pending[2:0]==3?(ae>>1):(ae>>3);if(delta<1)delta=1;next_ae=ae-MIN_AE<delta?MIN_AE:ae-delta;end
    end
    4,5:begin
     if(ae<cap)begin delta=pending[2:0]==5?(ae>>3):(ae>>4);if(delta<1)delta=1;next_ae=ae+delta>cap?cap:ae+delta;end
     else begin delta=pending[2:0]==5?4:2;next_ag=ag+delta>MAX_AG?MAX_AG:ag+delta;end
    end
    default:;
   endcase
  end
 end
 always @(posedge ctrl_clk or negedge ctrl_rst_n)begin
  if(!ctrl_rst_n)begin
   pending<=0;pending_valid<=0;inflight<=0;seen_busy<=0;settle<=0;
   ae_req<=0;ae<=MAX_AE;ag<=MAX_AG;
  end else begin
   ae_req<=0;
   if(record_valid)begin pending<=record;pending_valid<=1;if(settle!=0)settle<=settle-1'b1;end
   if(inflight)begin
    if(!ae_cfg_done)seen_busy<=1;
    if(seen_busy && ae_cfg_done)begin inflight<=0;seen_busy<=0;end
   end
   if(pending_valid && cam_cfg_done && ae_cfg_done && !inflight && settle==0)begin
    pending_valid<=0;
    if(next_ae!=ae || next_ag!=ag)begin
     ae<=next_ae;ag<=next_ag;ae_req<=1;inflight<=1;seen_busy<=0;settle<=SETTLE_FRAMES;
    end
   end
  end
 end
endmodule
