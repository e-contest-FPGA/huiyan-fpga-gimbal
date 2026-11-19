// V19 real-time color correction in front of all display/analysis paths.
// The official ISP (demosaic + AWB, no color matrix) gives a dark, cyan-cast
// and weakly saturated picture. Per frame, the means of R, G and B are taken
// on a 4x4 grid; each channel gets gain = TARGET / mean (gray world plus
// digital exposure), limited to 0.5..5x and to +-RATIO_LIMIT around the
// green gain, and approaches its target by 1/8 per frame. Each pixel is then
// scaled and its chroma expanded: out = Y + SAT/4 * (c - Y).
// Gains change only between frames. Latency: four clocks, video never stalls.
module video_color_correct #(
 parameter integer WIDTH=1280, HEIGHT=720,
 parameter integer TARGET_METER_ENABLE=0,
 parameter integer TARGET=64,          // desired channel mean; the display curve lifts it further
 parameter integer SAT=5,              // chroma gain in quarters (5 = 1.25x)
 parameter integer GAIN_MIN=64, GAIN_MAX=256,  // Q7: 0.5x .. 2x; more amplifies sensor noise
 parameter integer RATIO_NUM=3, RATIO_DEN=2    // R,B gain within 2/3..3/2 of G
)(
 input wire clk,rst_n,enable,
 input wire target_priority,
 input wire in_vs,in_hs,in_de,
 input wire [23:0] in_rgb,
 output reg out_vs,out_hs,out_de,
 output reg [23:0] out_rgb,
 output reg [9:0] gain_r,gain_g,gain_b  // Q7, for display/diagnostics
);
 localparam SAMPLES=(WIDTH/4)*(HEIGHT/4);
 // Numerator of TARGET / mean in Q7: TARGET * SAMPLES * 128.
 localparam [35:0] NUMER=TARGET*SAMPLES*128;
 // ---------------- statistics ----------------
 reg [11:0] px_reg,py;
 reg de_prev,vs_prev;
 wire [11:0] px=de_prev?px_reg:12'd0;
 wire sample=in_de && px[1:0]==0 && py[1:0]==0;
 reg [23:0] sum_r,sum_g,sum_b;
 reg [16:0] count;
 // ---------------- per-frame gain update ----------------
 reg [2:0] state;
 reg [1:0] channel;
 reg [5:0] iter;
 reg [35:0] div_q;
 reg [24:0] div_r;
 reg [23:0] div_v;
 wire [24:0] div_t={div_r[23:0],div_q[35]};
 wire div_take=div_t>={1'b0,div_v};
 reg [9:0] want_r,want_g,want_b;
 reg [23:0] frame_r,frame_g,frame_b;
 function [9:0] clamp_gain;
  input [35:0] q;
  begin clamp_gain=(q<GAIN_MIN)?GAIN_MIN:(q>GAIN_MAX)?GAIN_MAX:q[9:0]; end
 endfunction
 // Keep R and B within RATIO_NUM/RATIO_DEN of the green gain so that a scene
 // dominated by one strong color is not neutralised completely.
 function [9:0] limit_ratio;
  input [9:0] g,reference_g;
  reg [13:0] hi,lo;
  begin
   hi=(reference_g*RATIO_NUM)/RATIO_DEN; lo=(reference_g*RATIO_DEN)/RATIO_NUM;
   limit_ratio=(g>hi)?hi[9:0]:(g<lo)?lo[9:0]:g;
  end
 endfunction
 function [9:0] approach;
  input [9:0] cur,want;
  begin approach=(want>cur)?cur+((want-cur+7)>>3):cur-((cur-want+7)>>3); end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   px_reg<=0;py<=0;de_prev<=0;vs_prev<=0;sum_r<=0;sum_g<=0;sum_b<=0;count<=0;
   state<=0;channel<=0;iter<=0;div_q<=0;div_r<=0;div_v<=0;
   want_r<=128;want_g<=128;want_b<=128;frame_r<=0;frame_g<=0;frame_b<=0;
   gain_r<=128;gain_g<=128;gain_b<=128;
  end else begin
   de_prev<=in_de;vs_prev<=in_vs;
   if(in_de)px_reg<=px+1'b1;else begin px_reg<=0;if(de_prev)py<=py+1'b1; end
   if(sample) begin
    sum_r<=sum_r+in_rgb[23:16];sum_g<=sum_g+in_rgb[15:8];sum_b<=sum_b+in_rgb[7:0];
    count<=count+1'b1;
   end
   case(state)
    3'd0:;
    // Three divisions TARGET*N*128 / sum, one per channel.
    3'd1: begin
     div_q<=NUMER;div_r<=0;iter<=0;
     div_v<=(channel==0)?((frame_r==0)?24'd1:frame_r):(channel==1)?((frame_g==0)?24'd1:frame_g):((frame_b==0)?24'd1:frame_b);
     state<=3'd2;
    end
    3'd2: begin
     div_r<=div_take?div_t-{1'b0,div_v}:div_t;div_q<={div_q[34:0],div_take};
     if(iter==35) state<=3'd3; else iter<=iter+1'b1;
    end
    3'd3: begin
     case(channel)
      0: want_r<=clamp_gain(div_q);
      1: want_g<=clamp_gain(div_q);
      default: want_b<=clamp_gain(div_q);
     endcase
     if(channel==2) begin channel<=0;state<=3'd4; end
     else begin channel<=channel+1'b1;state<=3'd1; end
    end
    3'd4: begin
     // Apply in vertical blank, 1/8 of the way per frame.
     gain_g<=approach(gain_g,want_g);
     gain_r<=approach(gain_r,limit_ratio(want_r,want_g));
     gain_b<=approach(gain_b,limit_ratio(want_b,want_g));
     state<=3'd0;
    end
    default:state<=3'd0;
   endcase
   if(in_vs && !vs_prev) begin
    py<=0;
    if(count==SAMPLES && state==0) begin
     frame_r<=sum_r;frame_g<=sum_g;frame_b<=sum_b;channel<=0;state<=3'd1;
    end
    sum_r<=0;sum_g<=0;sum_b<=0;count<=0;
   end
   if(!enable) begin gain_r<=128;gain_g<=128;gain_b<=128; end
  end
 end
 // ---------------- pixel path ----------------
 // ROI exposure already controls capture brightness. Do not undo it by
 // attenuating all channels against the global mean of a bright background.
 wire protect_target=TARGET_METER_ENABLE && target_priority;
 wire [9:0] pixel_gain_r=(protect_target && gain_r<128)?10'd128:gain_r;
 wire [9:0] pixel_gain_g=(protect_target && gain_g<128)?10'd128:gain_g;
 wire [9:0] pixel_gain_b=(protect_target && gain_b<128)?10'd128:gain_b;
 reg [2:0] c1,c2,c3;
 reg [17:0] m_r,m_g,m_b;                 // channel x gain (Q7)
 reg [8:0] r2,g2,b2;
 reg [15:0] y_sum;
 reg [7:0] r3,g3,b3;
 // Shift-add product in logic: the DSP blocks are fully used elsewhere.
 function [17:0] scale;
  input [7:0] c; input [9:0] g; integer i; reg [17:0] acc;
  begin acc=0; for(i=0;i<10;i=i+1) if(g[i]) acc=acc+({10'd0,c}<<i); scale=acc; end
 endfunction
 function [7:0] sat8;
  input signed [11:0] v; begin sat8=(v<0)?8'd0:(v>255)?8'd255:v[7:0]; end
 endfunction
 function [7:0] boost;
  input [7:0] c,y;
  reg signed [11:0] d,v;
  begin d=$signed({4'b0,c})-$signed({4'b0,y}); v=$signed({4'b0,y})+((d*SAT)>>>2); boost=sat8(v); end
 endfunction
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   c1<=0;c2<=0;c3<=0;m_r<=0;m_g<=0;m_b<=0;r2<=0;g2<=0;b2<=0;y_sum<=0;r3<=0;g3<=0;b3<=0;
   out_vs<=0;out_hs<=0;out_de<=0;out_rgb<=0;
  end else begin
   c1<={in_vs,in_hs,in_de};c2<=c1;c3<=c2;
   m_r<=scale(in_rgb[23:16],pixel_gain_r);m_g<=scale(in_rgb[15:8],pixel_gain_g);m_b<=scale(in_rgb[7:0],pixel_gain_b);
   r2<=(m_r[17:7]>255)?9'd255:m_r[15:7];g2<=(m_g[17:7]>255)?9'd255:m_g[15:7];b2<=(m_b[17:7]>255)?9'd255:m_b[15:7];
   y_sum<=77*r2+150*g2+29*b2+128;
   r3<=r2[7:0];g3<=g2[7:0];b3<=b2[7:0];
   {out_vs,out_hs,out_de}<=c3;
   // Disabled: unity gains (see above) and no chroma expansion.
   out_rgb<=!c3[0]?24'd0:enable?{boost(r3,y_sum[15:8]),boost(g3,y_sum[15:8]),boost(b3,y_sum[15:8])}:{r3,g3,b3};
  end
 end
endmodule
