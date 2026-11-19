// V20 display-only processing. Analysis uses the unstyled bypasses.
// Every output, including the true pre-correction RGB, has three clocks latency.
module video_visual_effects #(parameter integer WIDTH=1280,TARGET_METER_ENABLE=0)(
 input wire clk,rst_n,in_vs,in_hs,in_de,
 input wire [23:0] in_raw,in_color,in_gray,in_binary,in_edge,
 input wire [7:0] scene_low,scene_mid,scene_high,
 input wire target_priority,
 output wire out_vs,out_hs,out_de,
 output reg [23:0] raw_rgb,color_rgb,gray_rgb,binary_rgb,edge_rgb,
 output reg [23:0] enhanced_rgb,cartoon_rgb,compare_rgb,
 output reg [4:0] strength
);
 reg vs_prev,de_prev;
 reg [11:0] px;
 wire [11:0] x=de_prev?px:12'd0;
 wire [4:0] requested=(TARGET_METER_ENABLE && target_priority)?5'd16:(scene_low<64 && scene_high>192)?5'd16:
                     (scene_mid<64)?5'd16:(scene_mid<112)?5'd8:5'd0;
 // Frame-rate slew prevents a scene change from abruptly changing the curve.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin vs_prev<=0;de_prev<=0;px<=0;strength<=8;end
  else begin
   vs_prev<=in_vs;de_prev<=in_de;
   if(in_de)px<=x+1'b1;else px<=0;
   if(in_vs && !vs_prev)begin
    px<=0;
    if(requested>strength)strength<=strength+1'b1;
    else if(requested<strength)strength<=strength-1'b1;
   end
  end
 end
 // Monotone lifted brightness, preserving black and white endpoints.
 function [6:0] shadow_lift;
  input [7:0] y;
  begin
   if(y<16)shadow_lift=y>>1;
   else if(y<48)shadow_lift=7'd8+((y-8'd16)<<1);
   else if(y<96)shadow_lift=7'd72-((y-8'd48)>>1);
   else if(y<160)shadow_lift=7'd48-((y-8'd96)>>1);
   else if(y<224)shadow_lift=7'd16-((y-8'd160)>>2);
   else shadow_lift=0;
  end
 endfunction
 function [6:0] scale_lift;
  input [6:0] v;input [4:0] s;
  reg [11:0] a;
  begin
   a=0;
   if(s[0])a=a+{5'd0,v};
   if(s[1])a=a+{4'd0,v,1'b0};
   if(s[2])a=a+{3'd0,v,2'b0};
   if(s[3])a=a+{2'd0,v,3'b0};
   if(s[4])a=a+{1'd0,v,4'b0};
   scale_lift=(a+12'd8)>>4;
  end
 endfunction
 function [7:0] add_lift;
  input [7:0] c;input [6:0] lift;reg [8:0] v;
  begin v={1'b0,c}+lift;add_lift=v[8]?8'd255:v[7:0];end
 endfunction
 function [7:0] poster;
  input [7:0] c;begin poster={c[7:5],c[7:5],c[7:6]};end
 endfunction
 reg [2:0] c0,c1,c2;
 reg [119:0] p0,p1;
 reg [7:0] y0;
 reg [6:0] lift1;
 reg [11:0] x0,x1;
 wire [23:0] corrected=p1[95:72];
 wire [23:0] lifted={add_lift(corrected[23:16],lift1),add_lift(corrected[15:8],lift1),add_lift(corrected[7:0],lift1)};
 wire [23:0] posterized={poster(lifted[23:16]),poster(lifted[15:8]),poster(lifted[7:0])};
 assign {out_vs,out_hs,out_de}=c2;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   c0<=0;c1<=0;c2<=0;p0<=0;p1<=0;y0<=0;lift1<=0;x0<=0;x1<=0;
   raw_rgb<=0;color_rgb<=0;gray_rgb<=0;binary_rgb<=0;edge_rgb<=0;
   enhanced_rgb<=0;cartoon_rgb<=0;compare_rgb<=0;
  end else begin
   c0<={in_vs,in_hs,in_de};c1<=c0;c2<=c1;
   p0<=in_de?{in_raw,in_color,in_gray,in_binary,in_edge}:120'd0;p1<=p0;
   // Shift/add brightness estimate is for display only, never classification.
   y0<=({2'd0,in_color[23:16]}+{1'd0,in_color[15:8],1'b0}+{2'd0,in_color[7:0]}+10'd2)>>2;
   lift1<=scale_lift(shadow_lift(y0),strength);x0<=x;x1<=x0;
   raw_rgb<=c1[0]?p1[119:96]:24'd0;color_rgb<=c1[0]?p1[95:72]:24'd0;
   gray_rgb<=c1[0]?p1[71:48]:24'd0;binary_rgb<=c1[0]?p1[47:24]:24'd0;edge_rgb<=c1[0]?p1[23:0]:24'd0;
   enhanced_rgb<=c1[0]?lifted:24'd0;
   cartoon_rgb<=!c1[0]?24'd0:(p1[23:0]!=0)?24'd0:posterized;
   compare_rgb<=!c1[0]?24'd0:(x1==WIDTH/2-1 || x1==WIDTH/2)?24'hffffff:(x1<WIDTH/2)?p1[119:96]:lifted;
  end
 end
endmodule
