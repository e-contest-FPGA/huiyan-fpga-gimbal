// Control consumes the latest observation. Video consumes the record whose
// frame ID belongs to the DDR slot actually being displayed. Missing history
// hides the box; it never draws a newer box on an older frame.
module servo_overlay_history(
 input wire clk,rst_n,record_valid,input wire [93:0] record,
 input wire vs,de,input wire [15:0] display_frame,
 output reg [93:0] overlay,output reg matched,output reg updated
);
 reg [93:0] history[0:3];reg [3:0] valid;
 reg started,vs_prev;integer i;
 wire [93:0] chosen=history[display_frame[1:0]];
 wire found=valid[display_frame[1:0]] && chosen[93:78]==display_frame;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   valid<=0;started<=0;vs_prev<=0;overlay<=0;matched<=0;updated<=0;
   for(i=0;i<4;i=i+1)history[i]<=0;
  end else begin
   vs_prev<=vs;updated<=0;
   if(record_valid)begin history[record[79:78]]<=record;valid[record[79:78]]<=1;end
   if(vs && !vs_prev)started<=0;
   else if(de)started<=1;
   if(!started && !de)begin
    if(overlay!=(found?chosen:94'd0) || matched!=found)updated<=1;
    overlay<=found?chosen:94'd0;matched<=found;
   end
  end
 end
endmodule

module servo_bcd_pair(
 input wire clk,rst_n,start,input wire [11:0] x,y,
 output reg [15:0] x_bcd,y_bcd
);
 reg [11:0] sx,sy;reg [15:0] bx,by;reg [3:0] step;reg busy;
 function [15:0] add3;
  input [15:0] value;integer j;
  begin add3=value;for(j=0;j<4;j=j+1)if(value[j*4+:4]>=5)add3[j*4+:4]=value[j*4+:4]+4'd3;end
 endfunction
 wire [15:0] ax=add3(bx),ay=add3(by);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin sx<=0;sy<=0;bx<=0;by<=0;step<=0;busy<=0;x_bcd<=0;y_bcd<=0;end
  else if(start)begin sx<=x;sy<=y;bx<=0;by<=0;step<=0;busy<=1;end
  else if(busy)begin
   bx<={ax[14:0],sx[11]};by<={ay[14:0],sy[11]};sx<=sx<<1;sy<=sy<<1;
   if(step==11)begin x_bcd<={ax[14:0],sx[11]};y_bcd<={ay[14:0],sy[11]};busy<=0;end
   else step<=step+1'b1;
  end
 end
endmodule
