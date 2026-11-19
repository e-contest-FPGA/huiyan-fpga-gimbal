// Tap AWB's four RGB pixels before DDR packing. Sample the first pixel of
// each four-pixel beat and one row in four; gaps do not imply line endings.
// tlast is authoritative. Incomplete/malformed/error frames are invalidated.
module camera_analysis_stream #(parameter WIDTH=1280,HEIGHT=720)(
 input wire clk,rst_n,sof,valid,eol,input wire [95:0] rgb4,input wire rx_error,
 output reg frame_start,frame_end,sample_valid,frame_bad,
 output reg [23:0] rgb,output reg [11:0] x,y,
 output reg [15:0] frame_id,output reg [15:0] bad_frames
);
 reg [11:0] column,row;
 reg active,bad,error_prev;
 reg sample_stage;reg [23:0] rgb_stage;reg [11:0] x_stage,y_stage;
 wire [11:0] current_x=sof?12'd0:column,current_y=sof?12'd0:row;
 wire last_word=current_x==WIDTH-4;
 wire error_now=rx_error && !error_prev;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   column<=0;row<=0;active<=0;bad<=0;error_prev<=0;
   frame_start<=0;frame_end<=0;sample_valid<=0;frame_bad<=0;
   rgb<=0;x<=0;y<=0;frame_id<=0;bad_frames<=0;
   sample_stage<=0;rgb_stage<=0;x_stage<=0;y_stage<=0;
  end else begin
   frame_start<=0;frame_end<=0;sample_stage<=0;error_prev<=rx_error;
   sample_valid<=sample_stage;rgb<=rgb_stage;x<=x_stage;y<=y_stage;
   if(error_now)bad<=1;
   if(sof)begin
    if(active && bad_frames!=16'hffff)bad_frames<=bad_frames+1'b1;
    column<=0;row<=0;active<=1;bad<=error_now;frame_bad<=0;
    frame_id<=frame_id+1'b1;frame_start<=1;
   end
   if(valid && (active || sof))begin
    x_stage<=current_x;y_stage<=current_y;rgb_stage<=rgb4[95:72];
    sample_stage<=current_y[1:0]==0 && current_x<WIDTH && current_y<HEIGHT;
    if(current_y>=HEIGHT || current_x>=WIDTH || eol!=last_word)bad<=1;
    if(eol)begin
     column<=0;row<=current_y+1'b1;
     if(current_y==HEIGHT-1)begin
      active<=0;frame_end<=1;frame_bad<=bad || error_now || !last_word;
      if((bad || error_now || !last_word) && bad_frames!=16'hffff)bad_frames<=bad_frames+1'b1;
     end
    end else column<=current_x+4;
   end
  end
 end
endmodule
