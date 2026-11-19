// V19 acquisition policy for video_color_tracker.
// SW1 (system_on) alone: automatic mode. A motion target observed for CONFIRM
//   consecutive frames, while the tracker is idle or searching, is learned.
// SW2 (lock) rising: learn whatever is inside the centre reticle now.
// SW2 falling: drop the model for one frame, then return to automatic mode.
// acquire is a level held until the tracker reports LEARN (handshake), so the
// tracker's own frame-start sampling decides the exact learning frame.
module video_track_manager #(
 parameter integer WIDTH=1280, HEIGHT=720,
 parameter integer LOCK_HALF=64,      // centre reticle half size
 parameter integer CONFIRM=3,         // consecutive motion frames before learning
 parameter integer MIN_SIZE=16, MAX_SIZE=480,RETRY_FRAMES=60
)(
 input wire clk,rst_n,vs,de,
 input wire system_on,lock,
 input wire motion_valid,motion_observed,
 input wire [11:0] motion_x0,motion_y0,motion_x1,motion_y1,
 input wire [2:0] tracker_status,
 output wire tracker_enable,
 output reg acquire,
 output reg [11:0] acq_x0,acq_y0,acq_x1,acq_y1,
 output reg lock_mode
);
 localparam ST_IDLE=0,ST_LEARN=1,ST_SEARCH=4;
 reg vs_prev,de_prev,lock_prev,cancel;
 reg [7:0] de_history;
 reg [3:0] motion_frames;
 reg [7:0] retry_frames;
 wire start=vs && !vs_prev && !de && de_history==0;
 wire [11:0] motion_w=motion_x1-motion_x0+1'b1, motion_h=motion_y1-motion_y0+1'b1;
 wire motion_ok=motion_valid && motion_observed && motion_x1>motion_x0 && motion_y1>motion_y0 &&
                motion_w>=MIN_SIZE && motion_h>=MIN_SIZE && motion_w<=MAX_SIZE && motion_h<=MAX_SIZE;
 // Preserve the current model while SEARCH attempts to find the same target.
 wire tracker_free=tracker_status==ST_IDLE;
 assign tracker_enable=(system_on || lock) && !cancel;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   vs_prev<=0;de_prev<=0;lock_prev<=0;cancel<=0;de_history<=0;motion_frames<=0;
   acquire<=0;acq_x0<=0;acq_y0<=0;acq_x1<=0;acq_y1<=0;lock_mode<=0;retry_frames<=0;
  end else begin
   vs_prev<=vs;de_prev<=de;de_history<={de_history[6:0],de};
   if(acquire && tracker_status==ST_LEARN)acquire<=0;
   if(start) begin
    cancel<=0;lock_prev<=lock;lock_mode<=lock;
    if(lock && tracker_status==5 && !acquire)begin
     if(retry_frames<RETRY_FRAMES)retry_frames<=retry_frames+1'b1;
    end else retry_frames<=0;
    motion_frames<=motion_ok?((motion_frames==4'hf)?motion_frames:motion_frames+1'b1):4'd0;
    if(lock && (!lock_prev || (tracker_status==5 && retry_frames>=RETRY_FRAMES))) begin
     acquire<=1;
     acq_x0<=WIDTH/2-LOCK_HALF;acq_x1<=WIDTH/2+LOCK_HALF-1;
     acq_y0<=HEIGHT/2-LOCK_HALF;acq_y1<=HEIGHT/2+LOCK_HALF-1;
    end else if(!lock && lock_prev) begin
     acquire<=0;cancel<=1;motion_frames<=0;
    end else if(!lock && system_on && !acquire && tracker_free && motion_ok && motion_frames>=CONFIRM-1) begin
     acquire<=1;acq_x0<=motion_x0;acq_y0<=motion_y0;acq_x1<=motion_x1;acq_y1<=motion_y1;
     motion_frames<=0;
    end else if(acquire && !lock && motion_ok) begin
     // Still waiting for the tracker: follow the newest motion box.
     acq_x0<=motion_x0;acq_y0<=motion_y0;acq_x1<=motion_x1;acq_y1<=motion_y1;
    end
    if(!system_on && !lock)acquire<=0;
   end
  end
 end
endmodule
