// V21: camera-domain learned colour + spatial components. One sample per
// 4x4 sensor pixels, 16x16 component grid. Complete components are associated
// with the previous reliable target; equally plausible candidates HOLD.
// No display clock/frame buffer in the control path. Bounded BFS aborts on
// a new frame rather than issuing an observation from incomplete work.
module servo_stream_tracker #(
 parameter WIDTH=1280,HEIGHT=720,MIN_CELLS=2,MAX_JUMP=128,
 parameter CONFIRM_FRAMES=3,HOLD_FRAMES=30,SEARCH_FRAMES=600,AMBIGUITY_GAP=24,
 parameter CELL_MIN_HITS=4
)(
 input wire clk,rst_n,enable,acquire,
 input wire [11:0] acq_x0,acq_y0,acq_x1,acq_y1,
 input wire frame_start,frame_end,frame_bad,sample_valid,
 input wire [11:0] sample_x,sample_y,input wire [23:0] rgb,
 input wire [15:0] frame_id,
 output reg result_valid,output reg [15:0] result_frame,
 output reg [2:0] status,output reg track_valid,track_observed,
 output reg [1:0] quality,output reg ambiguous,
 output reg [11:0] x0,y0,x1,y1,cx,cy,
 output reg [14:0] model,output reg [3:0] color_id,
 output reg [15:0] overruns,output reg [15:0] decision_cycles
);
 localparam COLS=WIDTH/16,ROWS=HEIGHT/16,CELLS=COLS*ROWS;
 localparam AW=$clog2(CELLS),XW=$clog2(COLS),YW=$clog2(ROWS),QW=XW+YW;
 localparam IDLE=0,COPY_R=1,COPY_W=2,ROOT_R=3,ROOT_C=4,POP_R=5,POP_U=6,
            NEIGHBOR=7,COMPLETE=8,DECIDE=9,PUBLISH=10,LEARN_R=11,
            LEARN_M=12,LEARN_BIN=13,LEARN_END=14,POP_META=15;
 reg [3:0] state;
 reg [15:0] work_cycles;
 reg learning,acquire_seen,frame_on,frame_invalid,have_reference;
 reg [11:0] lx0,ly0,lx1,ly1,reference_w,reference_h,reference_x,reference_y;
 reg [11:0] bx0,by0,bx1,by1;
 reg signed [12:0] vx,vy;
 reg [9:0] missing;
 reg [3:0] confirmation;
 reg [11:0] confirm_x,confirm_y;
 reg [15:0] sampled,target_total,reference_total;
 reg [15:0] target_hist[0:14],reference_hist[0:14];
 reg [3:0] bin_index,best_bin;
 reg [15:0] best_bin_count,coverage,m_t,m_f;
 reg [31:0] product_t,product_f;
 reg [4:0] multiply_step;
 wire [3:0] bin;
 servo_color_bin u_bin(.rgb(rgb),.bin(bin));
 wire inside_learning=sample_x>=lx0 && sample_x<=lx1 && sample_y>=ly0 && sample_y<=ly1;
 wire inside_background=sample_x>=bx0 && sample_x<=bx1 && sample_y>=by0 && sample_y<=by1 && !inside_learning;
 wire [12:0] acq_w={1'b0,acq_x1}-{1'b0,acq_x0}+1'b1;
 wire [12:0] acq_h={1'b0,acq_y1}-{1'b0,acq_y0}+1'b1;
 wire pixel=sample_valid && frame_on && !frame_invalid && state==IDLE;
 wire hit=pixel && !learning && model[bin];
 reg [2:0] row_hits;
 reg [1:0] row_min_x,row_max_x;
 wire [AW-1:0] pixel_address=(sample_y>>4)*COLS+(sample_x>>4);
 // Count plus subcell bounds. Bounds have 4-pixel resolution instead of
 // padding every component to complete 16x16 cells.
 wire [12:0] count_data;
 wire [4:0] previous_count=(sample_y[3:0]==0)?5'd0:count_data[4:0];
 wire [4:0] next_count=previous_count+row_hits+hit;
 wire row_nonempty=(row_hits!=0)||hit;
 wire [1:0] row_lo=(hit && sample_x[3:2]<row_min_x)?sample_x[3:2]:row_min_x;
 wire [1:0] row_hi=(hit && sample_x[3:2]>row_max_x)?sample_x[3:2]:row_max_x;
 wire [1:0] cell_l=previous_count==0 ? row_lo :
                         (row_nonempty && row_lo<count_data[6:5] ? row_lo : count_data[6:5]);
 wire [1:0] cell_r=previous_count==0 ? row_hi :
                         (row_nonempty && row_hi>count_data[8:7] ? row_hi : count_data[8:7]);
 wire [1:0] cell_t=previous_count==0 ? sample_y[3:2] : count_data[10:9];
 wire [1:0] cell_b=row_nonempty ? sample_y[3:2] : count_data[12:11];
 wire [12:0] count_word=next_count==0 ? 13'd0 : {cell_b,cell_t,cell_r,cell_l,next_count};
 wire pixel_read=pixel && sample_x[3:0]==0;
 wire pixel_write=pixel && sample_x[3:0]==12;
 reg [AW-1:0] scan_index,n_address,copy_write_index;
 reg [XW-1:0] scan_x,pop_x,read_x;
 reg [YW-1:0] scan_y,pop_y,read_y;
 reg [AW:0] head,tail,component_count;
 reg [1:0] direction;
 reg read_valid,n_valid;
 wire [QW-1:0] queued;
 reg [XW-1:0] n_x;reg [YW-1:0] n_y;
 wire [XW-1:0] queued_x=queued[XW-1:0];
 wire [YW-1:0] queued_y=queued[QW-1:XW];
 wire mask_data;
 wire root_hit=state==ROOT_C && mask_data;
 wire enqueue=state==NEIGHBOR && n_valid && mask_data;
 wire [AW+1:0] tail_next={1'b0,tail}+enqueue;
 function [AW-1:0] address;
  input [XW-1:0] ax;input [YW-1:0] ay;
  begin address=ay*COLS+ax;end
 endfunction
 always @*begin
  read_x=pop_x;read_y=pop_y;read_valid=0;
  if(state==POP_META)begin read_x=pop_x-1'b1;read_y=pop_y;read_valid=pop_x!=0;end
  else if(state==NEIGHBOR)case(direction)
   0:begin read_x=pop_x+1'b1;read_valid=pop_x<COLS-1;end
   1:begin read_y=pop_y-1'b1;read_valid=pop_y!=0;end
   2:begin read_y=pop_y+1'b1;read_valid=pop_y<ROWS-1;end
   default:read_valid=0;
  endcase
 end
 video_target_ram #(.WIDTH(13),.DEPTH(CELLS)) u_counts(
  .clk(clk),.read_enable(pixel_read || state==COPY_R || state==COPY_W || state==POP_U),
  .read_address(pixel_read?pixel_address:(state==POP_U?address(queued_x,queued_y):scan_index)),.read_data(count_data),
  .write_enable(pixel_write),.write_address(pixel_address),.write_data(count_word));
 video_target_ram #(.WIDTH(1),.DEPTH(CELLS)) u_mask(
  .clk(clk),.read_enable(state==ROOT_R || state==POP_META || (state==NEIGHBOR && direction!=3)),
  .read_address(state==ROOT_R?scan_index:(read_valid?address(read_x,read_y):{AW{1'b0}})),
  .read_data(mask_data),.write_enable(state==COPY_W || root_hit || enqueue),
  .write_address(state==COPY_W?copy_write_index:(root_hit?scan_index:n_address)),
  .write_data(state==COPY_W && count_data[4:0]>=CELL_MIN_HITS));
 video_target_ram #(.WIDTH(QW),.DEPTH(CELLS)) u_queue(
  .clk(clk),.read_enable(state==POP_R),.read_address(head[AW-1:0]),.read_data(queued),
  .write_enable(root_hit || (enqueue && tail<CELLS)),
  .write_address(root_hit?{AW{1'b0}}:tail[AW-1:0]),
  .write_data(root_hit?{scan_y,scan_x}:{n_y,n_x}));
 // Queue data describes the neighbour whose synchronous mask read completed.
 reg [XW-1:0] cl,cr;
 reg [YW-1:0] ct,cb;
 reg [11:0] fine_l,fine_r,fine_t,fine_b;
 reg seed_touched;
 reg [11:0] chosen_x0,chosen_y0,chosen_x1,chosen_y1;
 reg [12:0] best_distance,second_distance;
 reg candidate_found;
 wire [11:0] component_x0=fine_l,component_x1=fine_r;
 wire [11:0] component_y0=fine_t,component_y1=fine_b;
 wire [11:0] cell_x0=(pop_x<<4)+{count_data[6:5],2'b00};
 wire [11:0] cell_x1=(pop_x<<4)+{count_data[8:7],2'b11};
 wire [11:0] cell_y0=(pop_y<<4)+{count_data[10:9],2'b00};
 wire [11:0] cell_y1=(pop_y<<4)+{count_data[12:11],2'b11};
 wire [12:0] component_sum_x={1'b0,component_x0}+component_x1;
 wire [12:0] component_sum_y={1'b0,component_y0}+component_y1;
 wire [11:0] component_cx=component_sum_x[12:1],component_cy=component_sum_y[12:1];
 wire [11:0] component_w=component_x1-component_x0+1'b1;
 wire [11:0] component_h=component_y1-component_y0+1'b1;
 wire [XW:0] tile_w={1'b0,cr}-{1'b0,cl}+1'b1;
 wire [YW:0] tile_h={1'b0,cb}-{1'b0,ct}+1'b1;
 wire [XW+YW+1:0] tile_area=tile_w*tile_h;
 wire compact=({component_count,2'b00}>=tile_area);
 function [11:0] predict;
  input [11:0] position;input signed [12:0] velocity;input integer limit;
  reg signed [13:0] p;
  begin p=$signed({2'b00,position})+velocity;predict=(p<0)?0:(p>=limit)?limit-1:p[11:0];end
 endfunction
 wire [11:0] prediction_x=predict(reference_x,vx,WIDTH),prediction_y=predict(reference_y,vy,HEIGHT);
 function [12:0] distance;
  input [11:0] ax,ay,bx,by;
  reg [12:0] dx,dy;
  begin dx=ax>bx?ax-bx:bx-ax;dy=ay>by?ay-by:by-ay;distance=dx+dy;end
 endfunction
 wire [12:0] component_distance=distance(component_cx,component_cy,prediction_x,prediction_y);
 wire [12:0] gate=missing>=HOLD_FRAMES?WIDTH+HEIGHT:MAX_JUMP+{missing[4:0],3'b000};
 wire [14:0] cw3={3'b000,component_w}+{2'b00,component_w,1'b0};
 wire [14:0] ch3={3'b000,component_h}+{2'b00,component_h,1'b0};
 wire [14:0] rw3={3'b000,reference_w}+{2'b00,reference_w,1'b0};
 wire [14:0] rh3={3'b000,reference_h}+{2'b00,reference_h,1'b0};
 wire size_ok=!have_reference ||
  (cw3>={2'b00,reference_w,1'b0} && {2'b00,component_w,1'b0}<=rw3 &&
   ch3>={2'b00,reference_h,1'b0} && {2'b00,component_h,1'b0}<=rh3);
 wire initial_size_ok=have_reference || (component_w<=WIDTH*3/4 && component_h<=HEIGHT*3/4);
 wire anchor_ok=have_reference || confirmation!=0 || seed_touched;
 wire suitable=component_count>=MIN_CELLS && compact && size_ok && initial_size_ok && anchor_ok && component_distance<=gate;
 wire [12:0] chosen_sum_x={1'b0,chosen_x0}+chosen_x1,chosen_sum_y={1'b0,chosen_y0}+chosen_y1;
 wire [11:0] chosen_cx=chosen_sum_x[12:1],chosen_cy=chosen_sum_y[12:1];
 wire candidate_ambiguous=candidate_found && second_distance!=13'h1fff && second_distance-best_distance<=AMBIGUITY_GAP;
 integer i;
 function [3:0] color_of;
  input [3:0] value;
  begin case(value)
   0,11:color_of=1;1:color_of=2;2:color_of=3;3,4:color_of=4;5,6:color_of=5;
   7,8:color_of=6;9,10:color_of=7;12:color_of=8;13:color_of=9;14:color_of=10;
   default:color_of=0;
  endcase end
 endfunction
 function signed [12:0] velocity;
  input [11:0] current,previous;
  reg signed [12:0] d;
  begin d=$signed({1'b0,current})-$signed({1'b0,previous});velocity=d>64?64:d< -64?-64:d;end
 endfunction
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;learning<=0;acquire_seen<=0;frame_on<=0;frame_invalid<=0;have_reference<=0;
   lx0<=0;ly0<=0;lx1<=0;ly1<=0;reference_w<=0;reference_h<=0;reference_x<=0;reference_y<=0;
   bx0<=0;by0<=0;bx1<=0;by1<=0;
   vx<=0;vy<=0;missing<=0;confirmation<=0;confirm_x<=0;confirm_y<=0;
   sampled<=0;target_total<=0;reference_total<=0;bin_index<=0;best_bin<=0;
   best_bin_count<=0;coverage<=0;m_t<=0;m_f<=0;product_t<=0;product_f<=0;multiply_step<=0;
   row_hits<=0;row_min_x<=0;row_max_x<=0;scan_index<=0;copy_write_index<=0;scan_x<=0;scan_y<=0;head<=0;tail<=0;component_count<=0;
   pop_x<=0;pop_y<=0;direction<=0;n_valid<=0;n_address<=0;n_x<=0;n_y<=0;
   cl<=0;cr<=0;ct<=0;cb<=0;chosen_x0<=0;chosen_y0<=0;chosen_x1<=0;chosen_y1<=0;
   fine_l<=0;fine_r<=0;fine_t<=0;fine_b<=0;seed_touched<=0;
   best_distance<=13'h1fff;second_distance<=13'h1fff;candidate_found<=0;
   result_valid<=0;result_frame<=0;status<=0;track_valid<=0;track_observed<=0;quality<=0;ambiguous<=0;
   x0<=0;y0<=0;x1<=0;y1<=0;cx<=0;cy<=0;model<=0;color_id<=0;overruns<=0;decision_cycles<=0;work_cycles<=0;
   for(i=0;i<15;i=i+1)begin target_hist[i]<=0;reference_hist[i]<=0;end
  end else begin
   result_valid<=0;
   if(state!=IDLE && work_cycles!=16'hffff)work_cycles<=work_cycles+1'b1;
   if(!acquire)acquire_seen<=0;
   if(pixel)begin
    sampled<=sampled+1'b1;
    if(sample_x[3:0]==0)begin
     row_hits<=hit;row_min_x<=hit?sample_x[3:2]:2'd3;row_max_x<=hit?sample_x[3:2]:2'd0;
    end else begin
     row_hits<=row_hits+hit;
     if(hit && sample_x[3:2]<row_min_x)row_min_x<=sample_x[3:2];
     if(hit && sample_x[3:2]>row_max_x)row_max_x<=sample_x[3:2];
    end
    if(learning)begin
     if(inside_learning)begin target_hist[bin]<=target_hist[bin]+1'b1;target_total<=target_total+1'b1;end
     else if(inside_background)begin reference_hist[bin]<=reference_hist[bin]+1'b1;reference_total<=reference_total+1'b1;end
    end
   end
   if(state==POP_META || (state==NEIGHBOR && direction!=3))begin
    n_x<=read_x;n_y<=read_y;n_valid<=read_valid;n_address<=address(read_x,read_y);
   end
   case(state)
    IDLE:;
    // Prime the synchronous count read, then overlap the next count read
    // with the previous mask write. Exactly one mask entry per clock.
    COPY_R:begin copy_write_index<=0;scan_index<=CELLS>1?1:0;state<=COPY_W;end
    COPY_W:begin
     if(copy_write_index==CELLS-1)begin scan_index<=0;scan_x<=0;scan_y<=0;state<=ROOT_R;end
     else begin
      copy_write_index<=copy_write_index+1'b1;
      if(scan_index<CELLS-1)scan_index<=scan_index+1'b1;
     end
    end
    ROOT_R:state<=ROOT_C;
    ROOT_C:begin
     if(mask_data)begin
      head<=0;tail<=1;component_count<=0;cl<=COLS-1;cr<=0;ct<=ROWS-1;cb<=0;
      fine_l<=WIDTH-1;fine_r<=0;fine_t<=HEIGHT-1;fine_b<=0;seed_touched<=0;state<=POP_R;
     end
     else if(scan_index==CELLS-1)state<=DECIDE;
     else begin scan_index<=scan_index+1'b1;if(scan_x==COLS-1)begin scan_x<=0;scan_y<=scan_y+1'b1;end else scan_x<=scan_x+1'b1;state<=ROOT_R;end
    end
    POP_R:state<=POP_U;
    POP_U:begin
     pop_x<=queued_x;pop_y<=queued_y;head<=head+1'b1;component_count<=component_count+1'b1;
     if(queued_x<cl)cl<=queued_x;if(queued_x>cr)cr<=queued_x;
     if(queued_y<ct)ct<=queued_y;if(queued_y>cb)cb<=queued_y;
     state<=POP_META;
    end
    POP_META:begin
     if(cell_x0<fine_l)fine_l<=cell_x0;if(cell_x1>fine_r)fine_r<=cell_x1;
     if(cell_y0<fine_t)fine_t<=cell_y0;if(cell_y1>fine_b)fine_b<=cell_y1;
     if(cell_x1>=lx0 && cell_x0<=lx1 && cell_y1>=ly0 && cell_y0<=ly1)seed_touched<=1;
     direction<=0;state<=NEIGHBOR;
    end
    NEIGHBOR:begin
     if(enqueue)begin
      if(tail<CELLS)tail<=tail+1'b1;
      else begin state<=PUBLISH;quality<=0;track_observed<=0;ambiguous<=1;if(overruns!=16'hffff)overruns<=overruns+1'b1;end
     end
     if(direction==3)state<=({1'b0,head}<tail_next)?POP_R:COMPLETE;
     else direction<=direction+1'b1;
    end
    COMPLETE:begin
     if(suitable)begin
      if(!candidate_found || component_distance<best_distance)begin
       if(candidate_found)second_distance<=best_distance;
       candidate_found<=1;best_distance<=component_distance;
       chosen_x0<=component_x0;chosen_y0<=component_y0;chosen_x1<=component_x1;chosen_y1<=component_y1;
      end else if(component_distance<second_distance)second_distance<=component_distance;
     end
     if(scan_index==CELLS-1)state<=DECIDE;
     else begin scan_index<=scan_index+1'b1;if(scan_x==COLS-1)begin scan_x<=0;scan_y<=scan_y+1'b1;end else scan_x<=scan_x+1'b1;state<=ROOT_R;end
    end
    DECIDE:begin
     state<=PUBLISH;track_observed<=0;quality<=0;ambiguous<=candidate_ambiguous;
     if(candidate_found && !candidate_ambiguous)begin
      confirm_x<=chosen_cx;confirm_y<=chosen_cy;
      if((have_reference && missing==0) || CONFIRM_FRAMES<=1 ||
         (confirmation>=CONFIRM_FRAMES-1 && distance(chosen_cx,chosen_cy,confirm_x,confirm_y)<=MAX_JUMP))begin
       x0<=chosen_x0;y0<=chosen_y0;x1<=chosen_x1;y1<=chosen_y1;cx<=chosen_cx;cy<=chosen_cy;
       reference_w<=chosen_x1-chosen_x0+1'b1;reference_h<=chosen_y1-chosen_y0+1'b1;
       vx<=(have_reference && missing==0)?velocity(chosen_cx,reference_x):13'sd0;
       vy<=(have_reference && missing==0)?velocity(chosen_cy,reference_y):13'sd0;
       reference_x<=chosen_cx;reference_y<=chosen_cy;have_reference<=1;
       missing<=0;confirmation<=0;track_valid<=1;track_observed<=1;quality<=2;status<=2;
      end else begin
       confirmation<=(confirmation!=0 && distance(chosen_cx,chosen_cy,confirm_x,confirm_y)<=MAX_JUMP)?confirmation+1'b1:4'd1;
       quality<=1;status<=3;track_valid<=have_reference && missing<HOLD_FRAMES;
       if(missing!=1023)missing<=missing+1'b1;
      end
     end else begin
      confirmation<=0;vx<=0;vy<=0;if(missing!=1023)missing<=missing+1'b1;
      status<=missing<HOLD_FRAMES?3:4;track_valid<=have_reference && missing<HOLD_FRAMES;
      if(missing>=HOLD_FRAMES+SEARCH_FRAMES-1)begin model<=0;status<=0;track_valid<=0;have_reference<=0;end
     end
    end
    PUBLISH:begin result_valid<=1;decision_cycles<=work_cycles;state<=IDLE;end
    LEARN_R:begin m_t<=target_hist[bin_index];m_f<=reference_hist[bin_index];product_t<=0;product_f<=0;multiply_step<=0;state<=LEARN_M;end
    LEARN_M:begin
     if(reference_total[multiply_step[3:0]])product_t<=product_t+({16'd0,m_t}<<multiply_step);
     if(target_total[multiply_step[3:0]])product_f<=product_f+({16'd0,m_f}<<multiply_step);
     if(multiply_step==15)state<=LEARN_BIN;else multiply_step<=multiply_step+1'b1;
    end
    LEARN_BIN:begin
     if(m_t!=0 && ({m_t,6'b000000}>=target_total) && {1'b0,product_t}>{product_f,1'b0})begin
      model[bin_index]<=1;coverage<=coverage+m_t;
      if(m_t>best_bin_count)begin best_bin_count<=m_t;best_bin<=bin_index;end
     end
     if(bin_index==14)state<=LEARN_END;else begin bin_index<=bin_index+1'b1;state<=LEARN_R;end
    end
    LEARN_END:begin
     learning<=0;state<=PUBLISH;missing<=0;confirmation<=0;quality<=0;
     if(target_total>=8 && reference_total>=8 && model!=0 && {coverage,2'b00}>=target_total)begin status<=3;color_id<=color_of(best_bin);end
     else begin model<=0;status<=5;color_id<=0;end
    end
    default:state<=IDLE;
   endcase
   if(frame_end)begin
    result_frame<=frame_id;frame_on<=0;work_cycles<=0;
    if(frame_on && !frame_invalid && !frame_bad && sampled==(WIDTH/4)*(HEIGHT/4) && state==IDLE)begin
     if(learning)begin bin_index<=0;coverage<=0;best_bin_count<=0;model<=0;state<=LEARN_R;end
     else if(model!=0)begin scan_index<=0;scan_x<=0;scan_y<=0;candidate_found<=0;best_distance<=13'h1fff;second_distance<=13'h1fff;state<=COPY_R;end
     else begin track_observed<=0;quality<=0;state<=PUBLISH;end
    end else begin
     track_observed<=0;quality<=0;ambiguous<=0;status<=have_reference?3:0;state<=PUBLISH;confirmation<=0;
     if(missing!=1023)missing<=missing+1'b1;
    end
   end
   if(frame_start)begin
    sampled<=0;row_hits<=0;frame_on<=enable;frame_invalid<=0;
    if(state!=IDLE)begin
     state<=IDLE;frame_invalid<=1;track_observed<=0;quality<=0;
     if(overruns!=16'hffff)overruns<=overruns+1'b1;
    end
    if(!enable)begin
     model<=0;learning<=0;have_reference<=0;missing<=0;confirmation<=0;
     track_valid<=0;track_observed<=0;quality<=0;ambiguous<=0;status<=0;
    end else if(acquire && !acquire_seen)begin
     acquire_seen<=1;learning<=1;model<=0;have_reference<=0;track_valid<=0;track_observed<=0;
     quality<=0;ambiguous<=0;status<=1;result_valid<=1;result_frame<=frame_id;
     lx0<=acq_x0+((acq_x1-acq_x0)>>2);lx1<=acq_x1-((acq_x1-acq_x0)>>2);
     ly0<=acq_y0+((acq_y1-acq_y0)>>2);ly1<=acq_y1-((acq_y1-acq_y0)>>2);
     bx0<=acq_x0>acq_w?acq_x0-acq_w:0;by0<=acq_y0>acq_h?acq_y0-acq_h:0;
     bx1<=({1'b0,acq_x1}+acq_w>=WIDTH)?WIDTH-1:acq_x1+acq_w;
     by1<=({1'b0,acq_y1}+acq_h>=HEIGHT)?HEIGHT-1:acq_y1+acq_h;
     reference_x<=({1'b0,acq_x0}+acq_x1)>>1;reference_y<=({1'b0,acq_y0}+acq_y1)>>1;
     vx<=0;vy<=0;target_total<=0;reference_total<=0;
     for(i=0;i<15;i=i+1)begin target_hist[i]<=0;reference_hist[i]<=0;end
    end
   end
  end
 end
endmodule
