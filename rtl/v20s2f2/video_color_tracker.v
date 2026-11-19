// V19 color-histogram target tracker. Works while the camera moves.
// LEARN: one frame builds a 51-bin color histogram of the inner half of the
// acquisition box (target) and of the whole frame (reference). Bins that are
// RATIO times denser inside the target form a binary color model.
// TRACK: every frame, model-colored pixels inside a search window contribute
// count, sum and sum-of-squares moments (one mean-shift step per frame). The
// centroid and 1.75-sigma extents give the box; the next window follows it.
// All per-frame arithmetic is sequential and finishes early in vertical
// blank; outputs change only there, never inside active video.
// Bins 0..47: 24 hue sectors (15 degrees) x 2 saturation levels, found with
// comparisons only (no divider). Bins 48..50: dark, gray, bright achromatic.
module video_color_tracker #(
 parameter integer WIDTH=1280, HEIGHT=720,
 parameter integer MIN_PIXELS=48,     // paired model pixels per observation
 parameter integer HOLD_FRAMES=30,    // keep the last box while unobserved
 parameter integer SEARCH_FRAMES=600, // full-frame search before dropping the model
 parameter integer RATIO=2,           // target vs frame density for a model bin
 parameter integer COVER_NUM=1, COVER_DEN=4, // model must explain >=25% of target
 parameter integer MIN_BIN_SHIFT=6,   // a model bin holds >= 1/64 of the target
 parameter integer REJECT_FRAMES=60,  // show NO MODEL before returning to idle
 parameter integer EX_X0=0, EX_Y0=0, EX_X1=0, EX_Y1=0 // ignored rectangle [x0,x1)
)(
 input wire clk,rst_n,enable,vs,de,
 input wire [23:0] rgb,
 input wire acquire,                   // level, sampled at frame start
 input wire [11:0] acq_x0,acq_y0,acq_x1,acq_y1,
 output reg [2:0] status,              // ST_* below
 output reg track_valid,track_observed,
 output reg [11:0] x0,y0,x1,y1,cx,cy,
 output reg [15:0] cx_bcd,cy_bcd,
 output reg [3:0] color_id,            // 0 none,1 red,2 orange,3 yellow,4 green,5 cyan,
                                       // 6 blue,7 purple,8 black,9 gray,10 white
 // Diagnostics of the last learning frame, BCD: target samples, share of the
 // target explained by the model (%), number of model bins.
 output reg model_info,
 output reg [15:0] model_t_bcd,
 output reg [7:0] model_pct_bcd,model_n_bcd,
 output wire track_mask               // model pixel counted this clock (stage C)
);
 localparam ST_IDLE=0,ST_LEARN=1,ST_TRACK=2,ST_HOLD=3,ST_SEARCH=4,ST_REJECT=5;
 localparam M_IDLE=0,M_LEARN=1,M_TRACK=2;
 localparam F_IDLE=0,F_SCAN_R=1,F_SCAN_D=2,F_SCAN_M=3,F_MODEL=4,F_DIV=5,F_SQ=6,F_SQRT=7,
            F_UPDATE=8,F_BCD=9,F_PUBLISH=10,F_PCT=11,F_CLEAR=15;
 localparam HALF_MIN=6, HALF_MAX=WIDTH/2;

 // ---------------- pixel coordinates ----------------
 reg [11:0] px_reg,py;
 reg de_prev,vs_prev,bad;
 reg [7:0] de_history;
 reg [3:0] end_pipe;                   // frame end, delayed past the pixel pipeline
 reg [21:0] sq_reg,ysq;
 wire [11:0] px=de_prev?px_reg:12'd0;
 wire [21:0] xsq=de_prev?sq_reg:22'd0; // px squared, built incrementally
 wire start=vs && !vs_prev && !de && de_history==0;
 wire frame_end=de_prev && !de && py==HEIGHT-1;

 // ---------------- stage B: ordering of R,G,B ----------------
 wire [7:0] r=rgb[23:16],g=rgb[15:8],b=rgb[7:0];
 reg [2:0] sector_b;
 reg [7:0] max_b,c_b,d_b;
 reg asc_b,de_b;
 reg [11:0] x_b,y_b;
 reg [21:0] xsq_b,ysq_b;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin sector_b<=0;max_b<=0;c_b<=0;d_b<=0;asc_b<=0;de_b<=0;x_b<=0;y_b<=0;xsq_b<=0;ysq_b<=0; end
  else begin
   de_b<=de;x_b<=px;y_b<=py;xsq_b<=xsq;ysq_b<=ysq;
   // Hue sector by channel order; c=max-min, d=mid-min.
   if(r>=g && g>=b)      begin sector_b<=0;max_b<=r;c_b<=r-b;d_b<=g-b;asc_b<=1; end
   else if(g>r && r>=b)  begin sector_b<=1;max_b<=g;c_b<=g-b;d_b<=r-b;asc_b<=0; end
   else if(g>=b && b>r)  begin sector_b<=2;max_b<=g;c_b<=g-r;d_b<=b-r;asc_b<=1; end
   else if(b>g && g>r)   begin sector_b<=3;max_b<=b;c_b<=b-r;d_b<=g-r;asc_b<=0; end
   else if(b>r && r>=g)  begin sector_b<=4;max_b<=b;c_b<=b-g;d_b<=r-g;asc_b<=1; end
   else                  begin sector_b<=5;max_b<=r;c_b<=r-g;d_b<=b-g;asc_b<=0; end
  end
 end

 // ---------------- stage C: 6-bit color bin ----------------
 wire [9:0] d4={d_b,2'b00}, c1={2'b00,c_b}, c2={1'b0,c_b,1'b0}, c3=c1+c2;
 wire [1:0] q=(d4>=c3)?2'd3:(d4>=c2)?2'd2:(d4>=c1)?2'd1:2'd0;   // d/c in quarters
 wire [1:0] sub=asc_b?q:2'd3-q;
 wire [4:0] hue={sector_b,2'b00}+sub;
 // The raw camera RGB (no color-correction matrix, before the display lift)
 // is dark and weakly saturated: decide on relative saturation S>=1/16 with a
 // small noise floor. Gray noise spreads over many hue bins and is not learned.
 wire chroma=max_b>=8'd16 && c_b>=8'd8 && {c_b,4'b0000}>={4'b0000,max_b};
 wire sat_high={c_b,1'b0}>={1'b0,max_b};                                // S>=1/2
 // 12 hue bins of 30 degrees (0..11) keep a real object's shading in few
 // bins; 12/13/14 are dark, gray and bright achromatic.
 wire [5:0] bin_value=chroma?{2'b00,hue[4:1]}:(max_b<8'd48)?6'd12:(max_b<8'd160)?6'd13:6'd14;
 reg [5:0] bin_c;
 reg de_c;
 reg [11:0] x_c,y_c;
 reg [21:0] xsq_c,ysq_c;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin bin_c<=0;de_c<=0;x_c<=0;y_c<=0;xsq_c<=0;ysq_c<=0; end
  else begin bin_c<=bin_value;de_c<=de_b;x_c<=x_b;y_c<=y_b;xsq_c<=xsq_b;ysq_c<=ysq_b; end
 end
 wire excluded_c=(EX_X1>EX_X0) && x_c>=EX_X0 && x_c<EX_X1 && y_c>=EX_Y0 && y_c<EX_Y1;

 // ---------------- learning histograms ----------------
 reg [1:0] mode;
 reg [3:0] fstate;
 reg [5:0] scan_bin;
 reg [11:0] learn_x0,learn_y0,learn_x1,learn_y1;
 reg [11:0] win_x0,win_y0,win_x1,win_y1;
 // Every second pixel of every second row: read-modify-write needs two clocks.
 wire learn_sample=de_c && mode==M_LEARN && !x_c[0] && !y_c[0] && !excluded_c && !bad;
 wire in_learn_box=x_c>=learn_x0 && x_c<=learn_x1 && y_c>=learn_y0 && y_c<=learn_y1;
 reg sample_d,inside_d;
 reg [5:0] bin_d;
 reg [17:0] target_total,frame_total;
 wire scan_read=fstate==F_SCAN_R;
 wire hist_clear=fstate==F_CLEAR;
 wire [17:0] target_count,frame_count;
 video_target_ram #(.WIDTH(18),.DEPTH(64)) u_target_hist(
  .clk(clk),.read_enable(learn_sample || scan_read),.read_address(learn_sample?bin_c:scan_bin),
  .read_data(target_count),.write_enable(hist_clear || (sample_d && inside_d)),
  .write_address(hist_clear?scan_bin:bin_d),.write_data(hist_clear?18'd0:target_count+1'b1));
 video_target_ram #(.WIDTH(18),.DEPTH(64)) u_frame_hist(
  .clk(clk),.read_enable(learn_sample || scan_read),.read_address(learn_sample?bin_c:scan_bin),
  .read_data(frame_count),.write_enable(hist_clear || sample_d),
  .write_address(hist_clear?scan_bin:bin_d),.write_data(hist_clear?18'd0:frame_count+1'b1));

 // ---------------- tracking moments ----------------
 reg [63:0] model;
 wire in_window=x_c>=win_x0 && x_c<=win_x1 && y_c>=win_y0 && y_c<=win_y1;
 wire model_pixel=de_c && model[bin_c] && in_window && !excluded_c;
 reg previous_model;                   // a pixel counts only next to another model pixel
 wire counted=model_pixel && previous_model && mode==M_TRACK && !bad;
 assign track_mask=counted;
 reg [19:0] sum_n;
 reg [30:0] sum_x;
 reg [29:0] sum_y;
 reg [41:0] sum_xx;
 reg [39:0] sum_yy;

 // ---------------- per-frame sequential arithmetic ----------------
 reg [5:0] iter;
 reg job;                              // 0 = x axis, 1 = y axis
 reg [1:0] div_job;                    // sum_x, sum_y, sum_xx, sum_yy over N
 reg [21:0] div_q;
 reg [20:0] div_r;
 reg [19:0] div_v;
 wire [20:0] div_t={div_r[19:0],div_q[21]};
 wire div_take=div_t>={1'b0,div_v};
 wire [21:0] next_q={div_q[20:0],div_take};
 reg [17:0] m_t,m_f;                   // T[bin], F[bin]
 reg [37:0] prod_t,prod_f;             // T x frame_total, F x target_total
 reg [23:0] square;
 reg [21:0] sqrt_v,sqrt_res,sqrt_bit;
 reg [11:0] mean_x,mean_y;
 reg [21:0] ex2,ey2,var_x,var_y;
 reg [10:0] sigma_x,sigma_y;
 reg [17:0] cover,best_count;
 reg [5:0] best_bin;
 reg observed_now,acquiring;
 reg [11:0] half_w,half_h;             // smoothed half sizes
 reg [11:0] last_cx,last_cy;
 reg signed [12:0] vel_x,vel_y;
 reg first_track;
 reg [9:0] miss;
 reg [6:0] reject_count;
 reg [11:0] shift_x,shift_y;
 reg [15:0] bcd_x,bcd_y;
 reg [11:0] pend_x0,pend_y0,pend_x1,pend_y1,pend_cx,pend_cy;
 reg pend_valid,pend_observed;
 reg [2:0] pend_status;
 function [15:0] add3;
  input [15:0] value;integer j;
  begin add3=value;for(j=0;j<4;j=j+1)if(value[j*4+:4]>=5)add3[j*4+:4]=value[j*4+:4]+4'd3;end
 endfunction
 wire [15:0] adjusted_x=add3(bcd_x),adjusted_y=add3(bcd_y);
 function [15:0] bcd_inc;
  input [15:0] v; reg [15:0] r; integer j; reg carry;
  begin r=v;carry=1;
   for(j=0;j<4;j=j+1) if(carry) begin
    if(r[j*4+:4]==9) r[j*4+:4]=0; else begin r[j*4+:4]=r[j*4+:4]+1'b1;carry=0; end
   end
   bcd_inc=r;
  end
 endfunction
 reg pass2;
 reg [15:0] model1;
 wire [3:0] hue_prev=(scan_bin[3:0]==0)?4'd11:scan_bin[3:0]-1'b1;
 wire [3:0] hue_next=(scan_bin[3:0]==11)?4'd0:scan_bin[3:0]+1'b1;
 wire neighbor_selected=model1[hue_prev] || model1[hue_next];
 reg [15:0] t_bcd;
 reg [7:0] n_bcd,pct_bcd;
 reg [24:0] pct_acc;
 wire [24:0] pct_goal={1'b0,cover,6'b000000}+{2'b00,cover,5'b00000}+{5'b00000,cover,2'b00};
 function [3:0] color_of;
  input [5:0] bin;
  begin
   case(bin)
    0,11:color_of=1; 1:color_of=2; 2:color_of=3; 3,4:color_of=4; 5,6:color_of=5;
    7,8:color_of=6; 9,10:color_of=7; 12:color_of=8; 13:color_of=9; 14:color_of=10;
    default:color_of=0;
   endcase
  end
 endfunction
 function [11:0] clamp_low;
  input signed [13:0] v; begin clamp_low=(v<0)?12'd0:v[11:0]; end
 endfunction
 function [11:0] clamp_high;
  input signed [13:0] v; input integer limit; begin clamp_high=(v>=limit)?limit-1:(v<0)?12'd0:v[11:0]; end
 endfunction
 function signed [12:0] limit64;
  input signed [12:0] v; begin limit64=(v>64)?13'sd64:(v<-64)?-13'sd64:v; end
 endfunction
 // Half size = 1.75 sigma: a uniform blob of width W gives about 0.5 W.
 wire [11:0] hw_calc={1'b0,sigma_x}+{2'b00,sigma_x[10:1]}+{3'b000,sigma_x[10:2]};
 wire [11:0] hh_calc={1'b0,sigma_y}+{2'b00,sigma_y[10:1]}+{3'b000,sigma_y[10:2]};
 wire [11:0] hw_raw=(hw_calc<HALF_MIN)?HALF_MIN:(hw_calc>HALF_MAX)?HALF_MAX:hw_calc;
 wire [11:0] hh_raw=(hh_calc<HALF_MIN)?HALF_MIN:(hh_calc>HALF_MAX)?HALF_MAX:hh_calc;
 // Size follows observations through a 1/4 filter except on a fresh lock.
 wire fresh=first_track || miss>=HOLD_FRAMES;
 wire [11:0] hw_smooth=half_w+((hw_raw>half_w)?((hw_raw-half_w)>>2):-((half_w-hw_raw)>>2));
 wire [11:0] hh_smooth=half_h+((hh_raw>half_h)?((hh_raw-half_h)>>2):-((half_h-hh_raw)>>2));
 wire [11:0] box_w=fresh?hw_raw:hw_smooth, box_h=fresh?hh_raw:hh_smooth;
 wire [11:0] abs_vx=vel_x[12]?-vel_x:vel_x, abs_vy=vel_y[12]?-vel_y:vel_y;
 // Search window half size: twice the box, at least 48 px, plus the velocity;
 // each missed frame grows it by 32 px.
 wire [12:0] reach_x=((half_w<24)?13'd48:{half_w,1'b0})+abs_vx+{miss[6:0],5'b00000};
 wire [12:0] reach_y=((half_h<24)?13'd48:{half_h,1'b0})+abs_vy+{miss[6:0],5'b00000};
 wire signed [13:0] pred_x=$signed({2'b00,last_cx})+vel_x, pred_y=$signed({2'b00,last_cy})+vel_y;

 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   px_reg<=0;py<=0;de_prev<=0;vs_prev<=0;bad<=0;de_history<=0;end_pipe<=0;sq_reg<=0;ysq<=0;
   mode<=M_IDLE;fstate<=F_IDLE;scan_bin<=0;learn_x0<=0;learn_y0<=0;learn_x1<=0;learn_y1<=0;
   win_x0<=0;win_y0<=0;win_x1<=0;win_y1<=0;
   sample_d<=0;inside_d<=0;bin_d<=0;target_total<=0;frame_total<=0;model<=0;previous_model<=0;
   sum_n<=0;sum_x<=0;sum_y<=0;sum_xx<=0;sum_yy<=0;
   iter<=0;job<=0;div_job<=0;div_q<=0;div_r<=0;div_v<=0;m_t<=0;m_f<=0;prod_t<=0;prod_f<=0;square<=0;
   sqrt_v<=0;sqrt_res<=0;sqrt_bit<=0;mean_x<=0;mean_y<=0;ex2<=0;ey2<=0;var_x<=0;var_y<=0;
   sigma_x<=0;sigma_y<=0;cover<=0;best_count<=0;best_bin<=0;observed_now<=0;acquiring<=0;
   half_w<=0;half_h<=0;last_cx<=0;last_cy<=0;vel_x<=0;vel_y<=0;first_track<=0;miss<=0;reject_count<=0;
   shift_x<=0;shift_y<=0;bcd_x<=0;bcd_y<=0;
   pend_x0<=0;pend_y0<=0;pend_x1<=0;pend_y1<=0;pend_cx<=0;pend_cy<=0;
   pend_valid<=0;pend_observed<=0;pend_status<=ST_IDLE;
   status<=ST_IDLE;track_valid<=0;track_observed<=0;x0<=0;y0<=0;x1<=0;y1<=0;cx<=0;cy<=0;
   cx_bcd<=0;cy_bcd<=0;color_id<=0;
   pass2<=0;model1<=0;model_info<=0;model_t_bcd<=0;model_pct_bcd<=0;model_n_bcd<=0;t_bcd<=0;n_bcd<=0;pct_bcd<=0;pct_acc<=0;
  end else begin
   de_prev<=de;vs_prev<=vs;de_history<={de_history[6:0],de};
   end_pipe<={end_pipe[2:0],frame_end};
   if(de) begin
    px_reg<=px+1'b1;sq_reg<=xsq+{px,1'b0}+1'b1;
    if(px>=WIDTH || py>=HEIGHT)bad<=1;
   end else begin
    px_reg<=0;sq_reg<=0;
    if(de_prev) begin py<=py+1'b1;ysq<=ysq+{py,1'b0}+1'b1;if(px_reg!=WIDTH)bad<=1; end
   end
   sample_d<=learn_sample;inside_d<=learn_sample && in_learn_box;bin_d<=bin_c;
   if(learn_sample) begin
    frame_total<=frame_total+1'b1;
    if(in_learn_box) begin target_total<=target_total+1'b1;if(t_bcd!=16'h9999)t_bcd<=bcd_inc(t_bcd); end
   end
   if(de_c)previous_model<=model_pixel;else previous_model<=0;
   if(counted) begin
    sum_n<=sum_n+1'b1;sum_x<=sum_x+x_c;sum_y<=sum_y+y_c;sum_xx<=sum_xx+xsq_c;sum_yy<=sum_yy+ysq_c;
   end

   case(fstate)
    F_IDLE:;
    F_CLEAR: if(scan_bin==63) begin scan_bin<=0;fstate<=F_IDLE; end else scan_bin<=scan_bin+1'b1;
    // Model selection, bin by bin: T/target_total > RATIO x F/frame_total,
    // and the bin holds at least 1/32 of the target samples.
    F_SCAN_R: fstate<=F_SCAN_D;
    F_SCAN_D: begin m_t<=target_count;m_f<=frame_count;prod_t<=0;prod_f<=0;iter<=0;fstate<=F_SCAN_M; end
    F_SCAN_M: if(iter<18) begin
     if(frame_total[iter[4:0]])prod_t<=prod_t+({20'd0,m_t}<<iter[4:0]);
     if(target_total[iter[4:0]])prod_f<=prod_f+({20'd0,m_f}<<iter[4:0]);
     iter<=iter+1'b1;
    end else begin iter<=0;fstate<=F_MODEL; end
    // Pass 1 (bins 0..14): density ratio > RATIO. Pass 2 (hues 0..11): a hue
    // next to a pass-1 hue joins if it is merely denser on the target.
    F_MODEL: begin
     if(!model[scan_bin] && ({m_t,6'b000000}>>(6-MIN_BIN_SHIFT))>={6'd0,target_total} &&
        (pass2?(prod_t>prod_f && neighbor_selected):(prod_t>prod_f*RATIO))) begin
      model[scan_bin]<=1;cover<=cover+m_t;n_bcd<=bcd_inc({8'd0,n_bcd});
      if(m_t>best_count) begin best_count<=m_t;best_bin<=scan_bin; end
     end
     if(!pass2 && scan_bin==14) begin pass2<=1;model1<=model[15:0];scan_bin<=0;fstate<=F_SCAN_R; end
     else if(pass2 && scan_bin==11) begin pass2<=0;pct_acc<=0;pct_bcd<=0;fstate<=F_PCT; end
     else begin scan_bin<=scan_bin+1'b1;fstate<=F_SCAN_R; end
    end
    // Exact restoring division: centres fit 12 bits, second moments 22.
    // Seed the remainder with the high numerator bits, then process only
    // the quotient bits; completed-frame bounds guarantee seed < divisor.
    F_DIV: begin
     div_r<=div_take?div_t-{1'b0,div_v}:div_t;div_q<=next_q;
     if(iter==21) begin
      iter<=0;div_r<=0;
      case(div_job)
       2'd0:begin mean_x<=next_q[11:0];div_q<={sum_y[11:0],10'd0};div_r<=sum_y[29:12];iter<=10;div_job<=1; end
       2'd1:begin mean_y<=next_q[11:0];div_q<=sum_xx[21:0];div_r<=sum_xx[41:22];div_job<=2; end
       2'd2:begin ex2<=next_q[21:0];div_q<=sum_yy[21:0];div_r<=sum_yy[39:22];div_job<=3; end
       default:begin ey2<=next_q[21:0];div_job<=0;job<=0;square<=0;fstate<=F_SQ; end
      endcase
     end else iter<=iter+1'b1;
    end
    // Mean squared by shift-add, then variance = E[x^2] - mean^2.
    F_SQ: begin
     if(iter<12) begin
      if((job?mean_y[iter[3:0]]:mean_x[iter[3:0]]))square<=square+({12'd0,job?mean_y:mean_x}<<iter[3:0]);
      iter<=iter+1'b1;
     end else begin
      iter<=0;square<=0;
      if(!job) begin var_x<=(ex2>square[21:0])?ex2-square[21:0]:22'd0;job<=1; end
      else begin
       var_y<=(ey2>square[21:0])?ey2-square[21:0]:22'd0;job<=0;
       sqrt_v<=var_x;sqrt_res<=0;sqrt_bit<=22'h100000;fstate<=F_SQRT;
      end
     end
    end
    // Integer square root, two bits per step.
    F_SQRT: begin
     if(sqrt_bit!=0) begin
      if(sqrt_v>=sqrt_res+sqrt_bit) begin sqrt_v<=sqrt_v-(sqrt_res+sqrt_bit);sqrt_res<=(sqrt_res>>1)+sqrt_bit; end
      else sqrt_res<=sqrt_res>>1;
      sqrt_bit<=sqrt_bit>>2;
     end else if(!job) begin
      sigma_x<=sqrt_res[10:0];job<=1;sqrt_v<=var_y;sqrt_res<=0;sqrt_bit<=22'h100000;
     end else begin
      sigma_y<=sqrt_res[10:0];job<=0;fstate<=F_UPDATE;
     end
    end
    F_UPDATE: begin
     fstate<=F_BCD;
     pend_x0<=x0;pend_y0<=y0;pend_x1<=x1;pend_y1<=y1;pend_cx<=cx;pend_cy<=cy;
     pend_valid<=track_valid;pend_observed<=0;pend_status<=status;
     if(acquiring) begin
      // End of a learning frame: accept the model only if it explains enough
      // of the target and is distinct from the rest of the frame.
      acquiring<=0;
      model_info<=1;model_t_bcd<=t_bcd;model_pct_bcd<=pct_bcd;model_n_bcd<=n_bcd;
      if(target_total>=18'd32 && model!=0 && cover*COVER_DEN>=target_total*COVER_NUM) begin
       mode<=M_TRACK;first_track<=1;miss<=0;vel_x<=0;vel_y<=0;
       last_cx<=({1'b0,learn_x0}+{1'b0,learn_x1})>>1;last_cy<=({1'b0,learn_y0}+{1'b0,learn_y1})>>1;
       half_w<=learn_x1-learn_x0+1'b1;half_h<=learn_y1-learn_y0+1'b1;
       pend_status<=ST_TRACK;pend_valid<=0;color_id<=color_of(best_bin);
      end else begin
       mode<=M_IDLE;model<=0;pend_status<=ST_REJECT;pend_valid<=0;reject_count<=0;color_id<=0;
      end
     end else if(observed_now) begin
      miss<=0;first_track<=0;half_w<=box_w;half_h<=box_h;
      vel_x<=fresh?13'sd0:limit64($signed({1'b0,mean_x})-$signed({1'b0,last_cx}));
      vel_y<=fresh?13'sd0:limit64($signed({1'b0,mean_y})-$signed({1'b0,last_cy}));
      last_cx<=mean_x;last_cy<=mean_y;
      pend_valid<=1;pend_observed<=1;pend_status<=ST_TRACK;pend_cx<=mean_x;pend_cy<=mean_y;
      pend_x0<=clamp_low($signed({2'b00,mean_x})-$signed({2'b00,box_w}));
      pend_x1<=clamp_high($signed({2'b00,mean_x})+$signed({2'b00,box_w}),WIDTH);
      pend_y0<=clamp_low($signed({2'b00,mean_y})-$signed({2'b00,box_h}));
      pend_y1<=clamp_high($signed({2'b00,mean_y})+$signed({2'b00,box_h}),HEIGHT);
     end else if(mode==M_TRACK) begin
      if(miss!=10'h3ff)miss<=miss+1'b1;
      vel_x<=0;vel_y<=0;
      if(miss<HOLD_FRAMES-1) begin pend_status<=ST_HOLD;pend_valid<=track_valid; end
      else if(miss<HOLD_FRAMES+SEARCH_FRAMES-1) begin pend_status<=ST_SEARCH;pend_valid<=0; end
      else begin mode<=M_IDLE;model<=0;pend_status<=ST_IDLE;pend_valid<=0;color_id<=0; end
     end else if(status==ST_REJECT) begin
      if(reject_count>=REJECT_FRAMES)pend_status<=ST_IDLE;else reject_count<=reject_count+1'b1;
     end else pend_status<=ST_IDLE;
    end
    // Share of the target explained by the model, in whole percent (BCD).
    F_PCT: if(target_total!=0 && pct_acc+target_total<=pct_goal && pct_bcd!=8'h99) begin
     pct_acc<=pct_acc+target_total;pct_bcd<=bcd_inc({8'd0,pct_bcd});
    end else fstate<=F_UPDATE;
    F_BCD: begin
     if(iter==0) begin shift_x<=pend_cx;shift_y<=pend_cy;bcd_x<=0;bcd_y<=0;iter<=1; end
     else begin
      bcd_x<={adjusted_x[14:0],shift_x[11]};bcd_y<={adjusted_y[14:0],shift_y[11]};
      shift_x<=shift_x<<1;shift_y<=shift_y<<1;
      if(iter==12) begin iter<=0;fstate<=F_PUBLISH; end else iter<=iter+1'b1;
     end
    end
    F_PUBLISH: begin
     // Still in vertical blank: commit atomically and aim the next window.
     status<=pend_status;track_valid<=pend_valid;track_observed<=pend_valid && pend_observed;
     x0<=pend_x0;y0<=pend_y0;x1<=pend_x1;y1<=pend_y1;cx<=pend_cx;cy<=pend_cy;
     cx_bcd<=pend_valid?bcd_x:16'd0;cy_bcd<=pend_valid?bcd_y:16'd0;
     if(miss<HOLD_FRAMES) begin
      win_x0<=clamp_low(pred_x-$signed({1'b0,reach_x}));win_x1<=clamp_high(pred_x+$signed({1'b0,reach_x}),WIDTH);
      win_y0<=clamp_low(pred_y-$signed({1'b0,reach_y}));win_y1<=clamp_high(pred_y+$signed({1'b0,reach_y}),HEIGHT);
     end else begin win_x0<=0;win_y0<=0;win_x1<=WIDTH-1;win_y1<=HEIGHT-1; end
     fstate<=F_IDLE;
    end
    default:fstate<=F_IDLE;
   endcase

   // Frame end (after the last pixels left the pipeline).
   if(end_pipe[3] && fstate==F_IDLE) begin
    if(mode==M_LEARN && !bad) begin
     mode<=M_IDLE;acquiring<=1;scan_bin<=0;cover<=0;best_count<=0;best_bin<=0;model<=0;pass2<=0;fstate<=F_SCAN_R;
    end else if(mode==M_LEARN) begin
     mode<=M_IDLE;acquiring<=1;model<=0;fstate<=F_UPDATE;    // malformed learning frame
    end else if(mode==M_TRACK && !bad &&
                sum_n>=((miss<HOLD_FRAMES)?MIN_PIXELS:2*MIN_PIXELS)) begin
     // Search frames need twice the evidence to re-acquire anywhere.
     observed_now<=1;div_q<={sum_x[11:0],10'd0};div_r<=sum_x[30:12];div_v<=sum_n;iter<=10;div_job<=0;fstate<=F_DIV;
    end else begin observed_now<=0;fstate<=F_UPDATE; end
   end
   // Arithmetic never overlaps active video: abort and keep the old outputs.
   if(de && fstate!=F_IDLE && fstate!=F_CLEAR) begin fstate<=F_IDLE;bad<=1; end
   if(start) begin
    px_reg<=0;py<=0;ysq<=0;bad<=0;
    sum_n<=0;sum_x<=0;sum_y<=0;sum_xx<=0;sum_yy<=0;
    if(acquire && enable && mode!=M_LEARN && fstate==F_IDLE) begin
     // Learn from the inner half of the requested box.
     mode<=M_LEARN;target_total<=0;frame_total<=0;scan_bin<=0;fstate<=F_CLEAR;t_bcd<=0;n_bcd<=0;
     learn_x0<=acq_x0+((acq_x1-acq_x0)>>2);learn_x1<=acq_x1-((acq_x1-acq_x0)>>2);
     learn_y0<=acq_y0+((acq_y1-acq_y0)>>2);learn_y1<=acq_y1-((acq_y1-acq_y0)>>2);
     status<=ST_LEARN;track_valid<=0;track_observed<=0;color_id<=0;
    end
   end
   if(!enable) begin
    mode<=M_IDLE;model<=0;fstate<=F_IDLE;acquiring<=0;miss<=0;
    status<=ST_IDLE;track_valid<=0;track_observed<=0;color_id<=0;cx_bcd<=0;cy_bcd<=0;
   end
  end
 end
endmodule
