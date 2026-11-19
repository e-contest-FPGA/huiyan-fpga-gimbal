// Frame-age instrumentation only. It never stalls or modifies the image bus.
// Counter unit: one CSI frame-start period; endpoint: first active render pixel.
module video_frame_source (
 input wire clk,rst_n,capture_sof,isp_sof,
 output reg [15:0] capture_gray,
 output wire [15:0] isp_id,
 output wire isp_valid,
 output wire source_fault
);
 reg [15:0] capture_id,pending_id;
 reg cap_prev,isp_prev,pending,fault;
 wire cap_event=capture_sof && !cap_prev;
 wire isp_event=isp_sof && !isp_prev;
 wire [15:0] next_id=capture_id+16'd1;
 // Bypass also handles an ISP with zero frame-start latency.
 assign isp_id=pending?pending_id:next_id;
 assign isp_valid=!fault && (pending || cap_event);
 assign source_fault=fault;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   capture_id<=0;capture_gray<=0;pending_id<=0;
   cap_prev<=0;isp_prev<=0;pending<=0;fault<=0;
  end else begin
   cap_prev<=capture_sof;isp_prev<=isp_sof;
   if(cap_event) begin
    capture_id<=next_id;capture_gray<=(next_id>>1)^next_id;pending_id<=next_id;
   end
   case({cap_event,isp_event})
    2'b10:begin pending<=1;if(pending) fault<=1;end
    // A clean CSI->ISP pair re-establishes correspondence after a startup
    // orphan. Its current tag still sees the OLD fault and is rejected;
    // only a subsequent clean frame can become valid. A repeated CSI
    // without an ISP start still invalidates the pending generation.
    2'b01:begin pending<=0;fault<=!pending;end
    2'b11:begin pending<=pending;fault<=0;end
    default:pending<=pending;
   endcase
  end
 end
endmodule

module video_frame_store_tags #(
 parameter integer FRAME_WORDS=172800,
 parameter integer SLOT_STRIDE=7000000
)(
 input wire camera_clk,rst_n,camera_sof,
 input wire [15:0] camera_id,
 input wire camera_tag_valid,camera_overflow,
 input wire ddr_clk,frame_start,
 input wire [1:0] write_slot,
 input wire write_enable,
 input wire [24:0] write_address,
 input wire frame_error,
 output wire [63:0] frame_ids,
 output reg [3:0] frame_complete,
 output wire [19:0] diagnostic
);
 // Bundled data is held for the whole frame. The existing extended SOF
 // reaches frame_start after two DDR synchronizer stages. An extra DDR
 // guard cycle allows the bundled bus to miss the first sampling edge while
 // the faster SOF control path catches it (worst asynchronous phase/skew).
 reg [16:0] tag_hold;
 reg camera_sof_prev;
 (* async_reg="true" *) reg [16:0] tag_meta,tag_sync;
 reg [15:0] ids [0:3];
 reg [1:0] active_slot;
 reg [17:0] words;
 reg [24:0] expected_address;
 reg active,bad,start_pending;
 assign frame_ids={ids[3],ids[2],ids[1],ids[0]};
 // Read-only diagnostics: tag validity, write error, accepted-word count.
 // These signals are never used to qualify or alter video data.
 assign diagnostic={tag_sync[16],bad,words};
 always @(posedge camera_clk or negedge rst_n) begin
  if(!rst_n) begin tag_hold<=0;camera_sof_prev<=0;end
  else begin
   camera_sof_prev<=camera_sof;
   if(camera_sof && !camera_sof_prev) tag_hold<={camera_tag_valid,camera_id};
   else if(camera_overflow) tag_hold[16]<=0;
  end
 end
 always @(posedge ddr_clk or negedge rst_n) begin
  if(!rst_n) begin tag_meta<=0;tag_sync<=0;end
  else begin tag_meta<=tag_hold;tag_sync<=tag_meta;end
 end
 always @(posedge ddr_clk or negedge rst_n) begin
  if(!rst_n) begin
   ids[0]<=0;ids[1]<=0;ids[2]<=0;ids[3]<=0;frame_complete<=0;
   active_slot<=0;words<=0;expected_address<=0;active<=0;bad<=1;start_pending<=0;
  end else begin
  if(frame_start) begin
   active_slot<=write_slot;words<=0;active<=1;start_pending<=1;
   case(write_slot)
    0:expected_address<=0;
    1:expected_address<=SLOT_STRIDE;
    2:expected_address<=2*SLOT_STRIDE;
    3:expected_address<=3*SLOT_STRIDE;
   endcase
   frame_complete[write_slot]<=0;
   bad<=frame_error || write_enable;
  end else if(start_pending) begin
   start_pending<=0;ids[active_slot]<=tag_sync[15:0];
   bad<=bad || frame_error || !tag_sync[16] || write_enable;
  end else if(active && write_enable) begin
   expected_address<=expected_address+25'd8;
   if(words<FRAME_WORDS) words<=words+18'd1;
   if(frame_error || !tag_sync[16] || write_address!=expected_address || words>=FRAME_WORDS) begin
    bad<=1;frame_complete[active_slot]<=0;
   end else if(words==FRAME_WORDS-1)
    frame_complete[active_slot]<=!bad;
  end
  if(active && !frame_start && !start_pending && (!tag_sync[16] || frame_error)) begin
   bad<=1;frame_complete[active_slot]<=0;
  end
  end
 end
endmodule

module video_frame_read_tags #(
 parameter integer FRAME_WORDS=172800,
 parameter integer SLOT_STRIDE=7000000,
 parameter integer QUEUE_AW=10
)(
 input wire clk,rst_n,read_enable,read_conflict,
 input wire [24:0] read_address,
 input wire return_valid,
 input wire [63:0] frame_ids,
 input wire [3:0] frame_complete,
 output wire [18:0] return_tag,
 output reg fault
);
 wire [1:0] slot=(read_address<SLOT_STRIDE)?2'd0:
     (read_address<2*SLOT_STRIDE)?2'd1:(read_address<3*SLOT_STRIDE)?2'd2:2'd3;
 wire [24:0] base=(slot==0)?25'd0:(slot==1)?SLOT_STRIDE:(slot==2)?2*SLOT_STRIDE:3*SLOT_STRIDE;
 wire [24:0] offset=read_address-base;
 wire [15:0] selected_id=frame_ids >> (slot*16);
 wire eligible=!read_conflict && frame_complete[slot] && offset<FRAME_WORDS*8 && offset[2:0]==0;
 wire [20:0] head;
 wire full,empty;
 wire [15:0] current_id=frame_ids >> (head[19:18]*16);
 // One tag per accepted read request, NOT one tag per return-time VSYNC.
 // Old returns straddling a VSYNC therefore retain their original identity.
 soft_fifo_al_4057d6b76aa6 #(
  .DATA_WIDTH_W(21),.DATA_WIDTH_R(21),.ADDR_WIDTH_W(QUEUE_AW),.ADDR_WIDTH_R(QUEUE_AW),
  .AL_FULL_NUM((1<<QUEUE_AW)-8),.AL_EMPTY_NUM(2),.SHOW_AHEAD_EN(1),.OUTREG_EN("NOREG")
 ) u_request_tags (
  .rst(!rst_n),.clkw(clk),.clkr(clk),.we(read_enable),
  .di({eligible,slot,offset==0,offset==(FRAME_WORDS-1)*8,selected_id}),
  .re(return_valid),.dout(head),.full_flag(full),.empty_flag(empty),
  .valid(),.afull(),.aempty(),.wrusedw(),.rdusedw()
 );
 assign return_tag={head[20] && !empty && !fault &&
     frame_complete[head[19:18]] && current_id==head[15:0],head[17:0]};
 // Loss of queue correspondence is sticky until reset; never invent a tag.
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) fault<=0;
  else if((read_enable && full) || (return_valid && empty)) fault<=1;
 end
endmodule

module video_frame_latency #(
 parameter integer CLK_HZ=75000000
)(
 input wire clk,rst_n,
 input wire [15:0] capture_gray,
 input wire vs,de,
 input wire [15:0] display_id,
 input wire display_valid,
 output reg [1:0] status, // 0 WAIT, 1 valid, 2 STALE, 3 ERROR
 output reg [11:0] delay_bcd,
 output reg [15:0] capture_id,shown_id
);
 (* async_reg="true" *) reg [15:0] gray_meta,gray_sync;
 reg [15:0] live_id,last_id;
 integer i;
 always @* begin
  live_id[15]=gray_sync[15];
  for(i=14;i>=0;i=i-1) live_id[i]=live_id[i+1]^gray_sync[i];
 end
 reg [26:0] idle_cycles;
 reg seen_capture,vs_prev,armed;
 wire [15:0] age=live_id-display_id;
 reg [7:0] bin;
 reg [11:0] bcd,adjusted;
 reg [3:0] bits_left;
 reg [1:0] next_status;
 integer j;
 always @* begin
  adjusted=bcd;
  for(j=0;j<3;j=j+1)
   if(bcd[j*4+:4]>=5) adjusted[j*4+:4]=bcd[j*4+:4]+4'd3;
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   gray_meta<=0;gray_sync<=0;last_id<=0;idle_cycles<=0;seen_capture<=0;
   vs_prev<=0;armed<=0;status<=0;delay_bcd<=0;capture_id<=0;shown_id<=0;
   bin<=0;bcd<=0;bits_left<=0;next_status<=0;
  end else begin
   gray_meta<=capture_gray;gray_sync<=gray_meta;vs_prev<=vs;
   last_id<=live_id;
   if(live_id!=last_id) begin idle_cycles<=0;seen_capture<=1;end
   else if(idle_cycles<CLK_HZ) idle_cycles<=idle_cycles+27'd1;
   if(vs && !vs_prev) armed<=1;
   // The selected frame tag is available before the 1660+ clock render
   // pipeline. Take a fresh Gray-counter snapshot at first active render DE.
   if(armed && de) begin
    armed<=0;capture_id<=live_id;shown_id<=display_id;
    bin<=age[7:0];bcd<=0;bits_left<=8;
    if(!seen_capture) next_status<=0;
    else if(idle_cycles>=CLK_HZ) next_status<=2;
    else if(!display_valid || age>255) next_status<=3;
    else next_status<=1;
   end else if(bits_left!=0) begin
    bcd<={adjusted[10:0],bin[7]};bin<={bin[6:0],1'b0};bits_left<=bits_left-1'b1;
    if(bits_left==1) begin delay_bcd<={adjusted[10:0],bin[7]};status<=next_status;end
   end
   // A stopped capture must not keep displaying a reassuring old number.
   if(seen_capture && idle_cycles>=CLK_HZ) status<=2;
   else if(status==1 && !display_valid && de) status<=3;
  end
 end
endmodule

module video_fps_meter #(
 parameter integer CLK_HZ=75000000
)(
 input wire src_clk,src_rst_n,src_frame_start,
 input wire clk,rst_n,
 output reg fps_valid,
 output reg [7:0] fps_bcd
);
 reg src_prev,src_toggle;
 always @(posedge src_clk or negedge src_rst_n) begin
  if(!src_rst_n) begin src_prev<=0;src_toggle<=0; end
  else begin src_prev<=src_frame_start; if(src_frame_start && !src_prev)src_toggle<=~src_toggle; end
 end
 reg [2:0] sync;
 reg [26:0] window;
 reg [7:0] count;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin sync<=0;window<=0;count<=0;fps_valid<=0;fps_bcd<=0; end
  else begin
   sync<={sync[1:0],src_toggle};
   if(window==CLK_HZ-1) begin
    window<=0;fps_bcd<=count;fps_valid<=count!=0;count<=0;
   end else begin
    window<=window+1'b1;
    if(sync[2]^sync[1] && count!=8'h99)
     count<=(count[3:0]==4'd9)?{count[7:4]+4'd1,4'd0}:count+1'b1;
   end
  end
 end
endmodule

