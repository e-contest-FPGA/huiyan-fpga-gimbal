// V21 complete camera -> decision -> PWM path, independent of DDR/video.
module servo_vision_system #(
 parameter WIDTH=1280,HEIGHT=720,PIXEL_HZ=75000000,OPTICAL_LATENCY=1
)(
 input wire camera_clk,camera_rst_n,pixel_clk,pixel_rst_n,ctrl_clk,ctrl_rst_n,ddr_clk,
 input wire camera_sof,camera_valid,camera_eol,rx_error,input wire [95:0] camera_rgb4,
 input wire vs,de,input wire [23:0] raw_rgb,input wire [1:0] key_n,
 input wire system_on,lock,gimbal_on,target_priority,
 input wire motion_valid,motion_observed,
 input wire [11:0] motion_x0,motion_y0,motion_x1,motion_y1,
 input wire cam_cfg_done,ae_cfg_done,
 input wire [15:0] input_overflow,output_overflow,output_underflow,display_frame_ddr,
 output wire pwm_pan,pwm_tilt,led,uart_tx,motion_enable,measurement,meter_target,
 output wire ae_req,output wire [15:0] ae,ag,camera_frame_id,
 output wire camera_bad,camera_frame_end,
 output wire [2:0] track_status,output wire track_valid,track_observed,
 output wire [11:0] x0,y0,x1,y1,cx,cy,output wire [15:0] cx_bcd,cy_bcd,
 output wire [3:0] color_id,output wire track_mask,
 output wire lat_valid,output wire [15:0] lat_bcd,
 output wire [1:0] quality,output wire ambiguous,overlay_matched,control_stale,at_limit,
 output wire [15:0] rx_errors,underflows
);
 wire fs,fe,sv,fb;wire [23:0] camera_rgb;wire [11:0] sx,sy;
 wire [15:0] bad_frames;
 camera_analysis_stream #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_stream(
  .clk(camera_clk),.rst_n(camera_rst_n),.sof(camera_sof),.valid(camera_valid),.eol(camera_eol),
  .rgb4(camera_rgb4),.rx_error(rx_error),.frame_start(fs),.frame_end(fe),.sample_valid(sv),.frame_bad(fb),
  .rgb(camera_rgb),.x(sx),.y(sy),.frame_id(camera_frame_id),.bad_frames(bad_frames));
 assign camera_bad=fb;assign camera_frame_end=fe;
 wire moving,fast_motion;
 assign motion_enable=system_on && !moving && !measurement;
 servo_measure_control #(.CLK_HZ(PIXEL_HZ)) u_measure_switch(
  .clk(pixel_clk),.rst_n(pixel_rst_n),.vs(vs),.key_n(key_n),.measurement(measurement));
 wire tracker_enable,acquire,lock_mode;
 wire [11:0] ax0,ay0,ax1,ay1;
 video_track_manager #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_manager(
  .clk(pixel_clk),.rst_n(pixel_rst_n),.vs(vs),.de(de),.system_on(system_on),.lock(lock),
  .motion_valid(motion_valid),.motion_observed(motion_observed),
  .motion_x0(motion_x0),.motion_y0(motion_y0),.motion_x1(motion_x1),.motion_y1(motion_y1),
  .tracker_status(track_status),.tracker_enable(tracker_enable),.acquire(acquire),
  .acq_x0(ax0),.acq_y0(ay0),.acq_x1(ax1),.acq_y1(ay1),.lock_mode(lock_mode));
 wire [52:0] command={measurement,fast_motion,target_priority,tracker_enable,acquire,ax0,ay0,ax1,ay1};
 reg [52:0] last_command;reg command_sent;
 wire command_busy,command_valid;wire [52:0] camera_command;
 wire command_send=!command_busy && (!command_sent || command!=last_command);
 always @(posedge pixel_clk or negedge pixel_rst_n)begin
  if(!pixel_rst_n)begin last_command<=0;command_sent<=0;end
  else if(command_send)begin last_command<=command;command_sent<=1;end
 end
 servo_cdc #(.WIDTH(53)) u_command(
  .s_clk(pixel_clk),.s_rst_n(pixel_rst_n),.s_send(command_send),.s_data(command),.s_busy(command_busy),
  .d_clk(camera_clk),.d_rst_n(camera_rst_n),.d_valid(command_valid),.d_data(camera_command));
 wire c_measure,c_fast,c_priority,c_enable,c_acquire;
 wire [11:0] c_ax0,c_ay0,c_ax1,c_ay1;
 assign {c_measure,c_fast,c_priority,c_enable,c_acquire,c_ax0,c_ay0,c_ax1,c_ay1}=camera_command;
 wire result,observed,valid,ambiguity;
 wire [15:0] result_frame,overruns,decision_cycles;
 wire [2:0] status;wire [1:0] q;
 wire [11:0] tx0,ty0,tx1,ty1,tcx,tcy;
 wire [14:0] model;wire [3:0] colour;
 servo_stream_tracker #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_tracker(
  .clk(camera_clk),.rst_n(camera_rst_n),.enable(c_enable),.acquire(c_acquire),
  .acq_x0(c_ax0),.acq_y0(c_ay0),.acq_x1(c_ax1),.acq_y1(c_ay1),
  .frame_start(fs),.frame_end(fe),.frame_bad(fb),.sample_valid(sv),.sample_x(sx),.sample_y(sy),.rgb(camera_rgb),.frame_id(camera_frame_id),
  .result_valid(result),.result_frame(result_frame),.status(status),.track_valid(valid),.track_observed(observed),
  .quality(q),.ambiguous(ambiguity),.x0(tx0),.y0(ty0),.x1(tx1),.y1(ty1),.cx(tcx),.cy(tcy),
  .model(model),.color_id(colour),.overruns(overruns),.decision_cycles(decision_cycles));
 wire target_active,motion_limited;wire [15:0] ae_dropped;
 servo_exposure #(.WIDTH(WIDTH),.HEIGHT(HEIGHT)) u_exposure(
  .clk(camera_clk),.rst_n(camera_rst_n),.ctrl_clk(ctrl_clk),.ctrl_rst_n(ctrl_rst_n),
  .frame_start(fs),.frame_end(fe),.frame_bad(fb),.sample_valid(sv),.sample_x(sx),.sample_y(sy),.rgb(camera_rgb),
  .target_enable(c_priority),.observed(observed),.quality(q),.x0(tx0),.y0(ty0),.x1(tx1),.y1(ty1),
  .motion_fast(c_fast),.measurement(c_measure),.cam_cfg_done(cam_cfg_done),.ae_cfg_done(ae_cfg_done),
  .target_active(target_active),.motion_limited(motion_limited),.ae_req(ae_req),.ae(ae),.ag(ag),.commands_dropped(ae_dropped));
 wire [146:0] result_data={result_frame,status,valid,observed,q,ambiguity,tx0,ty0,tx1,ty1,tcx,tcy,model,colour,overruns,decision_cycles};
 wire record_busy,new_record;wire [146:0] record;
 reg [15:0] record_drops;
 always @(posedge camera_clk or negedge camera_rst_n)begin
  if(!camera_rst_n)record_drops<=0;
  else if(result && record_busy && record_drops!=16'hffff)record_drops<=record_drops+1'b1;
 end
 servo_cdc #(.WIDTH(147)) u_result(
  .s_clk(camera_clk),.s_rst_n(camera_rst_n),.s_send(result),.s_data(result_data),.s_busy(record_busy),
  .d_clk(pixel_clk),.d_rst_n(pixel_rst_n),.d_valid(new_record),.d_data(record));
 wire [15:0] live_frame,live_overruns,live_decision;
 wire live_valid,live_observed;wire [11:0] live_x0,live_y0,live_x1,live_y1,live_cx,live_cy;
 wire [14:0] live_model;wire [3:0] live_colour;
 assign {live_frame,track_status,live_valid,live_observed,quality,ambiguous,
         live_x0,live_y0,live_x1,live_y1,live_cx,live_cy,live_model,live_colour,live_overruns,live_decision}=record;
 wire [11:0] pan_us,tilt_us;wire [15:0] pwm_frame;wire [31:0] pwm_wait;
 gimbal_servo_quality_stable #(.CLK_HZ(PIXEL_HZ),.WIDTH(WIDTH),.HEIGHT(HEIGHT),.PAN_DIR(1),.TILT_DIR(-1),
  .TILT_MIN_US(700),.TILT_MAX_US(2300)) u_gimbal(
  .clk(pixel_clk),.rst_n(pixel_rst_n),.enable(gimbal_on),.vs(new_record),
  .target_observed(live_observed),.target_cx(live_cx),.target_cy(live_cy),
  .pwm_pan(pwm_pan),.pwm_tilt(pwm_tilt),.moving(moving),.pan_us(pan_us),.tilt_us(tilt_us),
  .quality(quality),.freeze(measurement),.command_frame(live_frame),.pwm_frame(pwm_frame),.pwm_wait_cycles(pwm_wait),
  .stale(control_stale),.fast_motion(fast_motion),.at_limit(at_limit));
 // Snapshot DDR-domain frame identity and overflow, and CSI-domain health.
 wire [31:0] ddr_health={display_frame_ddr,output_overflow};
 reg [31:0] last_ddr_health;wire ddr_busy,ddr_update;wire [31:0] pixel_ddr_health;
 wire ddr_send=!ddr_busy && ddr_health!=last_ddr_health;
 always @(posedge ddr_clk or negedge pixel_rst_n)begin
  if(!pixel_rst_n)last_ddr_health<=0;else if(ddr_send)last_ddr_health<=ddr_health;
 end
 servo_cdc #(.WIDTH(32)) u_ddr_health(
  .s_clk(ddr_clk),.s_rst_n(pixel_rst_n),.s_send(ddr_send),.s_data(ddr_health),.s_busy(ddr_busy),
  .d_clk(pixel_clk),.d_rst_n(pixel_rst_n),.d_valid(ddr_update),.d_data(pixel_ddr_health));
 wire [15:0] source_rx_errors;
 servo_event_counter u_rx_errors(.clk(camera_clk),.rst_n(camera_rst_n),.event_in(rx_error),.count(source_rx_errors));
 wire [97:0] camera_health={target_active,motion_limited,source_rx_errors,bad_frames,input_overflow,ae_dropped,record_drops,overruns};
 reg [97:0] last_camera_health;wire health_busy,health_update;wire [97:0] pixel_health;
 wire health_send=!health_busy && camera_health!=last_camera_health;
 always @(posedge camera_clk or negedge camera_rst_n)begin
  if(!camera_rst_n)last_camera_health<=0;else if(health_send)last_camera_health<=camera_health;
 end
 servo_cdc #(.WIDTH(98)) u_camera_health(
  .s_clk(camera_clk),.s_rst_n(camera_rst_n),.s_send(health_send),.s_data(camera_health),.s_busy(health_busy),
  .d_clk(pixel_clk),.d_rst_n(pixel_rst_n),.d_valid(health_update),.d_data(pixel_health));
 wire [15:0] h_bad,h_input,h_ae_drop,h_record_drop,h_overrun;
 wire h_fast;
 assign {meter_target,h_fast,rx_errors,h_bad,h_input,h_ae_drop,h_record_drop,h_overrun}=pixel_health;
 assign underflows=output_underflow;
 reg [31:0] last_exposure;wire exposure_busy,exposure_update;wire [31:0] pixel_exposure;
 wire exposure_send=!exposure_busy && {ae,ag}!=last_exposure;
 always @(posedge ctrl_clk or negedge ctrl_rst_n)begin
  if(!ctrl_rst_n)last_exposure<=0;else if(exposure_send)last_exposure<={ae,ag};
 end
 servo_cdc #(.WIDTH(32)) u_exposure_status(
  .s_clk(ctrl_clk),.s_rst_n(ctrl_rst_n),.s_send(exposure_send),.s_data({ae,ag}),.s_busy(exposure_busy),
  .d_clk(pixel_clk),.d_rst_n(pixel_rst_n),.d_valid(exposure_update),.d_data(pixel_exposure));
 wire [93:0] video_record={live_frame,live_valid,live_observed,live_x0,live_y0,live_x1,live_y1,live_cx,live_cy,live_colour};
 wire [93:0] overlay;wire overlay_update;wire [15:0] video_frame;
 assign video_frame=pixel_ddr_health[31:16];
 servo_overlay_history u_overlay(
  .clk(pixel_clk),.rst_n(pixel_rst_n),.record_valid(new_record),.record(video_record),
  .vs(vs),.de(de),.display_frame(video_frame),.overlay(overlay),.matched(overlay_matched),.updated(overlay_update));
 wire [15:0] overlay_frame;
 assign {overlay_frame,track_valid,track_observed,x0,y0,x1,y1,cx,cy,color_id}=overlay;
 servo_bcd_pair u_coordinates(.clk(pixel_clk),.rst_n(pixel_rst_n),.start(overlay_update),.x(cx),.y(cy),.x_bcd(cx_bcd),.y_bcd(cy_bcd));
 wire [3:0] display_bin;
 reg [11:0] raster_x,raster_y;reg de_previous,vs_previous;
 wire [11:0] current_x=de_previous?raster_x:12'd0;
 always @(posedge pixel_clk or negedge pixel_rst_n)begin
  if(!pixel_rst_n)begin raster_x<=0;raster_y<=0;de_previous<=0;vs_previous<=0;end
  else begin
   de_previous<=de;vs_previous<=vs;
   if(de)raster_x<=current_x+1'b1;else begin raster_x<=0;if(de_previous)raster_y<=raster_y+1'b1;end
   if(vs && !vs_previous)raster_y<=0;
  end
 end
 servo_color_bin u_display_bin(.rgb(raw_rgb),.bin(display_bin));
 assign track_mask=de && live_model[display_bin] && track_valid && !measurement &&
                   live_frame-overlay_frame<4 && current_x>=x0 && current_x<=x1 && raster_y>=y0 && raster_y<=y1;
 // Raw ISP output avoids V20 digital colour/tone changes in optical tests.
 wire [9:0] optical_sum={2'b00,raw_rgb[23:16]}+{1'b0,raw_rgb[15:8],1'b0}+{2'b00,raw_rgb[7:0]};
 wire latency_update;wire [31:0] latency_cycles;wire [15:0] latency_id,latency_timeouts;
 generate if(OPTICAL_LATENCY)begin:legacy_optical
 servo_latency_meter #(.CLK_HZ(PIXEL_HZ)) u_latency(
  .clk(pixel_clk),.rst_n(pixel_rst_n),.enable(measurement),.vs(vs),.de(de),.gray(optical_sum[9:2]),
  .led(led),.result_valid(latency_update),.lat_valid(lat_valid),.raw_cycles(latency_cycles),
  .sample_id(latency_id),.timeouts(latency_timeouts),.lat_bcd(lat_bcd));
 end else begin:frame_only
  assign led=1'b0;assign latency_update=1'b0;assign lat_valid=1'b0;
  assign latency_cycles=32'd0;assign latency_id=16'd0;assign latency_timeouts=16'd0;assign lat_bcd=16'd0;
 end endgenerate
 reg vs_prev;reg [16:0] millisecond_div;reg [31:0] uptime_ms;
 reg measure_previous;reg [15:0] measurement_epoch;
 always @(posedge pixel_clk or negedge pixel_rst_n)begin
  if(!pixel_rst_n)begin vs_prev<=0;millisecond_div<=0;uptime_ms<=0;measure_previous<=0;measurement_epoch<=0;end
  else begin
   vs_prev<=vs;
   measure_previous<=measurement;
   if(measurement && !measure_previous)measurement_epoch<=measurement_epoch+1'b1;
   if(millisecond_div==PIXEL_HZ/1000-1)begin millisecond_div<=0;uptime_ms<=uptime_ms+1'b1;end
   else millisecond_div<=millisecond_div+1'b1;
  end
 end
 wire [7:0] flags={control_stale,at_limit,overlay_matched,h_fast,meter_target,ambiguous,live_observed,measurement};
 wire [15:0] telemetry_dropped;wire telemetry_busy;
 // Offsets are documented in telemetry_v21.py; total payload is 78 bytes.
 wire [623:0] payload={32'h53563231,8'd1,flags,{6'd0,quality},{5'd0,track_status},
  live_frame,video_frame,4'd0,live_cx,4'd0,live_cy,4'd0,pan_us,4'd0,tilt_us,
  pixel_exposure,latency_cycles,latency_id,latency_timeouts,rx_errors,h_bad,h_input,
  pixel_ddr_health[15:0],output_underflow,live_overruns,h_ae_drop,h_record_drop,telemetry_dropped,
  uptime_ms,live_decision,pwm_frame,pwm_wait,{7'd0,lat_valid},8'd1,measurement_epoch,
  4'd0,live_x0,4'd0,live_y0,4'd0,live_x1,4'd0,live_y1,32'd0};
 servo_telemetry #(.CLK_HZ(PIXEL_HZ)) u_telemetry(
  .clk(pixel_clk),.rst_n(pixel_rst_n),.send(vs && !vs_prev),.payload(payload),
  .tx(uart_tx),.busy(telemetry_busy),.dropped(telemetry_dropped));
endmodule
