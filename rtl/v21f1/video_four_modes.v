// One pixel per clock. All four outputs refer to the same CENTER pixel.
// RGB/gray/control are delayed along with the centered 3x3 Sobel window.
// This avoids the one-pixel/one-line jump of a causal, unaligned edge filter.
module video_four_modes #(
    parameter integer H_TOTAL = 1650,
    parameter integer BINARY_THRESHOLD = 128,
    parameter integer BINARY_NORMALIZE = 0,
    parameter integer BINARY_MAJORITY = 0,
    parameter integer BINARY_MIN_VALUE = 8,
    parameter integer SOBEL_THRESHOLD = 160,
    parameter integer IMAGE_WIDTH = 1280,
    parameter integer IMAGE_HEIGHT = 720,
    parameter integer ADAPTIVE_ENABLE = 1,
    parameter integer GRAY_AUTO_GAIN = 0,
    parameter integer BINARY_AUTO_THRESHOLD = 0,
    parameter integer BINARY_LIGHT_STEP = 8,
    parameter integer SAMPLE_STRIDE = 64,
    parameter integer BINARY_INITIAL = 32,
    parameter integer EDGE_FLOOR = 24,
    parameter integer MIN_LOCAL_CONTRAST = 8
) (
    input wire clk, rst_n,
    input wire in_vs, in_hs, in_de,
    input wire [23:0] in_rgb,
    output reg out_vs, out_hs, out_de,
    output reg [23:0] color_rgb, gray_rgb, binary_rgb, edge_rgb,
    output wire [7:0] scene_low,scene_mid,scene_high
);
    // Integer BT.601 approximation: Y=(77R+150G+29B+128)>>8.
    reg [15:0] r_rg, r_b;
    reg [23:0] r_rgb1, r_rgb2;
    reg [2:0] r_ctrl1, r_ctrl2;
    reg [7:0] r_gray2;
    reg r_binary2;
    wire [16:0] gray_sum = {1'b0, r_rg} + {1'b0, r_b};
    wire [7:0] binary_value = max3(r_rgb1[23:16],r_rgb1[15:8],r_rgb1[7:0]);
    wire [15:0] binary_luma_sum = gray_sum[15:0]-16'd128;
    // Compare brightness-normalized luma to an ABSOLUTE threshold without a
    // divider: 255*(77R+150G+29B) >= 256*T*max(R,G,B).
    // Common scaling of all three channels leaves this decision unchanged.
    wire [23:0] binary_normalized_num = {binary_luma_sum,8'b0}-{8'b0,binary_luma_sum};
    wire [23:0] binary_normalized_cut = BINARY_THRESHOLD*256*binary_value;
    // ADAPTIVE_ENABLE controls Gaussian smoothing and local edge thresholds.
    // Frame-dependent gray gain is opt-in: changing the surrounding scene
    // must not change the gray level of an otherwise identical RGB patch.
    // Global illumination adaptation is independent of grayscale gain and
    // normalization. It runs in blanking, not in the pixel-rate datapath.
    wire [7:0] binary_light_threshold;
    generate if (BINARY_AUTO_THRESHOLD) begin : g_binary_light
        video_binary_light_adapt #(
            .WIDTH(IMAGE_WIDTH), .HEIGHT(IMAGE_HEIGHT), .SAMPLE_STRIDE(SAMPLE_STRIDE),
            .INITIAL_THRESHOLD(BINARY_INITIAL), .MAX_STEP(BINARY_LIGHT_STEP)
        ) u_light (
            .clk(clk), .rst_n(rst_n), .gray(r_gray2), .de(r_ctrl2[0]),
            .commit_vs(out_vs), .commit_de(out_de), .threshold(binary_light_threshold),
            .low_level(scene_low),.mid_level(scene_mid),.high_level(scene_high)
        );
    end else begin : g_binary_fixed
        assign binary_light_threshold = BINARY_THRESHOLD;
        assign scene_low=32;assign scene_mid=96;assign scene_high=192;
    end endgenerate
    wire [7:0] adaptive_gain_q4;
    generate if (ADAPTIVE_ENABLE && GRAY_AUTO_GAIN) begin : g_adaptive
        video_luma_adapt #(
            .WIDTH(IMAGE_WIDTH), .HEIGHT(IMAGE_HEIGHT), .SAMPLE_STRIDE(SAMPLE_STRIDE),
            .INITIAL_THRESHOLD(BINARY_INITIAL)
        ) u_luma_adapt (
            .clk(clk), .rst_n(rst_n), .gray(r_gray2), .de(r_ctrl2[0]),
            .commit_vs(out_vs), .commit_de(out_de),
            .threshold(), .gain_q4(adaptive_gain_q4)
        );
    end else begin : g_fixed
        assign adaptive_gain_q4 = 16;
    end endgenerate
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_rg <= 0; r_b <= 0; r_rgb1 <= 0; r_rgb2 <= 0;
            r_ctrl1 <= 0; r_ctrl2 <= 0; r_gray2 <= 0; r_binary2<=0;
        end else begin
            r_rg <= in_rgb[23:16] * 16'd77 + in_rgb[15:8] * 16'd150;
            r_b <= in_rgb[7:0] * 16'd29 + 16'd128;
            r_rgb1 <= in_de ? in_rgb : 24'd0;
            r_ctrl1 <= {in_vs, in_hs, in_de};
            r_gray2 <= r_ctrl1[0] ? gray_sum[15:8] : 8'd0;
            r_binary2 <= r_ctrl1[0] && (BINARY_AUTO_THRESHOLD ?
                (gray_sum[15:8]>=binary_light_threshold) : (BINARY_NORMALIZE ?
                (binary_value>=BINARY_MIN_VALUE && binary_normalized_num>=binary_normalized_cut) :
                (gray_sum[15:8]>=BINARY_THRESHOLD)));
            r_rgb2 <= r_rgb1;
            r_ctrl2 <= r_ctrl1;
        end
    end

    wire [35:0] middle;
    wire [9:0] upper;
    // Delay EVERY scan clock including blanking. Right/bottom pixels are
    // flushed during blanking, so the entire 1280x720 image is retained.
    video_scan_delay #(.WIDTH(36), .DEPTH(H_TOTAL)) u_middle_line (
        .clk(clk), .rst_n(rst_n),
        .in_data({r_ctrl2, r_rgb2, r_gray2, r_binary2}), .out_data(middle)
    );
    video_scan_delay #(.WIDTH(10), .DEPTH(2*H_TOTAL)) u_upper_line (
        .clk(clk), .rst_n(rst_n),
        .in_data({r_ctrl2[0], r_gray2, r_binary2}), .out_data(upper)
    );
    reg [7:0] r_current;
    reg r_current_de;
    reg r_current_binary;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin r_current <= 0; r_current_de <= 0; r_current_binary<=0; end
        else begin r_current <= r_gray2; r_current_de <= r_ctrl2[0]; r_current_binary<=r_binary2; end
    end

    reg [7:0] a0, a1, b0, b1, c0, c1;
    reg [1:0] a_valid, b_valid, c_valid;
    reg [1:0] a_binary, b_binary, c_binary;
    reg [23:0] r_mid_rgb_left;
    reg [2:0] r_mid_ctrl_left;
    reg signed [10:0] r_gx, r_gy;
    reg r_window_valid;
    reg [23:0] r_rgb_g;
    reg [7:0] r_gray_g;
    reg r_binary_class_g;
    reg [2:0] r_ctrl_g;
    wire [7:0] a2 = upper[9] ? upper[8:1] : 8'd0;
    wire [7:0] b2 = middle[33] ? middle[8:1] : 8'd0;
    wire [7:0] c2 = r_current_de ? r_current : 8'd0;
    wire [10:0] gx_right = {3'b0,a2} + {2'b0,b2,1'b0} + {3'b0,c2};
    wire [10:0] gx_left  = {3'b0,a0} + {2'b0,b0,1'b0} + {3'b0,c0};
    wire [10:0] gy_lower = {3'b0,c0} + {2'b0,c1,1'b0} + {3'b0,c2};
    wire [10:0] gy_upper = {3'b0,a0} + {2'b0,a1,1'b0} + {3'b0,a2};
    wire window_valid = (&a_valid) && (&b_valid) && (&c_valid)
                        && upper[9] && middle[33] && r_current_de;
    wire [3:0] binary_votes = {3'b0,a_binary[1]}+{3'b0,a_binary[0]}+{3'b0,upper[0]}+
        {3'b0,b_binary[1]}+{3'b0,b_binary[0]}+{3'b0,middle[0]}+
        {3'b0,c_binary[1]}+{3'b0,c_binary[0]}+{3'b0,r_current_binary};
    // Gaussian 1-2-1 / 2-4-2 / 1-2-1 kernel, divided by 16 and rounded.
    wire [9:0] gaussian_a = {2'b0,a0}+{1'b0,a1,1'b0}+{2'b0,a2};
    wire [9:0] gaussian_b = {2'b0,b0}+{1'b0,b1,1'b0}+{2'b0,b2};
    wire [9:0] gaussian_c = {2'b0,c0}+{1'b0,c1,1'b0}+{2'b0,c2};
    wire [11:0] gaussian_sum = {2'b0,gaussian_a}+{1'b0,gaussian_b,1'b0}
                              +{2'b0,gaussian_c}+12'd8;
    function [7:0] min3;
        input [7:0] p,q,r;
        reg [7:0] t;
        begin t=(p<q)?p:q; min3=(t<r)?t:r; end
    endfunction
    function [7:0] max3;
        input [7:0] p,q,r;
        reg [7:0] t;
        begin t=(p>q)?p:q; max3=(t>r)?t:r; end
    endfunction
    wire [7:0] local_min = min3(min3(a0,a1,a2),min3(b0,b1,b2),min3(c0,c1,c2));
    wire [7:0] local_max = max3(max3(a0,a1,a2),max3(b0,b1,b2),max3(c0,c1,c2));
    reg [7:0] r_contrast_g;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a0<=0; a1<=0; b0<=0; b1<=0; c0<=0; c1<=0;
            a_valid<=0; b_valid<=0; c_valid<=0;
            a_binary<=0; b_binary<=0; c_binary<=0;
            r_mid_rgb_left<=0; r_mid_ctrl_left<=0;
            r_gx<=0; r_gy<=0; r_window_valid<=0;
            r_rgb_g<=0; r_gray_g<=0; r_ctrl_g<=0; r_contrast_g<=0; r_binary_class_g<=0;
        end else begin
            a0<=a1; a1<=a2; b0<=b1; b1<=b2; c0<=c1; c1<=c2;
            a_valid<={a_valid[0],upper[9]};
            b_valid<={b_valid[0],middle[33]};
            c_valid<={c_valid[0],r_current_de};
            a_binary<={a_binary[0],upper[0]};
            b_binary<={b_binary[0],middle[0]};
            c_binary<={c_binary[0],r_current_binary};
            r_mid_rgb_left<=middle[32:9];
            r_mid_ctrl_left<=middle[35:33];
            r_gx <= $signed(gx_right) - $signed(gx_left);
            r_gy <= $signed(gy_lower) - $signed(gy_upper);
            r_window_valid <= window_valid;
            r_rgb_g <= r_mid_rgb_left;
            r_gray_g <= (ADAPTIVE_ENABLE && window_valid) ? gaussian_sum[11:4] : b1;
            // Majority is only a binary despeckler, never an adaptive threshold.
            // At incomplete border windows use the center classification.
            r_binary_class_g <= (BINARY_MAJORITY && window_valid) ? (binary_votes>=5) : b_binary[0];
            r_contrast_g <= local_max-local_min;
            r_ctrl_g <= r_mid_ctrl_left;
        end
    end

    wire [10:0] gx_abs = r_gx[10] ? -r_gx : r_gx;
    wire [10:0] gy_abs = r_gy[10] ? -r_gy : r_gy;
    reg [10:0] r_magnitude;
    reg r_edge_valid;
    reg [23:0] r_rgb_m;
    reg [7:0] r_gray_m;
    reg r_binary_class_m;
    reg [2:0] r_ctrl_m;
    reg [7:0] r_contrast_m;
    wire [15:0] boosted_gray = r_gray_m * adaptive_gain_q4 + 16'd8;
    wire [7:0] enhanced_gray = (boosted_gray[15:4]>255) ? 8'd255 : boosted_gray[11:4];
    wire [8:0] edge_cut = EDGE_FLOOR + {2'b0,r_gray_m[7:1]};
    wire binary_hit = r_binary_class_m;
    wire edge_hit = ADAPTIVE_ENABLE ? ((r_contrast_m>=MIN_LOCAL_CONTRAST) && (r_magnitude>=edge_cut))
                                   : (r_magnitude>=SOBEL_THRESHOLD);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_magnitude<=0; r_edge_valid<=0;
            r_rgb_m<=0; r_gray_m<=0; r_ctrl_m<=0; r_contrast_m<=0; r_binary_class_m<=0;
            out_vs<=0; out_hs<=0; out_de<=0;
            color_rgb<=0; gray_rgb<=0; binary_rgb<=0; edge_rgb<=0;
        end else begin
            r_magnitude <= gx_abs + gy_abs;
            r_edge_valid <= r_window_valid;
            r_rgb_m <= r_rgb_g; r_gray_m <= r_gray_g; r_ctrl_m <= r_ctrl_g;
            r_binary_class_m <= r_binary_class_g;
            r_contrast_m <= r_contrast_g;
            out_vs <= r_ctrl_m[2]; out_hs <= r_ctrl_m[1]; out_de <= r_ctrl_m[0];
            color_rgb <= r_ctrl_m[0] ? r_rgb_m : 24'd0;
            gray_rgb <= r_ctrl_m[0] ? (ADAPTIVE_ENABLE ? {enhanced_gray,enhanced_gray,enhanced_gray}
                                                                    : {r_gray_m,r_gray_m,r_gray_m}) : 24'd0;
            binary_rgb <= (r_ctrl_m[0] && binary_hit) ? 24'hffffff : 24'd0;
            edge_rgb <= (r_ctrl_m[0] && r_edge_valid && edge_hit) ? 24'hffffff : 24'd0;
        end
    end
endmodule

// Inferred synchronous READ_FIRST RAM. No reset loop over memory cells.
// The fill guard hides stale or unknown storage after startup and reset.
module video_scan_delay #(
    parameter integer WIDTH = 35,
    parameter integer DEPTH = 1650
) (
    input wire clk, rst_n,
    input wire [WIDTH-1:0] in_data,
    output wire [WIDTH-1:0] out_data
);
    localparam integer ADDR_W = (DEPTH>1) ? $clog2(DEPTH) : 1;
    localparam integer FILL_W = $clog2(DEPTH+1);
    // TD's automatic RAM inference selects ERAM for these scan buffers.
    reg [WIDTH-1:0] memory [0:DEPTH-1];
    reg [ADDR_W-1:0] addr;
    reg [FILL_W-1:0] filled;
    reg valid;
    reg [WIDTH-1:0] read_data;
    always @(posedge clk) begin
        if (rst_n) begin
            read_data <= memory[addr];
            memory[addr] <= in_data;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin addr<=0; filled<=0; valid<=0; end
        else begin
            addr <= (addr == DEPTH-1) ? 0 : addr+1'b1;
            if (filled != DEPTH) filled <= filled+1'b1;
            valid <= (filled == DEPTH);
        end
    end
    assign out_data = valid ? read_data : {WIDTH{1'b0}};
endmodule

// Keep adaptation in this existing compilation unit for TD cached projects.
// Sampled 32-bin raw-luma histogram. The 12.5th/87.5th percentiles ignore
// small bright lights and set the binary threshold and bounded shadow gain.
// Only a complete frame may update a target; active parameters slew ONLY at
// output vertical blank. No sensor/I2C/DDR reconfiguration is involved.
module video_luma_adapt #(
    parameter integer WIDTH=1280, HEIGHT=720, SAMPLE_STRIDE=64,
    parameter integer INITIAL_THRESHOLD=32,
    parameter integer THRESHOLD_STEP=2, GAIN_STEP=4
) (
    input wire clk, rst_n,
    input wire [7:0] gray,
    input wire de, commit_vs, commit_de,
    output reg [7:0] threshold, gain_q4
);
    localparam integer PIXELS=WIDTH*HEIGHT;
    localparam integer PW=$clog2(PIXELS+2);
    localparam integer SW=$clog2(PIXELS/SAMPLE_STRIDE+2);
    localparam integer CW=(SAMPLE_STRIDE>1)?$clog2(SAMPLE_STRIDE):1;
    reg [SW-1:0] histogram[0:31], snapshot[0:31];
    reg [PW-1:0] pixels;
    reg [SW-1:0] samples, snapshot_total;
    reg [CW-1:0] sample_phase;
    reg overflow, vs_prev;
    reg [7:0] de_history;
    wire commit=commit_vs && !vs_prev && !commit_de && de_history==0;
    wire complete=(pixels==PIXELS) && !overflow && (samples!=0);
    reg scanning, ready;
    reg [4:0] index, low_bin, high_bin;
    reg found_low, found_high;
    reg [SW:0] cdf;
    reg [7:0] target_threshold, target_gain;
    wire [SW:0] cdf_next=cdf+{1'b0,snapshot[index]};
    wire [SW:0] low_cut=(snapshot_total>=8)?({1'b0,snapshot_total}>>3):1;
    wire [SW:0] high_cut={1'b0,snapshot_total}-({1'b0,snapshot_total}>>3);
    wire [4:0] chosen_low=found_low?low_bin:((cdf_next>=low_cut)?index:low_bin);
    wire [4:0] chosen_high=found_high?high_bin:((cdf_next>=high_cut)?index:high_bin);
    wire [8:0] threshold_sum={1'b0,chosen_low,3'b0}+{1'b0,chosen_high,3'b0}+9'd7;
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            threshold<=INITIAL_THRESHOLD; gain_q4<=16;
            target_threshold<=INITIAL_THRESHOLD; target_gain<=16;
            pixels<=0; samples<=0; snapshot_total<=0; sample_phase<=0;
            overflow<=0; vs_prev<=0; de_history<=0;
            scanning<=0; ready<=0; index<=0; low_bin<=0; high_bin<=0;
            found_low<=0; found_high<=0; cdf<=0;
            for(i=0;i<32;i=i+1) begin histogram[i]<=0; snapshot[i]<=0; end
        end else begin
            vs_prev<=commit_vs;
            de_history<={de_history[6:0],commit_de};
            if(de && !commit) begin
                if(pixels<PIXELS) pixels<=pixels+1'b1;
                else overflow<=1;
                if(sample_phase==SAMPLE_STRIDE-1) begin
                    sample_phase<=0;
                    if(samples<PIXELS/SAMPLE_STRIDE) begin
                        samples<=samples+1'b1;
                        histogram[gray[7:3]]<=histogram[gray[7:3]]+1'b1;
                    end else overflow<=1;
                end else sample_phase<=sample_phase+1'b1;
            end
            if(scanning) begin
                cdf<=cdf_next;
                if(!found_low && cdf_next>=low_cut) begin low_bin<=index; found_low<=1; end
                if(!found_high && cdf_next>=high_cut) begin high_bin<=index; found_high<=1; end
                if(index==31) begin
                    target_threshold<=threshold_sum[8:1];
                    target_gain<=(chosen_high<=3)?128:((chosen_high<=7)?64:((chosen_high<=15)?32:16));
                    scanning<=0; ready<=1;
                end else index<=index+1'b1;
            end
            if(commit) begin
                pixels<=0; samples<=0; sample_phase<=0; overflow<=0;
                for(i=0;i<32;i=i+1) histogram[i]<=0;
                if(ready) begin
                    ready<=0;
                    if(complete) begin
                        if(target_threshold>threshold)
                            threshold<=(target_threshold-threshold>THRESHOLD_STEP)?threshold+THRESHOLD_STEP:target_threshold;
                        else
                            threshold<=(threshold-target_threshold>THRESHOLD_STEP)?threshold-THRESHOLD_STEP:target_threshold;
                        if(target_gain>gain_q4)
                            gain_q4<=(target_gain-gain_q4>GAIN_STEP)?gain_q4+GAIN_STEP:target_gain;
                        else
                            gain_q4<=(gain_q4-target_gain>GAIN_STEP)?gain_q4-GAIN_STEP:target_gain;
                    end
                end
                if(complete && !scanning) begin
                    for(i=0;i<32;i=i+1) snapshot[i]<=histogram[i];
                    snapshot_total<=samples;
                    index<=0; cdf<=0; low_bin<=0; high_bin<=0;
                    found_low<=0; found_high<=0; scanning<=1;
                end
            end
        end
    end
endmodule

// Lightweight global illumination estimate. Quantiles reject small extreme
// lights/shadows; the median supplies the scene's typical illumination.
// No divider, full-frame buffer, or per-pixel multiplication is required.
module video_binary_light_adapt #(
    parameter integer WIDTH=1280, HEIGHT=720, SAMPLE_STRIDE=64,
    parameter integer INITIAL_THRESHOLD=32, MAX_STEP=8
) (
    input wire clk, rst_n,
    input wire [7:0] gray,
    input wire de, commit_vs, commit_de,
    output reg [7:0] threshold,
    output reg [7:0] low_level,mid_level,high_level
);
    localparam integer PIXELS=WIDTH*HEIGHT;
    localparam integer SAMPLE_LIMIT=((WIDTH+SAMPLE_STRIDE-1)/SAMPLE_STRIDE)*HEIGHT;
    localparam integer PW=$clog2(PIXELS+2), SW=$clog2(SAMPLE_LIMIT+2);
    localparam integer CW=(SAMPLE_STRIDE>1)?$clog2(SAMPLE_STRIDE):1;
    reg [SW-1:0] histogram[0:63], snapshot[0:63];
    reg [PW-1:0] pixels;
    reg [SW-1:0] samples, snapshot_total;
    reg [CW-1:0] sample_phase, line_phase;
    reg de_prev, overflow, vs_prev;
    reg [7:0] de_history;
    wire [CW-1:0] phase=de_prev?sample_phase:line_phase;
    wire safe_blank=commit_vs && !commit_de && de_history==0 && !de;
    wire frame_start=safe_blank && !vs_prev;
    wire complete=pixels==PIXELS && !overflow && samples!=0;
    reg scanning, ready, blank_valid, applied, initialized;
    reg [5:0] index, low_bin, median_bin, high_bin;
    reg found_low, found_median, found_high;
    reg [SW:0] cdf;
    reg [7:0] target,pending_low,pending_mid,pending_high;
    wire [SW:0] cdf_next=cdf+{1'b0,snapshot[index]};
    wire [SW:0] low_cut=(snapshot_total>=8)?({1'b0,snapshot_total}>>3):1;
    wire [SW:0] median_cut=({1'b0,snapshot_total}+1)>>1;
    wire [SW:0] high_cut={1'b0,snapshot_total}-({1'b0,snapshot_total}>>3);
    wire [5:0] chosen_low=found_low?low_bin:((cdf_next>=low_cut)?index:low_bin);
    wire [5:0] chosen_median=found_median?median_bin:((cdf_next>=median_cut)?index:median_bin);
    wire [5:0] chosen_high=found_high?high_bin:((cdf_next>=high_cut)?index:high_bin);
    wire [7:0] low_center={chosen_low,2'b00}+8'd2;
    wire [7:0] median_center={chosen_median,2'b00}+8'd2;
    wire [7:0] high_center={chosen_high,2'b00}+8'd2;
    // Ttarget = round((3*P12.5 + 2*P50 + 3*P87.5)/8).
    // Four-level bins use their centers; thresholds stay within 4..251.
    wire [10:0] weighted={3'b0,low_center}+{2'b0,low_center,1'b0}+
                         {2'b0,median_center,1'b0}+
                         {3'b0,high_center}+{2'b0,high_center,1'b0}+11'd4;
    wire [7:0] candidate=weighted[10:3];
    wire [7:0] delta=(target>threshold)?target-threshold:threshold-target;
    wire [8:0] quarter_step=({1'b0,delta}+9'd3)>>2;
    wire [7:0] step=(quarter_step>MAX_STEP)?MAX_STEP:quarter_step[7:0];
    wire update_target=ready && safe_blank &&
        (frame_start?complete:(blank_valid && !applied));
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            threshold<=INITIAL_THRESHOLD; target<=INITIAL_THRESHOLD;
            low_level<=32;mid_level<=96;high_level<=192;pending_low<=32;pending_mid<=96;pending_high<=192;
            pixels<=0; samples<=0; snapshot_total<=0;
            sample_phase<=0; line_phase<=0; de_prev<=0; overflow<=0;
            vs_prev<=0; de_history<=0; scanning<=0; ready<=0;
            blank_valid<=0; applied<=0; initialized<=0;
            index<=0; low_bin<=0; median_bin<=0; high_bin<=0;
            found_low<=0; found_median<=0; found_high<=0; cdf<=0;
            for(i=0;i<64;i=i+1) begin histogram[i]<=0; snapshot[i]<=0; end
        end else begin
            vs_prev<=commit_vs; de_prev<=de;
            de_history<={de_history[6:0],commit_de};
            if(de) begin
                if(pixels<PIXELS) pixels<=pixels+1'b1;
                else overflow<=1;
                if(phase==SAMPLE_STRIDE-1) begin
                    sample_phase<=0;
                    if(samples<SAMPLE_LIMIT) begin
                        samples<=samples+1'b1;
                        histogram[gray[7:2]]<=histogram[gray[7:2]]+1'b1;
                    end else overflow<=1;
                end else sample_phase<=phase+1'b1;
            end
            // Move sample columns one position per row to reduce stride alias.
            if(!de && de_prev)
                line_phase<=(line_phase==SAMPLE_STRIDE-1)?0:line_phase+1'b1;
            if(scanning) begin
                cdf<=cdf_next;
                if(!found_low && cdf_next>=low_cut) begin low_bin<=index;found_low<=1;end
                if(!found_median && cdf_next>=median_cut) begin median_bin<=index;found_median<=1;end
                if(!found_high && cdf_next>=high_cut) begin high_bin<=index;found_high<=1;end
                if(index==63) begin
                    target<=(candidate<4)?4:((candidate>251)?251:candidate);
                    pending_low<=low_center;pending_mid<=median_center;pending_high<=high_center;
                    scanning<=0;ready<=1;
                end else index<=index+1'b1;
            end
            if(update_target) begin
                low_level<=pending_low;mid_level<=pending_mid;high_level<=pending_high;
                // First complete result initializes promptly. Later results
                // use a 1/4 IIR step capped at MAX_STEP and a 1-level deadband.
                if(!initialized) threshold<=target;
                else if(delta>1)
                    threshold<=(target>threshold)?threshold+step:threshold-step;
                initialized<=1;ready<=0;applied<=1;
            end
            if(frame_start) begin
                pixels<=0;samples<=0;sample_phase<=0;line_phase<=0;overflow<=0;
                blank_valid<=complete;applied<=update_target;
                for(i=0;i<64;i=i+1) histogram[i]<=0;
                // Invalid frames cannot publish a stale threshold result.
                if(!complete) ready<=0;
                if(complete && !scanning) begin
                    for(i=0;i<64;i=i+1) snapshot[i]<=histogram[i];
                    snapshot_total<=samples;index<=0;cdf<=0;
                    low_bin<=0;median_bin<=0;high_bin<=0;
                    found_low<=0;found_median<=0;found_high<=0;
                    scanning<=1;ready<=0;
                end
            end
        end
    end
endmodule

// Gentle luma contrast curve plus bounded chroma enhancement. Three stages
// keep all four bypass streams and enhanced RGB at exactly the same pixel.
// Defined in an existing TD compilation unit to support cached file lists.
module video_color_enhance (
    input wire clk, rst_n, in_vs, in_hs, in_de,
    input wire [23:0] in_color, in_gray, in_binary, in_edge,
    output wire out_vs, out_hs, out_de,
    output wire [23:0] color_rgb, gray_rgb, binary_rgb, edge_rgb,
    output reg [23:0] enhanced_rgb,
    output reg [23:0] display_rgb
);
    reg [15:0] r_luma_sum;
    reg [7:0] r_luma, r_tone;
    reg [95:0] r_pixels0, r_pixels1, r_pixels2;
    reg [2:0] r_ctrl0, r_ctrl1, r_ctrl2;

    // Nodes at Y=0,32,64,96,128,160,192,224,256:
    //                0,32,56,92,134,176,212,240,256.
    // Preserve deep shadows; increase midtone contrast; roll off highlights.
    function [7:0] tone_curve;
        input [7:0] y;
        integer base, slope, result;
        begin
            case(y[7:5])
                0: begin base=0;   slope=32; end
                1: begin base=32;  slope=24; end
                2: begin base=56;  slope=36; end
                3: begin base=92;  slope=42; end
                4: begin base=134; slope=42; end
                5: begin base=176; slope=36; end
                6: begin base=212; slope=28; end
                7: begin base=240; slope=16; end
                default: begin base=0; slope=32; end
            endcase
            result=base+((y[4:0]*slope+16)>>5);
            if(result>255) tone_curve=255;
            else tone_curve=result[7:0];
        end
    endfunction

    function [7:0] enhance_channel;
        input [7:0] c, y, mapped_y;
        reg signed [10:0] delta, chroma, result;
        begin
            delta=$signed({1'b0,c})-$signed({1'b0,y});
            // 1.25x chroma in midtones, 1.125x in deepest shadows/highlights.
            // Signed arithmetic and final saturation prevent wraparound.
            if(y<16 || y>=224) chroma=delta+((delta+11'sd4)>>>3);
            else chroma=delta+((delta+11'sd2)>>>2);
            result=$signed({3'b0,mapped_y})+chroma;
            if(result<0) enhance_channel=0;
            else if(result>255) enhance_channel=255;
            else enhance_channel=result[7:0];
        end
    endfunction

    // Display-only shadow/midtone lift. Same curve for all three channels;
    // raw color_rgb remains available for exposure and target analysis.
    // Input 0,32,64,96,128,160,192,224,256 maps to
    //       0,91,128,157,181,202,222,239,256.
    function [7:0] display_curve;
        input [7:0] c;
        reg [8:0] base;
        reg [6:0] slope;
        reg [11:0] product;
        reg [8:0] result;
        begin
            case(c[7:5])
                0: begin base=0;   slope=91; end
                1: begin base=91;  slope=37; end
                2: begin base=128; slope=29; end
                3: begin base=157; slope=24; end
                4: begin base=181; slope=21; end
                5: begin base=202; slope=20; end
                6: begin base=222; slope=17; end
                7: begin base=239; slope=17; end
                default: begin base=0; slope=32; end
            endcase
            product=c[4:0]*slope+12'd16;
            result=base+product[11:5];
            display_curve=(result>255)?8'd255:result[7:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r_luma_sum<=0; r_luma<=0; r_tone<=0;
            r_pixels0<=0; r_pixels1<=0; r_pixels2<=0;
            r_ctrl0<=0; r_ctrl1<=0; r_ctrl2<=0; enhanced_rgb<=0; display_rgb<=0;
        end else begin
            r_ctrl0<={in_vs,in_hs,in_de};
            r_ctrl1<=r_ctrl0; r_ctrl2<=r_ctrl1;
            r_pixels0<=in_de?{in_color,in_gray,in_binary,in_edge}:96'd0;
            r_pixels1<=r_pixels0; r_pixels2<=r_pixels1;
            r_luma_sum<=77*in_color[23:16]+150*in_color[15:8]+29*in_color[7:0]+128;
            r_luma<=r_luma_sum[15:8]; r_tone<=tone_curve(r_luma_sum[15:8]);
            if(r_ctrl1[0])
                enhanced_rgb<={enhance_channel(r_pixels1[95:88],r_luma,r_tone),
                              enhance_channel(r_pixels1[87:80],r_luma,r_tone),
                              enhance_channel(r_pixels1[79:72],r_luma,r_tone)};
            else enhanced_rgb<=0;
            if(r_ctrl1[0])
                display_rgb<={display_curve(r_pixels1[95:88]),
                              display_curve(r_pixels1[87:80]),
                              display_curve(r_pixels1[79:72])};
            else display_rgb<=0;
        end
    end
    assign {out_vs,out_hs,out_de}=r_ctrl2;
    assign {color_rgb,gray_rgb,binary_rgb,edge_rgb}=r_pixels2;
endmodule

// Frame metering before display processing. The camera is adjusted through
// the existing SC500 AE register table; no extra sensor registers are assumed.
module camera_auto_exposure #(
 parameter integer WIDTH=1280,HEIGHT=720,STRIDE=64,AUTO_ENABLE=1,TARGET_METER_ENABLE=0,
 parameter integer MIN_AE=8,MAX_AE=1990,MIN_AG=16,MAX_AG=80,
 parameter integer DARK_MEAN=64,LOW_MEAN=112,HIGH_MEAN=152,
 parameter integer VERY_HIGH_MEAN=160,EXTREME_MEAN=208,
 parameter integer BRIGHT_Y=225,CLIPPED_CHANNEL=235,SETTLE_FRAMES=4
)(
 input wire pixel_clk,pixel_rst_n,ctrl_clk,ctrl_rst_n,vs,de,
 input wire [23:0] rgb,
 input wire cam_cfg_done,ae_cfg_done,
 input wire target_enable,target_observed,
 input wire [11:0] target_x0,target_y0,target_x1,target_y1,
 output reg target_meter_active,
 output reg ae_req,
 output reg [15:0] ae,ag
);
 localparam PIXELS=WIDTH*HEIGHT,LIMIT=((WIDTH+STRIDE-1)/STRIDE)*HEIGHT;
 localparam PW=$clog2(PIXELS+2),CW=$clog2(LIMIT+2),PHW=$clog2(STRIDE);
 localparam SUMW=CW+8;
 reg [PW-1:0] pixels;
 reg [11:0] px_reg,rows;
 reg [CW-1:0] samples,bright,clipped,color_clipped,global_clipped;
 reg [11:0] roi_x0,roi_y0,roi_x1,roi_y1;
 reg [1:0] roi_streak;reg [3:0] roi_misses;
 wire roi_candidate=TARGET_METER_ENABLE && target_enable && target_observed &&
  target_x1<WIDTH && target_y1<HEIGHT && target_x1>target_x0 && target_y1>target_y0 &&
  target_x1-target_x0>=31 && target_y1-target_y0>=23;
 wire meter_sample=!target_meter_active || (px>=roi_x0 && px<=roi_x1 && rows>=roi_y0 && rows<=roi_y1);
 wire [CW+5:0] global_clip_scaled={1'b0,global_clipped,5'b0};
 wire global_white=target_meter_active && global_clip_scaled>=LIMIT*31;
 reg [SUMW-1:0] sum;
 reg [PHW-1:0] sample_phase,line_phase;
 reg de_prev,vs_prev,bad;
 reg [7:0] de_history;
 wire [11:0] px=de_prev?px_reg:12'd0;
 wire [PHW-1:0] phase=de_prev?sample_phase:line_phase;
 wire [15:0] luma_sum=77*rgb[23:16]+150*rgb[15:8]+29*rgb[7:0]+128;
 wire [7:0] y=luma_sum[15:8];
 wire is_clipped=rgb[23:16]>=CLIPPED_CHANNEL && rgb[15:8]>=CLIPPED_CHANNEL && rgb[7:0]>=CLIPPED_CHANNEL;
 wire is_color_clipped=rgb[23:16]>=250 || rgb[15:8]>=250 || rgb[7:0]>=250;
 wire start=vs && !vs_prev && !de && de_history==0;
 wire full=pixels==PIXELS && rows==HEIGHT && !bad && samples!=0;
 // Compare the sum to sample_count * threshold: no frame-rate divider.
 wire [SUMW-1:0] n=samples;
 wire extreme=global_white || sum>n*EXTREME_MEAN || ({7'd0,clipped}<<3)>=samples;
 wire severe=sum>n*VERY_HIGH_MEAN || ({7'd0,clipped}<<5)>=samples;
 wire too_bright=sum>n*HIGH_MEAN || ({7'd0,bright}<<4)>=samples || ({7'd0,color_clipped}<<3)>=samples;
 // Permit a small reflection without permanently blocking dark recovery:
 // near-white <= 1/128, any-channel saturation <= 1/64, bright < 1/64.
 // Broader saturation still uses the original fast reduction decisions.
 wire highlights_rare=({7'd0,clipped}<<7)<=samples &&
                      ({7'd0,color_clipped}<<6)<=samples &&
                      ({7'd0,bright}<<6)<samples;
 wire [2:0] decision=extreme?3'd3:(severe?3'd2:(too_bright?3'd1:
                      ((sum<n*DARK_MEAN && highlights_rare)?3'd5:
                       ((sum<n*LOW_MEAN && highlights_rare)?3'd4:3'd0))));
 reg [CW-1:0] snapshot_samples,snapshot_bright,snapshot_clipped,snapshot_color_clipped;
 reg [SUMW-1:0] snapshot_sum;
 // Bundled-data mailbox. Data stays fixed from request until acknowledgement.
 reg [2:0] r_action; // synthesis keep
 reg r_request; // synthesis keep
 reg r_ack_meta,r_ack_sync; // synthesis keep
 reg r_ack; // synthesis keep
 always @(posedge pixel_clk or negedge pixel_rst_n)begin
  if(!pixel_rst_n)begin
   pixels<=0;px_reg<=0;rows<=0;samples<=0;bright<=0;clipped<=0;color_clipped<=0;sum<=0;
   sample_phase<=0;line_phase<=0;de_prev<=0;vs_prev<=0;de_history<=0;bad<=0;
   target_meter_active<=0;roi_x0<=0;roi_y0<=0;roi_x1<=0;roi_y1<=0;
   roi_streak<=0;roi_misses<=0;global_clipped<=0;
   r_action<=0;r_request<=0;r_ack_meta<=0;r_ack_sync<=0;
   snapshot_samples<=0;snapshot_sum<=0;snapshot_bright<=0;snapshot_clipped<=0;snapshot_color_clipped<=0;
  end else begin
   r_ack_meta<=r_ack;r_ack_sync<=r_ack_meta;
   de_prev<=de;vs_prev<=vs;de_history<={de_history[6:0],de};
   if(de)begin
    px_reg<=px+1'b1;
    if(pixels<PIXELS)pixels<=pixels+1'b1;else bad<=1;
    if(px>=WIDTH || rows>=HEIGHT)bad<=1;
    if(phase==STRIDE-1)begin
     sample_phase<=0;
     global_clipped<=global_clipped+is_clipped;
     if(meter_sample)begin
     if(samples<LIMIT)begin
      samples<=samples+1'b1;sum<=sum+y;
      bright<=bright+(y>=BRIGHT_Y);clipped<=clipped+is_clipped;color_clipped<=color_clipped+is_color_clipped;
     end else bad<=1;
     end
    end else sample_phase<=phase+1'b1;
   end else begin
    px_reg<=0;
    if(de_prev)begin
     rows<=rows+1'b1;if(px_reg!=WIDTH)bad<=1;
     line_phase<=(line_phase==STRIDE-1)?0:line_phase+1'b1;
    end
   end
   if(start)begin
    // Bounds and policy are latched only between complete frames. Three
    // observations acquire ROI metering; eight misses return to global.
    global_clipped<=0;
    if(!TARGET_METER_ENABLE || !target_enable)begin
     target_meter_active<=0;roi_streak<=0;roi_misses<=0;
    end else if(roi_candidate)begin
     roi_misses<=0;if(roi_streak!=3)roi_streak<=roi_streak+1'b1;
     if(roi_streak>=2)target_meter_active<=1;
     roi_x0<=target_x0;roi_y0<=target_y0;roi_x1<=target_x1;roi_y1<=target_y1;
    end else begin
     roi_streak<=0;if(roi_misses!=15)roi_misses<=roi_misses+1'b1;
     if(roi_misses>=7)target_meter_active<=0;
    end
    if(full && (!target_meter_active || samples>=4) && r_request==r_ack_sync)begin
     r_action<=AUTO_ENABLE?decision:3'd0;r_request<=~r_request;
     snapshot_samples<=samples;snapshot_sum<=sum;
     snapshot_bright<=bright;snapshot_clipped<=clipped;snapshot_color_clipped<=color_clipped;
    end
    pixels<=0;px_reg<=0;rows<=0;samples<=0;bright<=0;clipped<=0;color_clipped<=0;sum<=0;
    sample_phase<=0;line_phase<=0;bad<=0;
   end
  end
 end
 reg r_req_meta,r_req_sync; // synthesis keep
 reg inflight,seen_busy;
 reg [3:0] settle;
 reg [2:0] r_sink_action; // synthesis keep
 integer next_ae,next_ag,delta;
 always @*begin
  next_ae=ae;next_ag=ag;delta=0;
  case(r_action)
   1,2,3:begin
    // Lower analogue gain first. At unity gain shorten integration time.
    if(ag>MIN_AG)begin
     delta=(r_action==3)?32:((r_action==2)?16:4);
     next_ag=(ag-MIN_AG<delta)?MIN_AG:ag-delta;
    end else begin
     delta=(r_action==3)?(ae>>1):((r_action==2)?(ae>>2):(ae>>3));
     if(delta<1)delta=1;
     next_ae=(ae-MIN_AE<delta)?MIN_AE:ae-delta;
    end
   end
   4,5:begin
    // Prefer longer exposure to extra analogue gain in dark scenes.
    if(ae<MAX_AE)begin
     delta=(r_action==5)?(ae>>3):(ae>>4);if(delta<1)delta=1;
     next_ae=(ae+delta>MAX_AE)?MAX_AE:ae+delta;
    end else begin
     delta=(r_action==5)?4:2;
     next_ag=(ag+delta>MAX_AG)?MAX_AG:ag+delta;
    end
   end
   default:;
  endcase
 end
 always @(posedge ctrl_clk or negedge ctrl_rst_n)begin
  if(!ctrl_rst_n)begin
   r_req_meta<=0;r_req_sync<=0;r_ack<=0;r_sink_action<=0;
   ae_req<=0;ae<=MAX_AE;ag<=MAX_AG;inflight<=0;seen_busy<=0;settle<=8;
  end else begin
   r_req_meta<=r_request;r_req_sync<=r_req_meta;ae_req<=0;
   if(inflight)begin
    if(!ae_cfg_done)seen_busy<=1;
    if(seen_busy && ae_cfg_done)begin inflight<=0;seen_busy<=0;end
   end
   if(r_req_sync!=r_ack && cam_cfg_done && ae_cfg_done && !inflight)begin
    r_ack<=r_req_sync;r_sink_action<=r_action;
    if(settle!=0)settle<=settle-1'b1;
    else if(next_ae!=ae || next_ag!=ag)begin
     ae<=next_ae;ag<=next_ag;ae_req<=1;inflight<=1;seen_busy<=0;settle<=SETTLE_FRAMES;
    end
   end
  end
 end
endmodule
