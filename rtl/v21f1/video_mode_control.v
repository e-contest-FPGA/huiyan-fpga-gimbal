// V20: RAW, GRAY, BINARY, EDGE, ADAPT, CARTOON, COMPARE; boot ADAPT.
// Debounced keys share the pixel clock with this controller: no bus CDC.
// Bit 0: K3 next; bit 1: K4 previous. Hold gives only one change.
module video_mode_control (
    input wire clk, rst_n,
    input wire [1:0] key_n,
    input wire in_vs, in_de,
    output reg [2:0] active_mode
);
    reg [1:0] r_key_prev;
    reg [2:0] r_requested;
    reg r_vs_prev;
    reg [7:0] r_de_history;
    wire [1:0] press = r_key_prev & ~key_n;
    reg [2:0] next_request;
    always @* begin
        next_request = r_requested;
        case (press)
            2'b01: next_request = (r_requested==3'd6)?3'd0:r_requested+3'd1;
            2'b10: next_request = (r_requested==0)?3'd6:r_requested-3'd1;
            default: next_request = r_requested; // simultaneous presses cancel
        endcase
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_key_prev<=2'b11; r_requested<=4; active_mode<=4;
            r_vs_prev<=0; r_de_history<=0;
        end else begin
            r_key_prev<=key_n;
            r_requested<=next_request;
            r_vs_prev<=in_vs;
            r_de_history<={r_de_history[6:0],in_de};
            // Only update in vertical blank, after the video pipeline drains.
            // A button event on this exact edge is included in the new mode.
            if (in_vs && !r_vs_prev && !in_de && r_de_history==0)
                active_mode<=next_request;
        end
    end
endmodule

// SW4 is a held level, unlike an edge-triggered key. Accept its boot position
// after debounce, then commit only in output vertical blank.
module video_enhance_switch #(
    parameter integer CLK_HZ=75000000, DEBOUNCE_MS=20
) (
    input wire clk, rst_n, sw4, in_vs, in_de,
    output reg active
);
    // Divide before multiplying to avoid overflowing a signed 32-bit integer.
    localparam integer CYCLES=(CLK_HZ/1000)*DEBOUNCE_MS +
                             ((CLK_HZ%1000)*DEBOUNCE_MS+999)/1000;
    localparam integer CW=(CYCLES>1)?$clog2(CYCLES):1;
    reg r_meta, r_sync; // synthesis keep
    reg r_stable, r_vs_prev;
    reg [CW-1:0] r_count;
    reg [7:0] r_de_history;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r_meta<=0; r_sync<=0; r_stable<=0; r_count<=0;
            active<=0; r_vs_prev<=0; r_de_history<=0;
        end else begin
            r_meta<=sw4; r_sync<=r_meta;
            if(r_sync==r_stable) r_count<=0;
            else if(r_count==CYCLES-1) begin
                r_stable<=r_sync; r_count<=0;
            end else r_count<=r_count+1'b1;
            r_vs_prev<=in_vs;
            r_de_history<={r_de_history[6:0],in_de};
            if(in_vs && !r_vs_prev && !in_de && r_de_history==0)
                active<=r_stable;
        end
    end
endmodule
