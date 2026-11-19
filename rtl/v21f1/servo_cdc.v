// V21 held-data mailbox. Payload is stable from request until round-trip ACK.
// A source may retry when busy; destination emits one pulse per accepted record.
module servo_cdc #(parameter WIDTH=32)(
 input wire s_clk,s_rst_n,s_send,input wire [WIDTH-1:0] s_data,
 output wire s_busy,
 input wire d_clk,d_rst_n,output reg d_valid,output reg [WIDTH-1:0] d_data
);
 reg [WIDTH-1:0] held;
 reg request,ack;
 reg [2:0] ack_sync,request_sync;
 assign s_busy=request!=ack_sync[2];
 always @(posedge s_clk or negedge s_rst_n)begin
  if(!s_rst_n)begin held<=0;request<=0;ack_sync<=0;end
  else begin
   ack_sync<={ack_sync[1:0],ack};
   if(s_send && !s_busy)begin held<=s_data;request<=~request;end
  end
 end
 always @(posedge d_clk or negedge d_rst_n)begin
  if(!d_rst_n)begin request_sync<=0;ack<=0;d_valid<=0;d_data<=0;end
  else begin
   request_sync<={request_sync[1:0],request};d_valid<=0;
   if(request_sync[2]!=ack)begin d_data<=held;d_valid<=1;ack<=request_sync[2];end
  end
 end
endmodule

// Sticky source-domain event counter. Snapshot crosses via servo_cdc, never
// by independently synchronising binary counter bits.
module servo_event_counter(input wire clk,rst_n,event_in,output reg [15:0] count);
 reg previous;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin previous<=0;count<=0;end
  else begin previous<=event_in;if(event_in && !previous && count!=16'hffff)count<=count+1'b1;end
 end
endmodule
