`timescale 1ns/1ps
module tb_frame_meter;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,cap=0,isp=0,vs=0,de=0,valid=0;
 wire [15:0] source_gray,isp_id;wire isp_valid;
 video_frame_source src(.clk(clk),.rst_n(rst),.capture_sof(cap),.isp_sof(isp),
 .capture_gray(source_gray),.isp_id(isp_id),.isp_valid(isp_valid));
 reg [15:0] gray=0,display_id=0;
 wire [1:0] status;wire [11:0] bcd;wire [15:0] c,d;
 video_frame_latency #(.CLK_HZ(300)) dut(clk,rst,gray,vs,de,display_id,valid,status,bcd,c,d);
 integer k,n,delta,expected,checks=0;
 task sample(input integer latest,input integer shown,input bit good,input integer state);
  begin
   @(negedge clk);gray=(latest>>1)^latest;display_id=shown;valid=good;
   repeat(5)@(negedge clk);vs=1;@(negedge clk);vs=0;
   repeat(3)@(negedge clk);de=1;repeat(12)@(negedge clk);de=0;
   expected=(latest-shown)&65535;
   if(status!==state || c!==latest[15:0] || d!==shown[15:0])
    $fatal(1,"FAIL sample C=%h D=%h state=%d wanted=%d",c,d,status,state);
   if(state==1 && bcd!==((expected/100)*256+((expected/10)%10)*16+expected%10))
    $fatal(1,"FAIL decimal delay %h expected %d",bcd,expected);
   checks++;
  end
 endtask
 initial begin
  #1;rst=0;repeat(4)@(negedge clk);rst=1;
  sample(0,0,0,0); // no source: WAIT
  for(k=1;k<=256;k++) begin
   n=(k*257)&65535;delta=k-1;
   sample(n,(n-delta)&65535,1,1);
  end
  sample(2,65535,1,1); // modulo rollover, age three
  sample(9,7,0,3);sample(10,11,1,3);sample(500,100,1,3);
  // OSD stays stable across later counter changes within the same frame.
  @(negedge clk);gray=(600>>1)^600;repeat(6)@(negedge clk);
  if(c!==500 || d!==100) $fatal(1,"FAIL snapshot not frame stable");
  repeat(310)@(negedge clk);if(status!==2)$fatal(1,"FAIL stale capture");
  sample(601,598,1,1);
  // Source pairing and real 16-bit wrap, plus duplicate/unpaired ISP faults.
  @(negedge clk);src.capture_id=16'hfffe;src.capture_gray=(16'hfffe>>1)^16'hfffe;
  cap=1;@(negedge clk);cap=0;
  if(source_gray!==((16'hffff>>1)^16'hffff) || isp_id!==16'hffff || !isp_valid)$fatal(1,"FAIL source SOF");
  isp=1;@(negedge clk);isp=0;@(negedge clk);cap=1;isp=1;
  #1;if(isp_id!==0 || !isp_valid)$fatal(1,"FAIL zero latency bypass wrap");
  @(negedge clk);cap=0;isp=0;@(negedge clk);isp=1;@(negedge clk);isp=0;
  if(isp_valid)$fatal(1,"FAIL unpaired source must invalidate");
  // Actual bench failure: one orphan start must not disable every later
  // frame. The resynchronizing pair itself remains invalid at its SOF.
  @(negedge clk);cap=1;@(negedge clk);cap=0;
  @(negedge clk);isp=1;#1;if(isp_valid)$fatal(1,"FAIL recovery pair accepted too early");
  @(negedge clk);isp=0;
  @(negedge clk);cap=1;@(negedge clk);cap=0;
  @(negedge clk);isp=1;#1;if(!isp_valid)$fatal(1,"FAIL clean pair cannot recover");
  @(negedge clk);isp=0;
  // Two CSI starts with no ISP in between invalidate the affected frame.
  @(negedge clk);cap=1;@(negedge clk);cap=0;
  @(negedge clk);cap=1;@(negedge clk);cap=0;
  @(negedge clk);isp=1;#1;if(isp_valid)$fatal(1,"FAIL duplicate CSI accepted");
  @(negedge clk);isp=0;
  $display("PASS frame_meter %0d samples: all 0..255 decimal ages, wrap, reset, WAIT/ERROR/STALE, frame snapshots and source pairing",checks);
  $finish;
 end
 initial begin #2000000;$fatal(1,"FAIL timeout");end
endmodule
