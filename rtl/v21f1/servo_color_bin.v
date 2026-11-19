// Division-free 12-sector hue classifier shared by acquisition and diagnostics.
module servo_color_bin(input wire [23:0] rgb,output wire [3:0] bin);
 wire [7:0] r=rgb[23:16],g=rgb[15:8],b=rgb[7:0];
 reg [2:0] sector;reg [7:0] maximum,span,mid;reg ascending;
 always @*begin
  if(r>=g && g>=b)begin sector=0;maximum=r;span=r-b;mid=g-b;ascending=1;end
  else if(g>r && r>=b)begin sector=1;maximum=g;span=g-b;mid=r-b;ascending=0;end
  else if(g>=b && b>r)begin sector=2;maximum=g;span=g-r;mid=b-r;ascending=1;end
  else if(b>g && g>r)begin sector=3;maximum=b;span=b-r;mid=g-r;ascending=0;end
  else if(b>r && r>=g)begin sector=4;maximum=b;span=b-g;mid=r-g;ascending=1;end
  else begin sector=5;maximum=r;span=r-g;mid=b-g;ascending=0;end
 end
 wire [9:0] m4={mid,2'b00},c={2'b00,span};
 wire [1:0] q=(m4>=c+(c<<1))?3:(m4>=(c<<1))?2:(m4>=c)?1:0;
 wire [4:0] hue={sector,2'b00}+(ascending?q:2'd3-q);
 wire chroma=maximum>=16 && span>=8 && {span,4'b0}>={4'b0,maximum};
 assign bin=chroma?{1'b0,hue[4:1]}:(maximum<48)?12:(maximum<160)?13:14;
endmodule
