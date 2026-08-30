module top (
    input  wire       clk,
    input  wire       btn1,
    input  wire       btn2,
    output wire [5:0] led
);

    // buttons are active low: pressed = 0
    wire b1_clean, b2_clean;

    debounce d1 (.clk(clk), .noisy(~btn1), .clean(b1_clean));
    debounce d2 (.clk(clk), .noisy(~btn2), .clean(b2_clean));

    // LEDs are active low too, so invert
    assign led = ~{4'b0000, b2_clean, b1_clean};

endmodule


module debounce (
    input  wire clk,
    input  wire noisy,
    output reg  clean = 0
);
    reg [17:0] count = 0;      // ~10 ms at 27 MHz
    reg        last  = 0;

    always @(posedge clk) begin
        if (noisy != last) begin
            last  <= noisy;
            count <= 0;
        end else if (count == 18'd270_000) begin
            clean <= last;
        end else begin
            count <= count + 1'b1;
        end
    end
endmodule