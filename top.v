module top (
    input  wire       clk,
    output wire [5:0] led
);

    reg [23:0] counter = 0;
    reg        state   = 1'b0;

    always @(posedge clk) begin
        if (counter == 24'd13_500_000) begin
            counter <= 0;
            state   <= ~state;
        end else begin
            counter <= counter + 1'b1;
        end
    end

    assign led = {6{state}};   // all six LEDs blink together

endmodule