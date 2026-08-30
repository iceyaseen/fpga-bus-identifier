`timescale 1ns/1ps
// ============================================================
//  Testbench for top.v - decodes the SPI byte stream that the
//  design actually drives onto lcd_sclk/lcd_mosi/lcd_dc.
//  top.v is instantiated unmodified (no parameter overrides) -
//  this exercises the exact same RTL that gets synthesized.
// ============================================================
module top_tb;

    reg clk = 1'b0;
    always #1 clk = ~clk;   // arbitrary sim time unit, only edge count matters

    wire lcd_rst, lcd_dc, lcd_cs, lcd_sclk, lcd_mosi;
    wire [5:0] led;

    initial begin
        $dumpfile("top_tb.vcd");
        $dumpvars(0, lcd_dc, lcd_cs, lcd_sclk, lcd_mosi, lcd_rst, led);
    end

    top dut (
        .clk      (clk),
        .lcd_rst  (lcd_rst),
        .lcd_dc   (lcd_dc),
        .lcd_cs   (lcd_cs),
        .lcd_sclk (lcd_sclk),
        .lcd_mosi (lcd_mosi),
        .led      (led)
    );

    // ---- SPI decoder: sample MOSI on each rising edge of SCLK,
    //      group MSB-first into bytes, tag each with lcd_dc ----
    reg [7:0] shreg      = 8'h00;
    reg [3:0] bitcnt     = 4'd0;
    integer   byte_count = 0;

    always @(posedge lcd_sclk) begin
        shreg  = {shreg[6:0], lcd_mosi};
        bitcnt = bitcnt + 1'b1;
        if (bitcnt == 4'd8) begin
            bitcnt     = 4'd0;
            byte_count = byte_count + 1;
            $display("BYTE %0d: 0x%02h  dc=%0d (%s)",
                      byte_count, shreg, lcd_dc,
                      lcd_dc ? "DATA" : "CMD");
        end
    end

    // ---- stop once the design reaches S_DONE (all LEDs on = 6'b000000) ----
    initial begin
        wait (led == 6'b000000);
        #100;
        $display("---- reached S_DONE, total bytes sent = %0d ----", byte_count);
        $finish;
    end

    // ---- watchdog: bail out if the design never finishes ----
    initial begin
        #900_000_000;
        $display("---- WATCHDOG TIMEOUT: design never reached S_DONE, bytes so far = %0d ----", byte_count);
        $finish;
    end

endmodule
