`timescale 1ns/1ps
// ============================================================
//  Testbench for top.v
//
//  - Instantiates top unmodified (its RTL, not a re-implementation)
//    with CYCLES_PER_MS shrunk so the ms-scale init/reset delays
//    finish quickly in simulation.
//  - Decodes the SPI stream the design actually drives onto
//    lcd_sclk/lcd_mosi/lcd_dc, exactly like real hardware would see.
//  - Captures each full 80x160 RAMWR frame and writes it out as a
//    plain-text PGM image (frame_blank.pgm / frame_left.pgm /
//    frame_right.pgm) so the rendered word can be viewed on a PC
//    before ever touching the board.
//  - Drives btn1 then btn2 (active low, debounced ~10ms in the DUT -
//    that debounce time is NOT shortened, it's only ~270k clocks,
//    which simulates in well under a second).
//
//  Note: the y*80+x indexing and PGM row/col loops below are
//  testbench-only file I/O, not synthesizable RTL - top.v itself
//  never computes a pixel index that way (see x_cnt/y_cnt there).
// ============================================================
module top_tb;

    localparam WIDTH  = 80;
    localparam HEIGHT = 160;
    localparam NPIX   = WIDTH * HEIGHT;

    reg clk = 1'b0;
    always #1 clk = ~clk;   // arbitrary sim time unit, only edge count matters

    reg btn1 = 1'b1;   // active low - idle high (not pressed)
    reg btn2 = 1'b1;

    wire lcd_rst, lcd_dc, lcd_cs, lcd_sclk, lcd_mosi;
    wire [5:0] led;

    initial begin
        $dumpfile("top_tb.vcd");
        $dumpvars(0, lcd_dc, lcd_cs, lcd_sclk, lcd_mosi, lcd_rst, led);
    end

    top #(.CYCLES_PER_MS(4)) dut (
        .clk      (clk),
        .btn1     (btn1),
        .btn2     (btn2),
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

    // ---- pixel capture: after a RAMWR command byte, the next
    //      WIDTH*HEIGHT*2 data bytes are pixels, hi byte first ----
    reg [15:0] frame [0:NPIX-1];
    reg        capturing   = 1'b0;
    reg        hi_phase    = 1'b1;
    reg [7:0]  hi_byte_val = 8'h00;
    integer    pix_index   = 0;
    reg        frame_ready = 1'b0;

    always @(posedge lcd_sclk) begin
        shreg  = {shreg[6:0], lcd_mosi};
        bitcnt = bitcnt + 1'b1;
        if (bitcnt == 4'd8) begin
            bitcnt     = 4'd0;
            byte_count = byte_count + 1;

            if (!lcd_dc) begin
                // command byte
                if (shreg == 8'h2C) begin       // RAMWR: pixel stream starts next
                    capturing <= 1'b1;
                    hi_phase  <= 1'b1;
                    pix_index <= 0;
                end else begin
                    capturing <= 1'b0;
                end
            end else if (capturing) begin
                // data byte, part of the pixel stream
                if (hi_phase) begin
                    hi_byte_val <= shreg;
                    hi_phase    <= 1'b0;
                end else begin
                    frame[pix_index] <= {hi_byte_val, shreg};
                    hi_phase         <= 1'b1;
                    if (pix_index == NPIX - 1) begin
                        capturing   <= 1'b0;
                        frame_ready <= 1'b1;
                    end else begin
                        pix_index <= pix_index + 1;
                    end
                end
            end
        end
    end

    // ---- write the captured frame out as a plain-text PGM ----
    task write_pgm(input [8*32-1:0] fname);
        integer fh, x, y, idx, val;
        begin
            fh = $fopen(fname, "w");
            $fdisplay(fh, "P2");
            $fdisplay(fh, "%0d %0d", WIDTH, HEIGHT);
            $fdisplay(fh, "255");
            for (y = 0; y < HEIGHT; y = y + 1) begin
                for (x = 0; x < WIDTH; x = x + 1) begin
                    idx = y * WIDTH + x;   // sim-only: matches the DUT's raster order
                    val = (frame[idx] == 16'hFFFF) ? 255 : 0;
                    $fwrite(fh, "%0d ", val);
                end
                $fdisplay(fh, "");
            end
            $fclose(fh);
            $display("wrote %0s", fname);
        end
    endtask

    // ---- test sequence: capture blank, then LEFT, then RIGHT ----
    initial begin
        frame_ready = 1'b0;

        // wait for the very first full-screen draw (blank, WORD_NONE) to finish
        wait (frame_ready == 1'b1);
        @(posedge clk);
        write_pgm("frame_blank.pgm");
        frame_ready = 1'b0;
        wait (dut.state == dut.S_IDLE);

        // press button 1 -> expect "LEFT"
        repeat (5) @(posedge clk);
        btn1 = 1'b0;
        repeat (300_000) @(posedge clk);   // hold well past the ~270k-cycle debounce
        btn1 = 1'b1;

        wait (frame_ready == 1'b1);
        @(posedge clk);
        write_pgm("frame_left.pgm");
        frame_ready = 1'b0;
        wait (dut.state == dut.S_IDLE);

        // press button 2 -> expect "RIGHT"
        repeat (5) @(posedge clk);
        btn2 = 1'b0;
        repeat (300_000) @(posedge clk);
        btn2 = 1'b1;

        wait (frame_ready == 1'b1);
        @(posedge clk);
        write_pgm("frame_right.pgm");

        $display("---- done, total SPI bytes seen = %0d ----", byte_count);
        $finish;
    end

    // ---- watchdog: bail out if the design never finishes ----
    initial begin
        #200_000_000;
        $display("---- WATCHDOG TIMEOUT: bytes so far = %0d ----", byte_count);
        $finish;
    end

endmodule
