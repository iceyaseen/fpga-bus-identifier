// ============================================================
//  A3 - ST7735S 80x160 screen: init + fill with one colour
//  Board: Sipeed Tang Nano 9K  (27 MHz clock)
// ============================================================

module top (
    input  wire       clk,        // 27 MHz, pin 52

    output reg        lcd_rst = 1'b0,
    output reg        lcd_dc  = 1'b0,
    output wire       lcd_cs,
    output wire       lcd_sclk,
    output wire       lcd_mosi,

    output wire [5:0] led
);

    // ---- settings you may want to change -------------------
    localparam [15:0] COLOR = 16'hF800;   // try 16'h07E0 (green), 16'h001F (blue)
    localparam        WIDTH  = 80;
    localparam        HEIGHT = 160;
    // window edges, already including this panel's offset (26 in x, 1 in y)
    localparam [7:0]  X_START = 8'd26;
    localparam [7:0]  X_END   = 8'd105;   // 26 + 80 - 1
    localparam [7:0]  Y_START = 8'd1;
    localparam [7:0]  Y_END   = 8'd160;   // 1 + 160 - 1
    localparam        CYCLES_PER_MS = 27000;
    // --------------------------------------------------------

    assign lcd_cs = 1'b0;                 // only one device on the bus, keep selected

    // ---- SPI byte sender -----------------------------------
    reg        spi_start = 1'b0;
    reg  [7:0] spi_data  = 8'h00;
    wire       spi_busy;

    spi_byte #(.DIV(4)) spi (          // 27MHz / (2*4) = 3.375 MHz, safe
        .clk   (clk),
        .start (spi_start),
        .data  (spi_data),
        .sclk  (lcd_sclk),
        .mosi  (lcd_mosi),
        .busy  (spi_busy)
    );

    // ---- init sequence ROM ---------------------------------
    // word[9] = 1 -> wait word[7:0] milliseconds
    // word[8] = 0 -> command byte,  1 -> data byte
    localparam ROM_LAST = 5'd21;

    reg  [4:0] rom_addr = 5'd0;
    wire [9:0] rom_word;

    // continuous assignment + function: guaranteed to settle at t=0,
    // unlike an always @(*) block keyed off a reg's declaration-time
    // initial value (which some simulators never re-trigger for the
    // very first case, leaving rom_word at X for rom_addr==0)
    function [9:0] rom_lookup(input [4:0] addr);
        case (addr)
            5'd0  : rom_lookup = {2'b00, 8'h01};          // SWRESET
            5'd1  : rom_lookup = {2'b10, 8'd150};         // wait 150 ms
            5'd2  : rom_lookup = {2'b00, 8'h11};          // SLPOUT  (wake up)
            5'd3  : rom_lookup = {2'b10, 8'd255};         // wait 255 ms
            5'd4  : rom_lookup = {2'b00, 8'h3A};          // COLMOD
            5'd5  : rom_lookup = {2'b01, 8'h05};          //   16 bits per pixel
            5'd6  : rom_lookup = {2'b00, 8'h36};          // MADCTL
            5'd7  : rom_lookup = {2'b01, 8'h08};          //   BGR order  (try 8'h00 if R/B swapped)
            5'd8  : rom_lookup = {2'b00, 8'h20};          // INVOFF   (try 8'h21 if colours look negative)
            5'd9  : rom_lookup = {2'b00, 8'h29};          // DISPON
            5'd10 : rom_lookup = {2'b10, 8'd100};         // wait 100 ms

            5'd11 : rom_lookup = {2'b00, 8'h2A};          // CASET - column range
            5'd12 : rom_lookup = {2'b01, 8'h00};
            5'd13 : rom_lookup = {2'b01, X_START};
            5'd14 : rom_lookup = {2'b01, 8'h00};
            5'd15 : rom_lookup = {2'b01, X_END};

            5'd16 : rom_lookup = {2'b00, 8'h2B};          // RASET - row range
            5'd17 : rom_lookup = {2'b01, 8'h00};
            5'd18 : rom_lookup = {2'b01, Y_START};
            5'd19 : rom_lookup = {2'b01, 8'h00};
            5'd20 : rom_lookup = {2'b01, Y_END};

            5'd21 : rom_lookup = {2'b00, 8'h2C};          // RAMWR - pixels follow
            default: rom_lookup = {2'b00, 8'h00};
        endcase
    endfunction

    assign rom_word = rom_lookup(rom_addr);

    // ---- main state machine --------------------------------
    localparam S_RST_LOW   = 3'd0,
               S_RST_WAIT  = 3'd1,
               S_INIT      = 3'd2,
               S_INIT_WAIT = 3'd3,
               S_DELAY     = 3'd4,
               S_FILL      = 3'd5,
               S_FILL_WAIT = 3'd6,
               S_DONE      = 3'd7;

    reg [2:0]  state     = S_RST_LOW;
    reg [23:0] delay_cnt = 24'd0;   // only used for the two fixed reset waits
    reg [14:0] ms_div    = 15'd0;   // counts clocks inside one millisecond
    reg [7:0]  ms_cnt    = 8'd0;    // counts milliseconds
    reg [7:0]  delay_ms  = 8'd0;
    reg [13:0] pix_cnt   = 14'd0;
    reg        hi_byte   = 1'b1;

    always @(posedge clk) begin
        spi_start <= 1'b0;                  // one-cycle pulse by default

        case (state)

        // hold reset low for 10 ms
        S_RST_LOW: begin
            lcd_rst <= 1'b0;
            if (delay_cnt == CYCLES_PER_MS * 10) begin
                delay_cnt <= 0;
                lcd_rst   <= 1'b1;
                state     <= S_RST_WAIT;
            end else begin
                delay_cnt <= delay_cnt + 1'b1;
            end
        end

        // let the chip settle for 120 ms
        S_RST_WAIT: begin
            if (delay_cnt == CYCLES_PER_MS * 120) begin
                delay_cnt <= 0;
                state     <= S_INIT;
            end else begin
                delay_cnt <= delay_cnt + 1'b1;
            end
        end

        // walk through the ROM
        S_INIT: begin
            if (rom_word[9]) begin              // it is a delay entry
                delay_ms <= rom_word[7:0];
                ms_div   <= 0;
                ms_cnt   <= 0;
                state    <= S_DELAY;
            end else begin                      // it is a byte to send
                lcd_dc    <= rom_word[8];
                spi_data  <= rom_word[7:0];
                spi_start <= 1'b1;
                state     <= S_INIT_WAIT;
            end
        end

        S_INIT_WAIT: begin
            if (!spi_busy) begin
                if (rom_addr == ROM_LAST) begin
                    pix_cnt <= 0;
                    hi_byte <= 1'b1;
                    state   <= S_FILL;
                end else begin
                    rom_addr <= rom_addr + 1'b1;
                    state    <= S_INIT;
                end
            end
        end

        // count milliseconds one at a time - no multiplier needed
        S_DELAY: begin
            if (ms_div == CYCLES_PER_MS - 1) begin
                ms_div <= 0;
                if (ms_cnt >= delay_ms) begin
                    ms_cnt   <= 0;
                    rom_addr <= rom_addr + 1'b1;
                    state    <= S_INIT;
                end else begin
                    ms_cnt <= ms_cnt + 1'b1;
                end
            end else begin
                ms_div <= ms_div + 1'b1;
            end
        end

        // stream WIDTH*HEIGHT pixels, 2 bytes each
        S_FILL: begin
            lcd_dc    <= 1'b1;                  // pixel data
            spi_data  <= hi_byte ? COLOR[15:8] : COLOR[7:0];
            spi_start <= 1'b1;
            state     <= S_FILL_WAIT;
        end

        S_FILL_WAIT: begin
            if (!spi_busy) begin
                if (hi_byte) begin
                    hi_byte <= 1'b0;
                    state   <= S_FILL;
                end else begin
                    hi_byte <= 1'b1;
                    if (pix_cnt == (WIDTH * HEIGHT - 1)) begin
                        state <= S_DONE;
                    end else begin
                        pix_cnt <= pix_cnt + 1'b1;
                        state   <= S_FILL;
                    end
                end
            end
        end

        S_DONE: begin
            state <= S_DONE;
        end

        endcase
    end

    // all LEDs turn on when the fill is finished (LEDs are active low)
    assign led = (state == S_DONE) ? 6'b000000 : 6'b111111;

endmodule


// ============================================================
//  SPI mode 0 byte sender - MSB first
//  clock idles low, the slave reads MOSI on the rising edge
// ============================================================
module spi_byte #(
    parameter DIV = 4                    // sclk period = 2 * DIV clocks
)(
    input  wire       clk,
    input  wire       start,
    input  wire [7:0] data,
    output reg        sclk = 1'b0,
    output reg        mosi = 1'b0,
    output wire       busy
);

    reg [7:0] shifter = 8'h00;
    reg [3:0] bits    = 4'd0;
    reg [7:0] div_cnt = 8'd0;
    reg       phase   = 1'b0;
    reg       busy_r  = 1'b0;

    // busy must go high in the SAME cycle as start, otherwise the caller
    // looks one cycle too early, sees busy=0, and thinks the byte is done
    assign busy = busy_r | start;

    always @(posedge clk) begin
        if (!busy_r) begin
            sclk <= 1'b0;
            if (start) begin
                shifter <= data;
                mosi    <= data[7];
                bits    <= 4'd8;
                div_cnt <= 0;
                phase   <= 1'b0;
                busy_r  <= 1'b1;
            end
        end else begin
            if (div_cnt == DIV - 1) begin
                div_cnt <= 0;
                if (phase == 1'b0) begin
                    sclk  <= 1'b1;              // rising edge - slave samples here
                    phase <= 1'b1;
                end else begin
                    sclk  <= 1'b0;
                    phase <= 1'b0;
                    bits  <= bits - 1'b1;
                    if (bits == 4'd1) begin
                        busy_r <= 1'b0;
                    end else begin
                        shifter <= {shifter[6:0], 1'b0};
                        mosi    <= shifter[6];
                    end
                end
            end else begin
                div_cnt <= div_cnt + 1'b1;
            end
        end
    end

endmodule