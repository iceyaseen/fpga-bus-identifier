// ============================================================
//  A3 - ST7735S 80x160 screen: show "LEFT" / "RIGHT" on button press
//  Board: Sipeed Tang Nano 9K  (27 MHz clock)
//
//  No framebuffer: each pixel's colour is decided while it is being
//  streamed out over SPI, using two counters (x_cnt, y_cnt) that
//  just increment - never computed as index % WIDTH / index / WIDTH.
// ============================================================

module top #(
    parameter CYCLES_PER_MS = 27000     // override in simulation to speed up delays
) (
    input  wire       clk,        // 27 MHz, pin 52

    input  wire        btn1,       // active low, pin 3
    input  wire        btn2,       // active low, pin 4

    input  wire       uart_rx,    // from host (USB-serial TXD), pin 18
    output wire       uart_tx,    // to host (USB-serial RXD), pin 17

    output reg        lcd_rst = 1'b0,
    output reg        lcd_dc  = 1'b0,
    output wire       lcd_cs,
    output wire       lcd_sclk,
    output wire       lcd_mosi,

    output wire [5:0] led
);

    // ---- settings -------------------------------------------
    localparam        WIDTH  = 80;
    localparam        HEIGHT = 160;
    // window edges, already including this panel's offset (26 in x, 1 in y)
    localparam [7:0]  X_START = 8'd26;
    localparam [7:0]  X_END   = 8'd105;   // 26 + 80 - 1
    localparam [7:0]  Y_START = 8'd1;
    localparam [7:0]  Y_END   = 8'd160;   // 1 + 160 - 1

    // text colours (RGB565 - all-1s / all-0s so BGR vs RGB order doesn't matter)
    localparam [15:0] COLOR_WHITE = 16'hFFFF;
    localparam [15:0] COLOR_BLACK = 16'h0000;

    // which word is currently shown
    localparam [1:0]  WORD_NONE  = 2'd0,
                       WORD_LEFT = 2'd1,
                       WORD_RIGHT= 2'd2;

    // 8x8 glyph ids - only the letters LEFT/RIGHT need
    localparam [2:0]  GLYPH_L = 3'd0,
                       GLYPH_E = 3'd1,
                       GLYPH_F = 3'd2,
                       GLYPH_T = 3'd3,
                       GLYPH_R = 3'd4,
                       GLYPH_I = 3'd5,
                       GLYPH_G = 3'd6,
                       GLYPH_H = 3'd7;

    // text box: 16x16 px per glyph (8x8 font at 2x scale), centred.
    // "RIGHT" = 5 chars = 80 px = exactly the screen width -> x offset 0.
    // "LEFT"  = 4 chars = 64 px -> x offset (80-64)/2 = 8.
    // Both words are vertically centred: (160-16)/2 = 72.
    localparam [7:0]  TEXT_Y_START  = 8'd72,
                       TEXT_Y_END   = 8'd87;
    localparam [7:0]  LEFT_X_START  = 8'd8,
                       LEFT_X_END   = 8'd71;
    localparam [7:0]  RIGHT_X_START = 8'd0,
                       RIGHT_X_END  = 8'd79;
    // --------------------------------------------------------

    assign lcd_cs = 1'b0;                 // only one device on the bus, keep selected

    // ---- buttons: debounce + rising-edge detect --------------
    wire b1_clean, b2_clean;

    debounce d1 (.clk(clk), .noisy(~btn1), .clean(b1_clean));
    debounce d2 (.clk(clk), .noisy(~btn2), .clean(b2_clean));

    reg b1_prev = 1'b0;
    reg b2_prev = 1'b0;
    wire b1_edge = b1_clean & ~b1_prev;
    wire b2_edge = b2_clean & ~b2_prev;

    always @(posedge clk) begin
        b1_prev <= b1_clean;
        b2_prev <= b2_clean;
    end

    // ---- UART: send "Hello from FPGA\r\n" once a second, and echo
    //      back whatever the host sends, showing it on the LEDs ----
    wire       uart_tx_busy;
    reg        uart_tx_start = 1'b0;
    reg  [7:0] uart_tx_data  = 8'h00;
    wire [7:0] uart_rx_data;
    wire       uart_rx_valid;

    uart_tx tx_unit (
        .clk   (clk),
        .start (uart_tx_start),
        .data  (uart_tx_data),
        .tx    (uart_tx),
        .busy  (uart_tx_busy)
    );

    uart_rx rx_unit (
        .clk   (clk),
        .rx    (uart_rx),
        .data  (uart_rx_data),
        .valid (uart_rx_valid)
    );

    // low 6 bits of the last byte received, shown on the LEDs
    reg [7:0] rx_byte_latch = 8'h00;
    always @(posedge clk) begin
        if (uart_rx_valid) rx_byte_latch <= uart_rx_data;
    end

    // "Hello from FPGA\r\n" - 17 bytes, sent once per second
    localparam MSG_LAST = 5'd16;

    function [7:0] msg_lookup(input [4:0] addr);
        case (addr)
            5'd0  : msg_lookup = 8'h48;   // H
            5'd1  : msg_lookup = 8'h65;   // e
            5'd2  : msg_lookup = 8'h6C;   // l
            5'd3  : msg_lookup = 8'h6C;   // l
            5'd4  : msg_lookup = 8'h6F;   // o
            5'd5  : msg_lookup = 8'h20;   // ' '
            5'd6  : msg_lookup = 8'h66;   // f
            5'd7  : msg_lookup = 8'h72;   // r
            5'd8  : msg_lookup = 8'h6F;   // o
            5'd9  : msg_lookup = 8'h6D;   // m
            5'd10 : msg_lookup = 8'h20;   // ' '
            5'd11 : msg_lookup = 8'h46;   // F
            5'd12 : msg_lookup = 8'h50;   // P
            5'd13 : msg_lookup = 8'h47;   // G
            5'd14 : msg_lookup = 8'h41;   // A
            5'd15 : msg_lookup = 8'h0D;   // \r
            5'd16 : msg_lookup = 8'h0A;   // \n
            default: msg_lookup = 8'h00;
        endcase
    endfunction

    // once-a-second tick - reuses CYCLES_PER_MS so simulation can shrink it too
    reg [24:0] sec_cnt  = 25'd0;
    reg        sec_tick = 1'b0;

    always @(posedge clk) begin
        sec_tick <= 1'b0;
        if (sec_cnt == (CYCLES_PER_MS * 1000) - 1'b1) begin
            sec_cnt  <= 25'd0;
            sec_tick <= 1'b1;
        end else begin
            sec_cnt <= sec_cnt + 1'b1;
        end
    end

    // single arbiter owns the one physical TX line: echo takes priority
    // over the housekeeping message, since it's a direct response to the host
    localparam TX_IDLE      = 2'd0,
               TX_HELLO     = 2'd1,
               TX_HELLO_WAIT= 2'd2,
               TX_ECHO_WAIT = 2'd3;

    reg [1:0] tx_state     = TX_IDLE;
    reg [4:0] msg_addr     = 5'd0;
    reg       hello_pending= 1'b0;
    reg       echo_pending = 1'b0;
    reg [7:0] echo_byte    = 8'h00;

    always @(posedge clk) begin
        uart_tx_start <= 1'b0;

        case (tx_state)
        TX_IDLE: begin
            if (echo_pending) begin
                uart_tx_data  <= echo_byte;
                uart_tx_start <= 1'b1;
                echo_pending  <= 1'b0;
                tx_state      <= TX_ECHO_WAIT;
            end else if (hello_pending) begin
                msg_addr <= 5'd0;
                tx_state <= TX_HELLO;
            end
        end

        TX_HELLO: begin
            uart_tx_data  <= msg_lookup(msg_addr);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_HELLO_WAIT;
        end

        TX_HELLO_WAIT: begin
            if (!uart_tx_busy) begin
                if (msg_addr == MSG_LAST) begin
                    hello_pending <= 1'b0;
                    tx_state      <= TX_IDLE;
                end else begin
                    msg_addr <= msg_addr + 1'b1;
                    tx_state <= TX_HELLO;
                end
            end
        end

        TX_ECHO_WAIT: begin
            if (!uart_tx_busy) tx_state <= TX_IDLE;
        end

        default: tx_state <= TX_IDLE;
        endcase

        // a fresh request always wins over a same-cycle "done" clear above,
        // so a byte that lands exactly as one job finishes is never dropped
        if (sec_tick) hello_pending <= 1'b1;
        if (uart_rx_valid) begin
            echo_byte    <= uart_rx_data;
            echo_pending <= 1'b1;
        end
    end

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

    // ---- init ROM: runs once after power-up ------------------
    // word[9] = 1 -> wait word[7:0] milliseconds
    // word[8] = 0 -> command byte,  1 -> data byte
    localparam ROM_INIT_LAST = 4'd10;

    function [9:0] rom_init_lookup(input [3:0] addr);
        case (addr)
            4'd0  : rom_init_lookup = {2'b00, 8'h01};          // SWRESET
            4'd1  : rom_init_lookup = {2'b10, 8'd150};         // wait 150 ms
            4'd2  : rom_init_lookup = {2'b00, 8'h11};          // SLPOUT  (wake up)
            4'd3  : rom_init_lookup = {2'b10, 8'd255};         // wait 255 ms
            4'd4  : rom_init_lookup = {2'b00, 8'h3A};          // COLMOD
            4'd5  : rom_init_lookup = {2'b01, 8'h05};          //   16 bits per pixel
            4'd6  : rom_init_lookup = {2'b00, 8'h36};          // MADCTL
            4'd7  : rom_init_lookup = {2'b01, 8'h08};          //   BGR order
            4'd8  : rom_init_lookup = {2'b00, 8'h20};          // INVOFF
            4'd9  : rom_init_lookup = {2'b00, 8'h29};          // DISPON
            4'd10 : rom_init_lookup = {2'b10, 8'd100};         // wait 100 ms
            default: rom_init_lookup = {2'b00, 8'h00};
        endcase
    endfunction

    // ---- draw ROM: CASET/RASET/RAMWR, re-run before every redraw ----
    localparam ROM_DRAW_LAST = 4'd10;

    function [9:0] rom_draw_lookup(input [3:0] addr);
        case (addr)
            4'd0  : rom_draw_lookup = {2'b00, 8'h2A};          // CASET - column range
            4'd1  : rom_draw_lookup = {2'b01, 8'h00};
            4'd2  : rom_draw_lookup = {2'b01, X_START};
            4'd3  : rom_draw_lookup = {2'b01, 8'h00};
            4'd4  : rom_draw_lookup = {2'b01, X_END};

            4'd5  : rom_draw_lookup = {2'b00, 8'h2B};          // RASET - row range
            4'd6  : rom_draw_lookup = {2'b01, 8'h00};
            4'd7  : rom_draw_lookup = {2'b01, Y_START};
            4'd8  : rom_draw_lookup = {2'b01, 8'h00};
            4'd9  : rom_draw_lookup = {2'b01, Y_END};

            4'd10 : rom_draw_lookup = {2'b00, 8'h2C};          // RAMWR - pixels follow
            default: rom_draw_lookup = {2'b00, 8'h00};
        endcase
    endfunction

    // ---- font ROM: which glyph goes in column `col` of each word ----
    function [2:0] glyph_for_col(input [1:0] word, input [2:0] col);
        case (word)
            WORD_LEFT: case (col)
                3'd0: glyph_for_col = GLYPH_L;
                3'd1: glyph_for_col = GLYPH_E;
                3'd2: glyph_for_col = GLYPH_F;
                3'd3: glyph_for_col = GLYPH_T;
                default: glyph_for_col = GLYPH_L;
            endcase
            WORD_RIGHT: case (col)
                3'd0: glyph_for_col = GLYPH_R;
                3'd1: glyph_for_col = GLYPH_I;
                3'd2: glyph_for_col = GLYPH_G;
                3'd3: glyph_for_col = GLYPH_H;
                3'd4: glyph_for_col = GLYPH_T;
                default: glyph_for_col = GLYPH_R;
            endcase
            default: glyph_for_col = GLYPH_L;
        endcase
    endfunction

    // 8x8 bitmaps, MSB = leftmost column, row 0 = top
    function [7:0] font_row(input [2:0] glyph, input [2:0] row);
        case (glyph)
            GLYPH_L: case (row)
                3'd0: font_row = 8'b10000000;
                3'd1: font_row = 8'b10000000;
                3'd2: font_row = 8'b10000000;
                3'd3: font_row = 8'b10000000;
                3'd4: font_row = 8'b10000000;
                3'd5: font_row = 8'b10000000;
                3'd6: font_row = 8'b10000000;
                3'd7: font_row = 8'b11111110;
            endcase
            GLYPH_E: case (row)
                3'd0: font_row = 8'b11111110;
                3'd1: font_row = 8'b10000000;
                3'd2: font_row = 8'b10000000;
                3'd3: font_row = 8'b11111100;
                3'd4: font_row = 8'b10000000;
                3'd5: font_row = 8'b10000000;
                3'd6: font_row = 8'b10000000;
                3'd7: font_row = 8'b11111110;
            endcase
            GLYPH_F: case (row)
                3'd0: font_row = 8'b11111110;
                3'd1: font_row = 8'b10000000;
                3'd2: font_row = 8'b10000000;
                3'd3: font_row = 8'b11111100;
                3'd4: font_row = 8'b10000000;
                3'd5: font_row = 8'b10000000;
                3'd6: font_row = 8'b10000000;
                3'd7: font_row = 8'b10000000;
            endcase
            GLYPH_T: case (row)
                3'd0: font_row = 8'b11111110;
                3'd1: font_row = 8'b00010000;
                3'd2: font_row = 8'b00010000;
                3'd3: font_row = 8'b00010000;
                3'd4: font_row = 8'b00010000;
                3'd5: font_row = 8'b00010000;
                3'd6: font_row = 8'b00010000;
                3'd7: font_row = 8'b00010000;
            endcase
            GLYPH_R: case (row)
                3'd0: font_row = 8'b11111100;
                3'd1: font_row = 8'b10000010;
                3'd2: font_row = 8'b10000010;
                3'd3: font_row = 8'b11111100;
                3'd4: font_row = 8'b10010000;
                3'd5: font_row = 8'b10001000;
                3'd6: font_row = 8'b10000100;
                3'd7: font_row = 8'b10000010;
            endcase
            GLYPH_I: case (row)
                3'd0: font_row = 8'b11111110;
                3'd1: font_row = 8'b00010000;
                3'd2: font_row = 8'b00010000;
                3'd3: font_row = 8'b00010000;
                3'd4: font_row = 8'b00010000;
                3'd5: font_row = 8'b00010000;
                3'd6: font_row = 8'b00010000;
                3'd7: font_row = 8'b11111110;
            endcase
            GLYPH_G: case (row)
                3'd0: font_row = 8'b01111100;
                3'd1: font_row = 8'b10000010;
                3'd2: font_row = 8'b10000000;
                3'd3: font_row = 8'b10001110;
                3'd4: font_row = 8'b10000010;
                3'd5: font_row = 8'b10000010;
                3'd6: font_row = 8'b10000010;
                3'd7: font_row = 8'b01111100;
            endcase
            GLYPH_H: case (row)
                3'd0: font_row = 8'b10000010;
                3'd1: font_row = 8'b10000010;
                3'd2: font_row = 8'b10000010;
                3'd3: font_row = 8'b11111110;
                3'd4: font_row = 8'b10000010;
                3'd5: font_row = 8'b10000010;
                3'd6: font_row = 8'b10000010;
                3'd7: font_row = 8'b10000010;
            endcase
            default: font_row = 8'b00000000;
        endcase
    endfunction

    // ---- pixel colour, decided combinationally from x_cnt/y_cnt ----
    reg  [1:0] current_word = WORD_NONE;
    reg  [7:0] x_cnt = 8'd0;   // 0..79
    reg  [7:0] y_cnt = 8'd0;   // 0..159

    wire word_is_right = (current_word == WORD_RIGHT);
    wire [7:0] text_x_start = word_is_right ? RIGHT_X_START : LEFT_X_START;
    wire [7:0] text_x_end   = word_is_right ? RIGHT_X_END   : LEFT_X_END;

    wire in_y_band    = (y_cnt >= TEXT_Y_START) && (y_cnt <= TEXT_Y_END);
    wire in_x_band    = (x_cnt >= text_x_start) && (x_cnt <= text_x_end);
    wire in_glyph_box = (current_word != WORD_NONE) && in_y_band && in_x_band;

    // local_x/local_y: 0..15 inside the current glyph's 16x16 cell.
    // char_col: which of the (4 or 5) letters. font_col/font_row: which
    // pixel of the underlying 8x8 bitmap (2x scale -> shift right by 1,
    // a constant shift, never a runtime divide).
    wire [7:0] local_x = x_cnt - text_x_start;
    wire [7:0] local_y = y_cnt - TEXT_Y_START;
    wire [2:0] char_col = local_x[6:4];
    wire [2:0] font_col = local_x[3:1];
    wire [2:0] font_row_idx = local_y[3:1];

    wire [2:0] glyph_id    = glyph_for_col(current_word, char_col);
    wire [7:0] row_bits    = font_row(glyph_id, font_row_idx);
    wire [7:0] shifted_row = row_bits << font_col;
    wire       pixel_on    = in_glyph_box && shifted_row[7];
    wire [15:0] pixel_color = pixel_on ? COLOR_WHITE : COLOR_BLACK;

    // ---- main state machine --------------------------------
    localparam S_RST_LOW    = 4'd0,
               S_RST_WAIT   = 4'd1,
               S_INIT       = 4'd2,
               S_INIT_WAIT  = 4'd3,
               S_DELAY      = 4'd4,
               S_DRAW       = 4'd5,
               S_DRAW_WAIT  = 4'd6,
               S_PIXEL      = 4'd7,
               S_PIXEL_WAIT = 4'd8,
               S_IDLE       = 4'd9;

    reg [3:0]  state     = S_RST_LOW;
    reg [3:0]  rom_addr  = 4'd0;
    reg [23:0] delay_cnt = 24'd0;   // only used for the two fixed reset waits
    reg [14:0] ms_div    = 15'd0;   // counts clocks inside one millisecond
    reg [7:0]  ms_cnt    = 8'd0;    // counts milliseconds
    reg [7:0]  delay_ms  = 8'd0;
    reg        hi_byte   = 1'b1;

    wire [9:0] init_word = rom_init_lookup(rom_addr);
    wire [9:0] draw_word = rom_draw_lookup(rom_addr);
    wire [9:0] rom_word  = (state == S_DRAW) ? draw_word : init_word;

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

        // walk through the init ROM (runs once)
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
                if (rom_addr == ROM_INIT_LAST) begin
                    rom_addr <= 4'd0;
                    state    <= S_DRAW;
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
                    ms_cnt <= 0;
                    if (rom_addr == ROM_INIT_LAST) begin
                        // the last init entry happens to be a delay (wait 100ms) -
                        // must check for end-of-table here too, not just in S_INIT_WAIT
                        rom_addr <= 4'd0;
                        state    <= S_DRAW;
                    end else begin
                        rom_addr <= rom_addr + 1'b1;
                        state    <= S_INIT;
                    end
                end else begin
                    ms_cnt <= ms_cnt + 1'b1;
                end
            end else begin
                ms_div <= ms_div + 1'b1;
            end
        end

        // walk through the draw ROM (CASET/RASET/RAMWR) - re-run every redraw
        S_DRAW: begin
            lcd_dc    <= rom_word[8];
            spi_data  <= rom_word[7:0];
            spi_start <= 1'b1;
            state     <= S_DRAW_WAIT;
        end

        S_DRAW_WAIT: begin
            if (!spi_busy) begin
                if (rom_addr == ROM_DRAW_LAST) begin
                    x_cnt   <= 8'd0;
                    y_cnt   <= 8'd0;
                    hi_byte <= 1'b1;
                    state   <= S_PIXEL;
                end else begin
                    rom_addr <= rom_addr + 1'b1;
                    state    <= S_DRAW;
                end
            end
        end

        // stream WIDTH*HEIGHT pixels, 2 bytes each, colour picked live
        S_PIXEL: begin
            lcd_dc    <= 1'b1;                  // pixel data
            spi_data  <= hi_byte ? pixel_color[15:8] : pixel_color[7:0];
            spi_start <= 1'b1;
            state     <= S_PIXEL_WAIT;
        end

        S_PIXEL_WAIT: begin
            if (!spi_busy) begin
                if (hi_byte) begin
                    hi_byte <= 1'b0;
                    state   <= S_PIXEL;
                end else begin
                    hi_byte <= 1'b1;
                    if (x_cnt == WIDTH - 1) begin
                        x_cnt <= 8'd0;
                        if (y_cnt == HEIGHT - 1) begin
                            state <= S_IDLE;
                        end else begin
                            y_cnt <= y_cnt + 1'b1;
                            state <= S_PIXEL;
                        end
                    end else begin
                        x_cnt <= x_cnt + 1'b1;
                        state <= S_PIXEL;
                    end
                end
            end
        end

        // watch the buttons; jump back to the draw section on a new press
        S_IDLE: begin
            if (b1_edge) begin
                current_word <= WORD_LEFT;
                rom_addr     <= 4'd0;
                state        <= S_DRAW;
            end else if (b2_edge) begin
                current_word <= WORD_RIGHT;
                rom_addr     <= 4'd0;
                state        <= S_DRAW;
            end
        end

        default: state <= S_RST_LOW;

        endcase
    end

    // LEDs show the low 6 bits of the last byte received over UART (active low)
    assign led = ~rx_byte_latch[5:0];

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


// ============================================================
//  ~10 ms debounce - same pattern as the earlier button test
// ============================================================
module debounce (
    input  wire clk,
    input  wire noisy,
    output reg  clean = 1'b0
);
    reg [18:0] count = 19'd0;      // ~10 ms at 27 MHz (270_000 needs 19 bits, not 18)
    reg        last  = 1'b0;

    always @(posedge clk) begin
        if (noisy != last) begin
            last  <= noisy;
            count <= 19'd0;
        end else if (count == 19'd270_000) begin
            clean <= last;
        end else begin
            count <= count + 1'b1;
        end
    end
endmodule
