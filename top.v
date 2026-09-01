// ============================================================
//  A3 - ST7735S 80x160 screen: show "LEFT" / "RIGHT" on button press
//  Board: Sipeed Tang Nano 9K  (27 MHz clock)
//
//  No framebuffer: each pixel's colour is decided while it is being
//  streamed out over SPI, using two counters (x_cnt, y_cnt) that
//  just increment - never computed as index % WIDTH / index / WIDTH.
// ============================================================

module top #(
    parameter CYCLES_PER_MS = 27000,    // override in simulation to speed up delays
    parameter NUM_CHANNELS  = 2,        // probe channel count - bump this (and the .cst) for more
    parameter FIFO_DEPTH    = 128       // must be a power of 2 - see fifo.v
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

    output wire [5:0] led,

    inout  wire [NUM_CHANNELS-1:0] probe   // logic-analyser probe pins: pins 27/28, see .cst
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

    // ---- UART: framed protocol only - no housekeeping text, no byte
    //      echo. Commands get a real ACK/ERROR/status reply instead ----
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

    // ---- edge capture engine -------------------------------
    // probe is inout so active probing can be added later; nothing
    // drives it in this version.
    //
    // This pin's read side used to be `wire probe_in = probe;`, reading
    // straight off the inout port. That is legal Verilog, and simulates
    // fine (edge_capture_tb.v passed), but real synthesis broke it: with
    // synth_gowin, since probe_drive_en never varies (nothing drives
    // it), Yosys constant-folded probe's own drive expression at
    // compile time and used THAT as the value of probe_in too - i.e. it
    // treated "what we're (not) driving" as "what the pin reads",
    // instead of wiring the read back to the IOBUF's real O pin. Traced
    // in the synthesized netlist: probe_in ended up tied to a literal
    // constant 'z', completely disconnected from the actual pad voltage
    // - so no amount of external toggling on pins 27/28 could ever have
    // registered, with no error or warning pointing at it directly.
    //
    // Fix: instantiate Gowin's IOBUF primitive by name for real builds.
    // That primitive's O pin is a genuine hardware read-back path, not
    // something Yosys can fold away. `SYNTHESIS` is defined
    // automatically by `read_verilog` (confirmed: `yosys -h read_verilog`
    // says so, and the actual build command this project's toolchain
    // runs never passes -nosynthesis), so this reliably picks the real
    // primitive for every real build and the simple behavioral version
    // for iverilog simulation, which doesn't know about Gowin's IOBUF.
    reg                      probe_drive_en  = 1'b0;
    reg  [NUM_CHANNELS-1:0]  probe_drive_val = {NUM_CHANNELS{1'b0}};
    wire [NUM_CHANNELS-1:0] probe_in;

`ifdef SYNTHESIS
    genvar probe_i;
    generate
        for (probe_i = 0; probe_i < NUM_CHANNELS; probe_i = probe_i + 1) begin : probe_iobuf
            IOBUF u_probe_iobuf (
                .O   (probe_in[probe_i]),
                .IO  (probe[probe_i]),
                .I   (probe_drive_val[probe_i]),
                .OEN (~probe_drive_en)   // OEN=1 -> tri-stated, per Gowin's IOBUF model
            );
        end
    endgenerate
`else
    assign probe    = probe_drive_en ? probe_drive_val : {NUM_CHANNELS{1'bz}};
    assign probe_in = probe;
`endif

    // 'S'/'X'/'R'/'V' commands from the host. Any other byte is
    // unrecognised and gets a real ERROR reply, not a silent drop or
    // an echo of the byte back.
    reg capture_en = 1'b0;
    reg ts_reset   = 1'b0;   // one-cycle pulse: zeroes the timestamp counter + clears FIFO overflow
    reg status_req = 1'b0;   // one-cycle pulse: latch+queue a 'V' status reply
    reg ack_req    = 1'b0;   // one-cycle pulse: latch+queue an ACK for S/X/R
    reg [7:0] ack_cmd = 8'h00;
    reg err_req       = 1'b0;   // one-cycle pulse: latch+queue an ERROR reply
    reg [7:0] err_bad_byte = 8'h00;

    always @(posedge clk) begin
        ts_reset   <= 1'b0;
        status_req <= 1'b0;
        ack_req    <= 1'b0;
        err_req    <= 1'b0;
        if (uart_rx_valid) begin
            case (uart_rx_data)
                8'h53: begin capture_en <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end   // 'S'
                8'h58: begin capture_en <= 1'b0; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end   // 'X'
                8'h52: begin ts_reset   <= 1'b1; ack_req <= 1'b1; ack_cmd <= uart_rx_data; end   // 'R'
                8'h56: status_req <= 1'b1;                                                       // 'V'
                default: begin
                    err_req      <= 1'b1;
                    err_bad_byte <= uart_rx_data;
                end
            endcase
        end
    end

    // TX arbiter state names + the regs it owns, declared up here (ahead
    // of the popper below, which needs TX_REC_WAIT/tx_state/rec_byte_idx
    // for rec_tx_done) so every reference in this file is to an
    // already-declared symbol - Icarus's default elaboration mode
    // rejects forward references even though plain Verilog allows them,
    // and depending on forward references made the multi-driver bug
    // below easy to miss. The arbiter's actual always block (the logic
    // that drives these regs) still lives further down, where it reads
    // naturally alongside the states it walks through.
    //
    // 9 states now (IDLE + 2 each for REC/STAT/ACK/ERR) need 4 bits.
    localparam TX_IDLE      = 4'd0,
               TX_REC       = 4'd1,
               TX_REC_WAIT  = 4'd2,
               TX_STAT      = 4'd3,
               TX_STAT_WAIT = 4'd4,
               TX_ACK       = 4'd5,
               TX_ACK_WAIT  = 4'd6,
               TX_ERR       = 4'd7,
               TX_ERR_WAIT  = 4'd8;

    reg [3:0] tx_state      = TX_IDLE;
    reg [2:0] rec_byte_idx  = 3'd0;   // EDGE_RECORD is 7 bytes: idx 0..6
    reg [2:0] stat_byte_idx = 3'd0;   // STATUS_REPLY is 7 bytes: idx 0..6
    reg [1:0] ack_byte_idx  = 2'd0;   // ACK is 3 bytes: idx 0..2
    reg [1:0] err_byte_idx  = 2'd0;   // ERROR is 4 bytes: idx 0..3

    wire [NUM_CHANNELS-1:0] probe_sync;
    sync #(.WIDTH(NUM_CHANNELS)) probe_sync_unit (
        .clk      (clk),
        .async_in (probe_in),
        .sync_out (probe_sync)
    );

    wire        rec_valid;
    wire [31:0] rec_ts;
    wire [7:0]  rec_st;

    edge_capture #(.NUM_CHANNELS(NUM_CHANNELS)) capture_unit (
        .clk           (clk),
        .capture_en    (capture_en),
        .ts_reset      (ts_reset),
        .chan_sync     (probe_sync),
        .rec_timestamp (rec_ts),
        .rec_state     (rec_st),
        .rec_valid     (rec_valid)
    );

    wire        fifo_full;
    wire        fifo_empty;
    wire        fifo_overflow;
    // width tracks FIFO_DEPTH automatically (fifo.v's own PTR_W does the
    // same $clog2 derivation) so this can't drift out of sync either
    wire [$clog2(FIFO_DEPTH):0] fifo_high_water;
    wire [39:0] fifo_rd_data;
    wire        fifo_rd_en;

    fifo_ff #(.DEPTH(FIFO_DEPTH), .WIDTH(40)) record_fifo (
        .clk        (clk),
        .ovf_clear  (ts_reset),
        .wr_en      (rec_valid),
        .wr_data    ({rec_ts, rec_st}),
        .full       (fifo_full),
        .rd_en      (fifo_rd_en),
        .rd_data    (fifo_rd_data),
        .empty      (fifo_empty),
        .overflow   (fifo_overflow),
        .high_water (fifo_high_water)
    );

    // ---- record popper: pulls one record out of the FIFO, latches
    //      it (rd_data is only valid one cycle after rd_en), then
    //      hands it to the TX arbiter below as rec_pending ----
    localparam POP_IDLE = 2'd0,
               POP_LATCH= 2'd1,
               POP_HOLD = 2'd2;

    reg [1:0]  pop_state   = POP_IDLE;
    reg [39:0] rec_latch   = 40'd0;
    reg        rec_pending = 1'b0;

    // combinational, not registered: if this were a reg (rd_en <= 1 in
    // POP_IDLE), the FIFO wouldn't see rd_en=1 until the cycle after
    // POP_IDLE, and POP_LATCH would then read rd_data one cycle too
    // early - a stale read that silently corrupts every record
    assign fifo_rd_en = (pop_state == POP_IDLE) && !fifo_empty && !rec_pending;

    // combinational flag: "the TX arbiter just sent this record's last
    // byte". rec_pending must have exactly ONE always-block driver: this
    // used to be written from here AND from the TX arbiter, which Yosys
    // correctly rejected as a multi-driver conflict and silently
    // resolved by tying it to a constant - so no status/record ever
    // actually got sent on real hardware even though it simulated fine
    // in Icarus. This wire lets the popper stay the sole driver while
    // still reacting to the arbiter finishing.
    wire rec_tx_done = (tx_state == TX_REC_WAIT) && !uart_tx_busy && (rec_byte_idx == 3'd6);

    always @(posedge clk) begin
        case (pop_state)
            POP_IDLE: begin
                if (fifo_rd_en) pop_state <= POP_LATCH;
            end
            POP_LATCH: begin
                rec_latch   <= fifo_rd_data;
                rec_pending <= 1'b1;
                pop_state   <= POP_HOLD;
            end
            POP_HOLD: begin
                if (rec_tx_done) begin
                    rec_pending <= 1'b0;
                    pop_state   <= POP_IDLE;
                end
            end
            default: pop_state <= POP_IDLE;
        endcase
    end

    // ---- reply state for status/ack/error. The actual set/clear of
    // each *_pending lives in the TX arbiter's always block below
    // (set at its tail, cleared in that message's own *_WAIT state),
    // never in a separate always block - see the big comment up at the
    // popper for why a second driver on the same reg is dangerous, not
    // just untidy: it's the exact bug that silently killed the status
    // reply the first time this project had one. ----
    reg        stat_pending    = 1'b0;
    reg        stat_overflow   = 1'b0;
    reg [15:0] stat_high_water = 16'd0;

    reg        ack_pending   = 1'b0;
    reg [7:0]  ack_cmd_latch = 8'h00;

    reg        err_pending   = 1'b0;
    reg [7:0]  err_bad_latch = 8'h00;

    // depth never changes at runtime, so no need to latch it - just
    // feed FIFO_DEPTH straight into the status byte lookup below
    localparam [15:0] STATUS_DEPTH = FIFO_DEPTH;

    // ---- framed protocol: [MARKER][PAYLOAD...][CHECKSUM]. The marker
    // byte alone tells the host both "this is a frame start" and,
    // since payload length is fixed per marker, exactly how many more
    // bytes to expect - no separate length byte needed. CHECKSUM is
    // the 8-bit sum (mod 256, via plain truncation on assignment to an
    // 8-bit function return - no multiply/divide anywhere) of the
    // marker plus every payload byte. If a host's checksum check fails
    // - false marker match, or real corruption - it should resync by
    // advancing exactly one byte and rescanning, not by skipping this
    // frame's whole claimed length. ----
    localparam [7:0] MARKER_EDGE = 8'hA5;
    localparam [7:0] MARKER_STAT = 8'hA6;
    localparam [7:0] MARKER_ACK  = 8'hA7;
    localparam [7:0] MARKER_ERR  = 8'hA8;

    localparam [7:0] ERR_UNKNOWN_CMD = 8'h01;

    // EDGE_RECORD - 7 bytes: marker, ts[31:24..7:0], state, checksum
    function [7:0] rec_checksum(input [39:0] rec);
        reg [7:0] sum;
        begin
            sum = MARKER_EDGE + rec[39:32] + rec[31:24] + rec[23:16] + rec[15:8] + rec[7:0];
            rec_checksum = sum;
        end
    endfunction

    function [7:0] rec_byte_lookup(input [39:0] rec, input [2:0] idx);
        case (idx)
            3'd0: rec_byte_lookup = MARKER_EDGE;
            3'd1: rec_byte_lookup = rec[39:32];    // timestamp[31:24]
            3'd2: rec_byte_lookup = rec[31:24];    // timestamp[23:16]
            3'd3: rec_byte_lookup = rec[23:16];    // timestamp[15:8]
            3'd4: rec_byte_lookup = rec[15:8];     // timestamp[7:0]
            3'd5: rec_byte_lookup = rec[7:0];      // channel state
            3'd6: rec_byte_lookup = rec_checksum(rec);
            default: rec_byte_lookup = 8'h00;
        endcase
    endfunction

    // STATUS_REPLY - 7 bytes: marker, overflow, high_water(2B), depth(2B), checksum
    function [7:0] stat_checksum(input ovf, input [15:0] hw, input [15:0] depth);
        reg [7:0] sum;
        begin
            sum = MARKER_STAT + {7'd0, ovf} + hw[15:8] + hw[7:0] + depth[15:8] + depth[7:0];
            stat_checksum = sum;
        end
    endfunction

    function [7:0] stat_byte_lookup(input ovf, input [15:0] hw, input [15:0] depth, input [2:0] idx);
        case (idx)
            3'd0: stat_byte_lookup = MARKER_STAT;
            3'd1: stat_byte_lookup = {7'd0, ovf};
            3'd2: stat_byte_lookup = hw[15:8];
            3'd3: stat_byte_lookup = hw[7:0];
            3'd4: stat_byte_lookup = depth[15:8];
            3'd5: stat_byte_lookup = depth[7:0];
            3'd6: stat_byte_lookup = stat_checksum(ovf, hw, depth);
            default: stat_byte_lookup = 8'h00;
        endcase
    endfunction

    // ACK - 3 bytes: marker, command byte, checksum
    function [7:0] ack_byte_lookup(input [7:0] cmd, input [1:0] idx);
        case (idx)
            2'd0: ack_byte_lookup = MARKER_ACK;
            2'd1: ack_byte_lookup = cmd;
            2'd2: ack_byte_lookup = MARKER_ACK + cmd;   // checksum
            default: ack_byte_lookup = 8'h00;
        endcase
    endfunction

    // ERROR - 4 bytes: marker, error code, offending byte, checksum
    function [7:0] err_byte_lookup(input [7:0] code, input [7:0] bad, input [1:0] idx);
        case (idx)
            2'd0: err_byte_lookup = MARKER_ERR;
            2'd1: err_byte_lookup = code;
            2'd2: err_byte_lookup = bad;
            2'd3: err_byte_lookup = MARKER_ERR + code + bad;   // checksum
            default: err_byte_lookup = 8'h00;
        endcase
    endfunction

    // single arbiter owns the one physical TX line. Priority, highest
    // first: capture records (losing these would silently corrupt a
    // capture), error replies (you want to know immediately if the
    // host sent something unrecognised), command acks, then status
    // replies (already sent only on request, fine to wait a beat).
    always @(posedge clk) begin
        uart_tx_start <= 1'b0;

        case (tx_state)
        TX_IDLE: begin
            if (rec_pending) begin
                rec_byte_idx <= 3'd0;
                tx_state     <= TX_REC;
            end else if (err_pending) begin
                err_byte_idx <= 2'd0;
                tx_state     <= TX_ERR;
            end else if (ack_pending) begin
                ack_byte_idx <= 2'd0;
                tx_state     <= TX_ACK;
            end else if (stat_pending) begin
                stat_byte_idx <= 3'd0;
                tx_state      <= TX_STAT;
            end
        end

        TX_REC: begin
            uart_tx_data  <= rec_byte_lookup(rec_latch, rec_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_REC_WAIT;
        end

        TX_REC_WAIT: begin
            if (!uart_tx_busy) begin
                if (rec_byte_idx == 3'd6) begin
                    // rec_pending is cleared by the popper's own always
                    // block (it watches rec_tx_done, computed from
                    // tx_state/rec_byte_idx right here) - see the big
                    // comment up at the popper for why this can't also
                    // write rec_pending directly
                    tx_state <= TX_IDLE;
                end else begin
                    rec_byte_idx <= rec_byte_idx + 1'b1;
                    tx_state     <= TX_REC;
                end
            end
        end

        TX_STAT: begin
            uart_tx_data  <= stat_byte_lookup(stat_overflow, stat_high_water, STATUS_DEPTH, stat_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_STAT_WAIT;
        end

        TX_STAT_WAIT: begin
            if (!uart_tx_busy) begin
                if (stat_byte_idx == 3'd6) begin
                    stat_pending <= 1'b0;
                    tx_state     <= TX_IDLE;
                end else begin
                    stat_byte_idx <= stat_byte_idx + 1'b1;
                    tx_state      <= TX_STAT;
                end
            end
        end

        TX_ACK: begin
            uart_tx_data  <= ack_byte_lookup(ack_cmd_latch, ack_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_ACK_WAIT;
        end

        TX_ACK_WAIT: begin
            if (!uart_tx_busy) begin
                if (ack_byte_idx == 2'd2) begin
                    ack_pending <= 1'b0;
                    tx_state    <= TX_IDLE;
                end else begin
                    ack_byte_idx <= ack_byte_idx + 1'b1;
                    tx_state     <= TX_ACK;
                end
            end
        end

        TX_ERR: begin
            uart_tx_data  <= err_byte_lookup(ERR_UNKNOWN_CMD, err_bad_latch, err_byte_idx);
            uart_tx_start <= 1'b1;
            tx_state      <= TX_ERR_WAIT;
        end

        TX_ERR_WAIT: begin
            if (!uart_tx_busy) begin
                if (err_byte_idx == 2'd3) begin
                    err_pending <= 1'b0;
                    tx_state    <= TX_IDLE;
                end else begin
                    err_byte_idx <= err_byte_idx + 1'b1;
                    tx_state     <= TX_ERR;
                end
            end
        end

        default: tx_state <= TX_IDLE;
        endcase

        // set logic for stat/ack/err pending lives here (not in
        // separate always blocks) so each has exactly one driver - see
        // the big comment up at the popper for why. ack/err additionally
        // guard on "not already pending": if a second S/X/R (or a
        // second bad byte) arrives before the first ack/error finished
        // sending, it still takes effect immediately (capture_en etc.
        // are set unconditionally in the command decode block above),
        // it just won't get its own separate ack/error frame - in
        // practice these are ~260us round-trips and commands don't
        // arrive faster than a human or script can send them.
        if (status_req) begin
            stat_overflow   <= fifo_overflow;
            // zero-pad fifo_high_water (width tracks FIFO_DEPTH, see its
            // declaration above) out to the fixed 16-bit wire field -
            // a hardcoded pad count here would silently truncate the
            // top bit if FIFO_DEPTH ever needed a wider pointer
            stat_high_water <= {{(15 - $clog2(FIFO_DEPTH)){1'b0}}, fifo_high_water};
            stat_pending    <= 1'b1;
        end
        if (ack_req && !ack_pending) begin
            ack_cmd_latch <= ack_cmd;
            ack_pending   <= 1'b1;
        end
        if (err_req && !err_pending) begin
            err_bad_latch <= err_bad_byte;
            err_pending   <= 1'b1;
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

    // LEDs (active low): led[0] lit while capturing, led[1] lit (sticky)
    // once the FIFO has overflowed, led[5:2] still show the low nibble
    // of the last byte received over UART, as before
    assign led[0]   = ~capture_en;
    assign led[1]   = ~fifo_overflow;
    assign led[5:2] = ~rx_byte_latch[3:0];

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
