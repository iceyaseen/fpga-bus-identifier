// ============================================================
//  i2c_scanner.v - sweeps the 7-bit I2C address space (0x08-0x77 by
//  default - the valid range; 0x00-0x07 and 0x78-0x7F are reserved)
//  using i2c_master, and reports every address that ACKs.
//
//  ADDR_FIRST/ADDR_LAST are real parameters (not localparams)
//  specifically so a testbench can override them to a much narrower
//  range for a fast, still fully representative simulation - the
//  scanning MECHANISM is what's being verified, not the specific
//  address value, and sweeping the full 112-address range in
//  simulation would just make every test run slow for no benefit.
//  Real hardware builds get the full default range automatically,
//  since nothing overrides it there.
//
//  hit_pending/hit_ack follow the same single-slot handshake as
//  top.v's rec_pending/rec_tx_done (see top.v's own comment on that,
//  and i2c_prescan.v's matching note): this module is the SOLE driver
//  of hit_pending/hit_address, top.v only ever computes hit_ack
//  combinationally from its own TX arbiter state.
// ============================================================
module i2c_scanner #(
    parameter [7:0] ADDR_FIRST = 8'h08,
    parameter [7:0] ADDR_LAST  = 8'h77
) (
    input  wire clk,

    input  wire start,       // one-cycle pulse
    output reg  busy,

    output reg         master_start,
    output reg  [7:0]  master_tx_byte,
    input  wire         master_done,
    input  wire         master_ack,

    output reg        hit_pending,
    output reg [7:0]  hit_address,
    input  wire        hit_ack
);

    // ~2.2ms at 27MHz. Originally 50us ("let a device settle"), but
    // that's far too short for a second reason that matters more when
    // capture is running (Part 6): one transaction generates ~27 edge
    // records (START + 9 bits x 2 SCL edges + ~7 SDA edges + STOP) in
    // ~110us, but the 921600-baud UART can only drain a 7-byte
    // EDGE_RECORD frame every ~76us - draining one transaction's worth
    // needs ~2050us, not 50us. Confirmed on real hardware: with the old
    // 50us delay, a full 112-address sweep overflowed the 256-entry
    // FIFO (net backlog growth ~25 records/address, every address),
    // dropping most of the scan's own edges and leaving the captured
    // waveform full of gaps - which then fed a wrong dominant pulse
    // width into the protocol hint (a large gap-inflated number instead
    // of the true ~5us I2C clock). This delay is sized so the FIFO
    // actually drains back down between addresses, so it never
    // accumulates a growing backlog no matter how many addresses are
    // swept. A full 112-address scan now takes ~112 * (2050 + txn
    // time) =~ 240ms - still effectively instant for a one-time scan.
    localparam [15:0] INTER_ADDR_DELAY = 16'd60000;

    localparam [2:0] S_IDLE       = 3'd0,
                      S_START_TXN  = 3'd1,
                      S_WAIT_DONE  = 3'd2,
                      S_REPORT_HIT = 3'd3,
                      S_DELAY      = 3'd4,
                      S_NEXT       = 3'd5;

    reg [2:0]  state = S_IDLE;
    reg [7:0]  addr  = ADDR_FIRST;
    reg [15:0] cnt   = 16'd0;

    initial begin
        busy           = 1'b0;
        master_start   = 1'b0;
        master_tx_byte = 8'h00;
        hit_pending    = 1'b0;
        hit_address    = 8'h00;
    end

    always @(posedge clk) begin
        master_start <= 1'b0;

        case (state)
        S_IDLE: begin
            if (start) begin
                busy  <= 1'b1;
                addr  <= ADDR_FIRST;
                state <= S_START_TXN;
            end
        end

        S_START_TXN: begin
            master_tx_byte <= {addr[6:0], 1'b0};   // 7-bit address + write bit
            master_start   <= 1'b1;
            state          <= S_WAIT_DONE;
        end

        S_WAIT_DONE: begin
            if (master_done) begin
                if (master_ack) begin
                    hit_address <= addr;
                    hit_pending <= 1'b1;
                    state       <= S_REPORT_HIT;
                end else begin
                    cnt   <= 16'd0;
                    state <= S_DELAY;
                end
            end
        end

        S_REPORT_HIT: begin
            if (hit_ack) begin
                hit_pending <= 1'b0;
                cnt         <= 16'd0;
                state       <= S_DELAY;
            end
        end

        S_DELAY: begin
            if (cnt == INTER_ADDR_DELAY - 1'b1) begin
                state <= S_NEXT;
            end else begin
                cnt <= cnt + 1'b1;
            end
        end

        S_NEXT: begin
            if (addr == ADDR_LAST) begin
                busy  <= 1'b0;
                state <= S_IDLE;
            end else begin
                addr  <= addr + 1'b1;
                state <= S_START_TXN;
            end
        end

        default: state <= S_IDLE;
        endcase
    end

endmodule
