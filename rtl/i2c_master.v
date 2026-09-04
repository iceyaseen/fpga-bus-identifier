// ============================================================
//  i2c_master.v - I2C master, one full write transaction per
//  operation: START, 8 data bits MSB-first, ACK/NACK sample on the
//  9th clock, STOP. That is exactly what a device-presence scan
//  needs (see i2c_scanner.v) - a general multi-byte streaming master
//  can come later, once this project actually reads data back from a
//  found device instead of just checking who answers.
//
//  100 kHz default at 27 MHz: HALF_PERIOD_CYCLES=135 gives an exact
//  270-cycle (10.0us) SCL period. Every phase (bit setup/hold, START/
//  STOP setup/hold) reuses this same half-period as its timing, which
//  clears every Standard-mode (100kHz) I2C spec minimum (4.0-4.7us)
//  with margin, since 135 cycles at 27MHz = 5.0us.
//
//  Clock stretching: after releasing SCL, the FSM does not advance on
//  a fixed schedule - it polls scl_sync every cycle in *_SCL_WAIT and
//  only starts counting the high-period hold once scl_sync actually
//  reads 1, however long that takes.
//
//  sda_drive_low/scl_drive_low: 1 = actively drive that line low, 0 =
//  release (Hi-Z, let the pull-up bring it high) - matches the
//  tristate-only contract of probe_driver, never asserts "drive high".
// ============================================================
module i2c_master #(
    parameter [15:0] HALF_PERIOD_CYCLES = 16'd135   // 27_000_000 / 100_000 / 2
) (
    input  wire       clk,

    input  wire        start,        // one-cycle pulse: begin a transaction
    input  wire [7:0]  tx_byte,      // address<<1 | rw (or any byte, for later reuse)
    output wire         busy,
    output reg          done,         // one-cycle pulse when the transaction finishes
    output reg          ack_received, // valid the same cycle as done

    output reg         scl_drive_low,
    output reg         sda_drive_low,
    input  wire        scl_sync,
    input  wire        sda_sync
);

    localparam [3:0] S_IDLE         = 4'd0,
                      S_START        = 4'd1,
                      S_BIT_LOW      = 4'd2,
                      S_BIT_SCL_WAIT = 4'd3,
                      S_BIT_HIGH     = 4'd4,
                      S_STOP_SDA_LOW = 4'd5,
                      S_STOP_SCL_WAIT= 4'd6,
                      S_STOP_HOLD    = 4'd7,
                      S_STOP_FINAL   = 4'd8;

    reg [3:0]  state        = S_IDLE;
    reg [15:0] phase_cnt    = 16'd0;
    reg [3:0]  bit_idx      = 4'd0;     // 0..7 data bits, 8 = ACK phase
    reg [7:0]  tx_shift     = 8'h00;
    reg        ack_captured = 1'b0;

    // busy must go high in the SAME cycle as start, same fix as
    // uart_tx.v's busy - otherwise a caller checking busy right after
    // issuing start looks one cycle too early and thinks it's idle
    reg  busy_r = 1'b0;
    assign busy = busy_r | start;

    // the bit currently on SDA during a data phase (MSB first): the
    // shift register's top bit, shifted left after each bit
    wire cur_bit = tx_shift[7];

    initial begin
        done          = 1'b0;
        ack_received  = 1'b0;
        scl_drive_low = 1'b0;
        sda_drive_low = 1'b0;
    end

    always @(posedge clk) begin
        done <= 1'b0;

        case (state)
        S_IDLE: begin
            if (start) begin
                tx_shift      <= tx_byte;
                bit_idx       <= 4'd0;
                busy_r        <= 1'b1;
                sda_drive_low <= 1'b1;   // SDA falls...
                scl_drive_low <= 1'b0;   // ...while SCL is still released/high: START condition
                phase_cnt     <= 16'd0;
                state         <= S_START;
            end
        end

        // START condition asserted; hold, then begin clocking by
        // driving SCL low
        S_START: begin
            if (phase_cnt == HALF_PERIOD_CYCLES - 1'b1) begin
                phase_cnt     <= 16'd0;
                scl_drive_low <= 1'b1;
                state         <= S_BIT_LOW;
            end else begin
                phase_cnt <= phase_cnt + 1'b1;
            end
        end

        // SCL held low: set up SDA for this bit (or release it during
        // the ACK phase, bit_idx==8), then hold for the setup time
        S_BIT_LOW: begin
            sda_drive_low <= (bit_idx == 4'd8) ? 1'b0 : ~cur_bit;
            if (phase_cnt == HALF_PERIOD_CYCLES - 1'b1) begin
                phase_cnt     <= 16'd0;
                scl_drive_low <= 1'b0;   // release SCL - let it rise
                state         <= S_BIT_SCL_WAIT;
            end else begin
                phase_cnt <= phase_cnt + 1'b1;
            end
        end

        // clock stretching: wait however long the device needs,
        // not a fixed schedule
        S_BIT_SCL_WAIT: begin
            if (scl_sync) begin
                phase_cnt <= 16'd0;
                state     <= S_BIT_HIGH;
            end
        end

        // SCL confirmed high: hold for the high period. Sample SDA
        // for ACK/NACK on the last cycle of the ACK phase's high hold.
        S_BIT_HIGH: begin
            if (phase_cnt == HALF_PERIOD_CYCLES - 1'b1) begin
                phase_cnt     <= 16'd0;
                scl_drive_low <= 1'b1;
                if (bit_idx == 4'd8) begin
                    ack_captured  <= ~sda_sync;   // low = ACK
                    sda_drive_low <= 1'b1;        // begin STOP setup: drive SDA low with SCL low
                    state         <= S_STOP_SDA_LOW;
                end else begin
                    tx_shift <= {tx_shift[6:0], 1'b0};
                    bit_idx  <= bit_idx + 1'b1;
                    state    <= S_BIT_LOW;
                end
            end else begin
                phase_cnt <= phase_cnt + 1'b1;
            end
        end

        S_STOP_SDA_LOW: begin
            if (phase_cnt == HALF_PERIOD_CYCLES - 1'b1) begin
                phase_cnt     <= 16'd0;
                scl_drive_low <= 1'b0;   // release SCL
                state         <= S_STOP_SCL_WAIT;
            end else begin
                phase_cnt <= phase_cnt + 1'b1;
            end
        end

        S_STOP_SCL_WAIT: begin
            if (scl_sync) begin
                phase_cnt <= 16'd0;
                state     <= S_STOP_HOLD;
            end
        end

        S_STOP_HOLD: begin
            if (phase_cnt == HALF_PERIOD_CYCLES - 1'b1) begin
                phase_cnt     <= 16'd0;
                sda_drive_low <= 1'b0;   // release SDA while SCL is high: STOP condition
                state         <= S_STOP_FINAL;
            end else begin
                phase_cnt <= phase_cnt + 1'b1;
            end
        end

        S_STOP_FINAL: begin
            if (phase_cnt == HALF_PERIOD_CYCLES - 1'b1) begin
                busy_r       <= 1'b0;
                done         <= 1'b1;
                ack_received <= ack_captured;
                state        <= S_IDLE;
            end else begin
                phase_cnt <= phase_cnt + 1'b1;
            end
        end

        default: state <= S_IDLE;
        endcase
    end

endmodule
