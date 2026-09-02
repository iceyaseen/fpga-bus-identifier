// ============================================================
//  edge_capture.v
//
//  Free-running 32-bit timestamp counter + change detector for
//  NUM_CHANNELS already-synchronised probe signals (feed it the
//  output of sync.v, never the raw async pins).
//
//  Every clock, the synchronised channel bus is compared against
//  its value on the previous clock. Any difference produces one
//  record: {timestamp[31:0], state[7:0]}, where state holds ALL
//  channels (not just which one changed) - low NUM_CHANNELS bits
//  used, the rest held at zero. NUM_CHANNELS must stay <= 8 since
//  that's what fits in the 8-bit state field.
//
//  rec_valid/rec_timestamp/rec_state are all registered together
//  in the same always block, so a consumer sampling rec_valid on
//  a clock edge already sees matching, non-stale data that same
//  cycle - no extra pipeline delay to account for downstream.
// ============================================================
module edge_capture #(
    parameter NUM_CHANNELS = 2
) (
    input  wire                    clk,
    input  wire                    capture_en,
    input  wire                    ts_reset,      // one-cycle pulse: zero the timestamp counter

    input  wire [NUM_CHANNELS-1:0] chan_sync,     // already through sync.v

    output reg  [31:0]             rec_timestamp,
    output reg  [7:0]              rec_state,
    output reg                     rec_valid
);

    reg [31:0]              timestamp;
    reg [NUM_CHANNELS-1:0]  prev_state;

    initial begin
        timestamp     = 32'd0;
        prev_state    = {NUM_CHANNELS{1'b0}};
        rec_timestamp = 32'd0;
        rec_state     = 8'd0;
        rec_valid     = 1'b0;
    end

    wire changed = (chan_sync != prev_state);

    always @(posedge clk) begin
        // free-running: always counts, 'R' snaps it back to zero
        if (ts_reset)
            timestamp <= 32'd0;
        else
            timestamp <= timestamp + 1'b1;

        // tracked every cycle regardless of capture_en, so resuming
        // capture after a stop never compares against a stale value
        // and fires a spurious "changed" record on the first cycle
        prev_state <= chan_sync;

        rec_valid <= 1'b0;
        if (capture_en && changed) begin
            rec_timestamp <= timestamp;
            rec_state     <= {{(8-NUM_CHANNELS){1'b0}}, chan_sync};
            rec_valid     <= 1'b1;
        end
    end

endmodule
