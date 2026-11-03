module quadrature_counter #(
    // Both encoder inputs are asynchronous and pass through independent
    // synchronizers.  Requiring the combined AB state to stay unchanged for
    // a few clocks rejects contact/driver spikes before the Gray-code decode.
    parameter integer STABLE_CYCLES = 4,
    // Do not stop the machine for one rejected transition.  A real wiring or
    // signal-integrity fault must produce this many bad transitions before
    // illegal_seen is asserted.
    parameter integer ILLEGAL_LIMIT = 8,
    parameter integer ILLEGAL_DECAY_CYCLES = 500000
) (
    input wire clk,
    input wire rst_n,
    input wire phase_a,
    input wire phase_b,
    output reg signed [31:0] position,
    output reg illegal_seen
);
    reg [2:0] a_sync;
    reg [2:0] b_sync;
    reg [1:0] previous;
    reg [1:0] candidate;
    reg [7:0] stable_count;
    reg [7:0] illegal_score;
    reg [31:0] illegal_decay_count;
    reg [2:0] startup_count;
    reg initialized;
    wire [1:0] current = {a_sync[2], b_sync[2]};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_sync <= 0;
            b_sync <= 0;
            previous <= 0;
            candidate <= 0;
            stable_count <= 0;
            illegal_score <= 0;
            illegal_decay_count <= 0;
            startup_count <= 0;
            initialized <= 1'b0;
            position <= 0;
            illegal_seen <= 1'b0;
        end else begin
            a_sync <= {a_sync[1:0], phase_a};
            b_sync <= {b_sync[1:0], phase_b};
            if (illegal_decay_count == ILLEGAL_DECAY_CYCLES - 1) begin
                illegal_decay_count <= 0;
                if (illegal_score != 0)
                    illegal_score <= illegal_score - 1'b1;
            end else begin
                illegal_decay_count <= illegal_decay_count + 1'b1;
            end

            if (current != candidate) begin
                candidate <= current;
                stable_count <= 0;
            end else if (stable_count < STABLE_CYCLES) begin
                stable_count <= stable_count + 1'b1;
            end

            if (!initialized) begin
                // Establish a zero reference at the actual idle AB state.
                // Reset may be released with either input high.
                if (startup_count == 7 && stable_count >= STABLE_CYCLES) begin
                    previous <= candidate;
                    initialized <= 1'b1;
                end else begin
                    startup_count <= startup_count + 1'b1;
                end
            end else if (stable_count == STABLE_CYCLES && candidate != previous) begin
                // Same sign convention as the proven Gowin example:
                // 00 -> 10 -> 11 -> 01 -> 00 is positive.
                case ({previous, candidate})
                    4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00:
                        position <= position + 1'b1;
                    4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00:
                        position <= position - 1'b1;
                    default: begin
                        if (illegal_score >= ILLEGAL_LIMIT - 1)
                            illegal_seen <= 1'b1;
                        else
                            illegal_score <= illegal_score + 1'b1;
                    end
                endcase
                previous <= candidate;
                // Consume this stable transition once.  Another update is
                // accepted only after a newly stable candidate is observed.
                stable_count <= STABLE_CYCLES + 1;
            end
        end
    end
endmodule

// Backlash (gear lash) estimator on the raw encoder count.
// The motor encoder must travel through a dead gap before the output shaft
// follows: BL_POS counts after reversing to increasing, BL_NEG counts after
// reversing to decreasing.  gap tracks where the motor sits inside that dead
// zone; shaft_position only moves once gap is pinned at an edge.
// Set both widths to 0 to bypass.
module backlash_comp #(
    parameter integer BL_POS = 8,
    parameter integer BL_NEG = 2
) (
    input wire clk,
    input wire rst_n,
    input wire signed [31:0] encoder_position,
    output reg signed [31:0] shaft_position
);
    reg signed [31:0] previous;
    reg signed [15:0] gap;
    wire signed [31:0] delta = encoder_position - previous;
    wire signed [31:0] gap_next = gap + delta;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            previous <= 0;
            gap <= 0;
            shaft_position <= 0;
        end else begin
            previous <= encoder_position;
            if (gap_next > BL_POS) begin
                shaft_position <= shaft_position + (gap_next - BL_POS);
                gap <= BL_POS;
            end else if (gap_next < -BL_NEG) begin
                shaft_position <= shaft_position + (gap_next + BL_NEG);
                gap <= -BL_NEG;
            end else begin
                gap <= gap_next[15:0];
            end
        end
    end
endmodule
