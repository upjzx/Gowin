module adc_capture #(
    parameter integer SAMPLE_DIV = 10000,
    parameter integer AVERAGE_LOG2 = 4,
    parameter integer MEDIAN_ENABLE = 1
) (
    input wire clk,
    input wire rst_n,
    input wire [9:0] adc_data,
    output reg adc_clk,
    output wire adc_oe_n,
    output reg [9:0] angle_raw,
    output reg [10:0] angle_x2,
    output reg [17:0] angle_q8,
    output reg angle_valid,
    output reg angle_update
);
    localparam integer ACC_WIDTH = 10 + AVERAGE_LOG2;
    reg [31:0] sample_divider;
    localparam integer AVERAGE_POINTS = 1 << AVERAGE_LOG2;
    reg [AVERAGE_LOG2-1:0] sample_count;
    reg [AVERAGE_LOG2-1:0] write_index;
    reg [9:0] samples [0:AVERAGE_POINTS-1];
    integer i;
    reg [ACC_WIDTH-1:0] sample_sum, shifted_sum;
    reg [9:0] raw_current, raw_previous, raw_older, accepted_data;
    reg [1:0] raw_fill;
    reg median_pending, average_pending;
    wire [9:0] median_sample = (raw_current>=raw_previous) ?
        ((raw_previous>=raw_older) ? raw_previous : ((raw_current>=raw_older) ? raw_older : raw_current)) :
        ((raw_current>=raw_older) ? raw_current : ((raw_previous>=raw_older) ? raw_older : raw_previous));
    // Two fixed coordinate charts avoid the seam without integrating an
    // unwrap branch. A transient conversion cannot leave a permanent offset.
    wire [9:0] shifted_data = accepted_data ^ 10'd512;
    wire [9:0] shifted_old = samples[write_index] ^ 10'd512;
    wire [ACC_WIDTH-1:0] completed_raw_sum = sample_sum + accepted_data - samples[write_index];
    wire [ACC_WIDTH-1:0] completed_shifted_sum = shifted_sum + shifted_data - shifted_old;
    wire use_shifted = accepted_data<256 || accepted_data>=768;
    wire [ACC_WIDTH-1:0] completed_sum = use_shifted ? completed_shifted_sum : completed_raw_sum;
    wire [31:0] rounded_sum = completed_sum + (1 << (AVERAGE_LOG2 - 1));
    wire [31:0] half_rounded_sum = completed_sum +
        ((AVERAGE_LOG2 > 1) ? (1 << (AVERAGE_LOG2 - 2)) : 0);
    // Preserve the moving mean's fractional bits for control. Truncate to
    // the circular 10-bit ADC domain before changing coordinate charts.
    wire [31:0] precise_sum = (AVERAGE_LOG2 <= 8) ?
        (completed_sum << (8-AVERAGE_LOG2)) : (completed_sum >> (AVERAGE_LOG2-8));

    assign adc_oe_n = 1'b0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_clk <= 1'b0;
            sample_divider <= 0;
            sample_count <= 0;
            write_index <= 0;
            for (i = 0; i < AVERAGE_POINTS; i = i + 1) samples[i] <= 0;
            sample_sum <= 0;
            shifted_sum <= {10'd512,{AVERAGE_LOG2{1'b0}}};
            raw_current <= 0; raw_previous <= 0; raw_older <= 0; accepted_data <= 0;
            raw_fill <= 0; median_pending <= 0; average_pending <= 0;
            angle_raw <= 0;
            angle_x2 <= 0;
            angle_q8 <= 0;
            angle_valid <= 1'b0;
            angle_update <= 1'b0;
        end else begin
            angle_update <= 1'b0;
            median_pending <= 0;
            average_pending <= median_pending;
            if(median_pending) accepted_data <= MEDIAN_ENABLE!=0 ? median_sample : raw_current;
            // 50 MHz / 2 = 25 MHz, matching the board-proven example.
            adc_clk <= ~adc_clk;
            if (sample_divider == SAMPLE_DIV - 1)
                sample_divider <= 0;
            else
                sample_divider <= sample_divider + 1'b1;

            // SAMPLE_DIV is even, so this captures just before the next
            // ADC rising edge, after the previous conversion has settled.
            if (sample_divider == 0 && !adc_clk) begin
                // Register the external bus before comparators and adders.
                // No combinational median/sum path starts at live ADC pins.
                raw_current <= adc_data; raw_previous <= raw_current; raw_older <= raw_previous;
                if(raw_fill<2) raw_fill <= raw_fill+1'b1;
                else median_pending <= 1;
            end
            if(average_pending) begin
                samples[write_index] <= accepted_data;
                write_index <= write_index + 1'b1;
                sample_sum <= completed_raw_sum;
                shifted_sum <= completed_shifted_sum;
                // Sliding average: after warm-up, update at every sample instead
                // of waiting for a complete new block. Round to nearest ADC count.
                if (angle_valid || sample_count == AVERAGE_POINTS - 1) begin
                    angle_raw <= rounded_sum[AVERAGE_LOG2+9:AVERAGE_LOG2] ^ (use_shifted ? 10'd512 : 10'd0);
                    // Preserve one fractional ADC bit before whole-count rounding.
                    // 1527 means 763.5; the physical converter remains 10-bit.
                    angle_x2 <= half_rounded_sum[AVERAGE_LOG2+9:AVERAGE_LOG2-1] ^ (use_shifted ? 11'd1024 : 11'd0);
                    angle_q8 <= precise_sum[17:0] ^ (use_shifted ? 18'd131072 : 18'd0);
                    angle_valid <= 1'b1;
                    angle_update <= 1'b1;
                end else begin
                    sample_count <= sample_count + 1'b1;
                end
            end
        end
    end
endmodule
