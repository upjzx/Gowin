module key_debounce #(
    parameter integer DEBOUNCE_CYCLES = 1000000
) (
    input wire clk,
    input wire rst_n,
    input wire key_n,
    output wire pressed,
    output reg press_pulse
);
    reg [1:0] key_sync;
    reg stable_n;
    reg [31:0] count;

    // A released key disables motor drive after at most two clock cycles.
    assign pressed = !stable_n && !key_sync[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_sync <= 2'b11;
            stable_n <= 1'b1;
            count <= 0;
            press_pulse <= 1'b0;
        end else begin
            key_sync <= {key_sync[0], key_n};
            press_pulse <= 1'b0;
            if (key_sync[1] == stable_n) begin
                count <= 0;
            end else if (count == DEBOUNCE_CYCLES - 1) begin
                stable_n <= key_sync[1];
                count <= 0;
                if (!key_sync[1])
                    press_pulse <= 1'b1;
            end else begin
                count <= count + 1'b1;
            end
        end
    end
endmodule
