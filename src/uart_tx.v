module uart_tx #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire start,
    input wire [7:0] data,
    output reg tx,
    output reg busy,
    output reg done
);
    localparam integer BIT_CYCLES = (CLOCK_HZ + BAUD / 2) / BAUD;
    reg [31:0] baud_count;
    reg [3:0] bit_index;
    reg [7:0] data_latched;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx <= 1'b1;
            busy <= 1'b0;
            done <= 1'b0;
            baud_count <= 0;
            bit_index <= 0;
            data_latched <= 0;
        end else begin
            done <= 1'b0;
            if (!busy) begin
                if (start) begin
                    data_latched <= data;
                    tx <= 1'b0;
                    busy <= 1'b1;
                    bit_index <= 0;
                    baud_count <= BIT_CYCLES - 1;
                end
            end else if (baud_count != 0) begin
                baud_count <= baud_count - 1'b1;
            end else begin
                baud_count <= BIT_CYCLES - 1;
                if (bit_index < 8) begin
                    tx <= data_latched[bit_index];
                    bit_index <= bit_index + 1'b1;
                end else if (bit_index == 8) begin
                    tx <= 1'b1;
                    bit_index <= 9;
                end else begin
                    tx <= 1'b1;
                    busy <= 1'b0;
                    done <= 1'b1;
                end
            end
        end
    end
endmodule
