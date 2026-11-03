module telemetry_v1 #(
    parameter integer CLOCK_HZ = 50000000,
    parameter integer BAUD = 115200,
    parameter integer FRAME_DIV = 500000,
    parameter integer EXTENDED_ANGLE = 0,
    parameter integer EXTENDED_STATE = 0,
    parameter integer SUPERVISOR_STATE = 0, ROUNDTRIP_STATE=0, KEY_STATE=0
) (
    input wire clk,
    input wire rst_n,
    input wire enable,
    input wire [7:0] status,
    input wire [31:0] supervisor_data, goal_position, anchor_position, board_ticks,
    input wire [15:0] dwell_ticks,
    input wire [31:0] key_data,
    input wire [9:0] angle_raw,
    input wire [5:0] angle_metadata,
    input wire [31:0] precise_angle_q8,
    input wire signed [31:0] correction_q8, position_velocity_q8, position_integral_q16, reference_velocity_q8,
    input wire signed [31:0] encoder1,
    input wire signed [31:0] encoder2,
    input wire signed [15:0] motor_command,
    output wire uart_tx_pin
);
    localparam [1:0] IDLE = 0, SEND = 1, WAIT_DONE = 2;
    reg [1:0] state;
    reg [31:0] frame_count;
    localparam integer FRAME_LENGTH=KEY_STATE!=0 && ROUNDTRIP_STATE!=0 ? 61 : ROUNDTRIP_STATE!=0 ? 57 : SUPERVISOR_STATE!=0 ? 43 : EXTENDED_STATE!=0 ? 39 : 19;
    reg [5:0] byte_index;
    reg [7:0] frame_sequence;
    reg [7:0] status_hold;
    reg [15:0] angle_hold;
    reg [31:0] enc1_hold;
    reg [31:0] enc2_hold;
    reg [15:0] command_hold;
    reg [335:0] extended_hold;
    reg [15:0] crc;
    reg tx_start;
    reg [7:0] tx_data;
    wire tx_busy;
    wire tx_done;
    reg [7:0] selected_byte;

    function [15:0] crc_byte;
        input [15:0] old_crc;
        input [7:0] value;
        reg [15:0] work;
        integer bit_number;
        begin
            work = old_crc ^ {value, 8'h00};
            for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
                if (work[15])
                    work = (work << 1) ^ 16'h1021;
                else
                    work = work << 1;
            crc_byte = work;
        end
    endfunction

    always @* begin
        case (byte_index)
            0: selected_byte = 8'hA5;
            1: selected_byte = 8'h5A;
            2: selected_byte = KEY_STATE!=0 && ROUNDTRIP_STATE!=0 ? 8'h05 : ROUNDTRIP_STATE!=0 ? 8'h04 : SUPERVISOR_STATE!=0 ? 8'h03 : EXTENDED_STATE!=0 ? 8'h02 : 8'h01;
            3: selected_byte = frame_sequence;
            4: selected_byte = status_hold;
            5: selected_byte = angle_hold[7:0];
            6: selected_byte = angle_hold[15:8];
            7: selected_byte = enc1_hold[7:0];
            8: selected_byte = enc1_hold[15:8];
            9: selected_byte = enc1_hold[23:16];
            10: selected_byte = enc1_hold[31:24];
            11: selected_byte = enc2_hold[7:0];
            12: selected_byte = enc2_hold[15:8];
            13: selected_byte = enc2_hold[23:16];
            14: selected_byte = enc2_hold[31:24];
            15: selected_byte = command_hold[7:0];
            16: selected_byte = command_hold[15:8];
            default: begin
                if(byte_index==FRAME_LENGTH-2) selected_byte=crc[7:0];
                else if(byte_index==FRAME_LENGTH-1) selected_byte=crc[15:8];
                else if(EXTENDED_STATE!=0 && byte_index>=17 && byte_index<(KEY_STATE!=0 && ROUNDTRIP_STATE!=0 ? 59 : ROUNDTRIP_STATE!=0 ? 55 : SUPERVISOR_STATE!=0 ? 41 : 37))
                    selected_byte=extended_hold[(byte_index-17)*8 +: 8];
                else selected_byte=0;
            end
        endcase
    end

    uart_tx #(.CLOCK_HZ(CLOCK_HZ), .BAUD(BAUD)) sender (
        .clk(clk), .rst_n(rst_n), .start(tx_start), .data(tx_data),
        .tx(uart_tx_pin), .busy(tx_busy), .done(tx_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            frame_count <= 0;
            byte_index <= 0;
            frame_sequence <= 0;
            status_hold <= 0;
            angle_hold <= 0;
            enc1_hold <= 0;
            enc2_hold <= 0;
            command_hold <= 0;
            extended_hold<=0;
            crc <= 16'hFFFF;
            tx_start <= 1'b0;
            tx_data <= 0;
        end else begin
            tx_start <= 1'b0;
            if (frame_count == FRAME_DIV - 1)
                frame_count <= 0;
            else
                frame_count <= frame_count + 1'b1;

            case (state)
                IDLE: if (enable && frame_count == FRAME_DIV - 1) begin
                    status_hold <= status;
                    angle_hold <= {EXTENDED_ANGLE!=0 ? angle_metadata : 6'b0, angle_raw};
                    enc1_hold <= encoder1;
                    enc2_hold <= encoder2;
                    command_hold <= motor_command;
                    extended_hold<={key_data,board_ticks,dwell_ticks,anchor_position,goal_position,supervisor_data,reference_velocity_q8,position_integral_q16,position_velocity_q8,correction_q8,precise_angle_q8};
                    crc <= 16'hFFFF;
                    byte_index <= 0;
                    state <= SEND;
                end
                SEND: if (!tx_busy) begin
                    tx_data <= selected_byte;
                    tx_start <= 1'b1;
                    if (byte_index >= 2 && byte_index <= FRAME_LENGTH-3)
                        crc <= crc_byte(crc, selected_byte);
                    state <= WAIT_DONE;
                end
                WAIT_DONE: if (tx_done) begin
                    if (byte_index == FRAME_LENGTH-1) begin
                        frame_sequence <= frame_sequence + 1'b1;
                        state <= IDLE;
                    end else begin
                        byte_index <= byte_index + 1'b1;
                        state <= SEND;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule
