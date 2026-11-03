// Signed PWM command to TB6612 direction pins, with all-low reversal blanking.
// All-low describes the FPGA pins; it is not a claim about H-bridge high-Z mode.
module motor_signed_pwm #(
    parameter integer PWM_PERIOD = 5000,
    parameter integer REVERSE_BLANK_CYCLES = 5000
) (
    input wire clk,
    input wire rst_n,
    input wire drive_enable,
    input wire fault,
    input wire signed [15:0] command,
    output reg in1,
    output reg in2,
    output reg pwm
);
    reg [15:0] pwm_count;
    reg [15:0] blank_count;
    reg [15:0] idle_count;
    reg direction_valid;
    reg direction_positive;
    wire desired_positive = !command[15];
    wire [15:0] command_magnitude = command[15] ? -command : command;
    wire [15:0] duty = (command_magnitude > PWM_PERIOD) ?
                       PWM_PERIOD : command_magnitude;
    wire [31:0] remaining_blank = REVERSE_BLANK_CYCLES - idle_count;
    wire active = drive_enable && !fault && (command != 0) &&
                  direction_valid && (blank_count == 0) &&
                  (direction_positive == desired_positive);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_count <= 0;
            blank_count <= 0;
            idle_count <= REVERSE_BLANK_CYCLES;
            direction_valid <= 1'b0;
            direction_positive <= 1'b0;
        end else begin
            // Start a complete pulse as soon as drive is allowed. Free-running
            // while inhibited can add up to one carrier period after blanking
            // and can resume with only the tail of a pulse. Blanking duration
            // and the asynchronous output inhibit below remain unchanged.
            if (!active || pwm_count == PWM_PERIOD - 1)
                pwm_count <= 0;
            else
                pwm_count <= pwm_count + 1'b1;

            if (!active) begin
                if (idle_count < REVERSE_BLANK_CYCLES)
                    idle_count <= idle_count + 1'b1;
            end else idle_count <= 0;

            if (!drive_enable || fault || command == 0) begin
                // Keep the last direction: a short zero/disable must not bypass
                // reversal blanking when output resumes in the opposite direction.
                if (blank_count != 0) blank_count <= blank_count - 1'b1;
            end else if (blank_count != 0) begin
                blank_count <= blank_count - 1'b1;
                if (blank_count == 1)
                    direction_positive <= desired_positive;
            end else if (!direction_valid) begin
                direction_positive <= desired_positive;
                direction_valid <= 1'b1;
            end else if (direction_positive != desired_positive) begin
                if (idle_count >= REVERSE_BLANK_CYCLES)
                    direction_positive <= desired_positive;
                else
                    blank_count <= remaining_blank[15:0];
            end
        end
    end

    always @* begin
        in1 = 1'b0;
        in2 = 1'b0;
        pwm = 1'b0;
        if (active) begin
            in1 = direction_positive;
            in2 = !direction_positive;
            pwm = (pwm_count < duty);
        end
    end
endmodule
