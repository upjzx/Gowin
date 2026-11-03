// Short unconfirmed-center hold, then bounded reduction. Direction reversal,
// reset and safety disable never wait for center qualification.
module balance_pwm_limiter #(
    parameter integer MAX_COMMAND = 2000,
    parameter integer MIN_ACTIVE_COMMAND = 600,
    parameter integer OUTPUT_RELEASE_STEP = 100,
    parameter integer START_COMMAND = 1200,
    parameter integer START_TICKS = 8,
    parameter integer LINEAR_ENABLE = 0,
    parameter integer LINEAR_MAX_STEP = 0,
    parameter integer LINEAR_BOOST_COMMAND = 0
) (
    input wire clk, rst_n, enable, update, inside_deadband, deadband_confirmed, position_needs_action,
    input wire signed [15:0] request,
    input wire linear_mode,
    output reg signed [15:0] command
);
    reg [31:0] start_left;
    reg start_positive, previous_force_min, pending, hold_reduce;
    reg signed [15:0] desired_hold;
    reg prepare, positive_hold, zero_hold, boost_hold;
    reg [31:0] normal_hold;
    wire force_min = !deadband_confirmed || position_needs_action;
    wire positive = !request[15];
    wire [15:0] magnitude = request[15] ? -request : request;
    wire use_linear = LINEAR_ENABLE!=0 && linear_mode;
    // Continuous odd compensation: double small requests, then add only a
    // bounded amount. There is no nonzero floor or discontinuity at zero.
    wire [31:0] linear_extra = magnitude<LINEAR_BOOST_COMMAND ? magnitude : LINEAR_BOOST_COMMAND;
    wire [31:0] linear_amplitude = magnitude + linear_extra;
    wire [31:0] normal_amplitude = use_linear ? linear_amplitude :
        (force_min && magnitude < MIN_ACTIVE_COMMAND ? MIN_ACTIVE_COMMAND : magnitude);
    wire start_now = !use_linear && request != 0 && force_min && START_TICKS > 0 && START_COMMAND > 0 &&
        (command == 0 || !previous_force_min);
    wire continuing_start = !use_linear && start_left != 0 && force_min && request != 0 && positive == start_positive;
    wire [31:0] boosted_amplitude = normal_hold < START_COMMAND ? START_COMMAND : normal_hold;
    wire [31:0] amplitude = zero_hold ? 0 : (boost_hold ? boosted_amplitude : normal_hold);
    wire [15:0] limited = amplitude > MAX_COMMAND ? MAX_COMMAND : amplitude[15:0];
    wire [15:0] current_magnitude = command[15] ? -command : command;
    wire [15:0] target_magnitude = desired_hold[15] ? -desired_hold : desired_hold;
    wire signed [31:0] release_up = $signed(command) + OUTPUT_RELEASE_STEP;
    wire signed [31:0] release_down = $signed(command) - OUTPUT_RELEASE_STEP;
    wire signed [31:0] linear_delta = $signed(desired_hold) - $signed(command);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            start_left <= 0; start_positive <= 0; previous_force_min <= 0;
            pending <= 0; hold_reduce <= 0; desired_hold <= 0; command <= 0;
            prepare<=0;positive_hold<=0;zero_hold<=1;boost_hold<=0;normal_hold<=0;
        end else if(!enable) begin
            start_left <= 0; previous_force_min <= 0; pending <= 0;
            hold_reduce <= 0; desired_hold <= 0; command <= 0;
            prepare<=0;positive_hold<=0;zero_hold<=1;boost_hold<=0;normal_hold<=0;
        end else if(update) begin
            previous_force_min <= force_min; prepare <= 1; pending <= 0;
            normal_hold<=normal_amplitude;positive_hold<=positive;
            zero_hold<=request==0;boost_hold<=start_now || continuing_start;
            hold_reduce <= !use_linear && inside_deadband && !deadband_confirmed && !position_needs_action;

            if(start_now) begin start_left <= START_TICKS - 1; start_positive <= positive; end
            else if(continuing_start) start_left <= start_left - 1'b1;
            else start_left <= 0;
        end else if(prepare) begin
            // One extra 50MHz cycle, still completed before the next 0.5ms tick.
            prepare<=0;pending<=1;
            desired_hold<=positive_hold ? $signed(limited) : -$signed(limited);
        end else if(pending) begin
            pending <= 0;
            if(use_linear && LINEAR_MAX_STEP>0 && linear_delta>LINEAR_MAX_STEP)
                command <= $signed(command)+LINEAR_MAX_STEP;
            else if(use_linear && LINEAR_MAX_STEP>0 && linear_delta< -LINEAR_MAX_STEP)
                command <= $signed(command)-LINEAR_MAX_STEP;
            else if(use_linear) command <= desired_hold;
            else if(command != 0 && desired_hold != 0 && desired_hold[15] != command[15]) command <= desired_hold;
            else if(hold_reduce && current_magnitude > target_magnitude) command <= command;
            else if(OUTPUT_RELEASE_STEP > 0 && current_magnitude > target_magnitude &&
                    current_magnitude - target_magnitude > OUTPUT_RELEASE_STEP)
                command <= command[15] ? release_up[15:0] : release_down[15:0];
            else command <= desired_hold;
        end
    end
endmodule
