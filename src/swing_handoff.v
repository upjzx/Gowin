// Condition only swing-up and the initial PID handoff. Normal balance bypasses
// this limiter. The top-level safety mux always cuts output immediately.
module swing_handoff #(
    parameter integer ENTRY_ADC=40, EXIT_ADC=64,
    parameter integer SWING_STEP=128, CAPTURE_STEP=256
) (
    input wire clk,rst_n,enable,update,policy_tick,swing_active,balance_enable,capture,
    input wire signed [15:0] angle_error,pump_command,tracking_command,balance_command,
    output wire signed [15:0] command,
    output reg soft_owner
);
    reg signed [15:0] limited_command;
    reg handoff_active;
    reg [2:0] capture_age;
    wire signed [15:0] request=balance_enable ? balance_command : soft_owner ? tracking_command : pump_command;
    wire signed [16:0] difference=$signed({request[15],request})-$signed({limited_command[15],limited_command});
    wire signed [16:0] step_limit=balance_enable ? CAPTURE_STEP : SWING_STEP;
    assign command=!enable ? 16'sd0 : (balance_enable && !handoff_active && !capture) ? balance_command : limited_command;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) soft_owner<=0;
        else if(!enable || !swing_active) soft_owner<=0;
        else if(policy_tick) begin
            if(!soft_owner && angle_error<=ENTRY_ADC && angle_error>=-ENTRY_ADC) soft_owner<=1;
            else if(soft_owner && (angle_error>EXIT_ADC || angle_error< -EXIT_ADC)) soft_owner<=0;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin limited_command<=0;handoff_active<=1;capture_age<=0;end
        else if(!enable) begin limited_command<=0;handoff_active<=1;capture_age<=0;end
        else if(capture) begin handoff_active<=1;capture_age<=0;end
        else if(update) begin
            if(!balance_enable) begin handoff_active<=1;capture_age<=0;end
            else if(capture_age<4) capture_age<=capture_age+1'b1;
            // Warm accepted PID pipeline before deciding that the handoff has
            // converged; never seed from its reset-to-zero output on capture.
            if(balance_enable && capture_age>=4 && difference<=step_limit && difference>=-step_limit)
                handoff_active<=0;
            if(difference>step_limit) limited_command<=limited_command+step_limit;
            else if(difference< -step_limit) limited_command<=limited_command-step_limit;
            else limited_command<=request;
        end
    end
endmodule
