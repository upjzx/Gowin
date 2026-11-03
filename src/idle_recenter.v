// Fixed-target recovery ONLY while the adapter asserts idle hold mode.
// Output is a small velocity request; the saved target is never moved.
module idle_recenter #(
    parameter integer ENABLE=1, MAX_SPEED_Q8=64, START_COUNTS=6, STOP_COUNTS=3,
    parameter integer ACCEL_Q8=4, GAIN_Q8=4, ANGLE_WINDOW_ADC=12
)(
    input wire clk,rst_n,enable,update,
    input wire signed [31:0] target,position,angle_error_q8,
    output wire signed [31:0] velocity_q8,
    output reg active
);
    reg signed [32:0] error;
    reg signed [63:0] proportional_velocity;
    reg signed [31:0] requested_velocity,speed;
    reg posture_ok;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) posture_ok<=0;
        else posture_ok<=angle_error_q8<=ANGLE_WINDOW_ADC*256 && angle_error_q8>=-ANGLE_WINDOW_ADC*256;
    end
    wire allowed=ENABLE!=0 && enable && posture_ok;
    wire outside_start=error>START_COUNTS || error< -START_COUNTS;
    wire inside_stop=error<=STOP_COUNTS && error>=-STOP_COUNTS;
    wire wrong_direction=(speed>0 && error<0) || (speed<0 && error>0);
    assign velocity_q8=allowed && active ? speed : 32'sd0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin error<=0;proportional_velocity<=0;requested_velocity<=0;end
        else begin
            error<=$signed({target[31],target})-$signed({position[31],position});
            proportional_velocity<=$signed(error)*GAIN_Q8;
            requested_velocity<=proportional_velocity>MAX_SPEED_Q8 ? MAX_SPEED_Q8 :
                proportional_velocity< -MAX_SPEED_Q8 ? -MAX_SPEED_Q8 : proportional_velocity;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin active<=0;speed<=0;end
        else if(!allowed) begin active<=0;speed<=0;end
        else if(update) begin
            if(inside_stop || (!active && !outside_start)) begin active<=0;speed<=0;end
            else begin
                active<=1;
                if(wrong_direction) speed<=0;
                else if(requested_velocity>speed+ACCEL_Q8) speed<=speed+ACCEL_Q8;
                else if(requested_velocity<speed-ACCEL_Q8) speed<=speed-ACCEL_Q8;
                else speed<=requested_velocity;
            end
        end
    end
endmodule
