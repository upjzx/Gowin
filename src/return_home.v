// 5ms update: acquisition(12), local stability(13), return(14), hold(15).
module return_home #(
    parameter integer COUNTS_PER_REV=1080, QUALIFY_TICKS=20,
    parameter integer POSITION_WINDOW=30, ANGLE_WINDOW_ADC=20,
    parameter integer SPEED_WINDOW_Q8=256, ANGLE_SPEED_Q8=512,
    parameter integer STILL_EXCURSION_COUNTS=30,
    parameter integer MAX_SPEED_Q8=256, ACCEL_Q8=4, APPROACH_SHIFT=5
) (
    input wire clk,rst_n,enable,capture,update,target_locked,measurement_valid,
    input wire signed [31:0] position,acquired_target,zero_angle,
    input wire signed [31:0] velocity_q8,angle_error_q8,angle_velocity_q8,
    output reg override_enable,
    output wire signed [31:0] reference_position,reference_velocity_q8,
    output reg [3:0] phase,
    output reg signed [31:0] goal,anchor,
    output reg [15:0] dwell_ticks
);
    reg signed [39:0] reference_q8;
    reg signed [31:0] speed_q8;
    reg signed [32:0] nearest_zero;
    reg signed [31:0] stability_origin;
    wire signed [32:0] stability_excursion=$signed({position[31],position})-$signed({stability_origin[31],stability_origin});
    wire signed [32:0] zero_delta=$signed({position[31],position})-nearest_zero;
    // Constant iterative reduction avoids a combinational modulo divider.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) nearest_zero<=0;
        else if(!enable || capture) nearest_zero<=$signed(zero_angle);
        else if(!override_enable) begin
            if(zero_delta>COUNTS_PER_REV/2) nearest_zero<=nearest_zero+COUNTS_PER_REV;
            else if(zero_delta< -(COUNTS_PER_REV/2)) nearest_zero<=nearest_zero-COUNTS_PER_REV;
        end
    end
    assign reference_position=reference_q8 >>> 8;
    assign reference_velocity_q8=phase==14 ? speed_q8 : 32'sd0;
    wire signed [32:0] goal_error=$signed(goal)-$signed({position[31],position});
    wire posture_ok=angle_error_q8<=ANGLE_WINDOW_ADC*256 && angle_error_q8>=-ANGLE_WINDOW_ADC*256 &&
        angle_velocity_q8<=ANGLE_SPEED_Q8 && angle_velocity_q8>=-ANGLE_SPEED_Q8;
    // Temporary-target error is deliberately NOT an arrival gate. A quiet
    // arm can stop far from its first turning point and still return home.
    // Also allow a quiet catch without an eventual direction reversal.
    wire locally_ready=measurement_valid && posture_ok &&
        (dwell_ticks==0 || (stability_excursion<=STILL_EXCURSION_COUNTS && stability_excursion>=-STILL_EXCURSION_COUNTS)) &&
        velocity_q8<=SPEED_WINDOW_Q8 && velocity_q8>=-SPEED_WINDOW_Q8;
    wire arrived=posture_ok && goal_error<=POSITION_WINDOW && goal_error>=-POSITION_WINDOW && velocity_q8<=64 && velocity_q8>=-64;
    reg signed [39:0] distance_q8,next_reference;
    reg signed [32:0] follow_error;
    reg signed [31:0] desired_speed,next_speed;
    reg path_allowed;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin distance_q8<=0;follow_error<=0;path_allowed<=0;desired_speed<=0;next_speed<=0;next_reference<=0;end
        else begin
            distance_q8<=$signed({{8{goal[31]}},goal})*40'sd256-reference_q8;
            follow_error<=$signed(reference_position)-$signed({position[31],position});
            path_allowed<=angle_error_q8<=24*256 && angle_error_q8>=-24*256 && follow_error<=90 && follow_error>=-90;
            desired_speed<=!path_allowed ? 0 : (distance_q8 >>> APPROACH_SHIFT)>MAX_SPEED_Q8 ? MAX_SPEED_Q8 :
                (distance_q8 >>> APPROACH_SHIFT)< -MAX_SPEED_Q8 ? -MAX_SPEED_Q8 : (distance_q8 >>> APPROACH_SHIFT);
            next_speed<=desired_speed>speed_q8+ACCEL_Q8 ? speed_q8+ACCEL_Q8 :
                desired_speed<speed_q8-ACCEL_Q8 ? speed_q8-ACCEL_Q8 : desired_speed;
            next_reference<=reference_q8+next_speed;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin phase<=0;override_enable<=0;reference_q8<=0;speed_q8<=0;goal<=0;anchor<=0;dwell_ticks<=0;stability_origin<=0;end
        else if(!enable || capture) begin
            phase<=enable ? 12 : 0;override_enable<=0;reference_q8<=$signed(position)*40'sd256;
            speed_q8<=0;goal<=position;anchor<=position;dwell_ticks<=0;
            stability_origin<=position;
        end else if(update) begin
            if(!override_enable) begin
                reference_q8<=$signed(acquired_target)*40'sd256;goal<=acquired_target;anchor<=position;
                phase<=measurement_valid ? 13 : 12;
                if(dwell_ticks==0) stability_origin<=position;
                if(locally_ready && zero_delta<=COUNTS_PER_REV/2 && zero_delta>=-(COUNTS_PER_REV/2)) begin
                    if(dwell_ticks>=QUALIFY_TICKS-1) begin
                        goal<=nearest_zero[31:0];anchor<=position;
                        reference_q8<=$signed(position)*40'sd256;speed_q8<=0;
                        override_enable<=1;phase<=14;dwell_ticks<=0;
                    end else dwell_ticks<=dwell_ticks+1'b1;
                end else dwell_ticks<=0;
            end else if(phase==14) begin
                if(distance_q8<=256 && distance_q8>=-256 && speed_q8<=ACCEL_Q8 && speed_q8>=-ACCEL_Q8) begin
                    reference_q8<=$signed(goal)*40'sd256;speed_q8<=0;
                    if(arrived) begin
                        if(dwell_ticks>=QUALIFY_TICKS-1) begin phase<=15;dwell_ticks<=0;end
                        else dwell_ticks<=dwell_ticks+1'b1;
                    end else dwell_ticks<=0;
                end else begin reference_q8<=next_reference;speed_q8<=next_speed;dwell_ticks<=0;end
            end else begin reference_q8<=$signed(goal)*40'sd256;speed_q8<=0;end
        end
    end
endmodule
