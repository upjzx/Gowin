// Before the unchanged 360deg/sine mission: acquire local balance, then
// approach the nearest saved zero with a speed/acceleration limited reference.
module home_zero_direct #(
    parameter integer COUNTS_PER_REV=1080, LOCAL_TICKS=20, ARRIVAL_TICKS=40,
    parameter integer LOCAL_SPEED_Q8=256, LOCAL_EXCURSION=20, TEMP_TARGET_TOLERANCE=40,
    parameter integer MOVE_ANGLE_ADC=6, RESUME_ANGLE_ADC=4, ARRIVAL_COUNTS=6,
    parameter integer MAX_SPEED_Q8=64, ACCEL_Q8=1, APPROACH_SHIFT=6, MAX_FOLLOW_ERROR=24,
    // After arrival, re-run the return when an idle hold is pushed more than
    // REHOME_COUNTS from the nearest saved zero for REHOME_TICKS. 0 = off.
    parameter integer REHOME_COUNTS=0, REHOME_TICKS=100,
    parameter integer TURN_FAST_START_ENABLE=0, TURN_START_COUNTS=36,
    parameter integer TURN_START_SPEED_Q8=384, TURN_START_ANGLE_ADC=18,
    parameter integer READY_ON_ARRIVAL=0, DIRECT_TO_ZERO=0,
    parameter integer HANDOFF_COUNTS=12, HANDOFF_SPEED_Q8=128, HANDOFF_ANGLE_ADC=12
) (
    input wire clk,rst_n,enable,capture,update,velocity_valid,upright_entry,temp_target_locked,
    input wire rehome_allowed,motion_request_pending,turn_request_pending,
    // Optional one-clock request; legacy instances may leave these unconnected.
    input wire force_return,
    input wire signed [31:0] force_reference_position,
    input wire signed [31:0] position,zero_position,temporary_target,velocity_q8,angle_error_q8,angle_velocity_q8,
    output reg reference_enable,done,
    output reg signed [31:0] goal,anchor,
    output wire signed [31:0] reference_position,reference_velocity_q8,
    output wire pause_drive,
    output wire [3:0] phase,
    output reg [15:0] qualified
);
    reg paused;
    reg signed [32:0] nearest_zero;
    reg signed [31:0] quiet_origin;
    reg signed [47:0] reference_q16;
    reg signed [31:0] speed_q16;
    wire signed [47:0] rounded_reference=reference_q16+48'sd32768;
    reg signed [31:0] reference_position_hold;
    assign reference_position=reference_position_hold;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) reference_position_hold<=0;
        // Seed the visible reference on the same clock as ownership changes.
        else if(force_return===1'b1) reference_position_hold<=force_reference_position;
        else reference_position_hold<=rounded_reference >>> 16;
    end
    assign reference_velocity_q8=reference_enable && !done ? speed_q16/256 : 32'sd0;
    localparam signed [31:0] MAX_SPEED_Q16=MAX_SPEED_Q8*256, ACCEL_Q16=ACCEL_Q8*256;
    wire signed [47:0] goal_q16=$signed(goal)*48'sd65536;
    reg signed [47:0] distance_q16,next_reference;
    reg signed [32:0] following_error;
    reg path_allowed;
    reg signed [31:0] desired_speed,next_speed;
    wire signed [47:0] requested_speed=distance_q16 >>> APPROACH_SHIFT;
    wire reference_finished=distance_q16<=256 && distance_q16>=-256 &&
        speed_q16<=ACCEL_Q16 && speed_q16>=-ACCEL_Q16;
    // Short pipelines settle well before each 5ms position update.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin distance_q16<=0;following_error<=0;path_allowed<=0;desired_speed<=0;next_speed<=0;next_reference<=0;end
        else begin
            distance_q16<=goal_q16-reference_q16;
            following_error<=$signed({reference_position[31],reference_position})-$signed({position[31],position});
            // Wait only if the reference leads the plant toward its goal.
            // A plant ahead of the reference must not strand it behind.
            path_allowed<=!pause_drive && (goal>=anchor ? following_error<=MAX_FOLLOW_ERROR : following_error>=-MAX_FOLLOW_ERROR);
            desired_speed<=!path_allowed ? 0 : requested_speed>MAX_SPEED_Q16 ? MAX_SPEED_Q16 :
                requested_speed< -MAX_SPEED_Q16 ? -MAX_SPEED_Q16 : requested_speed[31:0];
            next_speed<=desired_speed>speed_q16+ACCEL_Q16 ? speed_q16+ACCEL_Q16 :
                desired_speed<speed_q16-ACCEL_Q16 ? speed_q16-ACCEL_Q16 : desired_speed;
            next_reference<=reference_q16+next_speed;
        end
    end
    wire signed [32:0] zero_delta=$signed({position[31],position})-nearest_zero;
    reg [15:0] rehome_count;
    wire rehome_far=zero_delta>REHOME_COUNTS || zero_delta< -REHOME_COUNTS;
    wire signed [32:0] excursion=$signed({position[31],position})-$signed({quiet_origin[31],quiet_origin});
    wire signed [32:0] final_error=$signed({goal[31],goal})-$signed({position[31],position});
    wire signed [32:0] temporary_error=$signed({temporary_target[31],temporary_target})-$signed({position[31],position});
    // Qualify local rest by measured motion, not distance from the first
    // turning point: coast after capture can leave that point far behind.
    wire local_ok=velocity_valid && temp_target_locked &&
        velocity_q8<=LOCAL_SPEED_Q8 && velocity_q8>=-LOCAL_SPEED_Q8 &&
        angle_error_q8<=13*256 && angle_error_q8>=-13*256 && angle_velocity_q8<=512 && angle_velocity_q8>=-512 &&
        (qualified==0 || (excursion<=LOCAL_EXCURSION && excursion>=-LOCAL_EXCURSION));
    wire angle_out=angle_error_q8>MOVE_ANGLE_ADC*256 || angle_error_q8< -MOVE_ANGLE_ADC*256;
    wire angle_recovered=angle_error_q8<=RESUME_ANGLE_ADC*256 && angle_error_q8>=-RESUME_ANGLE_ADC*256;
    assign pause_drive=reference_enable && !done && (paused || angle_out);
    assign phase=!enable ? 4'd0 : done ? 4'd15 : !reference_enable ? 4'd12 : pause_drive ? 4'd13 : 4'd14;
    // Actual arrival still requires valid low measured speed and recovered posture.
    // A nearby reference can be snapped to goal; do not wait for sub-count decay.
    localparam signed [47:0] ARRIVAL_REFERENCE_Q16=HANDOFF_COUNTS*48'sd65536;
    wire reference_near_goal=distance_q16<=ARRIVAL_REFERENCE_Q16 && distance_q16>=-ARRIVAL_REFERENCE_Q16;
    wire arrival_reference_ok=READY_ON_ARRIVAL!=0 ? reference_near_goal : reference_finished;
    wire legacy_arrived=reference_finished && velocity_valid && !pause_drive &&
        final_error<=ARRIVAL_COUNTS && final_error>=-ARRIVAL_COUNTS && velocity_q8<=64 && velocity_q8>=-64;
    wire handoff_arrived=arrival_reference_ok && velocity_valid &&
        final_error<=HANDOFF_COUNTS && final_error>=-HANDOFF_COUNTS &&
        velocity_q8<=HANDOFF_SPEED_Q8 && velocity_q8>=-HANDOFF_SPEED_Q8 &&
        angle_error_q8<=HANDOFF_ANGLE_ADC*256 && angle_error_q8>=-HANDOFF_ANGLE_ADC*256 &&
        angle_velocity_q8<=512 && angle_velocity_q8>=-512;
    // Selecting K3/K4 alone never loosens stationary arrival. A pending motion
    // request may use the wider handoff; otherwise require actual low-speed rest.
    wire requested_handoff=READY_ON_ARRIVAL!=0 && motion_request_pending===1'b1;
    wire turn_fast_requested=TURN_FAST_START_ENABLE!=0 && turn_request_pending===1'b1;
    wire turn_fast_posture=velocity_valid && temp_target_locked &&
        velocity_q8<=TURN_START_SPEED_Q8 && velocity_q8>=-TURN_START_SPEED_Q8 &&
        angle_error_q8<=TURN_START_ANGLE_ADC*256 && angle_error_q8>=-TURN_START_ANGLE_ADC*256 &&
        angle_velocity_q8<=1024 && angle_velocity_q8>=-1024;
    wire turn_fast_arrived=turn_fast_requested && turn_fast_posture &&
        final_error<=TURN_START_COUNTS && final_error>=-TURN_START_COUNTS;
    wire turn_fast_local=turn_fast_requested && turn_fast_posture &&
        zero_delta<=TURN_START_COUNTS && zero_delta>=-TURN_START_COUNTS;
    wire stationary_arrived=velocity_valid && !pause_drive &&
        final_error<=ARRIVAL_COUNTS && final_error>=-ARRIVAL_COUNTS &&
        distance_q16<=ARRIVAL_COUNTS*48'sd65536 && distance_q16>=-ARRIVAL_COUNTS*48'sd65536 &&
        velocity_q8<=32 && velocity_q8>=-32 && angle_velocity_q8<=128 && angle_velocity_q8>=-128;
    wire arrived=READY_ON_ARRIVAL!=0 ? (turn_fast_requested ? turn_fast_arrived : requested_handoff ? handoff_arrived : stationary_arrived) : legacy_arrived;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) nearest_zero<=0;
        else if(!enable || capture) nearest_zero<=$signed(zero_position);
        else if(!reference_enable || done) begin
            if(zero_delta>COUNTS_PER_REV/2) nearest_zero<=nearest_zero+COUNTS_PER_REV;
            else if(zero_delta< -(COUNTS_PER_REV/2)) nearest_zero<=nearest_zero-COUNTS_PER_REV;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin rehome_count<=0;reference_enable<=0;done<=0;goal<=0;anchor<=0;quiet_origin<=0;qualified<=0;paused<=0;reference_q16<=0;speed_q16<=0;end
        else if(!enable || capture) begin
            rehome_count<=0;reference_enable<=0;done<=0;goal<=position;anchor<=position;quiet_origin<=position;qualified<=0;paused<=0;
            reference_q16<=$signed(position)*48'sd65536;speed_q16<=0;
        end else if(force_return===1'b1) begin
            // Same entry as the post-catch return: hold the temporary target
            // until the arm is locally quiet, then ramp from the MEASURED
            // position to nearest_zero (still tracked whole revolutions).
            // Starting at once from the path endpoint carried the turn's
            // residual motion and lag into the return.
            rehome_count<=0;reference_enable<=0;done<=0;qualified<=0;paused<=0;
            goal<=position;anchor<=position;quiet_origin<=position;
            reference_q16<=$signed(position)*48'sd65536;speed_q16<=0;
        end else begin
            if(angle_out) paused<=1;
            else if(angle_recovered) paused<=0;
            if(update && done) begin
                // Displaced while idle: restart local qualification + approach.
                if(REHOME_COUNTS>0 && rehome_allowed===1'b1 && rehome_far) begin
                    if(rehome_count>=REHOME_TICKS-1) begin
                        rehome_count<=0;done<=0;reference_enable<=0;qualified<=0;paused<=0;
                    end else rehome_count<=rehome_count+1'b1;
                end else rehome_count<=0;
            end
            if(update && !done) begin
                if(!reference_enable) begin
                    goal<=position;anchor<=position;
                    reference_q16<=$signed(position)*48'sd65536;speed_q16<=0;
                    if(qualified==0) quiet_origin<=position;
                    if(turn_fast_local) begin
                        goal<=nearest_zero[31:0];reference_q16<=nearest_zero*48'sd65536;
                        reference_enable<=1;done<=1;qualified<=0;speed_q16<=0;
                    end else if((DIRECT_TO_ZERO!=0 || local_ok) && zero_delta<=COUNTS_PER_REV/2 && zero_delta>=-(COUNTS_PER_REV/2)) begin
                        if(DIRECT_TO_ZERO!=0 || qualified>=LOCAL_TICKS-1) begin
                            goal<=nearest_zero[31:0];anchor<=position;reference_enable<=1;qualified<=0;
                        end else qualified<=qualified+1'b1;
                    end else qualified<=0;
                end else begin
                    if(reference_finished || (READY_ON_ARRIVAL!=0 && stationary_arrived)) begin reference_q16<=goal_q16;speed_q16<=0;end
                    else begin reference_q16<=next_reference;speed_q16<=next_speed;end
                    if(arrived) begin
                    if(turn_fast_requested || requested_handoff || qualified>=ARRIVAL_TICKS-1) begin
                        done<=1;qualified<=0;
                        if(READY_ON_ARRIVAL!=0) begin reference_q16<=goal_q16;speed_q16<=0;end
                    end
                    else qualified<=qualified+1'b1;
                    end else qualified<=0;
                end
            end
        end
    end
endmodule
