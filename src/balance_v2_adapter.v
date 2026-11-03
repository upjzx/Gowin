// PID cascade from gowin-pid. D history tracks during swing-up; the position
// target, integral/center qualification and PWM become active only at catch.
module balance_v2_adapter #(
    parameter integer CAPTURE_VELOCITY_SEED_ENABLE=0,
    parameter integer RETURN_START_ZERO_ENABLE=0, ARM_COUNTS_PER_REV=1080, HOME_MOVE_ANGLE_ADC=6,
    parameter integer POST_MOTION_HOME_ENABLE=0,
    parameter integer HOME_REHOME_COUNTS=0, HOME_REHOME_TICKS=100,
    parameter integer HOME_READY_ON_ARRIVAL=0, HOME_DIRECT_TO_ZERO=0, HOME_LOCAL_TICKS=20,
    parameter integer HOME_HANDOFF_COUNTS=12, HOME_HANDOFF_SPEED_Q8=128, HOME_HANDOFF_ANGLE_ADC=12,
    // Relaxed handoff applies only to explicit K3/K1 or K3/K2 commands.
    parameter integer TURN_FAST_START_ENABLE=1, TURN_START_COUNTS=36,
    parameter integer TURN_START_SPEED_Q8=384, TURN_START_ANGLE_ADC=18,
    parameter integer HOLD_ARRIVAL_COUNTS=3,
    // Idle-only recenter: Q8 counts per 5ms, 64 = 50 counts/s.
    parameter integer IDLE_RECENTER_ENABLE=1, IDLE_RETURN_MAX_SPEED_Q8=64,
    parameter integer IDLE_RETURN_START_COUNTS=6, IDLE_RETURN_STOP_COUNTS=3,
    parameter integer IDLE_RETURN_ACCEL_Q8=4, IDLE_RETURN_GAIN_Q8=4,
    parameter integer IDLE_HOLD_KD_Q8=3200, IDLE_HOLD_P_LIMIT_ADC=4,
    // 128 adds up to 50% velocity damping inside the endpoint window.
    parameter integer TURN_CW_BRAKE_STRENGTH_Q8=128, TURN_CCW_BRAKE_STRENGTH_Q8=128,
    parameter integer TURN_BRAKE_MAX_EXTRA_SPEED_Q8=512,
    parameter integer HOLD_REFERENCE_KP_Q8=32, HOLD_REFERENCE_KI_Q16=4,
    parameter integer WAYPOINT_TICKS_PER_30DEG=200, WAYPOINT_ARRIVAL_COUNTS=12,
    parameter integer WAYPOINT_DWELL_TICKS=40, WAYPOINT_TIMEOUT_TICKS=4000,
    parameter integer WAYPOINT_PASS_THROUGH=0, WAYPOINT_PASS_COUNTS=3,
    parameter integer TURN_ENDPOINT_BRAKE_ENABLE=0, TURN_ENDPOINT_WINDOW_COUNTS=256,
    // CW/CCW name the commanded turn, independently of encoder polarity.
    parameter integer TURN_CW_ENDPOINT_WINDOW_COUNTS=TURN_ENDPOINT_WINDOW_COUNTS,
    parameter integer TURN_CCW_ENDPOINT_WINDOW_COUNTS=TURN_ENDPOINT_WINDOW_COUNTS,
    // Positive/negative select the next sine/waypoint travel direction.
    parameter integer SINE_ENDPOINT_BRAKE_ENABLE=1,
    parameter integer SINE_POS_ENDPOINT_WINDOW_COUNTS=90, SINE_NEG_ENDPOINT_WINDOW_COUNTS=90,
    parameter integer HOME_MAX_SPEED_Q8=64, HOME_ACCEL_Q8=1, HOME_APPROACH_SHIFT=6,
    parameter integer UPRIGHT_ADC=767,
    // 1: upright point comes from the upright_adc_q8 port (K1 calibration).
    parameter integer RUNTIME_CAL=0,
    // 0: the 360deg sequence waits for turn_trigger only (no 10 s auto start).
    parameter integer TURN_AUTO_START=1,
    parameter integer AUTO_SEQUENCE_ENABLE=1, ROUNDTRIP_ENABLE=0, DIRECT_TURN_ENABLE=0, STEP_PERIOD_TICKS=1200, SEQUENCE_DWELL_TICKS=4000,
    parameter integer MOVE_SPEED_LIMIT_Q16=32768,
    parameter integer SMOOTH_STEP_ENABLE=0, MINJERK_ENABLE=0, ACCEL_FF_Q8=0,
    parameter integer SINE_ACCEL_FF_Q8=0, SINE_DIRECTION=1,
    parameter integer SINE_ENABLE=0, SINE_AMPLITUDE=15, SINE_PERIOD_TICKS=4000, SINE_CYCLES=3,
    parameter integer WAVEPACKET_ENABLE=0, WAVEPACKET_AMPLITUDE=SINE_AMPLITUDE,
    parameter integer WAVEPACKET_PERIOD_TICKS=SINE_PERIOD_TICKS, WAVEPACKET_CYCLES=8,
    parameter integer WAVEPACKET_DIRECTION=SINE_DIRECTION,
    // 1: original sin^2 envelope; 2: stronger contrast with sin^4 envelope.
    parameter integer WAVEPACKET_ENVELOPE_POWER=2,
    // Profile 1: seven explicit alternating C2 peaks; profile 0: legacy envelope.
    parameter integer WAVEPACKET_PROFILE=0,
    parameter integer WAVEPACKET_PEAK_1=90, WAVEPACKET_PEAK_2=240,
    parameter integer WAVEPACKET_PEAK_3=390, WAVEPACKET_PEAK_4=540,
    parameter integer EXTENSION_ENABLE=0, EXTENSION_STEP_COUNTS=540, EXTENSION_DIRECTION=1,
    parameter integer HIGH_RES_ANGLE=0,
    parameter integer LINEAR_STEADY_ENABLE=0,
    parameter integer RECOVERY_ENABLE=0, RECOVERY_ENTRY_ADC=12,
    parameter integer RECOVERY_EXIT_ADC=4, RECOVERY_QUIET_TICKS=1000,
    parameter integer LINEAR_STEADY_DELAY_TICKS=2000,
    parameter integer LINEAR_MAX_STEP=0,
    parameter integer LINEAR_BOOST_COMMAND=0,
    parameter integer CONTROL_DIV=25000,
    parameter integer ANGLE_DEADBAND_X2=8,
    parameter integer DEADBAND_CONFIRM_TICKS=8000,
    parameter signed [15:0] ANGLE_KP=16'sd220,
    parameter signed [15:0] STEADY_ANGLE_KP=16'sd0,
    parameter signed [15:0] ANGLE_KD=16'sd400,
    parameter signed [15:0] STEADY_ANGLE_KD=16'sd0,
    parameter signed [15:0] ANGLE_KI_Q8=16'sd0,
    parameter integer INTEGRAL_LIMIT_COMMAND=200,
    parameter integer INTEGRAL_WINDOW_ADC=10,
    parameter integer MAX_D_COMMAND=1400,
    parameter integer DERIVATIVE_WINDOW_TICKS=5,
    parameter integer DERIVATIVE_FILTER_SHIFT=1,
    parameter integer MAX_COMMAND=4000,
    parameter integer MOTOR_POLARITY=0,
    parameter integer POSITION_ENABLE=1,
    parameter signed [15:0] POSITION_KP_Q8=16'sd4,
    parameter signed [15:0] POSITION_KD_Q8=16'sd1790,
    parameter signed [15:0] POSITION_STEADY_KD_Q8=16'sd0,
    parameter signed [15:0] POSITION_KI_Q16=16'sd0,
    parameter integer POSITION_POLARITY=1,
    parameter integer POSITION_I_LIMIT_ADC=1,
    parameter integer POSITION_INTEGRAL_WINDOW_COUNTS=0,
    parameter integer POSITION_INTEGRAL_QUIET_TICKS=0,
    parameter integer POSITION_INTEGRAL_VELOCITY_LIMIT_Q8=64,
    parameter integer POSITION_INTEGRAL_MOVING_DECAY_SHIFT=0,
    parameter integer POSITION_INTEGRAL_DEADBAND_COUNTS=0,
    parameter integer POSITION_INTEGRAL_OVERSHOOT_COUNTS=0,
    parameter integer POSITION_INTEGRAL_UNWIND_SHIFT=0,
    parameter integer REFERENCE_HOLD_I_DEADBAND_COUNTS=6, REFERENCE_SETTLE_I_DEADBAND_COUNTS=4,
    parameter integer MAX_TARGET_OFFSET_ADC=6,
    parameter integer STEADY_MAX_TARGET_OFFSET_ADC=0,
    parameter integer REFERENCE_POSITION_P_LIMIT_ADC=4,
    parameter integer TURN_DISTANCE_BRAKE_ENABLE=0, TURN_DISTANCE_WINDOW_SHIFT=9, TURN_DISTANCE_WINDOW_COUNTS=540,
    parameter integer TURN_BRAKE_ENABLE=0, TURN_BRAKE_LOOKAHEAD_TICKS=64,
    parameter integer TURN_BRAKE_MAX_LEAD_COUNTS=128,
    parameter integer POSITION_STEADY_P_LIMIT_ADC=0,
    parameter integer POSITION_STEADY_P_DEADBAND_COUNTS=0,
    parameter integer MAX_TARGET_STEP_Q8=14,
    parameter integer STEADY_POSITION_STEP_Q8=0, STEADY_DAMPING_STEP_Q8=0,
    parameter integer TURN_POSITION_STEP_Q8=0, TURN_LAUNCH_STEP_Q8=0, TURN_LAUNCH_TICKS=40,
    parameter integer POSITION_TOLERANCE_COUNTS=2,
    parameter integer POSITION_TARGET_MODE=0,
    parameter integer POSITION_TURN_MIN_TRAVEL_COUNTS=20,
    parameter integer POSITION_TURN_REVERSE_COUNTS=30,
    parameter integer POSITION_TURN_REVERSE_TICKS=4,
    parameter integer POSITION_TURN_STOP_TICKS=20,
    parameter signed [31:0] POSITION_TARGET_COUNTS=32'sd0,
    parameter signed [31:0] DIRECT_POSITION_TARGET_COUNTS=POSITION_TARGET_COUNTS,
    parameter integer POSITION_VELOCITY_FILTER_SHIFT=1,
    parameter integer POSITION_VELOCITY_WINDOW_TICKS=1,
    parameter integer POSITION_VELOCITY_OBSERVER_ENABLE=0,
    parameter integer MIN_ACTIVE_COMMAND=1200,
    parameter integer OUTPUT_RELEASE_STEP=100,
    parameter integer START_COMMAND=2000,
    parameter integer START_TICKS=40
) (
    input wire clk, rst_n, track, enable, start, inner_tick, outer_tick,
    input wire signed [31:0] capture_velocity_q8,
    input wire capture_here,
    input wire [2:0] motion_command,
    input wire [10:0] angle_x2,
    input wire [17:0] angle_q8,
    input wire [17:0] upright_adc_q8,
    input wire signed [31:0] position, run_origin,
    input wire signed [31:0] startup_zero,
    output wire signed [15:0] command,
    // Continuously measured P/D for the top-level soft catch. Normal command
    // still waits for full enable, target capture and accepted pipeline data.
    output wire signed [15:0] tracking_command,
    output wire signed [31:0] correction_q8,
    output wire signed [31:0] angle_target_q8,
    output wire signed [32:0] target_position,
    output wire signed [31:0] position_velocity,
    output wire signed [31:0] angle_delta_q8,
    output wire position_target_locked,
    output wire quiet_servo_active,
    output wire [3:0] mission_phase,
    output wire signed [31:0] extension_goal, extension_anchor,
    output wire [15:0] extension_dwell,
    output wire signed [31:0] position_integral_q16, mission_velocity_q8
);
    wire signed [31:0] position_offset_q8;
    wire signed [31:0] position_target;
    wire signed [32:0] position_error;
    wire signed [31:0] position_velocity_q8;
    wire signed [15:0] pd_command;
    wire pd_command_update, inside_deadband, deadband_confirmed;
    wire signed [31:0] measured_angle_velocity_q8;
    wire signed [31:0] upright_q8=RUNTIME_CAL!=0 ?
        $signed({14'b0,upright_adc_q8}) : UPRIGHT_ADC*256;
    reg signed [31:0] mission_angle_error_q8;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) mission_angle_error_q8<=0;
        else mission_angle_error_q8<=$signed({1'b0,angle_q8})-upright_q8;
    end
    wire home_reference_enable,home_done,home_pause;
    wire post_home_request;
    wire home_owner=RETURN_START_ZERO_ENABLE!=0 && !home_done;
    wire signed [31:0] home_goal,home_anchor;
    wire signed [31:0] home_reference_position,home_reference_velocity;
    wire [3:0] home_phase;
    wire [15:0] home_qualified;
    // Movement tilt is bounded, and actual angle outside +/-6 suppresses
    // position drive while the original angle PD continues to balance.
    wire signed [31:0] home_limited_offset=home_pause ? 32'sd0 :
        position_offset_q8>HOME_MOVE_ANGLE_ADC*256 ? HOME_MOVE_ANGLE_ADC*256 :
        position_offset_q8< -HOME_MOVE_ANGLE_ADC*256 ? -HOME_MOVE_ANGLE_ADC*256 : position_offset_q8;
    // Keep velocity damping when home position pushing pauses. The P/I
    // component is gated inside the position controller, independently of D.
    wire signed [31:0] effective_offset_q8=position_offset_q8;
    wire signed [15:0] limited_combined_pd=pd_command;
    assign quiet_servo_active=1'b0;
    reg [31:0] balance_age;
    reg direct_capture;
    reg balance_age_ready;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) balance_age_ready<=LINEAR_STEADY_DELAY_TICKS==0;
        else if(start || !enable || !track) balance_age_ready<=LINEAR_STEADY_DELAY_TICKS==0;
        else if(inner_tick && balance_age>=LINEAR_STEADY_DELAY_TICKS-1) balance_age_ready<=1;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin balance_age<=0; direct_capture<=0; end
        else if(start) begin balance_age<=0; direct_capture<=(capture_here===1'b1); end
        else if(!enable || !track) begin balance_age<=0; direct_capture<=0; end
        else if(inner_tick && balance_age<LINEAR_STEADY_DELAY_TICKS)
            balance_age<=balance_age+1'b1;
    end
    wire linear_mode=LINEAR_STEADY_ENABLE!=0 && position_target_locked &&
        (direct_capture || balance_age_ready);
    // A pre-catch result can still be in the angle controller's pipeline.
    // Only results launched after target capture may reach the PWM limiter.
    reg [3:0] accepted_update;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) accepted_update<=0;
        else if(!enable || !track || start) accepted_update<=0;
        else accepted_update<={accepted_update[2:0],inner_tick};
    end
    wire [32:0] abs_position_error=position_error[32] ?
        -position_error : position_error;
    wire [31:0] abs_position_velocity=position_velocity_q8[31] ?
        -position_velocity_q8 : position_velocity_q8;
    wire position_needs_action_live=(POSITION_ENABLE!=0) &&
        (((POSITION_KP_Q8!=0 || POSITION_KI_Q16!=0) &&
          abs_position_error>POSITION_TOLERANCE_COUNTS) ||
         (POSITION_KD_Q8!=0 && abs_position_velocity>=256));
    reg position_needs_action;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) position_needs_action<=0;
        else position_needs_action<=position_needs_action_live;
    end

    wire signed [31:0] qualification_velocity_q8;
    wire qualification_valid,qualification_still;
    wire signed [31:0] mission_reference_position, mission_reference_velocity;
    wire mission_reference_enable;
    wire sequence_reference_enable,waypoint_settling;
    wire home_idle_hold=RETURN_START_ZERO_ENABLE!=0 && home_done && !sequence_reference_enable;
    wire signed [31:0] sequence_reference_position,sequence_reference_velocity,sequence_goal,sequence_anchor;
    wire signed [31:0] sequence_reference_accel;
    wire sequence_sine_endpoint_active, sequence_sine_endpoint_positive;
    wire signed [31:0] sequence_sine_endpoint_position;
    wire [3:0] sequence_phase;
    // Only a successful path-to-settling edge requests post-motion homing.
    // Refused commands, timeout holds and ordinary idle holds do not retrigger it.
    reg [3:0] previous_sequence_phase;
    assign post_home_request=POST_MOTION_HOME_ENABLE!=0 && RETURN_START_ZERO_ENABLE!=0 &&
        enable && track && !start && !home_owner && home_done && sequence_phase==4'd5 &&
        (previous_sequence_phase==4'd2 || previous_sequence_phase==4'd4 ||
         previous_sequence_phase==4'd7 || previous_sequence_phase==4'd11);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) previous_sequence_phase<=0;
        else if(!enable || !track || start) previous_sequence_phase<=0;
        else previous_sequence_phase<=sequence_phase;
    end
    wire sine_reference_active=!home_owner && (sequence_phase==4'd11 || sequence_phase==4'd2);
    wire motion_accel_active=!home_owner && (sequence_phase==4'd4 || sequence_phase==4'd7 ||
        sequence_phase==4'd11 || (WAVEPACKET_ENABLE!=0 && sequence_phase==4'd2));
    wire [15:0] sequence_dwell;
    assign mission_reference_enable=home_owner ? home_reference_enable : (sequence_reference_enable || home_idle_hold);
    assign mission_reference_position=home_owner ? home_reference_position : home_idle_hold ? home_goal : sequence_reference_position;
    wire idle_hold_mode=enable && track && !start && !home_owner && motion_command==0 &&
        (sequence_phase==3 || sequence_phase==5 || sequence_phase==8 || sequence_phase==10);
    wire signed [31:0] idle_return_velocity_q8;
    wire idle_return_active;
    idle_recenter #(.ENABLE(IDLE_RECENTER_ENABLE),.MAX_SPEED_Q8(IDLE_RETURN_MAX_SPEED_Q8),
        .START_COUNTS(IDLE_RETURN_START_COUNTS),.STOP_COUNTS(IDLE_RETURN_STOP_COUNTS),
        .ACCEL_Q8(IDLE_RETURN_ACCEL_Q8),.GAIN_Q8(IDLE_RETURN_GAIN_Q8)) idle_return (
        .clk(clk),.rst_n(rst_n),.enable(idle_hold_mode && qualification_valid),.update(outer_tick),
        .target(mission_reference_position),.position(position),
        .angle_error_q8(mission_angle_error_q8),
        .velocity_q8(idle_return_velocity_q8),.active(idle_return_active));
    assign mission_reference_velocity=home_owner ? home_reference_velocity :
        idle_hold_mode ? idle_return_velocity_q8 : home_idle_hold ? 32'sd0 : sequence_reference_velocity;
    assign mission_phase=home_owner ? home_phase : sequence_phase;
    assign extension_goal=(home_owner || home_idle_hold) ? home_goal : sequence_goal;
    assign extension_anchor=home_owner ? home_anchor : sequence_anchor;
    assign extension_dwell=home_owner ? home_qualified : sequence_dwell;
    encoder_quiet_window #(.WINDOW_LOG2(4)) quiet_measurement (
        .clk(clk),.rst_n(rst_n),.enable(enable && track),.capture(start),.update(outer_tick),
        .position(position),.velocity_q8(qualification_velocity_q8),.valid(qualification_valid),.stable_window(qualification_still));
    home_zero_direct #(.COUNTS_PER_REV(ARM_COUNTS_PER_REV),.LOCAL_TICKS(HOME_LOCAL_TICKS),.MOVE_ANGLE_ADC(HOME_MOVE_ANGLE_ADC),
        .RESUME_ANGLE_ADC(HOME_MOVE_ANGLE_ADC>2 ? HOME_MOVE_ANGLE_ADC-2 : 0),
        .MAX_SPEED_Q8(HOME_MAX_SPEED_Q8),.ACCEL_Q8(HOME_ACCEL_Q8),.APPROACH_SHIFT(HOME_APPROACH_SHIFT),
        .ARRIVAL_COUNTS(HOLD_ARRIVAL_COUNTS),.TURN_FAST_START_ENABLE(TURN_FAST_START_ENABLE),
        .TURN_START_COUNTS(TURN_START_COUNTS),.TURN_START_SPEED_Q8(TURN_START_SPEED_Q8),.TURN_START_ANGLE_ADC(TURN_START_ANGLE_ADC),
        .REHOME_COUNTS(HOME_REHOME_COUNTS),.REHOME_TICKS(HOME_REHOME_TICKS),
        .READY_ON_ARRIVAL(HOME_READY_ON_ARRIVAL),.DIRECT_TO_ZERO(HOME_DIRECT_TO_ZERO),
        .HANDOFF_COUNTS(HOME_HANDOFF_COUNTS),.HANDOFF_SPEED_Q8(HOME_HANDOFF_SPEED_Q8),
        .HANDOFF_ANGLE_ADC(HOME_HANDOFF_ANGLE_ADC)) zero_return (
        // Idle holds only: 3 = holding after return, 10 = holding after a move.
        .rehome_allowed(IDLE_RECENTER_ENABLE==0 && (sequence_phase==4'd3 || sequence_phase==4'd10)),
        // A queued K1/K2 uses the same relaxed handoff as after swing-up, also
        // during post-motion homing; motion_command clears when a motion starts.
        .motion_request_pending(motion_command!=0),
        .turn_request_pending(motion_command==1 || motion_command==2),
        .force_return(post_home_request),.force_reference_position(sequence_reference_position),
        .clk(clk),.rst_n(rst_n),.enable(enable && track && RETURN_START_ZERO_ENABLE!=0),
        .capture(start),.update(outer_tick),.velocity_valid(qualification_valid),
        .upright_entry(capture_here),.temp_target_locked(position_target_locked),
        .position(position),.zero_position(startup_zero),.temporary_target(position_target),.velocity_q8(qualification_velocity_q8),
        .angle_error_q8(mission_angle_error_q8),.angle_velocity_q8(measured_angle_velocity_q8),
        .reference_enable(home_reference_enable),.done(home_done),.goal(home_goal),.anchor(home_anchor),
        .reference_position(home_reference_position),.reference_velocity_q8(home_reference_velocity),
        .pause_drive(home_pause),.phase(home_phase),.qualified(home_qualified));
    position_move #(.STEP_COUNTS(EXTENSION_STEP_COUNTS),.DIRECTION(EXTENSION_DIRECTION),
        .COUNTS_PER_REV(ARM_COUNTS_PER_REV),.WAYPOINT_TICKS_PER_30DEG(WAYPOINT_TICKS_PER_30DEG),
        .WAYPOINT_ARRIVAL_COUNTS(WAYPOINT_ARRIVAL_COUNTS),.WAYPOINT_DWELL_TICKS(WAYPOINT_DWELL_TICKS),
        .WAYPOINT_TIMEOUT_TICKS(WAYPOINT_TIMEOUT_TICKS),.WAYPOINT_PASS_THROUGH(WAYPOINT_PASS_THROUGH),
        .WAYPOINT_PASS_COUNTS(WAYPOINT_PASS_COUNTS),
        .FOLLOW_ON_SINE(AUTO_SEQUENCE_ENABLE),.ROUNDTRIP_ENABLE(ROUNDTRIP_ENABLE),.DIRECT_TURN_ENABLE(DIRECT_TURN_ENABLE),.STEP_PERIOD_TICKS(STEP_PERIOD_TICKS),
        .SEQUENCE_DWELL_TICKS(SEQUENCE_DWELL_TICKS),.SMOOTH_STEP_ENABLE(SMOOTH_STEP_ENABLE),.MINJERK_ENABLE(MINJERK_ENABLE),.SINE_ENABLE(SINE_ENABLE),.SINE_AMPLITUDE(SINE_AMPLITUDE),
        .SINE_PERIOD_TICKS(SINE_PERIOD_TICKS),.SINE_CYCLES(SINE_CYCLES),.SINE_DIRECTION(SINE_DIRECTION),
        .WAVEPACKET_ENABLE(WAVEPACKET_ENABLE),.WAVEPACKET_AMPLITUDE(WAVEPACKET_AMPLITUDE),
        .WAVEPACKET_PERIOD_TICKS(WAVEPACKET_PERIOD_TICKS),.WAVEPACKET_CYCLES(WAVEPACKET_CYCLES),
        .WAVEPACKET_DIRECTION(WAVEPACKET_DIRECTION),
        .WAVEPACKET_ENVELOPE_POWER(WAVEPACKET_ENVELOPE_POWER),
        .WAVEPACKET_PROFILE(WAVEPACKET_PROFILE),
        .WAVEPACKET_PEAK_1(WAVEPACKET_PEAK_1),.WAVEPACKET_PEAK_2(WAVEPACKET_PEAK_2),
        .WAVEPACKET_PEAK_3(WAVEPACKET_PEAK_3),.WAVEPACKET_PEAK_4(WAVEPACKET_PEAK_4),
        .INITIAL_QUALIFY_TICKS((SINE_ENABLE!=0 || SMOOTH_STEP_ENABLE!=0) ? 40 : 200),
        .MAX_SPEED_Q16(MOVE_SPEED_LIMIT_Q16),
        .AUTO_ONCE(AUTO_SEQUENCE_ENABLE!=0 && TURN_AUTO_START!=0),.SPEED_WINDOW_Q8(32),.ARRIVAL_TOLERANCE(HOLD_ARRIVAL_COUNTS),
        .READY_TICKS(RETURN_START_ZERO_ENABLE!=0 && HOME_READY_ON_ARRIVAL!=0 ? 1 : 100)) extension (
        .clk(clk),.rst_n(rst_n),.enable(enable && track && EXTENSION_ENABLE!=0 && !home_owner),
        .capture(start),.update(outer_tick),.target_locked(position_target_locked),
        .steady_ready(linear_mode && qualification_valid && qualification_still),.command(motion_command),.position(position),
        .acquired_target(home_idle_hold ? home_goal : position_target),.velocity_q8(qualification_velocity_q8),
        .velocity_valid(qualification_valid),.waypoint_settling(waypoint_settling),
        .angle_error_q8(mission_angle_error_q8),.run_origin(run_origin),
        .override_enable(sequence_reference_enable),.reference_position(sequence_reference_position),
        .reference_velocity_q8(sequence_reference_velocity),.reference_accel_q16(sequence_reference_accel),.phase(sequence_phase),
        .sine_endpoint_active(sequence_sine_endpoint_active),.sine_endpoint_positive(sequence_sine_endpoint_positive),
        .sine_endpoint_position(sequence_sine_endpoint_position),
        .goal(sequence_goal),.anchor(sequence_anchor),.dwell_ticks(sequence_dwell));

    position_pid_controller #(
        .CAPTURE_VELOCITY_SEED_ENABLE(CAPTURE_VELOCITY_SEED_ENABLE),
        .HOLD_REFERENCE_KP_Q8(HOLD_REFERENCE_KP_Q8),.HOLD_REFERENCE_KI_Q16(HOLD_REFERENCE_KI_Q16),
        .TURN_ENDPOINT_BRAKE_ENABLE(TURN_ENDPOINT_BRAKE_ENABLE),.TURN_ENDPOINT_WINDOW_COUNTS(TURN_ENDPOINT_WINDOW_COUNTS),
        .TURN_CW_ENDPOINT_WINDOW_COUNTS(TURN_CW_ENDPOINT_WINDOW_COUNTS),.TURN_CCW_ENDPOINT_WINDOW_COUNTS(TURN_CCW_ENDPOINT_WINDOW_COUNTS),
        .SINE_ENDPOINT_BRAKE_ENABLE(SINE_ENDPOINT_BRAKE_ENABLE),
        .SINE_POS_ENDPOINT_WINDOW_COUNTS(SINE_POS_ENDPOINT_WINDOW_COUNTS),.SINE_NEG_ENDPOINT_WINDOW_COUNTS(SINE_NEG_ENDPOINT_WINDOW_COUNTS),
        .TURN_DIRECTION(EXTENSION_DIRECTION),
        .TURN_CW_BRAKE_STRENGTH_Q8(TURN_CW_BRAKE_STRENGTH_Q8),.TURN_CCW_BRAKE_STRENGTH_Q8(TURN_CCW_BRAKE_STRENGTH_Q8),
        .TURN_BRAKE_MAX_EXTRA_SPEED_Q8(TURN_BRAKE_MAX_EXTRA_SPEED_Q8),
        .IDLE_HOLD_KD_Q8(IDLE_HOLD_KD_Q8),.IDLE_HOLD_P_LIMIT_ADC(IDLE_HOLD_P_LIMIT_ADC),
        .EXTERNAL_REFERENCE_ENABLE(EXTENSION_ENABLE),.REFERENCE_QUIET_FILTER_ENABLE(1),
        .KP_Q8(POSITION_KP_Q8),.KD_Q8(POSITION_KD_Q8),.STEADY_KD_Q8(POSITION_STEADY_KD_Q8),
        .KI_Q16(POSITION_KI_Q16),.I_LIMIT_ADC(POSITION_I_LIMIT_ADC),
        .INTEGRAL_WINDOW_COUNTS(POSITION_INTEGRAL_WINDOW_COUNTS),
        .INTEGRAL_QUIET_TICKS(POSITION_INTEGRAL_QUIET_TICKS),
        .INTEGRAL_VELOCITY_LIMIT_Q8(POSITION_INTEGRAL_VELOCITY_LIMIT_Q8),
        .INTEGRAL_MOVING_DECAY_SHIFT(POSITION_INTEGRAL_MOVING_DECAY_SHIFT),
        .INTEGRAL_DEADBAND_COUNTS(POSITION_INTEGRAL_DEADBAND_COUNTS),
        .INTEGRAL_OVERSHOOT_COUNTS(POSITION_INTEGRAL_OVERSHOOT_COUNTS),
        .INTEGRAL_UNWIND_SHIFT(POSITION_INTEGRAL_UNWIND_SHIFT),
        .REFERENCE_HOLD_I_DEADBAND_COUNTS(REFERENCE_HOLD_I_DEADBAND_COUNTS),
        .REFERENCE_SETTLE_I_DEADBAND_COUNTS(REFERENCE_SETTLE_I_DEADBAND_COUNTS),
        .MAX_OFFSET_ADC(MAX_TARGET_OFFSET_ADC),
        .STEADY_MAX_OFFSET_ADC(STEADY_MAX_TARGET_OFFSET_ADC),
        .REFERENCE_P_LIMIT_ADC(REFERENCE_POSITION_P_LIMIT_ADC),
        .TURN_DISTANCE_BRAKE_ENABLE(TURN_DISTANCE_BRAKE_ENABLE),.TURN_DISTANCE_WINDOW_SHIFT(TURN_DISTANCE_WINDOW_SHIFT),.TURN_DISTANCE_WINDOW_COUNTS(TURN_DISTANCE_WINDOW_COUNTS),
        .TURN_BRAKE_ENABLE(TURN_BRAKE_ENABLE),.TURN_BRAKE_LOOKAHEAD_TICKS(TURN_BRAKE_LOOKAHEAD_TICKS),
        .TURN_BRAKE_MAX_LEAD_COUNTS(TURN_BRAKE_MAX_LEAD_COUNTS),
        .STEADY_P_LIMIT_ADC(POSITION_STEADY_P_LIMIT_ADC),
        .STEADY_P_DEADBAND_COUNTS(POSITION_STEADY_P_DEADBAND_COUNTS),
        .MAX_OFFSET_STEP_Q8(MAX_TARGET_STEP_Q8),
        .TURN_POSITION_STEP_Q8(TURN_POSITION_STEP_Q8),.TURN_LAUNCH_STEP_Q8(TURN_LAUNCH_STEP_Q8),.TURN_LAUNCH_TICKS(TURN_LAUNCH_TICKS),
        .STEADY_POSITION_STEP_Q8(STEADY_POSITION_STEP_Q8),.STEADY_DAMPING_STEP_Q8(STEADY_DAMPING_STEP_Q8),
        .VELOCITY_FILTER_SHIFT(POSITION_VELOCITY_FILTER_SHIFT),
        .VELOCITY_WINDOW_TICKS(POSITION_VELOCITY_WINDOW_TICKS),
        .VELOCITY_OBSERVER_ENABLE(POSITION_VELOCITY_OBSERVER_ENABLE),
        .TARGET_MODE(POSITION_TARGET_MODE),.TARGET_COUNTS(POSITION_TARGET_COUNTS),
        .DIRECT_TARGET_COUNTS(DIRECT_POSITION_TARGET_COUNTS),
        .TURN_MIN_TRAVEL_COUNTS(POSITION_TURN_MIN_TRAVEL_COUNTS),
        .TURN_REVERSE_COUNTS(POSITION_TURN_REVERSE_COUNTS),
        .TURN_REVERSE_TICKS(POSITION_TURN_REVERSE_TICKS),
        .TURN_STOP_TICKS(POSITION_TURN_STOP_TICKS),
        .OUTPUT_POLARITY(POSITION_POLARITY ^ MOTOR_POLARITY),.ACCEL_FF_Q8(ACCEL_FF_Q8),.SINE_ACCEL_FF_Q8(SINE_ACCEL_FF_Q8)
    ) position_controller (
        .clk(clk),.rst_n(rst_n),
        .enable(enable && POSITION_ENABLE!=0),.capture(start),.capture_here(capture_here),
        .capture_velocity_q8(capture_here===1'b1 ? 32'sd0 : capture_velocity_q8),
        .steady_mode(linear_mode),
        .reference_enable(mission_reference_enable),.stationary_hold_active(idle_hold_mode),
        .reference_pause(home_owner && home_pause),.sine_reference(sine_reference_active),
        .home_reference(home_owner && home_reference_enable),.home_p_limit_adc(HOME_MOVE_ANGLE_ADC),
        .turn_brake_active(DIRECT_TURN_ENABLE!=0 && (mission_phase==4 || mission_phase==7)),
        .turn_brake_positive(mission_phase==4 ? EXTENSION_DIRECTION>0 : EXTENSION_DIRECTION<0),
        .turn_brake_velocity_q8(qualification_velocity_q8),
        .turn_endpoint_active(!home_owner && (sequence_phase==4 || sequence_phase==7)),
        .turn_final_position(sequence_goal),
        .sine_endpoint_active(!home_owner && sequence_sine_endpoint_active),
        .sine_endpoint_positive(sequence_sine_endpoint_positive),.sine_endpoint_position(sequence_sine_endpoint_position),
        .reference_measured_slow(qualification_valid && qualification_still && qualification_velocity_q8<=32 && qualification_velocity_q8>=-32),
        .reference_settling(EXTENSION_ENABLE!=0 && (home_idle_hold || waypoint_settling || mission_phase==3 || mission_phase==5 || mission_phase==6 || mission_phase==8 || mission_phase==10)),
        .reference_position(mission_reference_position),
        .reference_velocity_q8(mission_reference_velocity),
        .reference_accel_q16(motion_accel_active ? sequence_reference_accel : 32'sd0),
        .update(outer_tick && enable && POSITION_ENABLE!=0),
        .position(position),.target_offset_q8(position_offset_q8),
        .target_position(position_target),.position_error(position_error),
        .velocity_q8(position_velocity_q8),.target_locked(position_target_locked),
        .integral_output_q16(position_integral_q16)
    );

    balance_pd_controller #(
        .UPRIGHT_ADC_X2(UPRIGHT_ADC*2),.RUNTIME_CAL(RUNTIME_CAL),.UPRIGHT_DITHER_ENABLE(0),.HIGH_RES_ANGLE(HIGH_RES_ANGLE),
        .CONTINUOUS_ENABLE(LINEAR_STEADY_ENABLE),
        .RECOVERY_ENABLE(RECOVERY_ENABLE),.RECOVERY_ENTRY_ADC(RECOVERY_ENTRY_ADC),
        .RECOVERY_EXIT_ADC(RECOVERY_EXIT_ADC),.RECOVERY_QUIET_TICKS(RECOVERY_QUIET_TICKS),
        .ANGLE_DEADBAND_X2(ANGLE_DEADBAND_X2),
        .DEADBAND_CONFIRM_TICKS(DEADBAND_CONFIRM_TICKS),
        .CONTROL_DIV(CONTROL_DIV),.KP(ANGLE_KP),.STEADY_KP(STEADY_ANGLE_KP),.KD(ANGLE_KD),.STEADY_KD(STEADY_ANGLE_KD),
        .KI_Q8(ANGLE_KI_Q8),.INTEGRAL_LIMIT_COMMAND(INTEGRAL_LIMIT_COMMAND),
        .INTEGRAL_WINDOW_ADC(INTEGRAL_WINDOW_ADC),
        .MAX_D_COMMAND(MAX_D_COMMAND),.MAX_COMMAND(MAX_COMMAND),
        .DERIVATIVE_WINDOW_TICKS(DERIVATIVE_WINDOW_TICKS),
        .DERIVATIVE_FILTER_SHIFT(DERIVATIVE_FILTER_SHIFT),
        .MOTOR_POLARITY(MOTOR_POLARITY)
    ) angle_controller (
        .clk(clk),.rst_n(rst_n),.enable(track),.track_only(!enable || start),
        .update(inner_tick && track && !start),.angle_x2(angle_x2),.angle_q8(angle_q8),.upright_adc_q8(upright_adc_q8),
        .continuous_mode(linear_mode),.reference_mode(mission_reference_enable),
        .target_offset_q8(POSITION_ENABLE!=0 ? effective_offset_q8 : 32'sd0),
        .measured_velocity_q8(measured_angle_velocity_q8),
        .command(pd_command),.command_update(pd_command_update),
        .inside_deadband(inside_deadband),.deadband_confirmed(deadband_confirmed)
    );

    balance_pwm_limiter #(
        .MAX_COMMAND(MAX_COMMAND),.MIN_ACTIVE_COMMAND(MIN_ACTIVE_COMMAND),
        .OUTPUT_RELEASE_STEP(OUTPUT_RELEASE_STEP),
        .LINEAR_ENABLE(LINEAR_STEADY_ENABLE),.LINEAR_MAX_STEP(LINEAR_MAX_STEP),
        .LINEAR_BOOST_COMMAND(LINEAR_BOOST_COMMAND),
        .START_COMMAND(START_COMMAND),.START_TICKS(START_TICKS)
    ) output_limiter (
        .clk(clk),.rst_n(rst_n),.enable(enable && track && !start),
        .update(pd_command_update && accepted_update[3]),
        .inside_deadband(inside_deadband),.deadband_confirmed(deadband_confirmed),
        .position_needs_action(position_needs_action),.request(limited_combined_pd),
        .linear_mode(linear_mode),
        .command(command)
    );

    assign correction_q8=effective_offset_q8;
    assign tracking_command=pd_command;
    assign angle_target_q8=upright_q8+effective_offset_q8;
    assign target_position=mission_reference_enable ?
        {mission_reference_position[31],mission_reference_position} : {position_target[31],position_target};
    assign position_velocity=position_velocity_q8;
    assign mission_velocity_q8=mission_reference_velocity;
    assign angle_delta_q8=measured_angle_velocity_q8;
endmodule
