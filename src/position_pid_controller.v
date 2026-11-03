// Jiangxie-style cascade: position PID changes the angle target, never PWM.
// Outer e = target - position; angle target offset = -outer output.
// Q8 gains/output allow fractional ADC targets without integer dead zones.
module position_pid_controller #(
    parameter integer CAPTURE_VELOCITY_SEED_ENABLE=0,
    parameter signed [15:0] KP_Q8 = 16'sd2,
    parameter signed [15:0] REFERENCE_KP_Q8 = 16'sd16,
    parameter signed [15:0] REFERENCE_KI_Q16 = 16'sd1,
    parameter signed [15:0] HOLD_REFERENCE_KP_Q8=REFERENCE_KP_Q8, HOLD_REFERENCE_KI_Q16=REFERENCE_KI_Q16,
    parameter integer REFERENCE_HOLD_I_DEADBAND_COUNTS=6,
    parameter integer REFERENCE_SETTLE_I_DEADBAND_COUNTS=4,
    parameter integer REFERENCE_I_LIMIT_ADC=8, REFERENCE_P_LIMIT_ADC=4,
    parameter integer REFERENCE_DAMPING_STEP_Q8=256,
    parameter integer TURN_ENDPOINT_BRAKE_ENABLE=0, TURN_ENDPOINT_WINDOW_COUNTS=256,
    // CW/CCW name the commanded turn, independently of encoder polarity.
    parameter integer TURN_CW_ENDPOINT_WINDOW_COUNTS=TURN_ENDPOINT_WINDOW_COUNTS,
    parameter integer TURN_CCW_ENDPOINT_WINDOW_COUNTS=TURN_ENDPOINT_WINDOW_COUNTS,
    // Positive/negative select the next sine/waypoint travel direction.
    parameter integer SINE_ENDPOINT_BRAKE_ENABLE=1,
    parameter integer SINE_POS_ENDPOINT_WINDOW_COUNTS=90, SINE_NEG_ENDPOINT_WINDOW_COUNTS=90,
    parameter integer TURN_DIRECTION=1,
    parameter integer TURN_CW_BRAKE_STRENGTH_Q8=0, TURN_CCW_BRAKE_STRENGTH_Q8=0,
    parameter integer TURN_BRAKE_MAX_EXTRA_SPEED_Q8=512,
    parameter integer IDLE_HOLD_KD_Q8=0, IDLE_HOLD_P_LIMIT_ADC=0,
    parameter integer TURN_DISTANCE_BRAKE_ENABLE=0, TURN_DISTANCE_WINDOW_SHIFT=9, TURN_DISTANCE_WINDOW_COUNTS=540,
    parameter integer TURN_BRAKE_ENABLE=0, TURN_BRAKE_LOOKAHEAD_TICKS=64,
    parameter integer TURN_BRAKE_MAX_LEAD_COUNTS=128,
    parameter integer REFERENCE_QUIET_FILTER_ENABLE=0,
    parameter signed [15:0] KD_Q8 = 16'sd64,
    parameter signed [15:0] STEADY_KD_Q8 = 16'sd0,
    parameter signed [15:0] KI_Q16 = 16'sd0,
    parameter integer I_LIMIT_ADC = 2,
    parameter integer INTEGRAL_WINDOW_COUNTS = 0,
    parameter integer INTEGRAL_QUIET_TICKS = 0,
    parameter integer INTEGRAL_VELOCITY_LIMIT_Q8 = 64,
    parameter integer INTEGRAL_MOVING_DECAY_SHIFT = 0,
    parameter integer INTEGRAL_DEADBAND_COUNTS = 0,
    parameter integer INTEGRAL_OVERSHOOT_COUNTS = 0,
    parameter integer INTEGRAL_UNWIND_SHIFT = 0,
    parameter integer MAX_OFFSET_ADC = 6,
    parameter integer STEADY_MAX_OFFSET_ADC = 0,
    parameter integer STEADY_P_LIMIT_ADC = 0,
    parameter integer STEADY_P_DEADBAND_COUNTS = 0,
    parameter integer MAX_OFFSET_STEP_Q8 = 64,
    parameter integer TURN_POSITION_STEP_Q8=0, TURN_LAUNCH_STEP_Q8=0, TURN_LAUNCH_TICKS=40,
    parameter integer STEADY_POSITION_STEP_Q8 = 0,
    parameter integer STEADY_DAMPING_STEP_Q8 = 0,
    parameter integer VELOCITY_FILTER_SHIFT = 1,
    parameter integer VELOCITY_WINDOW_TICKS = 1,
    parameter integer VELOCITY_OBSERVER_ENABLE = 0,
    parameter integer VELOCITY_OBSERVER_ALPHA_SHIFT = 1,
    parameter integer VELOCITY_OBSERVER_BETA_SHIFT = 3,
    parameter integer REFERENCE_OBSERVER_BETA_SHIFT = 4,
    parameter integer TARGET_MODE = 0,
    // Mode 2 acquires the first post-catch turning point, then holds it.
    parameter integer TURN_MIN_TRAVEL_COUNTS = 20,
    parameter integer TURN_REVERSE_COUNTS = 30,
    parameter integer TURN_REVERSE_TICKS = 4,
    parameter integer TURN_STOP_TICKS = 20,
    parameter signed [31:0] TARGET_COUNTS = 32'sd0,
    // Diagnostic direct capture may use a known position step; normal
    // swing-up acquisition keeps TARGET_COUNTS and its original turn logic.
    parameter signed [31:0] DIRECT_TARGET_COUNTS = TARGET_COUNTS,
    parameter integer OUTPUT_POLARITY = 0,
    // Planned-acceleration tilt: offset += accel_q16*ACCEL_FF_Q8/256 (Q8 ADC),
    // added with P so it shares polarity and limits. Reference mode only.
    parameter integer ACCEL_FF_Q8 = 0,
    parameter integer SINE_ACCEL_FF_Q8 = 0,
    parameter integer EXTERNAL_REFERENCE_ENABLE = 0
) (
    input wire clk, rst_n, enable, capture, update,
    input wire capture_here, steady_mode,
    input wire signed [31:0] capture_velocity_q8,
    input wire reference_enable, reference_settling, reference_measured_slow,
    input wire reference_pause,
    input wire sine_reference, stationary_hold_active,
    input wire home_reference,
    input wire signed [31:0] home_p_limit_adc,
    input wire turn_brake_active, turn_brake_positive,
    input wire turn_endpoint_active,
    input wire sine_endpoint_active, sine_endpoint_positive,
    input wire signed [31:0] sine_endpoint_position,
    input wire signed [31:0] turn_final_position,
    input wire signed [31:0] turn_brake_velocity_q8,
    input wire signed [31:0] reference_position, reference_velocity_q8,
    input wire signed [31:0] reference_accel_q16,
    input wire signed [31:0] position,
    output reg signed [31:0] target_offset_q8,
    output reg signed [31:0] target_position,
    output wire signed [32:0] position_error,
    output reg signed [31:0] velocity_q8,
    output reg target_locked,
    output wire signed [31:0] integral_output_q16
);
    reg signed [31:0] previous_position, integral_q16;
    assign integral_output_q16=integral_q16;
    reg signed [47:0] observer_error_q16, observer_velocity_q16;
    localparam integer VELOCITY_N=(VELOCITY_WINDOW_TICKS>0) ? VELOCITY_WINDOW_TICKS : 1;
    reg signed [31:0] velocity_history [0:VELOCITY_N-1];
    reg [31:0] velocity_index;
    integer velocity_i;
    reg signed [32:0] error_hold;
    reg signed [31:0] brake_bias_hold;
    reg signed [31:0] integral_candidate_q16, limited_hold;
    reg signed [48:0] p_hold, d_hold;
    reg signed [47:0] d_product_hold;
    reg signed [31:0] damping_velocity_hold;
    reg signed [15:0] kd_hold;
    reg [2:0] stage;
    reg signed [31:0] turn_origin, turn_extreme;
    reg signed [1:0] turn_direction;
    reg [31:0] turn_stop_count, turn_reverse_count;
    reg [31:0] integral_quiet_count;
    reg separated_active;
    reg integral_clear_hold, integral_commit_hold, integral_saturation_veto;
    reg signed [31:0] position_part_q8, damping_part_q8;
    reg signed [31:0] position_limited_hold;
    // Before target acquisition use velocity damping, without pulling back
    // toward catch or accumulating a position integral.
    assign position_error = (TARGET_MODE == 2 && !target_locked && !reference_active) ? 33'sd0 :
        $signed({target_position[31],target_position}) - $signed({position[31],position});
    wire signed [32:0] from_turn_origin = $signed({position[31],position}) -
                                               $signed({turn_origin[31],turn_origin});
    wire signed [32:0] from_turn_extreme = $signed({position[31],position}) -
                                                $signed({turn_extreme[31],turn_extreme});
    wire advancing_extreme = (turn_direction == 1 && position > turn_extreme) ||
                             (turn_direction == -1 && position < turn_extreme);
    wire turn_reversed = (turn_direction == 1 && from_turn_extreme <= -TURN_REVERSE_COUNTS) ||
                         (turn_direction == -1 && from_turn_extreme >= TURN_REVERSE_COUNTS);
    // Only a plateau without a new extreme can count as stopped. Tolerate
    // one encoder count of jitter; continued creeping resets the timer.
    wire at_turn_stop = from_turn_extreme >= -33'sd1 && from_turn_extreme <= 33'sd1;
    wire signed [32:0] delta = $signed({position[31],position}) -
                              $signed({previous_position[31],previous_position});
    wire signed [31:0] velocity_reference=VELOCITY_N>1 ? velocity_history[velocity_index] : previous_position;
    wire signed [32:0] window_delta=$signed({position[31],position})-$signed({velocity_reference[31],velocity_reference});
    wire moving_reverse = (turn_direction == 1 && delta < 0) ||
                          (turn_direction == -1 && delta > 0);
    wire reverse_confirmed = turn_reversed && moving_reverse &&
                             turn_reverse_count >= TURN_REVERSE_TICKS-1;
    // Clamp an impossible per-update delta before Q8 conversion, not a stop.
    wire signed [31:0] bounded_delta = (window_delta > 32767) ? 32'sd32767 :
        (window_delta < -32767) ? -32'sd32767 : {{16{window_delta[15]}},window_delta[15:0]};
    wire signed [31:0] window_step_q8 = bounded_delta <<< 8;
    wire signed [31:0] step_q8 = window_step_q8/VELOCITY_N;
    wire signed [31:0] velocity_filtered = velocity_q8 +
        ((step_q8 - velocity_q8) >>> VELOCITY_FILTER_SHIFT);
    wire signed [31:0] velocity_next = (step_q8 == 0 &&
        velocity_q8 < (1 << VELOCITY_FILTER_SHIFT) && velocity_q8 > -(1 << VELOCITY_FILTER_SHIFT)) ?
        32'sd0 : velocity_filtered;
    // Alpha-beta tracking estimator. Track only prediction error relative
    // to the live encoder, so an absolute 32-bit count does not get shifted
    // into a narrow fixed-point position register. Velocity units remain
    // counts per outer update, matching the existing window estimator.
    reg signed [31:0] observer_delta;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) observer_delta<=0;
        else observer_delta<=(delta>32767) ? 32'sd32767 : (delta< -32767) ? -32'sd32767 : delta[31:0];
    end
    wire signed [47:0] observer_delta_q16=$signed(observer_delta)*48'sd65536;
    wire signed [47:0] observer_prediction=observer_error_q16+observer_velocity_q16-observer_delta_q16;
    wire signed [47:0] observer_error_next=observer_prediction-(observer_prediction >>> VELOCITY_OBSERVER_ALPHA_SHIFT);
    // Reuse the same observer. Only after the extension reference starts,
    // use the smaller existing beta correction (1/16 instead of1/8).
    // Non-reference acquisition keeps the original coefficient.
    wire signed [47:0] observer_beta_correction=reference_active ?
        (observer_prediction >>> REFERENCE_OBSERVER_BETA_SHIFT) :
        (observer_prediction >>> VELOCITY_OBSERVER_BETA_SHIFT);
    wire signed [47:0] observer_velocity_next=observer_velocity_q16-observer_beta_correction;
    wire signed [47:0] observer_output_q8=observer_velocity_next >>> 8;
    // Remove only small steady position pull, continuously at both edges.
    // Keep measured-speed damping active even while the position P is zero.
    // Continuous position feedback through the same angle-target path.
    // Arrival tolerance qualifies the result; it does not switch P off.
    wire signed [32:0] p_error_request = (steady_mode && !reference_active && STEADY_P_DEADBAND_COUNTS>0) ?
        ((error_hold>STEADY_P_DEADBAND_COUNTS) ? error_hold-STEADY_P_DEADBAND_COUNTS :
         (error_hold< -STEADY_P_DEADBAND_COUNTS) ? error_hold+STEADY_P_DEADBAND_COUNTS : 33'sd0) : error_hold;
    // Split the P path across existing stages 6/7; stage 1 still commits
    // exactly the same outer-loop sample, without another 5 ms tick of delay.
    reg signed [32:0] p_error_hold;
    reg signed [33:0] brake_p_error;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin p_error_hold<=0;brake_p_error<=0;end
        else begin
            if(stage==6) p_error_hold<=p_error_request;
            if(stage==7) brake_p_error<=$signed({p_error_hold[32],p_error_hold})-
                $signed({{2{brake_bias_hold[31]}},brake_bias_hold});
        end
    end
    wire signed [48:0] baseline_p_term=p_error_hold * KP_Q8;
    // Fixed target remains unchanged. Only the direct-turn P error anticipates
    // near-end travel. Window is256 counts; weight ramps0..1 toward the goal.
    // Use the80ms measured displacement instead of a noisy single encoder tick.
    wire brake_enabled=TURN_BRAKE_ENABLE!=0 && reference_enable===1'b1 && turn_brake_active===1'b1;
    reg signed [32:0] brake_remaining, brake_toward_velocity;
    reg [8:0] brake_weight;
    reg signed [48:0] brake_prediction_q8;
    reg [31:0] brake_lead_q8;
    reg [40:0] brake_weighted_q16;
    reg signed [31:0] brake_bias_counts;
    wire [32:0] brake_rounded=(brake_weighted_q16+41'd32768) >> 16;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            brake_remaining<=0; brake_toward_velocity<=0; brake_weight<=0;
            brake_prediction_q8<=0; brake_lead_q8<=0; brake_weighted_q16<=0; brake_bias_counts<=0;
        end else begin
            brake_remaining<=turn_brake_positive ? position_error : -position_error;
            brake_toward_velocity<=turn_brake_positive ?
                $signed({turn_brake_velocity_q8[31],turn_brake_velocity_q8}) : -$signed({turn_brake_velocity_q8[31],turn_brake_velocity_q8});
            brake_weight<=!brake_enabled || brake_remaining>=256 ? 9'd0 :
                brake_remaining<=0 ? 9'd256 : 9'd256-brake_remaining[8:0];
            brake_prediction_q8<=brake_toward_velocity*TURN_BRAKE_LOOKAHEAD_TICKS;
            brake_lead_q8<=!brake_enabled || brake_prediction_q8<=0 ? 32'd0 :
                brake_prediction_q8>TURN_BRAKE_MAX_LEAD_COUNTS*256 ? TURN_BRAKE_MAX_LEAD_COUNTS*256 : brake_prediction_q8[31:0];
            brake_weighted_q16<=brake_lead_q8*brake_weight;
            brake_bias_counts<=!brake_enabled ? 32'sd0 :
                turn_brake_positive ? $signed(brake_rounded[31:0]) : -$signed(brake_rounded[31:0]);
        end
    end

    // Register the slow-reference decision before the P/I gain muxes.
    // Stage 6 preserves the original D/feedforward sample instant while
    // allowing this decision to settle before P/I arithmetic in stage 1.
    reg slow_reference_hold;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) slow_reference_hold<=0;
        else slow_reference_hold<=stationary_hold_active===1'b1 || (reference_settling===1'b1 && reference_measured_slow===1'b1 && reference_velocity_q8==0);
    end
    wire signed [49:0] moving_reference_p=brake_p_error * REFERENCE_KP_Q8;
    wire signed [49:0] holding_reference_p=brake_p_error * HOLD_REFERENCE_KP_Q8;
    wire signed [49:0] reference_p_term=slow_reference_hold ? holding_reference_p : moving_reference_p;
    wire signed [48:0] p_term=reference_active ? reference_p_term : baseline_p_term;
    // Bound only position pull. Preserve velocity damping and the total
    // tilt limit so stronger local stiffness does not create a large return.
    // Distance taper only: last256 encoder counts (85.33deg).
    // No elapsed-time or velocity prediction determines this P ceiling.
    // Keep actual-speed D and the fixed final target unchanged.
    wire distance_brake_active=TURN_DISTANCE_BRAKE_ENABLE!=0 && reference_active && turn_brake_active===1'b1;
    localparam integer DISTANCE_WINDOW=TURN_DISTANCE_WINDOW_COUNTS;
    wire [32:0] live_distance_remaining=position_error[32] ? -position_error : position_error;
    reg [9:0] distance_bounded;
    reg [23:0] distance_product;
    reg signed [31:0] distance_limit_live, distance_limit_hold;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin distance_bounded<=0;distance_product<=0;distance_limit_live<=0;end
        else begin
            distance_bounded<=live_distance_remaining>=DISTANCE_WINDOW ? DISTANCE_WINDOW : live_distance_remaining[9:0];
            distance_product<=distance_bounded*REFERENCE_P_LIMIT_ADC;
            distance_limit_live<=(distance_product*32'd256) / DISTANCE_WINDOW;
        end
    end
    wire signed [31:0] distance_p_limit_q8=distance_limit_hold;
    wire signed [48:0] active_p_limit_q8=reference_active ?
        (stationary_hold_active===1'b1 && IDLE_HOLD_P_LIMIT_ADC>0 ? IDLE_HOLD_P_LIMIT_ADC*256 : home_reference===1'b1 ? home_p_limit_adc*256 :
        (distance_brake_active ? distance_p_limit_q8 : REFERENCE_P_LIMIT_ADC*256)) : STEADY_P_LIMIT_ADC*256;
    wire signed [48:0] p_bounded = (STEADY_P_LIMIT_ADC>0 && steady_mode) ?
        ((p_term>active_p_limit_q8) ? active_p_limit_q8 :
         (p_term< -active_p_limit_q8) ? -active_p_limit_q8 : p_term) : p_term;
    wire signed [15:0] active_kd = stationary_hold_active===1'b1 && IDLE_HOLD_KD_Q8>0 ? IDLE_HOLD_KD_Q8 : (steady_mode && STEADY_KD_Q8>0) ? STEADY_KD_Q8 : KD_Q8;
    wire reference_active=EXTERNAL_REFERENCE_ENABLE!=0 && reference_enable===1'b1;
    // Taper the velocity target in D using measured distance to the FINAL
    // endpoint. Keep the position trajectory intact. Divisors are constants.
    localparam integer CW_WINDOW=TURN_CW_ENDPOINT_WINDOW_COUNTS>0 ? TURN_CW_ENDPOINT_WINDOW_COUNTS : 1;
    localparam integer CCW_WINDOW=TURN_CCW_ENDPOINT_WINDOW_COUNTS>0 ? TURN_CCW_ENDPOINT_WINDOW_COUNTS : 1;
    localparam integer POS_WINDOW=SINE_POS_ENDPOINT_WINDOW_COUNTS>0 ? SINE_POS_ENDPOINT_WINDOW_COUNTS : 1;
    localparam integer NEG_WINDOW=SINE_NEG_ENDPOINT_WINDOW_COUNTS>0 ? SINE_NEG_ENDPOINT_WINDOW_COUNTS : 1;
    wire endpoint_sine=sine_endpoint_active===1'b1;
    wire endpoint_positive=endpoint_sine ? sine_endpoint_positive : turn_brake_positive;
    wire endpoint_cw=turn_brake_positive==(TURN_DIRECTION>0);
    wire signed [31:0] endpoint_goal=endpoint_sine ? sine_endpoint_position : turn_final_position;
    wire [31:0] endpoint_window=endpoint_sine ? (endpoint_positive ? POS_WINDOW : NEG_WINDOW) :
        (endpoint_cw ? CW_WINDOW : CCW_WINDOW);
    wire endpoint_window_enabled=endpoint_sine ? (endpoint_positive ? SINE_POS_ENDPOINT_WINDOW_COUNTS>0 : SINE_NEG_ENDPOINT_WINDOW_COUNTS>0) :
        (endpoint_cw ? TURN_CW_ENDPOINT_WINDOW_COUNTS>0 : TURN_CCW_ENDPOINT_WINDOW_COUNTS>0);
    reg signed [32:0] endpoint_remaining;
    reg [31:0] endpoint_bounded;
    reg [8:0] endpoint_weight_q8;
    reg signed [41:0] endpoint_velocity_product;
    reg endpoint_gate;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin endpoint_remaining<=0;endpoint_bounded<=0;endpoint_weight_q8<=256;endpoint_velocity_product<=0;endpoint_gate<=0;end
        else begin
            endpoint_gate<=reference_active && endpoint_window_enabled &&
                (endpoint_sine ? SINE_ENDPOINT_BRAKE_ENABLE!=0 : (TURN_ENDPOINT_BRAKE_ENABLE!=0 && turn_endpoint_active===1'b1));
            endpoint_remaining<=endpoint_positive ?
                $signed({endpoint_goal[31],endpoint_goal})-$signed({position[31],position}) :
                $signed({position[31],position})-$signed({endpoint_goal[31],endpoint_goal});
            endpoint_bounded<=endpoint_remaining<=0 ? 0 : endpoint_remaining>=endpoint_window ? endpoint_window : endpoint_remaining[31:0];
            endpoint_weight_q8<=endpoint_sine ?
                (endpoint_positive ? (endpoint_bounded*32'd256)/POS_WINDOW : (endpoint_bounded*32'd256)/NEG_WINDOW) :
                (endpoint_cw ? (endpoint_bounded*32'd256)/CW_WINDOW : (endpoint_bounded*32'd256)/CCW_WINDOW);
            endpoint_velocity_product<=$signed(reference_velocity_q8)*$signed({1'b0,endpoint_weight_q8});
        end
    end
    // Never suppress an opposite-direction correction after an overshoot.
    wire reference_toward_goal=endpoint_positive ? reference_velocity_q8>=0 : reference_velocity_q8<=0;
    wire signed [31:0] effective_reference_velocity_q8=endpoint_gate && reference_toward_goal ?
        endpoint_velocity_product >>> 8 : reference_velocity_q8;
    // Additional turn-only braking from measured velocity. Separate constant
    // products keep the CW/CCW strength mux out of the multiplier input path.
    wire turn_toward_endpoint=turn_brake_positive ? velocity_q8>0 : velocity_q8<0;
    wire turn_extra_active=endpoint_gate && !endpoint_sine && turn_endpoint_active===1'b1 && turn_toward_endpoint;
    reg signed [63:0] turn_cw_gain_product,turn_ccw_gain_product;
    reg signed [31:0] turn_gain_velocity;
    reg signed [41:0] turn_weighted_velocity;
    reg signed [31:0] turn_extra_velocity;
    wire signed [41:0] turn_extra_scaled=turn_weighted_velocity >>> 8;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin turn_cw_gain_product<=0;turn_ccw_gain_product<=0;turn_gain_velocity<=0;turn_weighted_velocity<=0;turn_extra_velocity<=0;end
        else begin
            turn_cw_gain_product<=$signed(velocity_q8)*TURN_CW_BRAKE_STRENGTH_Q8;
            turn_ccw_gain_product<=$signed(velocity_q8)*TURN_CCW_BRAKE_STRENGTH_Q8;
            turn_gain_velocity<=turn_extra_active ? (endpoint_cw ? turn_cw_gain_product >>> 8 : turn_ccw_gain_product >>> 8) : 32'sd0;
            turn_weighted_velocity<=$signed(turn_gain_velocity)*$signed({1'b0,(9'd256-endpoint_weight_q8)});
            turn_extra_velocity<=turn_extra_scaled>TURN_BRAKE_MAX_EXTRA_SPEED_Q8 ? TURN_BRAKE_MAX_EXTRA_SPEED_Q8 :
                turn_extra_scaled< -TURN_BRAKE_MAX_EXTRA_SPEED_Q8 ? -TURN_BRAKE_MAX_EXTRA_SPEED_Q8 : turn_extra_scaled;
        end
    end
    reg signed [31:0] reference_damping_hold,measured_damping_velocity,turn_extra_applied;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin reference_damping_hold<=0;measured_damping_velocity<=0;turn_extra_applied<=0;end
        else begin
            reference_damping_hold<=reference_active ? effective_reference_velocity_q8 : 32'sd0;
            turn_extra_applied<=turn_extra_active ? turn_extra_velocity : 32'sd0;
            measured_damping_velocity<=velocity_q8+turn_extra_applied;
        end
    end
    wire signed [31:0] damping_velocity=measured_damping_velocity-reference_damping_hold;
    wire signed [47:0] d_product = damping_velocity_hold * kd_hold;
    wire signed [48:0] d_term = -(d_product_hold[47] ? -((-d_product_hold) >>> 8) : (d_product_hold >>> 8));
    wire signed [48:0] baseline_i_product=error_hold * KI_Q16;
    wire signed [48:0] moving_reference_i=error_hold * REFERENCE_KI_Q16;
    wire signed [48:0] holding_reference_i=error_hold * HOLD_REFERENCE_KI_Q16;
    wire signed [48:0] reference_i_product=slow_reference_hold ? holding_reference_i : moving_reference_i;
    wire signed [48:0] i_product=reference_active ? reference_i_product : baseline_i_product;
    wire active_i=reference_active ? (slow_reference_hold ? HOLD_REFERENCE_KI_Q16!=0 : REFERENCE_KI_Q16!=0) : KI_Q16!=0;
    // Retain the full Q16 increment: small count errors with KI=1 must not
    // disappear in a per-update Q8 right shift.
    wire signed [49:0] i_sum = $signed(integral_q16) + i_product;
    wire integral_allowed = INTEGRAL_WINDOW_COUNTS <= 0 ||
        (error_hold <= INTEGRAL_WINDOW_COUNTS && error_hold >= -INTEGRAL_WINDOW_COUNTS);
    wire integral_slow = velocity_q8<=INTEGRAL_VELOCITY_LIMIT_Q8 &&
        velocity_q8>=-INTEGRAL_VELOCITY_LIMIT_Q8;
    wire integral_ready = steady_mode && integral_slow &&
        integral_quiet_count>=INTEGRAL_QUIET_TICKS;
    wire integral_centered = INTEGRAL_DEADBAND_COUNTS>0 &&
        error_hold<=INTEGRAL_DEADBAND_COUNTS && error_hold>=-INTEGRAL_DEADBAND_COUNTS;
    wire [31:0] integral_magnitude = integral_q16[31] ? -integral_q16 : integral_q16;
    wire [31:0] decay_fraction = integral_magnitude >> INTEGRAL_MOVING_DECAY_SHIFT;
    wire [31:0] decay_step = decay_fraction!=0 ? decay_fraction :
        (integral_magnitude!=0 ? 32'd1 : 32'd0);
    wire signed [31:0] decayed_integral = integral_q16[31] ?
        integral_q16+$signed(decay_step) : integral_q16-$signed(decay_step);
    // Remove stored drive only after a meaningful overshoot. Retain the
    // learned static bias inside the tolerance band. Applied P/I still goes
    // through the normal slew limiter; this never changes PWM directly.
    wire integral_overshot = steady_mode && INTEGRAL_OVERSHOOT_COUNTS>0 &&
        INTEGRAL_UNWIND_SHIFT>0 &&
        ((integral_q16>0 && error_hold< -INTEGRAL_OVERSHOOT_COUNTS) ||
         (integral_q16<0 && error_hold>INTEGRAL_OVERSHOOT_COUNTS));
    wire [31:0] unwind_fraction=integral_magnitude >> INTEGRAL_UNWIND_SHIFT;
    wire [31:0] unwind_step=unwind_fraction!=0 ? unwind_fraction :
        (integral_magnitude!=0 ? 32'd1 : 32'd0);
    wire signed [31:0] unwound_integral=integral_q16[31] ?
        integral_q16+$signed(unwind_step) : integral_q16-$signed(unwind_step);
    // Standard bounded tracking integral: an opposite error subtracts normally.
    // Do not erase a learned equilibrium bias abruptly at an arrival-band edge.
    wire signed [31:0] active_reference_i_deadband=
        (reference_settling===1'b1 && reference_measured_slow===1'b1) ?
        REFERENCE_SETTLE_I_DEADBAND_COUNTS : REFERENCE_HOLD_I_DEADBAND_COUNTS;
    wire signed [31:0] reference_i_next=
        reference_pause===1'b1 ? integral_q16 :
        (error_hold>80 || error_hold< -80 ||
         (error_hold<=active_reference_i_deadband && error_hold>=-active_reference_i_deadband)) ? integral_q16 :
        i_sum>REFERENCE_I_LIMIT_ADC*65536 ? REFERENCE_I_LIMIT_ADC*65536 :
        i_sum< -REFERENCE_I_LIMIT_ADC*65536 ? -REFERENCE_I_LIMIT_ADC*65536 : i_sum[31:0];
    wire signed [31:0] i_next = (!active_i ||
                               (TARGET_MODE == 2 && !target_locked)) ? 32'sd0 :
        reference_active ? reference_i_next :
        integral_overshot ? unwound_integral :
        (INTEGRAL_MOVING_DECAY_SHIFT>0 && steady_mode && !integral_slow) ? decayed_integral :
        integral_centered ? integral_q16 :
        (INTEGRAL_QUIET_TICKS>0 && (!integral_ready || !integral_allowed)) ? integral_q16 :
        !integral_allowed ? 32'sd0 :
        (i_sum > I_LIMIT_ADC * 65536) ? I_LIMIT_ADC * 65536 :
        (i_sum < -I_LIMIT_ADC * 65536) ? -I_LIMIT_ADC * 65536 : i_sum[31:0];
    wire signed [31:0] i_output_q8 = integral_candidate_q16[31] ?
        -((-$signed(integral_candidate_q16)) >>> 8) : (integral_candidate_q16 >>> 8);
    // Multiply by constant gains separately; select the motion family explicitly.
    wire use_sine_ff=sine_reference===1'b1;
    wire signed [47:0] turn_accel_ff_product=$signed(reference_accel_q16)*ACCEL_FF_Q8;
    wire signed [47:0] sine_accel_ff_product=$signed(reference_accel_q16)*SINE_ACCEL_FF_Q8;
    wire signed [47:0] accel_ff_product=use_sine_ff ? sine_accel_ff_product : turn_accel_ff_product;
    reg signed [31:0] accel_ff_hold;
    wire signed [49:0] outer_sum = $signed(p_hold) + $signed(d_hold) + $signed(i_output_q8) + $signed(accel_ff_hold);
    reg signed [49:0] outer_sum_pipeline,position_sum_pipeline;
    wire signed [49:0] offset_request = OUTPUT_POLARITY ? outer_sum_pipeline : -outer_sum_pipeline;
    wire signed [31:0] active_limit_q8 = (STEADY_MAX_OFFSET_ADC>0 && steady_mode) ?
        STEADY_MAX_OFFSET_ADC*256 : MAX_OFFSET_ADC*256;
    wire signed [31:0] limited_offset = (offset_request > active_limit_q8) ? active_limit_q8 :
        (offset_request < -active_limit_q8) ? -active_limit_q8 : offset_request[31:0];
    wire signed [32:0] offset_step = $signed({limited_hold[31],limited_hold}) -
                                   $signed({target_offset_q8[31],target_offset_q8});
    wire signed [32:0] active_damping_step=reference_active ? REFERENCE_DAMPING_STEP_Q8 : STEADY_DAMPING_STEP_Q8;
    reg [31:0] turn_launch_age;
    wire turn_launch_active=TURN_LAUNCH_STEP_Q8>0 && reference_active && turn_brake_active===1'b1;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) turn_launch_age<=0;
        else if(!enable || capture || !turn_launch_active) turn_launch_age<=0;
        else if(update && turn_launch_age<TURN_LAUNCH_TICKS) turn_launch_age<=turn_launch_age+1'b1;
    end
    // Boost only the initial slew toward a distant direct-turn target.
    // Both CW/CCW re-arm during the intervening dwell. No endpoint/sine boost.
    wire signed [32:0] active_position_step=(turn_launch_active && turn_launch_age<TURN_LAUNCH_TICKS &&
        (error_hold>TURN_DISTANCE_WINDOW_COUNTS || error_hold< -TURN_DISTANCE_WINDOW_COUNTS)) ?
        TURN_LAUNCH_STEP_Q8 :
        (reference_active && turn_brake_active===1'b1 && TURN_POSITION_STEP_Q8>0) ? TURN_POSITION_STEP_Q8 : STEADY_POSITION_STEP_Q8;
    wire separate_slew=steady_mode && STEADY_POSITION_STEP_Q8>0 && STEADY_DAMPING_STEP_Q8>0;
    wire signed [49:0] position_sum=$signed(p_hold)+$signed(i_output_q8)+$signed(accel_ff_hold);
    wire signed [49:0] position_request=(home_reference===1'b1 && reference_pause===1'b1) ? 50'sd0 :
        OUTPUT_POLARITY ? position_sum_pipeline : -position_sum_pipeline;
    wire signed [48:0] damping_request=OUTPUT_POLARITY ? d_hold : -d_hold;
    wire signed [31:0] position_limit_q8=home_reference===1'b1 ? home_p_limit_adc*256 : active_limit_q8;
    wire signed [31:0] position_limited=(position_request>position_limit_q8) ? position_limit_q8 :
        (position_request< -position_limit_q8) ? -position_limit_q8 : position_request[31:0];
    wire signed [31:0] damping_limited=(damping_request>active_limit_q8) ? active_limit_q8 :
        (damping_request< -active_limit_q8) ? -active_limit_q8 : damping_request[31:0];
    wire signed [32:0] position_step=$signed({position_limited_hold[31],position_limited_hold})-$signed({position_part_q8[31],position_part_q8});
    wire signed [32:0] damping_step=$signed({damping_limited[31],damping_limited})-$signed({damping_part_q8[31],damping_part_q8});
    wire signed [32:0] separated_sum=$signed({position_part_q8[31],position_part_q8})+$signed({damping_part_q8[31],damping_part_q8});
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            observer_error_q16<=0; observer_velocity_q16<=0;
            target_offset_q8 <= 0; target_position <= 0; previous_position <= 0;
            velocity_q8 <= 0; integral_q16 <= 0; integral_candidate_q16 <= 0;
            error_hold <= 0; accel_ff_hold <= 0; brake_bias_hold<=0; distance_limit_hold<=0; outer_sum_pipeline<=0; position_sum_pipeline<=0; p_hold <= 0; d_hold <= 0; limited_hold <= 0; stage <= 0;
            kd_hold <= KD_Q8;
            d_product_hold <= 0; damping_velocity_hold<=0;
            target_locked <= (TARGET_MODE != 2); turn_origin <= 0; turn_extreme <= 0;
            turn_direction <= 0; turn_stop_count <= 0; turn_reverse_count <= 0;
            integral_quiet_count <= 0;
            integral_clear_hold<=0; integral_commit_hold<=0; integral_saturation_veto<=0;
            separated_active<=0; position_part_q8<=0; damping_part_q8<=0; position_limited_hold<=0;
            velocity_index<=0;
            for(velocity_i=0;velocity_i<VELOCITY_N;velocity_i=velocity_i+1) velocity_history[velocity_i]<=0;
        end else if (capture) begin
            observer_error_q16<=0;
            observer_velocity_q16<=CAPTURE_VELOCITY_SEED_ENABLE!=0 ? $signed(capture_velocity_q8)*48'sd256 : 48'sd0;
            target_position <= (TARGET_MODE == 1) ? TARGET_COUNTS :
                (capture_here===1'b1) ? position+DIRECT_TARGET_COUNTS : position+TARGET_COUNTS;
            previous_position <= position;
            velocity_q8 <= CAPTURE_VELOCITY_SEED_ENABLE!=0 ? capture_velocity_q8 : 32'sd0;
            integral_q16 <= 0;
            integral_candidate_q16 <= 0; target_offset_q8 <= 0; stage <= 0; brake_bias_hold<=0; distance_limit_hold<=0; outer_sum_pipeline<=0; position_sum_pipeline<=0;
            kd_hold <= KD_Q8;
            target_locked <= (TARGET_MODE != 2 || capture_here===1'b1); turn_origin <= position; turn_extreme <= position;
            turn_direction <= 0; turn_stop_count <= 0; turn_reverse_count <= 0;
            integral_quiet_count <= 0;
            integral_clear_hold<=0; integral_commit_hold<=0; integral_saturation_veto<=0;
            separated_active<=0; position_part_q8<=0; damping_part_q8<=0; position_limited_hold<=0;
            velocity_index<=0;
            for(velocity_i=0;velocity_i<VELOCITY_N;velocity_i=velocity_i+1) velocity_history[velocity_i]<=position;
        end else if (!enable) begin
            observer_error_q16<=0; observer_velocity_q16<=0;
            previous_position <= position; velocity_q8 <= 0; integral_q16 <= 0;
            integral_candidate_q16 <= 0; target_offset_q8 <= 0; stage <= 0; brake_bias_hold<=0; distance_limit_hold<=0; outer_sum_pipeline<=0; position_sum_pipeline<=0;
            kd_hold <= KD_Q8;
            target_locked <= (TARGET_MODE != 2); turn_origin <= position; turn_extreme <= position;
            turn_direction <= 0; turn_stop_count <= 0; turn_reverse_count <= 0;
            integral_quiet_count <= 0;
            integral_clear_hold<=0; integral_commit_hold<=0; integral_saturation_veto<=0;
            separated_active<=0; position_part_q8<=0; damping_part_q8<=0; position_limited_hold<=0;
            velocity_index<=0;
            for(velocity_i=0;velocity_i<VELOCITY_N;velocity_i=velocity_i+1) velocity_history[velocity_i]<=position;
        end else if (update) begin
            if(reference_active) begin target_position<=reference_position;target_locked<=1;end
            if(!steady_mode || !integral_slow || !integral_allowed)
                integral_quiet_count<=0;
            else if(integral_quiet_count<INTEGRAL_QUIET_TICKS)
                integral_quiet_count<=integral_quiet_count+1'b1;
            previous_position <= position;
            observer_error_q16<=observer_error_next;
            observer_velocity_q16<=observer_velocity_next;
            velocity_q8 <= VELOCITY_OBSERVER_ENABLE!=0 ? observer_output_q8[31:0] : velocity_next;
            velocity_history[velocity_index]<=position;
            velocity_index<=(velocity_index==VELOCITY_N-1) ? 32'd0 : velocity_index+1'b1;
            kd_hold <= active_kd;
            error_hold <= position_error; distance_limit_hold<=distance_limit_live; brake_bias_hold<=brake_enabled ? brake_bias_counts : 32'sd0; stage <= 6;
            if (TARGET_MODE == 2 && !target_locked && !reference_active) begin
                // Require consecutive strictly reverse updates as well as
                // sufficient retreat. A pause, forward step or new extreme
                // cancels the candidate; do not count elapsed time as motion.
                if (moving_reverse && !advancing_extreme) begin
                    if (turn_reverse_count < TURN_REVERSE_TICKS)
                        turn_reverse_count <= turn_reverse_count + 1'b1;
                end else turn_reverse_count <= 0;
                if (turn_direction == 0) begin
                    // Establish direction by actual departure, not one noisy
                    // increment immediately after catch.
                    turn_extreme <= position;
                    target_position <= position + TARGET_COUNTS;
                    turn_stop_count <= 0;
                    if (from_turn_origin >= TURN_MIN_TRAVEL_COUNTS) turn_direction <= 1;
                    else if (from_turn_origin <= -TURN_MIN_TRAVEL_COUNTS) turn_direction <= -1;
                end else if (advancing_extreme) begin
                    turn_extreme <= position;
                    target_position <= position + TARGET_COUNTS;
                    turn_stop_count <= 0;
                end else if (reverse_confirmed || (at_turn_stop && turn_stop_count >= TURN_STOP_TICKS-1)) begin
                    // Lock the point BEFORE reversal, rather than the sample
                    // already moving away. Preserve D/filter/slew state.
                    target_position <= turn_extreme + TARGET_COUNTS;
                    target_locked <= 1;
                    turn_stop_count <= 0;
                    turn_reverse_count <= 0;
                end else if (at_turn_stop) begin
                    turn_stop_count <= turn_stop_count + 1'b1;
                end else turn_stop_count <= 0;
            end
        end else begin
            case (stage)
                6: begin
                    accel_ff_hold <= (reference_active && (use_sine_ff ? SINE_ACCEL_FF_Q8!=0 : ACCEL_FF_Q8!=0)) ? accel_ff_product >>> 8 : 32'sd0;
                    stage <= 7; end
                7: begin damping_velocity_hold<=damping_velocity; stage<=1;end
                1: begin p_hold <= p_bounded; d_product_hold<=d_product; integral_candidate_q16 <= i_next; stage <= 2; end
                2: begin d_hold <= d_term; position_sum_pipeline<=position_sum; stage <= 5; end
                5: begin outer_sum_pipeline<=outer_sum; position_limited_hold<=position_limited; stage<=3;end
                3: begin
                    limited_hold <= limited_offset;
                    separated_active<=separate_slew;
                    if(!separate_slew || !separated_active) begin
                        position_part_q8<=target_offset_q8; damping_part_q8<=0;
                    end else begin
                        if(position_step>active_position_step) position_part_q8<=position_part_q8+active_position_step;
                        else if(position_step< -active_position_step) position_part_q8<=position_part_q8-active_position_step;
                        else position_part_q8<=position_limited_hold;
                        if(damping_step>active_damping_step) damping_part_q8<=damping_part_q8+active_damping_step;
                        else if(damping_step< -active_damping_step) damping_part_q8<=damping_part_q8-active_damping_step;
                        else damping_part_q8<=damping_limited;
                    end
                    // Register the existing anti-windup decision before its
                    // 32-bit write mux; numerical updates still finish in this tick.
                    integral_clear_hold<=!active_i || (!reference_active && !integral_allowed && INTEGRAL_QUIET_TICKS<=0);
                    integral_commit_hold<=(integral_candidate_q16<=integral_q16 && integral_candidate_q16>=0 && integral_q16>=0) ||
                             (integral_candidate_q16>=integral_q16 && integral_candidate_q16<=0 && integral_q16<=0);
                    integral_saturation_veto<=((outer_sum_pipeline > active_limit_q8 && error_hold > 0) ||
                               (outer_sum_pipeline < -active_limit_q8 && error_hold < 0));
                    stage <= 4;
                end
                4: begin
                    if(integral_clear_hold) integral_q16<=0;
                    else if(integral_commit_hold || !integral_saturation_veto) integral_q16<=integral_candidate_q16;
                    if(separate_slew) begin
                        if(separated_sum>active_limit_q8) target_offset_q8<=active_limit_q8;
                        else if(separated_sum< -active_limit_q8) target_offset_q8<=-active_limit_q8;
                        else target_offset_q8<=separated_sum[31:0];
                    end else if (MAX_OFFSET_STEP_Q8 > 0 && offset_step > MAX_OFFSET_STEP_Q8)
                        target_offset_q8 <= target_offset_q8 + MAX_OFFSET_STEP_Q8;
                    else if (MAX_OFFSET_STEP_Q8 > 0 && offset_step < -MAX_OFFSET_STEP_Q8)
                        target_offset_q8 <= target_offset_q8 - MAX_OFFSET_STEP_Q8;
                    else target_offset_q8 <= limited_hold;
                    stage <= 0;
                end
                default: stage <= 0;
            endcase
        end
    end
endmodule
