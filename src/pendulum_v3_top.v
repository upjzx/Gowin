module pendulum_v3_top #(
    parameter integer RETURN_START_ZERO_ENABLE=1, HOME_MOVE_ANGLE_ADC=6,
    // Complete the original K3/K4 path, then smoothly return to the saved K1 direction.
    parameter integer POST_MOTION_HOME_ENABLE=1,
    // Release motion once actual arrival is safe; no extra PID-settle timer.
    parameter integer HOME_READY_ON_ARRIVAL=1,
    // Retain first post-catch temporary target; only20ms low-speed confirmation.
    parameter integer HOME_DIRECT_TO_ZERO=0, HOME_LOCAL_TICKS=4,
    parameter integer HOME_HANDOFF_COUNTS=12, HOME_HANDOFF_SPEED_Q8=128, HOME_HANDOFF_ANGLE_ADC=12,
    // Relaxed handoff applies only to explicit K3/K1 or K3/K2 commands.
    parameter integer TURN_FAST_START_ENABLE=1, TURN_START_COUNTS=36,
    parameter integer TURN_START_SPEED_Q8=384, TURN_START_ANGLE_ADC=18,
    parameter integer HOLD_ARRIVAL_COUNTS=6,
    // Idle-only recenter: Q8 counts per 5ms, 64 = 50 counts/s.
    parameter integer IDLE_RECENTER_ENABLE=0, IDLE_RETURN_MAX_SPEED_Q8=64,
    parameter integer IDLE_RETURN_START_COUNTS=6, IDLE_RETURN_STOP_COUNTS=3,
    parameter integer IDLE_RETURN_ACCEL_Q8=4, IDLE_RETURN_GAIN_Q8=4,
    parameter integer IDLE_HOLD_KD_Q8=3200, IDLE_HOLD_P_LIMIT_ADC=6,
    // 128 adds up to 50% velocity damping inside the endpoint window.
    parameter integer TURN_CW_BRAKE_STRENGTH_Q8=280, TURN_CCW_BRAKE_STRENGTH_Q8=272,
    parameter integer TURN_BRAKE_MAX_EXTRA_SPEED_Q8=512,
    parameter integer HOLD_REFERENCE_KP_Q8=16, HOLD_REFERENCE_KI_Q16=0,
    parameter integer WAYPOINT_TICKS_PER_30DEG=200, WAYPOINT_ARRIVAL_COUNTS=12,
    parameter integer WAYPOINT_DWELL_TICKS=40, WAYPOINT_TIMEOUT_TICKS=4000,
    parameter integer WAYPOINT_PASS_THROUGH=1, WAYPOINT_PASS_COUNTS=3,
    parameter integer TURN_ENDPOINT_BRAKE_ENABLE=1, TURN_ENDPOINT_WINDOW_COUNTS=600,
    // CW/CCW name the commanded turn, independently of encoder polarity.
    parameter integer TURN_CW_ENDPOINT_WINDOW_COUNTS=800,
    parameter integer TURN_CCW_ENDPOINT_WINDOW_COUNTS=700,
    // Positive/negative select the next sine/waypoint travel direction.
    parameter integer SINE_ENDPOINT_BRAKE_ENABLE=1,
    parameter integer SINE_POS_ENDPOINT_WINDOW_COUNTS=30, SINE_NEG_ENDPOINT_WINDOW_COUNTS=50,
    // Q8 counts/5ms: 80 = 62.5 counts/s ~=20.8deg/s at1080 counts/rev.
    parameter integer HOME_MAX_SPEED_Q8=200, HOME_ACCEL_Q8=1,
    parameter integer HOME_APPROACH_SHIFT=7,
    // Re-run return-to-K1-zero after a push: >REHOME counts for TICKS*5ms.
    parameter integer HOME_REHOME_COUNTS=20, HOME_REHOME_TICKS=100,
    parameter integer SWING_ENERGY_GUARD_ENABLE=1,
    // Excess-energy approach brake (see swing_balance_fsm). 0 = old fixed brake.
    parameter integer ENERGY_BRAKE_ENABLE=1, ENERGY_BRAKE_C_Q8=10, ENERGY_BRAKE_GAIN=128,
    parameter integer AUTO_REPEAT_ENABLE=0, AUTO_REPEAT_ROUNDS=2,
    parameter integer RETRY_ENABLE=1, RETRY_TIMEOUT_TICKS=4000, RETRY_MAX_ATTEMPTS=3,
    parameter integer KEY_DEBOUNCE_CYCLES=1000000, ADC_SAMPLE_DIV=1250,
    parameter integer ADC_AVERAGE_LOG2=7,
    parameter integer HIGH_RES_ANGLE=1,
    parameter integer LINEAR_STEADY_ENABLE=1, LINEAR_STEADY_DELAY_TICKS=2000,
    // Hardware trial 09 showed sustained oscillation with recovery P enabled.
    // Keep the optional experimental path off in the basic operating profile.
    parameter integer RECOVERY_ENABLE=0, RECOVERY_ENTRY_ADC=12,
    parameter integer RECOVERY_EXIT_ADC=4, RECOVERY_QUIET_TICKS=1000,
    parameter integer LINEAR_MAX_STEP=0,
    parameter integer LINEAR_BOOST_COMMAND=80,
    parameter integer TELEMETRY_DIV=500000, UART_BAUD=115200,
    // Keep full diagnostics available in basic balance without commanding travel.
    parameter integer DRIFT_TELEMETRY=1, EXTENDED_TELEMETRY=1,
    parameter integer AUTO_SEQUENCE_ENABLE=1, ROUNDTRIP_ENABLE=1, DIRECT_TURN_ENABLE=0, STEP_PERIOD_TICKS=590, SEQUENCE_DWELL_TICKS=4000,
    parameter integer MOVE_SPEED_LIMIT_Q16=400000,
    parameter integer SMOOTH_STEP_ENABLE=1,
    // CW/CCW acceleration tilt feedforward; sine has its own gain (0 = off).
    parameter integer MINJERK_ENABLE=1, ACCEL_FF_Q8=1600,
    parameter integer SINE_ACCEL_FF_Q8=260,
    // K4/K1 only: +1 encoder-positive start, -1 encoder-negative start.
    parameter integer SINE_DIRECTION=1,
    parameter integer SINE_ENABLE=0, SINE_AMPLITUDE=540, SINE_PERIOD_TICKS=1750, SINE_CYCLES=3,
    // K4/K2 profile 1: seven explicit peaks. AMPLITUDE/CYCLES/POWER are profile-0 only.
    // 1428 * 5ms = 7.14s period; seven-peak path lasts 24.99s excluding pauses.
    parameter integer WAVEPACKET_ENABLE=1, WAVEPACKET_AMPLITUDE=540,
    parameter integer WAVEPACKET_PERIOD_TICKS=1428, WAVEPACKET_CYCLES=8,
    parameter integer WAVEPACKET_DIRECTION=1,
    // 1: original sin^2 envelope; 2: stronger contrast with sin^4 envelope.
    parameter integer WAVEPACKET_ENVELOPE_POWER=2,
    // Profile 1: seven explicit alternating C2 peaks; profile 0: legacy envelope.
    parameter integer WAVEPACKET_PROFILE=1,
    parameter integer WAVEPACKET_PEAK_1=90, WAVEPACKET_PEAK_2=270,
    parameter integer WAVEPACKET_PEAK_3=540, WAVEPACKET_PEAK_4=900,
    parameter integer EXTENSION_ENABLE=1, EXTENSION_STEP_COUNTS=1080, EXTENSION_DIRECTION=1,
    // Historical approximate calibration; one physical revolution is unverified.
    parameter integer ARM_COUNTS_PER_REV=1080,
    parameter integer PWM_PERIOD=5000, MOTOR_PORT=1,
    parameter integer TEST_MODE=2, ENABLE_MOTOR_OUTPUT=1,
    // Keys: K1 = calibrate (hold arm upright, press: records encoder zero and
    // upright ADC), K2 = start swing-up. While balancing: K3 = turn mode
    // (K1 CW 360, K2 CCW 360), K4 = function mode (K1 full-amplitude sine, K2 smooth explicit-peak wavepacket).
    // K5 (rst_n) resets everything.
    // Auto start is off: holding the arm upright for K1 must not start a run.
    parameter integer LED_ACTIVE_LOW=0,
    parameter integer AUTO_MODE2=0, AUTO_START_DELAY_TICKS=1000,
    parameter integer RUNTIME_CAL=1, TURN_AUTO_START=0, //用 RUNTIME_CAL=0 可以切写死的竖直摆杆adc。关闭 10 s 自动正反转的开关是 TURN_AUTO_START=0。
    parameter integer AUTO_UPRIGHT_ENABLE=1, AUTO_UPRIGHT_DELAY_TICKS=100,
    // Retain the user's calibration from the five-run baseline.
    parameter integer UPRIGHT_ADC=752, HANGING_ADC=214,
    parameter integer CAPTURE_WINDOW_ADC=8, TRIP_WINDOW_ADC=160,
    parameter integer SOFT_CAPTURE_WINDOW_ADC=40,
    parameter integer CAPTURE_TRIP_WINDOW_ADC=400, BALANCE_GRACE_TICKS=100,
    // Basic task holds continuously; finite diagnostic tests may override this.
    parameter integer BALANCE_TEST_TICKS=0,
    parameter integer CAPTURE_SPEED_ADC=2, CAPTURE_CART_SPEED_COUNTS=6,
    parameter integer CAPTURE_HOLD_TICKS=3,
    parameter integer SENSOR_TIMEOUT_CYCLES=1000000, CONTROL_DIV=250000,
    parameter integer BALANCE_CONTROL_DIV=25000, POSITION_DIV=250000,
    // Balance gains/friction compensation follow the user's five-run baseline. 
    parameter integer ANGLE_KP=140, ANGLE_KD=600, ANGLE_KI_Q8=4,
    parameter integer STEADY_ANGLE_KP=60,
    parameter integer STEADY_ANGLE_KD=400,
    parameter integer INTEGRAL_LIMIT_COMMAND=200, INTEGRAL_WINDOW_ADC=10,
    parameter integer D_FILTER_SHIFT=1, MOTOR_LIMIT=3000, MOTOR_POLARITY=0,
    parameter integer ANGLE_DEADBAND_X2=8, DEADBAND_CONFIRM_TICKS=8000,
    parameter integer MAX_D_COMMAND=6000, DERIVATIVE_WINDOW_TICKS=10,
    parameter integer MIN_ACTIVE_COMMAND=200, OUTPUT_RELEASE_STEP=0,
    parameter integer START_COMMAND=250, START_TICKS=2,
    parameter integer BALANCE_COMMAND_TRIM=0,
    parameter integer ENABLE_POSITION_LOOP=1, ENCODER_POLARITY=0,
    parameter integer BACKLASH_POS_COUNTS=5, BACKLASH_NEG_COUNTS=5,
    parameter integer POSITION_KP_Q8=25, POSITION_KD_Q8=2560, POSITION_KI_Q16=0,
    parameter integer POSITION_STEADY_KD_Q8=2560,
    parameter integer POSITION_POLARITY=1, POSITION_TARGET_MODE=2,
    parameter integer POSITION_TURN_MIN_TRAVEL_COUNTS=20,
    parameter integer POSITION_TURN_REVERSE_COUNTS=30, POSITION_TURN_REVERSE_TICKS=4,
    parameter integer POSITION_TURN_STOP_TICKS=18,
    parameter integer POSITION_I_LIMIT_ADC=8, POSITION_INTEGRAL_WINDOW_COUNTS=0,
    parameter integer POSITION_INTEGRAL_QUIET_TICKS=0,
    parameter integer POSITION_INTEGRAL_VELOCITY_LIMIT_Q8=64,
    parameter integer POSITION_INTEGRAL_MOVING_DECAY_SHIFT=0,
    parameter integer POSITION_INTEGRAL_DEADBAND_COUNTS=20,
    parameter integer POSITION_INTEGRAL_OVERSHOOT_COUNTS=20,
    parameter integer POSITION_INTEGRAL_UNWIND_SHIFT=4,
    parameter integer REFERENCE_HOLD_I_DEADBAND_COUNTS=6, REFERENCE_SETTLE_I_DEADBAND_COUNTS=3,
    parameter integer POSITION_TOLERANCE_COUNTS=2, POSITION_VELOCITY_FILTER_SHIFT=1,
    parameter integer POSITION_VELOCITY_WINDOW_TICKS=1,
    parameter integer POSITION_VELOCITY_OBSERVER_ENABLE=1,
    parameter integer BIAS_LIMIT_ADC=18, BIAS_SLEW_Q8=256,
    parameter integer STEADY_POSITION_STEP_Q8=256, STEADY_DAMPING_STEP_Q8=256,
    parameter integer TURN_POSITION_STEP_Q8=48, TURN_LAUNCH_STEP_Q8=64, TURN_LAUNCH_TICKS=40,
    parameter integer STEADY_BIAS_LIMIT_ADC=0,
    parameter integer REFERENCE_POSITION_P_LIMIT_ADC=12,
    parameter integer TURN_DISTANCE_BRAKE_ENABLE=1, TURN_DISTANCE_WINDOW_SHIFT=9, TURN_DISTANCE_WINDOW_COUNTS=540,
    parameter integer TURN_BRAKE_ENABLE=0, TURN_BRAKE_LOOKAHEAD_TICKS=96,
    parameter integer TURN_BRAKE_MAX_LEAD_COUNTS=192,
    parameter integer POSITION_STEADY_P_LIMIT_ADC=2,
    parameter integer POSITION_STEADY_P_DEADBAND_COUNTS=15,
    // Target modes: 0=catch position, 1=absolute, 2=first post-catch turn.
    // TARGET_OFFSET_COUNTS offsets the acquired point in modes 0 and 2.
    parameter integer TARGET_OFFSET_COUNTS=0, DIRECT_TARGET_OFFSET_COUNTS=0,
    parameter integer MOTION_DEVIATION_LIMIT_COUNTS=2160,
    parameter integer MAX_TRAVEL_COUNTS=5000,
    parameter integer SOFT_TRAVEL_COUNTS=400, ENERGY_COS_SWITCH_ADC=240,
    parameter integer TOP_BRAKE_WINDOW_ADC=160, TOP_BRAKE_DUTY=3000,
    // Early swing-up profile: brake has priority near the top so the arm does
    // not carry full energy through upright; smaller pulse increments avoid
    // the repeated over-acceleration seen in the later profile.
    parameter integer TOP_DEPARTURE_BRAKE_ENABLE=1,
    parameter integer TOP_SPEED_PRIORITY_ENABLE=1,
    parameter integer SWING_DUTY=1000, SWING_MAX_DUTY=2500, SWING_DUTY_STEP=150,
    parameter integer PULSE_TICKS=20, BRAKE_TICKS=10, SWING_POLARITY=0,
    parameter integer SWING_TIMEOUT_TICKS=20000, NO_EXTREMA_TICKS=1000,
    parameter integer MAX_PULSE_PAIRS=200, ADC_JUMP_LIMIT=96, ADC_RAIL_TICKS=20
) (
    input wire clk_50m, rst_n,
    input wire [3:0] key_sw,
    input wire [9:0] adc_data_in,
    input wire enc1_a, enc1_b, enc2_a, enc2_b,
    output wire adc_clk, adc_oe_n,
    output wire AN1, AN2, PWMA, BN1, BN2, PWMB, uart_tx,
    output wire [3:0] led
);
    wire start_pressed, serial_pressed, extension_pressed, extension_pulse;
    wire retry_inhibit, retry_start;
    wire [2:0] retry_phase;
    wire [7:0] retry_attempt;
    wire [15:0] retry_elapsed;
    wire stop_key=1'b0;
    wire retry_healthy=configuration_ok && sensor_fresh && !selected_encoder_fault;
    reg extension_abort;
    reg [3:0] extension_guard_age;
    // A direct final target legitimately starts1080 counts away. Guard its
    // allowed corridor and retreat from best measured progress instead.
    reg signed [32:0] direct_delta, direct_goal_delta, direct_progress, direct_distance, direct_best;
    reg signed [33:0] direct_retreat, direct_excess;
    reg signed [31:0] guard_goal, guard_anchor;
    reg guard_direct_mode;
    reg signed [31:0] guard_position_sample,guard_goal_sample,guard_anchor_sample;
    wire direct_guard_mode=DIRECT_TURN_ENABLE!=0 && mission_phase!=11;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin direct_delta<=0; direct_goal_delta<=0; direct_progress<=0;
            direct_distance<=0; direct_retreat<=0; direct_excess<=0;
            guard_position_sample<=0;guard_goal_sample<=0;guard_anchor_sample<=0;end
        else begin
            guard_position_sample<=selected_position;guard_goal_sample<=extension_goal;guard_anchor_sample<=extension_anchor;
            direct_delta<=$signed({guard_position_sample[31],guard_position_sample})-$signed({guard_anchor_sample[31],guard_anchor_sample});
            direct_goal_delta<=$signed({guard_goal_sample[31],guard_goal_sample})-$signed({guard_anchor_sample[31],guard_anchor_sample});
            direct_progress<=extension_goal>=extension_anchor ? direct_delta : -direct_delta;
            direct_distance<=direct_goal_delta<0 ? -direct_goal_delta : direct_goal_delta;
            direct_retreat<=$signed({direct_best[32],direct_best})-$signed({direct_progress[32],direct_progress});
            direct_excess<=$signed({direct_progress[32],direct_progress})-$signed({direct_distance[32],direct_distance});
        end
    end
    wire extension_operating=EXTENSION_ENABLE!=0 && (mission_phase==2 || mission_phase==4 || mission_phase==5 ||
        mission_phase==6 || mission_phase==7 || mission_phase==8 || mission_phase==10 || mission_phase==11 ||
        mission_phase==13 || mission_phase==14);
    // Catch gross departure from the commanded reference before multi-turn
    // travel. This latch is cleared only by reset, never by automatic retries.
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin extension_abort<=0; extension_guard_age<=0;
            direct_best<=0; guard_goal<=0; guard_anchor<=0; guard_direct_mode<=0; end
        else if(control_tick) begin
            guard_goal<=extension_goal; guard_anchor<=extension_anchor; guard_direct_mode<=direct_guard_mode;
            if(!extension_operating || guard_direct_mode!=direct_guard_mode || (direct_guard_mode &&
                (guard_goal!=extension_goal || guard_anchor!=extension_anchor))) begin
                extension_guard_age<=0; direct_best<=0;
            end else if(extension_guard_age<4) extension_guard_age<=extension_guard_age+1'b1;
            else if(direct_guard_mode) begin
                if(direct_progress>direct_best) direct_best<=direct_progress;
                if(direct_progress< -MOTION_DEVIATION_LIMIT_COUNTS || direct_excess>MOTION_DEVIATION_LIMIT_COUNTS || direct_retreat>MOTION_DEVIATION_LIMIT_COUNTS)
                    extension_abort<=1;
            end else if(telemetry_position_error>MOTION_DEVIATION_LIMIT_COUNTS || telemetry_position_error< -MOTION_DEVIATION_LIMIT_COUNTS)
                extension_abort<=1;
        end
    end
    key_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) extension_key_filter (
        .clk(clk_50m),.rst_n(rst_n),.key_n(key_sw[1]),
        .pressed(extension_pressed),.press_pulse(extension_pulse));
    wire start_press_pulse, serial_press_pulse, calibrate_pulse;
    assign start_press_pulse=extension_pulse && key_mode==0 && !run_active;   // K2
    reg serial_enabled, emergency_stop;
    reg [1:0] key_mode;
    reg [31:0] auto_start_count;
    reg auto_start_pulse, auto_started;
    wire [9:0] angle_raw;
    wire [10:0] angle_x2;
    wire [17:0] angle_q8;
    wire angle_valid, angle_update;
    reg [31:0] sensor_age, control_count, balance_control_count, position_count;
    wire control_tick=(control_count==CONTROL_DIV-1);
    wire balance_control_tick=(balance_control_count==BALANCE_CONTROL_DIV-1);
    wire sensor_fresh=angle_valid && sensor_age<SENSOR_TIMEOUT_CYCLES;
    // Upright auto-entry is explicitly enabled for direct position tests.
    // Require a stationary qualified interval and capture the current target.
    // K1 calibration. Upright keeps Q8 precision for the balance loop; the
    // hanging point follows from the fixed upright-to-hanging distance.
    // K1 never drives the motor, so a latched fault/protection state must not
    // block it; only a running controller or a stale ADC sample rejects it.
    wire calibrate_accept=calibrate_pulse && key_mode==0 && sensor_fresh && !run_active && configuration_ok;
    reg [17:0] upright_cal_q8;
    wire [9:0] upright_cal_raw=(upright_cal_q8+18'd128)>>8;
    wire [9:0] hanging_cal=upright_cal_raw-(UPRIGHT_ADC-HANGING_ADC);
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) upright_cal_q8<=UPRIGHT_ADC*256;
        else if(RUNTIME_CAL!=0 && calibrate_accept)
            upright_cal_q8<=angle_q8;
    end
    wire [9:0] auto_down_distance=angle_raw-hanging_cal;
    wire auto_hanging=(auto_down_distance<=10'd40 || auto_down_distance>=10'd984);
    wire signed [31:0] encoder1, encoder2;
    wire encoder1_illegal, encoder2_illegal;
    wire signed [31:0] raw_encoder_position=(MOTOR_PORT==1) ? encoder1 : encoder2;
    wire signed [31:0] raw_selected_position;
    // Raw encoder runs ~8 counts ahead of the shaft when increasing and ~2
    // when decreasing; feed every consumer the estimated shaft position.
    backlash_comp #(.BL_POS(BACKLASH_POS_COUNTS),.BL_NEG(BACKLASH_NEG_COUNTS)) backlash_motor (
        .clk(clk_50m),.rst_n(rst_n),
        .encoder_position(raw_encoder_position),.shaft_position(raw_selected_position)
    );
    wire signed [31:0] selected_position=(ENCODER_POLARITY!=0) ?
        -raw_selected_position : raw_selected_position;
    reg signed [31:0] startup_zero;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) startup_zero<=0;
        else if(calibrate_accept)
            startup_zero<=selected_position;
    end
    wire selected_encoder_fault=(MOTOR_PORT==1) ? encoder1_illegal : encoder2_illegal;
    reg [31:0] upright_start_count;
    reg signed [31:0] idle_previous_position;
    reg [9:0] idle_previous_angle;
    reg idle_history_valid, direct_entry;
    wire [9:0] auto_up_distance=angle_raw-upright_cal_raw;
    wire auto_upright=(auto_up_distance<=CAPTURE_WINDOW_ADC ||
        auto_up_distance>=1024-CAPTURE_WINDOW_ADC);
    wire signed [32:0] idle_position_delta=$signed({selected_position[31],selected_position})-
        $signed({idle_previous_position[31],idle_previous_position});
    wire signed [10:0] idle_angle_delta=$signed({1'b0,angle_raw})-$signed({1'b0,idle_previous_angle});
    wire upright_ready=AUTO_UPRIGHT_ENABLE!=0 && idle_history_valid && auto_upright &&
        idle_position_delta<=1 && idle_position_delta>=-1 &&
        idle_angle_delta<=1 && idle_angle_delta>=-1 && !selected_encoder_fault;
    wire [2:0] state;
    wire [3:0] fault_code;
    wire balance_start, balance_enable, run_active, swing_active, fault_pending;
    wire top_speed_brake_active;
    wire signed [31:0] run_origin;
    wire signed [31:0] capture_velocity_q8;
    wire outer_tick=balance_enable && position_count==POSITION_DIV-1;
    wire signed [15:0] swing_command, balance_command, tracking_command;
    reg signed [15:0] swing_command_registered;
    reg top_priority_registered;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin swing_command_registered<=0;top_priority_registered<=0;end
        else begin swing_command_registered<=swing_command;top_priority_registered<=top_speed_brake_active;end
    end
    // Use the continuously tracked, bounded P/D near upright, before the
    // full catch criteria enable position control and the output limiter.
    wire signed [15:0] soft_capture_error=$signed({1'b0,angle_raw})-$signed({1'b0,upright_cal_raw});
    wire soft_capture_request=(TEST_MODE==2) && swing_active &&
        SOFT_CAPTURE_WINDOW_ADC>0 &&
        soft_capture_error<=SOFT_CAPTURE_WINDOW_ADC &&
        soft_capture_error>=-SOFT_CAPTURE_WINDOW_ADC;
    reg soft_capture_active;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) soft_capture_active<=0;
        else soft_capture_active<=soft_capture_request;
    end
    localparam signed [16:0] BALANCE_COMMAND_TRIM_17=BALANCE_COMMAND_TRIM;
    wire signed [16:0] balance_command_with_trim=
        $signed({balance_command[15],balance_command})+BALANCE_COMMAND_TRIM_17;
    wire signed [15:0] balance_command_trimmed=
        (balance_command_with_trim>MOTOR_LIMIT) ? MOTOR_LIMIT[15:0] :
        (balance_command_with_trim< -MOTOR_LIMIT) ? -$signed(MOTOR_LIMIT[15:0]) :
        balance_command_with_trim[15:0];
    wire signed [31:0] correction_q8, angle_target_q8, position_velocity, angle_delta_q8;
    wire signed [32:0] target_position;
    wire position_target_locked;
    wire quiet_servo_active;
    wire [3:0] mission_phase;
    reg [3:0] previous_mission_phase;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) previous_mission_phase<=0;
        else previous_mission_phase<=mission_phase;
    end
    reg mission_cycle_complete;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) mission_cycle_complete<=0;
        else mission_cycle_complete<=AUTO_REPEAT_ENABLE!=0 && previous_mission_phase==11 &&
            mission_phase==5 && balance_enable && !extension_abort && !fault_pending;
    end
    wire signed [31:0] extension_goal, extension_anchor;
    wire [15:0] extension_dwell;
    reg [31:0] board_control_ticks;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) board_control_ticks<=0;
        else if(control_tick) board_control_ticks<=board_control_ticks+1'b1;
    end
    wire signed [31:0] position_integral_q16, mission_velocity_q8;
    wire motor_in1, motor_in2, motor_pwm;
    // Register direction bits for diagnostic packing only; actual output pins
    // retain immediate protection. Avoid encoder-safety fanout in UART data.
    reg telemetry_motor_in1,telemetry_motor_in2;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin telemetry_motor_in1<=0;telemetry_motor_in2<=0;end
        else begin telemetry_motor_in1<=motor_in1;telemetry_motor_in2<=motor_in2;end
    end
    // Interface 2 is unused when MOTOR_PORT=1.  In this dedicated Mode-2
    // build its telemetry word proves what reaches the FPGA output logic:
    // [27]=IN2, [26]=IN1, [25:22]=fault, [21:19]=state,
    // The low field is reserved.  An earlier PWM-high counter put the whole
    // encoder safety chain on a 50 MHz counter-enable path and broke timing.
    wire [31:0] mode2_debug_word={4'b0,telemetry_motor_in2,telemetry_motor_in1,
                                  fault_code,state,correction_q8[18:0]};
    reg signed [32:0] telemetry_position_error;
    reg signed [31:0] telemetry_position_sample, telemetry_correction_sample;
    reg telemetry_target_locked;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            telemetry_position_error<=0; telemetry_position_sample<=0;
            telemetry_correction_sample<=0; telemetry_target_locked<=0;
        end else begin
            telemetry_position_error<=target_position-$signed({selected_position[31],selected_position});
            telemetry_position_sample<=encoder1;
            telemetry_correction_sample<=correction_q8;
            telemetry_target_locked<=position_target_locked;
        end
    end
    wire signed [9:0] drift_error=(telemetry_position_error>511) ? 10'sd511 :
        (telemetry_position_error< -512) ? -10'sd512 : telemetry_position_error[9:0];
    wire signed [31:0] drift_correction_q2=telemetry_correction_sample >>> 6;
    wire signed [7:0] drift_correction=(drift_correction_q2>127) ? 8'sd127 :
        (drift_correction_q2< -128) ? -8'sd128 : drift_correction_q2[7:0];
    wire [31:0] drift_debug_word={(quiet_servo_active ? 4'h9 : 4'h8),telemetry_motor_in2,telemetry_motor_in1,fault_code,state,
        telemetry_target_locked,drift_error,drift_correction};
    wire signed [18:0] mission_error=(telemetry_position_error>262143) ? 19'sd262143 :
        (telemetry_position_error< -262144) ? -19'sd262144 : telemetry_position_error[18:0];
    wire [31:0] mission_debug_word={4'hc,telemetry_motor_in2,telemetry_motor_in1,fault_code,state,mission_error};
    wire use_drift_telemetry=(MOTOR_PORT==1 && TEST_MODE==2 && DRIFT_TELEMETRY!=0);
    wire signed [31:0] telemetry_encoder1=use_drift_telemetry ? telemetry_position_sample : encoder1;
    wire signed [31:0] telemetry_encoder2=
        (MOTOR_PORT==1 && TEST_MODE==2) ?
        $signed((EXTENSION_ENABLE!=0) ? mission_debug_word : use_drift_telemetry ? drift_debug_word : mode2_debug_word) : encoder2;
    reg signed [31:0] telemetry_encoder2_sample;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) telemetry_encoder2_sample<=0;
        else telemetry_encoder2_sample<=telemetry_encoder2;
    end
    wire configuration_ok=(MOTOR_PORT==1 || MOTOR_PORT==2) &&
        TEST_MODE>=0 && TEST_MODE<=2 && MOTOR_LIMIT>0 && MOTOR_LIMIT<=PWM_PERIOD &&
        BALANCE_COMMAND_TRIM>=-MOTOR_LIMIT && BALANCE_COMMAND_TRIM<=MOTOR_LIMIT &&
        SWING_DUTY>0 && SWING_DUTY<=SWING_MAX_DUTY && SWING_MAX_DUTY<=PWM_PERIOD &&
        PWM_PERIOD>0 && PWM_PERIOD<=32767 && CONTROL_DIV>0 &&
        BALANCE_CONTROL_DIV>=8 && POSITION_DIV>=5 &&
        (POSITION_TARGET_MODE>=0 && POSITION_TARGET_MODE<=2) &&
        (POSITION_TARGET_MODE!=2 || (POSITION_TURN_MIN_TRAVEL_COUNTS>0 &&
         POSITION_TURN_REVERSE_COUNTS>1 && POSITION_TURN_REVERSE_TICKS>0 &&
         POSITION_TURN_STOP_TICKS>0)) &&
        CAPTURE_WINDOW_ADC<TRIP_WINDOW_ADC && TRIP_WINDOW_ADC<CAPTURE_TRIP_WINDOW_ADC &&
        CAPTURE_TRIP_WINDOW_ADC<=511 && BALANCE_GRACE_TICKS>0 &&
        TOP_BRAKE_WINDOW_ADC>CAPTURE_WINDOW_ADC && TOP_BRAKE_WINDOW_ADC<=511 &&
        TOP_BRAKE_DUTY>0 && TOP_BRAKE_DUTY<=PWM_PERIOD &&
        ADC_SAMPLE_DIV>0 && ADC_SAMPLE_DIV%2==0 &&
        AUTO_START_DELAY_TICKS>0 && AUTO_UPRIGHT_DELAY_TICKS>0 && SOFT_TRAVEL_COUNTS>0 &&
        SOFT_TRAVEL_COUNTS<MAX_TRAVEL_COUNTS &&
        (EXTENSION_ENABLE==0 || (ENABLE_POSITION_LOOP!=0 && MOTOR_PORT==1 &&
         EXTENSION_STEP_COUNTS>0 && EXTENSION_STEP_COUNTS<MAX_TRAVEL_COUNTS-500 &&
         (EXTENSION_DIRECTION==1 || EXTENSION_DIRECTION==-1)));
    swing_retry_supervisor #(.ENABLE(RETRY_ENABLE),.REPEAT_ENABLE(AUTO_REPEAT_ENABLE),.REPEAT_MAX_ROUNDS(AUTO_REPEAT_ROUNDS),.TIMEOUT_TICKS(RETRY_TIMEOUT_TICKS),
        .MAX_ATTEMPTS(RETRY_MAX_ATTEMPTS)) retry_supervisor (
        .cycle_complete(mission_cycle_complete),
        .clk(clk_50m),.rst_n(rst_n),.tick(control_tick),.run_active(run_active),
        .balance_enable(balance_enable),.healthy(retry_healthy),
        .fault(state==6 || fault_pending || emergency_stop || extension_abort),.cancelled(stop_key),
        .stable_balance(EXTENSION_ENABLE!=0 ? balance_enable :
            (auto_upright && position_target_locked && position_velocity<=32 && position_velocity>=-32)),
        .stationary_down((auto_hanging && idle_history_valid && idle_position_delta==0 &&
            idle_angle_delta<=1 && idle_angle_delta>=-1) ||
            (retry_phase==7 && upright_ready)),
        .inhibit(retry_inhibit),.retry_start(retry_start),.phase(retry_phase),
        .attempt(retry_attempt),.elapsed_ticks(retry_elapsed)
    );
    wire safe_active=rst_n && configuration_ok && run_active && sensor_fresh &&
        !fault_pending && !emergency_stop && !stop_key && !retry_inhibit && !extension_abort;
    // Demand is transmitted even in dry-run. Only drive_enable controls the pins.
    wire signed [15:0] selected_motor_command=balance_enable ? balance_command_trimmed :
        (soft_capture_active && !top_priority_registered) ? tracking_command : swing_command_registered;
    wire signed [15:0] requested_motor_command=safe_active ? selected_motor_command : 16'sd0;
    // The same limit covers energy pumping, top braking and both catch paths.
    // Clamp before the immediate safety mux so travel protection does not
    // traverse two extra command comparators on the way to the output pins.
    wire signed [15:0] bounded_motor_request=(selected_motor_command>MOTOR_LIMIT) ? MOTOR_LIMIT[15:0] :
        (selected_motor_command< -MOTOR_LIMIT) ? -$signed(MOTOR_LIMIT[15:0]) : selected_motor_command;
    // Isolate the command mux/clamp from PWM direction and zero detection.
    // Raw safe_active still inhibits the pins immediately, without this delay.
    reg signed [15:0] bounded_motor_command;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) bounded_motor_command<=0;
        else if(!safe_active) bounded_motor_command<=0;
        else bounded_motor_command<=bounded_motor_request;
    end
    wire signed [15:0] motor_command=safe_active ? bounded_motor_command : 16'sd0;
    wire motor_drive_enable=(ENABLE_MOTOR_OUTPUT!=0) && safe_active && (bounded_motor_command!=0);
    wire [7:0] telemetry_status={
        (MOTOR_PORT==2),encoder2_illegal,encoder1_illegal,
        angle_valid,emergency_stop,(state==6),run_active,serial_enabled
    };

    key_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) start_key_filter (
        .clk(clk_50m),.rst_n(rst_n),.key_n(key_sw[0]),
        .pressed(start_pressed),.press_pulse(calibrate_pulse)
    );
    key_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) serial_key_filter (
        .clk(clk_50m),.rst_n(rst_n),.key_n(key_sw[2]),
        .pressed(serial_pressed),.press_pulse(serial_press_pulse)
    );
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin
            serial_enabled<=1; emergency_stop<=0;
            auto_start_count<=0; auto_start_pulse<=0; auto_started<=0;
            upright_start_count<=0; idle_previous_position<=0;
            idle_previous_angle<=0; idle_history_valid<=0; direct_entry<=0;
            sensor_age<=SENSOR_TIMEOUT_CYCLES; control_count<=0;
            balance_control_count<=0; position_count<=0;
        end else begin
            auto_start_pulse<=0;
            if(control_tick) begin
                idle_previous_position<=selected_position;
                idle_previous_angle<=angle_raw;
                idle_history_valid<=sensor_fresh;
            end
            if(retry_start) direct_entry<=upright_ready;
            else if(start_press_pulse) direct_entry<=(AUTO_UPRIGHT_ENABLE!=0 && upright_ready);
            if(angle_update) sensor_age<=0;
            else if(sensor_age<SENSOR_TIMEOUT_CYCLES) sensor_age<=sensor_age+1'b1;
            if(control_tick) control_count<=0;
            else control_count<=control_count+1'b1;
            if(!run_active) begin
                balance_control_count<=0;
                position_count<=0;
            end else begin
                if(balance_control_tick) balance_control_count<=0;
                else balance_control_count<=balance_control_count+1'b1;
                if(!balance_enable || balance_start || outer_tick) position_count<=0;
                else position_count<=position_count+1'b1;
            end

            // In the dedicated Mode-2 image, wait 5 s (default) after the
            // ADC becomes valid, then ask the FSM to start.  If the pendulum
            // is not yet stationary at the hanging point, retry after another
            // delay; once a run starts, never auto-restart without reset.
            if(run_active) begin
                auto_started<=1;
                auto_start_count<=0;
                upright_start_count<=0;
            end else if(AUTO_MODE2!=0 && TEST_MODE==2 && !auto_started &&
                        configuration_ok && sensor_fresh && upright_ready && key_sw[1] &&
                        !emergency_stop && control_tick) begin
                auto_start_count<=0;
                if(upright_start_count>=AUTO_UPRIGHT_DELAY_TICKS-1) begin
                    upright_start_count<=0; auto_start_pulse<=1; direct_entry<=1;
                end else upright_start_count<=upright_start_count+1'b1;
            end else if(AUTO_MODE2!=0 && TEST_MODE==2 && !auto_started &&
                        configuration_ok && sensor_fresh && auto_hanging && key_sw[1] &&
                        !emergency_stop && control_tick) begin
                if(auto_start_count>=AUTO_START_DELAY_TICKS-1) begin
                    auto_start_count<=0;
                    auto_start_pulse<=1;
                    direct_entry<=0;
                end else auto_start_count<=auto_start_count+1'b1;
                upright_start_count<=0;
            end else if(!sensor_fresh || !auto_hanging || !key_sw[1]) begin
                auto_start_count<=0;
                if(!upright_ready || !sensor_fresh || !key_sw[1]) upright_start_count<=0;
            end

        end
    end
    adc_capture #(.SAMPLE_DIV(ADC_SAMPLE_DIV),.AVERAGE_LOG2(ADC_AVERAGE_LOG2)) adc (
        .clk(clk_50m),.rst_n(rst_n),.adc_data(adc_data_in),
        .adc_clk(adc_clk),.adc_oe_n(adc_oe_n),.angle_raw(angle_raw),.angle_x2(angle_x2),.angle_q8(angle_q8),
        .angle_valid(angle_valid),.angle_update(angle_update)
    );
    quadrature_counter encoder_a (
        .clk(clk_50m),.rst_n(rst_n),.phase_a(enc1_a),.phase_b(enc1_b),
        .position(encoder1),.illegal_seen(encoder1_illegal)
    );
    quadrature_counter encoder_b (
        .clk(clk_50m),.rst_n(rst_n),.phase_a(enc2_a),.phase_b(enc2_b),
        .position(encoder2),.illegal_seen(encoder2_illegal)
    );
    swing_balance_fsm #(
        .TEST_MODE(TEST_MODE),.UPRIGHT_ADC(UPRIGHT_ADC),.HANGING_ADC(HANGING_ADC),.RUNTIME_CAL(RUNTIME_CAL),
        .CAPTURE_WINDOW_ADC(CAPTURE_WINDOW_ADC),.TRIP_WINDOW_ADC(TRIP_WINDOW_ADC),
        .CAPTURE_TRIP_WINDOW_ADC(CAPTURE_TRIP_WINDOW_ADC),
        .BALANCE_GRACE_TICKS(BALANCE_GRACE_TICKS),
        .BALANCE_TEST_TICKS(BALANCE_TEST_TICKS),
        .CAPTURE_SPEED_ADC(CAPTURE_SPEED_ADC),
        .CAPTURE_CART_SPEED_COUNTS(CAPTURE_CART_SPEED_COUNTS),
        .CAPTURE_HOLD_TICKS(CAPTURE_HOLD_TICKS),
        .ADC_JUMP_LIMIT(ADC_JUMP_LIMIT),.SWING_DUTY(SWING_DUTY),
        .SWING_MAX_DUTY(SWING_MAX_DUTY),.SWING_DUTY_STEP(SWING_DUTY_STEP),
        .PULSE_TICKS(PULSE_TICKS),.BRAKE_TICKS(BRAKE_TICKS),.SWING_POLARITY(SWING_POLARITY),
        .SWING_TIMEOUT_TICKS(SWING_TIMEOUT_TICKS),.NO_EXTREMA_TICKS(NO_EXTREMA_TICKS),
        .MAX_PULSE_PAIRS(MAX_PULSE_PAIRS),.MAX_TRAVEL_COUNTS(MAX_TRAVEL_COUNTS),
        .SOFT_TRAVEL_COUNTS(SOFT_TRAVEL_COUNTS),
        .ENERGY_COS_SWITCH_ADC(ENERGY_COS_SWITCH_ADC),
        .TOP_BRAKE_WINDOW_ADC(TOP_BRAKE_WINDOW_ADC),
        .TOP_BRAKE_DUTY(TOP_BRAKE_DUTY),
        .TOP_DEPARTURE_BRAKE_ENABLE(TOP_DEPARTURE_BRAKE_ENABLE),
        .TOP_SPEED_PRIORITY_ENABLE(TOP_SPEED_PRIORITY_ENABLE),
        .ENERGY_GUARD_ENABLE(SWING_ENERGY_GUARD_ENABLE),
        .ENERGY_BRAKE_ENABLE(ENERGY_BRAKE_ENABLE),.ENERGY_BRAKE_C_Q8(ENERGY_BRAKE_C_Q8),
        .ENERGY_BRAKE_GAIN(ENERGY_BRAKE_GAIN),
        .ADC_RAIL_TICKS(ADC_RAIL_TICKS)
    ) sequencer (
        .clk(clk_50m),.rst_n(rst_n),.tick(control_tick),
         .start((start_press_pulse || auto_start_pulse || retry_start) && configuration_ok && !retry_inhibit),
        .stop(stop_key || (retry_inhibit && !fault_pending && state!=6)),
        .emergency(emergency_stop),.sensor_fresh(sensor_fresh),
        .encoder_fault(selected_encoder_fault),.angle_raw(angle_raw),
        .upright_adc(upright_cal_raw),.hanging_adc(hanging_cal),
        .position(selected_position),.state(state),.fault_code(fault_code),
        .balance_start(balance_start),.balance_enable(balance_enable),
        .run_active(run_active),.swing_active(swing_active),.fault_pending(fault_pending),
        .top_speed_brake_active(top_speed_brake_active),
        .capture_velocity_q8(capture_velocity_q8),
        .swing_command(swing_command),.run_origin(run_origin)
    );
    // Cut the travel-check fanout into controller reset/track logic. The motor
    // output still uses raw safe_active/fault_pending for immediate inhibition.
    reg controller_safe;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) controller_safe<=0;
        else controller_safe<=safe_active;
    end
    wire controller_permit=EXTENSION_ENABLE!=0 ? controller_safe : safe_active;
    // Modes may be preselected during swing-up; motion waits for safe home handoff.
    // 0: K1 calibrate / K2 swing-up, 1 (K3): K1 CW / K2 CCW, 2 (K4): K1 sine / K2 smooth explicit-peak wavepacket.
    // A K1/K2 press in modes 1/2 is latched until the motion starts or is refused.
    wire mode_pressed, mode_press_pulse;
    key_debounce #(.DEBOUNCE_CYCLES(KEY_DEBOUNCE_CYCLES)) mode_key_filter (
        .clk(clk_50m),.rst_n(rst_n),.key_n(key_sw[3]),
        .pressed(mode_pressed),.press_pulse(mode_press_pulse));
    reg [2:0] motion_command;
    reg [2:0] telemetry_last_key,telemetry_last_motion;
    reg [15:0] telemetry_key_count;
    // Report every debounced key, including selection/cancellation without motion.
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin telemetry_last_key<=0;telemetry_key_count<=0;telemetry_last_motion<=0;end
        else begin
            if(serial_press_pulse || mode_press_pulse || calibrate_pulse || extension_pulse) begin
                telemetry_key_count<=telemetry_key_count+1'b1;
                telemetry_last_key<=serial_press_pulse ? 3'd3 : mode_press_pulse ? 3'd4 : calibrate_pulse ? 3'd1 : 3'd2;
            end
            if(!run_active || !balance_enable) telemetry_last_motion<=0;
            else if(mission_phase==4) telemetry_last_motion<=1;
            else if(mission_phase==7) telemetry_last_motion<=2;
            else if(mission_phase==11) telemetry_last_motion<=3;
            else if(mission_phase==2) telemetry_last_motion<=4;
        end
    end
    wire motion_started=mission_phase==4 || mission_phase==7 || mission_phase==11 ||
        mission_phase==2 || mission_phase==9;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) begin key_mode<=0; motion_command<=0; end
        else if(!run_active) begin key_mode<=0; motion_command<=0; end
        else begin
            if(serial_press_pulse) key_mode<=key_mode==1 ? 0 : 1;
            else if(mode_press_pulse) key_mode<=key_mode==2 ? 0 : 2;
            if(motion_started || serial_press_pulse || mode_press_pulse) motion_command<=0;
            else if(key_mode==1 && calibrate_pulse) motion_command<=1;
            else if(key_mode==1 && extension_pulse) motion_command<=2;
            else if(key_mode==2 && calibrate_pulse) motion_command<=3;
            else if(key_mode==2 && extension_pulse) motion_command<=4;
        end
    end
    balance_v2_adapter #(
        .CAPTURE_VELOCITY_SEED_ENABLE(1),
        .RETURN_START_ZERO_ENABLE(RETURN_START_ZERO_ENABLE),.ARM_COUNTS_PER_REV(ARM_COUNTS_PER_REV),
        .POST_MOTION_HOME_ENABLE(POST_MOTION_HOME_ENABLE),
        .HOME_REHOME_COUNTS(HOME_REHOME_COUNTS),.HOME_REHOME_TICKS(HOME_REHOME_TICKS),
        .HOME_MOVE_ANGLE_ADC(HOME_MOVE_ANGLE_ADC),.HOME_READY_ON_ARRIVAL(HOME_READY_ON_ARRIVAL),
        .HOME_DIRECT_TO_ZERO(HOME_DIRECT_TO_ZERO),.HOME_LOCAL_TICKS(HOME_LOCAL_TICKS),
        .HOME_HANDOFF_COUNTS(HOME_HANDOFF_COUNTS),
        .HOME_HANDOFF_SPEED_Q8(HOME_HANDOFF_SPEED_Q8),.HOME_HANDOFF_ANGLE_ADC(HOME_HANDOFF_ANGLE_ADC),
        .HOLD_REFERENCE_KP_Q8(HOLD_REFERENCE_KP_Q8),.HOLD_REFERENCE_KI_Q16(HOLD_REFERENCE_KI_Q16),
        .TURN_FAST_START_ENABLE(TURN_FAST_START_ENABLE),.TURN_START_COUNTS(TURN_START_COUNTS),
        .TURN_START_SPEED_Q8(TURN_START_SPEED_Q8),.TURN_START_ANGLE_ADC(TURN_START_ANGLE_ADC),
        .HOLD_ARRIVAL_COUNTS(HOLD_ARRIVAL_COUNTS),
        .IDLE_RECENTER_ENABLE(IDLE_RECENTER_ENABLE),.IDLE_RETURN_MAX_SPEED_Q8(IDLE_RETURN_MAX_SPEED_Q8),
        .IDLE_RETURN_START_COUNTS(IDLE_RETURN_START_COUNTS),.IDLE_RETURN_STOP_COUNTS(IDLE_RETURN_STOP_COUNTS),
        .IDLE_RETURN_ACCEL_Q8(IDLE_RETURN_ACCEL_Q8),.IDLE_RETURN_GAIN_Q8(IDLE_RETURN_GAIN_Q8),
        .IDLE_HOLD_KD_Q8(IDLE_HOLD_KD_Q8),.IDLE_HOLD_P_LIMIT_ADC(IDLE_HOLD_P_LIMIT_ADC),
        .TURN_CW_BRAKE_STRENGTH_Q8(TURN_CW_BRAKE_STRENGTH_Q8),.TURN_CCW_BRAKE_STRENGTH_Q8(TURN_CCW_BRAKE_STRENGTH_Q8),
        .TURN_BRAKE_MAX_EXTRA_SPEED_Q8(TURN_BRAKE_MAX_EXTRA_SPEED_Q8),
        .WAYPOINT_TICKS_PER_30DEG(WAYPOINT_TICKS_PER_30DEG),.WAYPOINT_ARRIVAL_COUNTS(WAYPOINT_ARRIVAL_COUNTS),
        .WAYPOINT_DWELL_TICKS(WAYPOINT_DWELL_TICKS),.WAYPOINT_TIMEOUT_TICKS(WAYPOINT_TIMEOUT_TICKS),
        .WAYPOINT_PASS_THROUGH(WAYPOINT_PASS_THROUGH),.WAYPOINT_PASS_COUNTS(WAYPOINT_PASS_COUNTS),
        .TURN_ENDPOINT_BRAKE_ENABLE(TURN_ENDPOINT_BRAKE_ENABLE),.TURN_ENDPOINT_WINDOW_COUNTS(TURN_ENDPOINT_WINDOW_COUNTS),
        .TURN_CW_ENDPOINT_WINDOW_COUNTS(TURN_CW_ENDPOINT_WINDOW_COUNTS),.TURN_CCW_ENDPOINT_WINDOW_COUNTS(TURN_CCW_ENDPOINT_WINDOW_COUNTS),
        .SINE_ENDPOINT_BRAKE_ENABLE(SINE_ENDPOINT_BRAKE_ENABLE),
        .SINE_POS_ENDPOINT_WINDOW_COUNTS(SINE_POS_ENDPOINT_WINDOW_COUNTS),.SINE_NEG_ENDPOINT_WINDOW_COUNTS(SINE_NEG_ENDPOINT_WINDOW_COUNTS),
        .HOME_MAX_SPEED_Q8(HOME_MAX_SPEED_Q8),.HOME_ACCEL_Q8(HOME_ACCEL_Q8),.HOME_APPROACH_SHIFT(HOME_APPROACH_SHIFT),
         .AUTO_SEQUENCE_ENABLE(AUTO_SEQUENCE_ENABLE),.ROUNDTRIP_ENABLE(ROUNDTRIP_ENABLE),.DIRECT_TURN_ENABLE(DIRECT_TURN_ENABLE),.STEP_PERIOD_TICKS(STEP_PERIOD_TICKS),
        .SEQUENCE_DWELL_TICKS(SEQUENCE_DWELL_TICKS),.MOVE_SPEED_LIMIT_Q16(MOVE_SPEED_LIMIT_Q16),.SMOOTH_STEP_ENABLE(SMOOTH_STEP_ENABLE),.MINJERK_ENABLE(MINJERK_ENABLE),.ACCEL_FF_Q8(ACCEL_FF_Q8),.SINE_ACCEL_FF_Q8(SINE_ACCEL_FF_Q8),.SINE_ENABLE(SINE_ENABLE),.SINE_AMPLITUDE(SINE_AMPLITUDE),
        .SINE_PERIOD_TICKS(SINE_PERIOD_TICKS),.SINE_CYCLES(SINE_CYCLES),.SINE_DIRECTION(SINE_DIRECTION),
        .WAVEPACKET_ENABLE(WAVEPACKET_ENABLE),.WAVEPACKET_AMPLITUDE(WAVEPACKET_AMPLITUDE),
        .WAVEPACKET_PERIOD_TICKS(WAVEPACKET_PERIOD_TICKS),.WAVEPACKET_CYCLES(WAVEPACKET_CYCLES),
        .WAVEPACKET_DIRECTION(WAVEPACKET_DIRECTION),
        .WAVEPACKET_ENVELOPE_POWER(WAVEPACKET_ENVELOPE_POWER),
        .WAVEPACKET_PROFILE(WAVEPACKET_PROFILE),
        .WAVEPACKET_PEAK_1(WAVEPACKET_PEAK_1),.WAVEPACKET_PEAK_2(WAVEPACKET_PEAK_2),
        .WAVEPACKET_PEAK_3(WAVEPACKET_PEAK_3),.WAVEPACKET_PEAK_4(WAVEPACKET_PEAK_4),
        .EXTENSION_ENABLE(EXTENSION_ENABLE),.EXTENSION_STEP_COUNTS(EXTENSION_STEP_COUNTS),
        .EXTENSION_DIRECTION(EXTENSION_DIRECTION),
        .HIGH_RES_ANGLE(HIGH_RES_ANGLE),
        .LINEAR_STEADY_ENABLE(LINEAR_STEADY_ENABLE),
        .LINEAR_STEADY_DELAY_TICKS(LINEAR_STEADY_DELAY_TICKS),.LINEAR_MAX_STEP(LINEAR_MAX_STEP),
        .LINEAR_BOOST_COMMAND(LINEAR_BOOST_COMMAND),
        .UPRIGHT_ADC(UPRIGHT_ADC),.RUNTIME_CAL(RUNTIME_CAL),.TURN_AUTO_START(TURN_AUTO_START),.CONTROL_DIV(BALANCE_CONTROL_DIV),
        .ANGLE_DEADBAND_X2(ANGLE_DEADBAND_X2),
        .DEADBAND_CONFIRM_TICKS(DEADBAND_CONFIRM_TICKS),
        .ANGLE_KP(ANGLE_KP),.STEADY_ANGLE_KP(STEADY_ANGLE_KP),.ANGLE_KD(ANGLE_KD),.STEADY_ANGLE_KD(STEADY_ANGLE_KD),
        .RECOVERY_ENABLE(RECOVERY_ENABLE),.RECOVERY_ENTRY_ADC(RECOVERY_ENTRY_ADC),
        .RECOVERY_EXIT_ADC(RECOVERY_EXIT_ADC),.RECOVERY_QUIET_TICKS(RECOVERY_QUIET_TICKS),
        .ANGLE_KI_Q8(ANGLE_KI_Q8),
        .INTEGRAL_LIMIT_COMMAND(INTEGRAL_LIMIT_COMMAND),
        .INTEGRAL_WINDOW_ADC(INTEGRAL_WINDOW_ADC),
        .MAX_D_COMMAND(MAX_D_COMMAND),
        .DERIVATIVE_WINDOW_TICKS(DERIVATIVE_WINDOW_TICKS),
        .DERIVATIVE_FILTER_SHIFT(D_FILTER_SHIFT),
        .MAX_COMMAND(MOTOR_LIMIT),.MOTOR_POLARITY(MOTOR_POLARITY),
        .POSITION_ENABLE(ENABLE_POSITION_LOOP),.POSITION_KP_Q8(POSITION_KP_Q8),
        .POSITION_KD_Q8(POSITION_KD_Q8),.POSITION_POLARITY(POSITION_POLARITY),
        .POSITION_STEADY_KD_Q8(POSITION_STEADY_KD_Q8),
        .POSITION_KI_Q16(POSITION_KI_Q16),
        .POSITION_I_LIMIT_ADC(POSITION_I_LIMIT_ADC),
        .POSITION_INTEGRAL_WINDOW_COUNTS(POSITION_INTEGRAL_WINDOW_COUNTS),
        .POSITION_INTEGRAL_QUIET_TICKS(POSITION_INTEGRAL_QUIET_TICKS),
        .POSITION_INTEGRAL_VELOCITY_LIMIT_Q8(POSITION_INTEGRAL_VELOCITY_LIMIT_Q8),
        .POSITION_INTEGRAL_MOVING_DECAY_SHIFT(POSITION_INTEGRAL_MOVING_DECAY_SHIFT),
        .POSITION_INTEGRAL_DEADBAND_COUNTS(POSITION_INTEGRAL_DEADBAND_COUNTS),
        .POSITION_INTEGRAL_OVERSHOOT_COUNTS(POSITION_INTEGRAL_OVERSHOOT_COUNTS),
        .POSITION_INTEGRAL_UNWIND_SHIFT(POSITION_INTEGRAL_UNWIND_SHIFT),
        .REFERENCE_HOLD_I_DEADBAND_COUNTS(REFERENCE_HOLD_I_DEADBAND_COUNTS),
        .REFERENCE_SETTLE_I_DEADBAND_COUNTS(REFERENCE_SETTLE_I_DEADBAND_COUNTS),
        .POSITION_TARGET_MODE(POSITION_TARGET_MODE),
        .POSITION_TURN_MIN_TRAVEL_COUNTS(POSITION_TURN_MIN_TRAVEL_COUNTS),
        .POSITION_TURN_REVERSE_COUNTS(POSITION_TURN_REVERSE_COUNTS),
        .POSITION_TURN_REVERSE_TICKS(POSITION_TURN_REVERSE_TICKS),
        .POSITION_TURN_STOP_TICKS(POSITION_TURN_STOP_TICKS),
        .POSITION_TARGET_COUNTS(TARGET_OFFSET_COUNTS),
        .DIRECT_POSITION_TARGET_COUNTS(DIRECT_TARGET_OFFSET_COUNTS),
        .POSITION_TOLERANCE_COUNTS(POSITION_TOLERANCE_COUNTS),
        .POSITION_VELOCITY_FILTER_SHIFT(POSITION_VELOCITY_FILTER_SHIFT),
        .POSITION_VELOCITY_WINDOW_TICKS(POSITION_VELOCITY_WINDOW_TICKS),
        .POSITION_VELOCITY_OBSERVER_ENABLE(POSITION_VELOCITY_OBSERVER_ENABLE),
        .MAX_TARGET_OFFSET_ADC(BIAS_LIMIT_ADC),
        .STEADY_MAX_TARGET_OFFSET_ADC(STEADY_BIAS_LIMIT_ADC),
        .REFERENCE_POSITION_P_LIMIT_ADC(REFERENCE_POSITION_P_LIMIT_ADC),
        .TURN_DISTANCE_BRAKE_ENABLE(TURN_DISTANCE_BRAKE_ENABLE),.TURN_DISTANCE_WINDOW_SHIFT(TURN_DISTANCE_WINDOW_SHIFT),.TURN_DISTANCE_WINDOW_COUNTS(TURN_DISTANCE_WINDOW_COUNTS),
        .TURN_BRAKE_ENABLE(TURN_BRAKE_ENABLE),.TURN_BRAKE_LOOKAHEAD_TICKS(TURN_BRAKE_LOOKAHEAD_TICKS),
        .TURN_BRAKE_MAX_LEAD_COUNTS(TURN_BRAKE_MAX_LEAD_COUNTS),
        .POSITION_STEADY_P_LIMIT_ADC(POSITION_STEADY_P_LIMIT_ADC),
        .POSITION_STEADY_P_DEADBAND_COUNTS(POSITION_STEADY_P_DEADBAND_COUNTS),
        .MAX_TARGET_STEP_Q8(BIAS_SLEW_Q8),
        .TURN_POSITION_STEP_Q8(TURN_POSITION_STEP_Q8),.TURN_LAUNCH_STEP_Q8(TURN_LAUNCH_STEP_Q8),.TURN_LAUNCH_TICKS(TURN_LAUNCH_TICKS),
        .STEADY_POSITION_STEP_Q8(STEADY_POSITION_STEP_Q8),.STEADY_DAMPING_STEP_Q8(STEADY_DAMPING_STEP_Q8),
        .MIN_ACTIVE_COMMAND(MIN_ACTIVE_COMMAND),
        .OUTPUT_RELEASE_STEP(OUTPUT_RELEASE_STEP),
        .START_COMMAND(START_COMMAND),.START_TICKS(START_TICKS)
    ) balance (
        .clk(clk_50m),.rst_n(rst_n),.track(run_active && controller_permit),
        .enable(balance_enable && controller_permit),
        .start(balance_start),.inner_tick(balance_control_tick),.outer_tick(outer_tick),
        .capture_velocity_q8(capture_velocity_q8),
        .capture_here(direct_entry),
        .motion_command(motion_command),.run_origin(run_origin),
        .angle_x2(angle_x2),.angle_q8(angle_q8),.upright_adc_q8(upright_cal_q8),.position(selected_position),.command(balance_command),
        .startup_zero(startup_zero),
        .tracking_command(tracking_command),
        .correction_q8(correction_q8),.angle_target_q8(angle_target_q8),
        .target_position(target_position),.position_velocity(position_velocity),
        .angle_delta_q8(angle_delta_q8),.position_target_locked(position_target_locked),
        .quiet_servo_active(quiet_servo_active),.mission_phase(mission_phase),
        .position_integral_q16(position_integral_q16),.mission_velocity_q8(mission_velocity_q8),
        .extension_goal(extension_goal),.extension_anchor(extension_anchor),.extension_dwell(extension_dwell)
    );
    // Mode 0: LED1 = calibrated (off while K1 held), LED2 = swinging up /
    //         settling until the arm is ready for K3/K4.
    // Mode 1 (LED3 on): LED1 = CW turn running, LED2 = CCW turn running.
    // Mode 2 (LED4 on): LED1 = sine running, LED2 = diverge/converge running.
    reg calibrated;
    always @(posedge clk_50m or negedge rst_n) begin
        if(!rst_n) calibrated<=0;
        else if(calibrate_accept) calibrated<=1;
        else if(calibrate_pulse && key_mode==0) calibrated<=0;
    end
    wire led_settling=run_active && (!balance_enable || mission_phase==0 || mission_phase==1);
    wire led1=key_mode==1 ? mission_phase==4 : key_mode==2 ? mission_phase==11 :
        calibrated && !(start_pressed && !run_active);
    wire led2=key_mode==1 ? mission_phase==7 : key_mode==2 ? mission_phase==2 : led_settling;
    wire [3:0] led_on={key_mode==2, key_mode==1, led2, led1};
    assign led=(LED_ACTIVE_LOW!=0) ? ~led_on : led_on;
    motor_signed_pwm #(
        .PWM_PERIOD(PWM_PERIOD),.REVERSE_BLANK_CYCLES(PWM_PERIOD)
    ) motor (
        .clk(clk_50m),.rst_n(rst_n),.drive_enable(motor_drive_enable),
        .fault(fault_pending || emergency_stop),.command(bounded_motor_command),
        .in1(motor_in1),.in2(motor_in2),.pwm(motor_pwm)
    );
    assign AN1=(MOTOR_PORT==2) ? motor_in1 : 1'b0;
    assign AN2=(MOTOR_PORT==2) ? motor_in2 : 1'b0;
    assign PWMA=(MOTOR_PORT==2) ? motor_pwm : 1'b0;
    assign BN1=(MOTOR_PORT==1) ? motor_in1 : 1'b0;
    assign BN2=(MOTOR_PORT==1) ? motor_in2 : 1'b0;
    assign PWMB=(MOTOR_PORT==1) ? motor_pwm : 1'b0;
    telemetry_v1 #(.FRAME_DIV(TELEMETRY_DIV),.BAUD(UART_BAUD),.EXTENDED_ANGLE(EXTENSION_ENABLE),
        .EXTENDED_STATE(EXTENDED_TELEMETRY),.SUPERVISOR_STATE(RETRY_ENABLE),.ROUNDTRIP_STATE(EXTENSION_ENABLE),.KEY_STATE(EXTENSION_ENABLE)) telemetry (
        .clk(clk_50m),.rst_n(rst_n),.enable(serial_enabled),.status(telemetry_status),
        .supervisor_data({4'b0,extension_abort,retry_phase,retry_attempt,retry_elapsed}),
        .goal_position(extension_goal),.anchor_position(extension_anchor),
        .dwell_ticks(extension_dwell),.board_ticks(board_control_ticks),
        .key_data({(WAVEPACKET_ENABLE==0),(WAVEPACKET_ENABLE==0 && WAYPOINT_PASS_THROUGH!=0),3'b0,motion_command,telemetry_key_count,telemetry_last_motion,telemetry_last_key,key_mode}),
        .angle_raw(angle_raw),.encoder1(telemetry_encoder1),.encoder2(telemetry_encoder2_sample),
        .angle_metadata({mission_phase,telemetry_target_locked,1'b0}),
        .precise_angle_q8({14'b0,angle_q8}),.correction_q8(correction_q8),
        .position_velocity_q8(position_velocity),.position_integral_q16(position_integral_q16),
        .reference_velocity_q8(mission_velocity_q8),
        .motor_command(motor_command),.uart_tx_pin(uart_tx)
    );
endmodule
