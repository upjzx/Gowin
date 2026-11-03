// Resonant push/brake pairs at local extrema; no global ADC-to-angle assumption.
module swing_balance_fsm #(
    parameter integer TEST_MODE=1, UPRIGHT_ADC=767, HANGING_ADC=225,
    // 1: use the upright_adc/hanging_adc ports (K1 calibration) instead of
    // the compiled constants. Ports are ignored when 0.
    parameter integer RUNTIME_CAL=0,
    parameter integer CAPTURE_WINDOW_ADC=24, TRIP_WINDOW_ADC=60,
    parameter integer CAPTURE_TRIP_WINDOW_ADC=400,
    parameter integer BALANCE_GRACE_TICKS=100,
    // Optional bounded experiment; zero permits continuous normal operation.
    parameter integer BALANCE_TEST_TICKS=0,
    parameter integer START_DOWN_WINDOW_ADC=40, CAPTURE_SPEED_ADC=4,
    parameter integer CAPTURE_CART_SPEED_COUNTS=3,
    parameter integer CAPTURE_HOLD_TICKS=2, ADC_JUMP_LIMIT=96,
    parameter integer TREND_STEP_ADC=2, EXTREMA_EXCURSION_ADC=12,
    parameter integer SWING_DUTY=1000, SWING_MAX_DUTY=1500, SWING_DUTY_STEP=100,
    parameter integer PULSE_TICKS=12, BRAKE_TICKS=12,
    parameter integer SWING_TIMEOUT_TICKS=2000, NO_EXTREMA_TICKS=160,
    parameter integer MAX_PULSE_PAIRS=16, SWING_POLARITY=0,
    parameter integer SOFT_TRAVEL_COUNTS=500,
    parameter integer ENERGY_COS_SWITCH_ADC=240,
    parameter integer TOP_BRAKE_WINDOW_ADC=110,
    parameter integer TOP_BRAKE_DUTY=3500,
    parameter integer TOP_DEPARTURE_BRAKE_ENABLE=0,
    parameter integer TOP_SPEED_PRIORITY_ENABLE=0,
    parameter integer ENERGY_GUARD_ENABLE=0, TOP_BRAKE_GAIN=512,
    // 1: approach braking inside TOP_BRAKE_WINDOW_ADC is sized by excess
    // energy instead of a fixed duty.  Near upright the arm just reaches the
    // top when |speed| = C*|error| (C = omega0*tick, Q8, ADC counts/tick per
    // count).  Brake = GAIN*(speed^2-(C*error)^2), zero when energy is short.
    parameter integer ENERGY_BRAKE_ENABLE=0, ENERGY_BRAKE_C_Q8=8,
    parameter integer ENERGY_BRAKE_GAIN=128,
    // Kept for source compatibility. End codes are valid angles for this sensor.
    parameter integer MAX_TRAVEL_COUNTS=400, ADC_RAIL_TICKS=20
) (
    input wire clk, rst_n, tick, start, stop, emergency,
    input wire sensor_fresh, encoder_fault,
    input wire [9:0] angle_raw,
    input wire [9:0] upright_adc, hanging_adc,
    input wire signed [31:0] position,
    output reg [2:0] state,
    output reg [3:0] fault_code,
    output reg balance_start,
    output wire balance_enable, run_active, swing_active, fault_pending,
    output wire top_speed_brake_active,
    output reg signed [31:0] capture_velocity_q8,
    output reg signed [15:0] swing_command,
    output reg signed [31:0] run_origin
);
    localparam [2:0] IDLE=3'd0, KICK=3'd1, MONITOR=3'd2, PUSH=3'd3,
                     BRAKE=3'd4, BALANCE=3'd5, FAULT=3'd6;
    localparam signed [15:0] UPRIGHT_16=UPRIGHT_ADC[15:0], HANGING_16=HANGING_ADC[15:0];
    localparam signed [15:0] TOP_BRAKE_DUTY_16=TOP_BRAKE_DUTY[15:0];
    reg [9:0] previous_angle;
    reg history_valid;
    reg signed [1:0] trend;
    reg [31:0] pulse_timer, total_timer, monitor_timer, pair_count;
    reg [31:0] capture_count, balance_timer;
    reg pulse_positive;
    reg signed [15:0] amplitude;
    reg signed [31:0] previous_position_tick, cart_velocity_tick;
    reg signed [15:0] swing_delta_sampled;
    reg swing_delta_valid;
    reg energy_guard,guard_positive,guard_brake;
    reg [1:0] guard_reverse_ticks,guard_slow_ticks;
    reg signed [15:0] guard_brake_command;
    // The pendulum sensor is circular: 0 follows 1023.  Always use the
    // shortest signed distance so crossing that seam is a one-count move.
    function signed [15:0] circular_difference;
        input [9:0] current_value;
        input [9:0] reference_value;
        reg signed [15:0] linear_difference;
        begin
            linear_difference=$signed({6'b0,current_value})-
                              $signed({6'b0,reference_value});
            if(linear_difference>16'sd511)
                circular_difference=linear_difference-16'sd1024;
            else if(linear_difference< -16'sd512)
                circular_difference=linear_difference+16'sd1024;
            else
                circular_difference=linear_difference;
        end
    endfunction
    wire [9:0] upright_ref=RUNTIME_CAL!=0 ? upright_adc : UPRIGHT_16[9:0];
    wire [9:0] hanging_ref=RUNTIME_CAL!=0 ? hanging_adc : HANGING_16[9:0];
    wire signed [15:0] angle_error=circular_difference(angle_raw,upright_ref);
    wire signed [15:0] abs_error=angle_error[15] ? -angle_error : angle_error;
    wire signed [15:0] down_error=circular_difference(angle_raw,hanging_ref);
    wire signed [15:0] abs_down=down_error[15] ? -down_error : down_error;
    wire signed [15:0] previous_down_error=
        circular_difference(previous_angle,hanging_ref);
    wire signed [15:0] delta=circular_difference(angle_raw,previous_angle);
    wire signed [15:0] abs_delta=delta[15] ? -delta : delta;
    wire local_sample=history_valid && (abs_delta<=ADC_JUMP_LIMIT);
    wire signed [15:0] previous_top_error=circular_difference(previous_angle,upright_ref);
    // Error also changes sign opposite upright. Check location as well as
    // sign, so the hanging seam cannot be mistaken for a full-speed top pass.
    wire fast_top_pass=abs_error<=TOP_BRAKE_WINDOW_ADC && abs_delta>CAPTURE_SPEED_ADC &&
        ((previous_top_error<0 && angle_error>=0) || (previous_top_error>0 && angle_error<=0));
    wire reverse_from_pass=(guard_positive && delta<=-TREND_STEP_ADC) ||
                           (!guard_positive && delta>=TREND_STEP_ADC);
    wire signed [31:0] brake_magnitude=$signed(abs_delta)*TOP_BRAKE_GAIN;
    wire signed [15:0] bounded_brake=brake_magnitude>TOP_BRAKE_DUTY ? TOP_BRAKE_DUTY_16 : brake_magnitude[15:0];
    // Excess-energy brake magnitude, registered once per control tick.
    wire signed [15:0] abs_delta_sampled=swing_delta_sampled[15] ? -swing_delta_sampled : swing_delta_sampled;
    reg signed [47:0] speed_sq_q16, needed_sq_q16, excess_scaled;
    reg [15:0] excess_brake;
    wire signed [31:0] speed_q8=$signed(abs_delta_sampled) <<< 8;
    // Break calibrated angle -> circular error -> gain -> square into stages.
    reg signed [31:0] needed_q8;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) needed_q8<=0;
        else needed_q8<=$signed(abs_error)*ENERGY_BRAKE_C_Q8;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin speed_sq_q16<=0;needed_sq_q16<=0;excess_scaled<=0;excess_brake<=0;end
        else begin
            speed_sq_q16<=speed_q8*speed_q8;
            needed_sq_q16<=needed_q8*needed_q8;
            excess_scaled<=(speed_sq_q16-needed_sq_q16)*ENERGY_BRAKE_GAIN;
            excess_brake<=excess_scaled<=0 ? 16'd0 :
                (excess_scaled>>>16)>TOP_BRAKE_DUTY ? TOP_BRAKE_DUTY_16 : excess_scaled[31:16];
        end
    end
    wire excess_mode=(ENERGY_BRAKE_ENABLE!=0);
    wire signed [15:0] excess_brake_s=$signed(excess_brake);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin energy_guard<=0;guard_positive<=0;guard_reverse_ticks<=0;
            guard_brake<=0;guard_slow_ticks<=0;guard_brake_command<=0;end
        else if(!swing_active || stop || emergency || !sensor_fresh || ENERGY_GUARD_ENABLE==0) begin
            energy_guard<=0;guard_positive<=0;guard_reverse_ticks<=0;
            guard_brake<=0;guard_slow_ticks<=0;guard_brake_command<=0;
        end else if(tick && local_sample) begin
            guard_brake_command<=(delta>0) ^ (SWING_POLARITY!=0) ? bounded_brake : -bounded_brake;
            if(abs_error>TOP_BRAKE_WINDOW_ADC+24) begin guard_brake<=0;guard_slow_ticks<=0;end
            else if(abs_error<=TOP_BRAKE_WINDOW_ADC && abs_delta>CAPTURE_SPEED_ADC) begin
                guard_brake<=1;guard_slow_ticks<=0;
            end else if(abs_delta<=1) begin
                if(guard_slow_ticks>=1) guard_brake<=0;
                else guard_slow_ticks<=guard_slow_ticks+1'b1;
            end else guard_slow_ticks<=0;
            if(fast_top_pass) begin energy_guard<=1;guard_positive<=delta>0;guard_reverse_ticks<=0;end
            else if(energy_guard && reverse_from_pass) begin
                if(guard_reverse_ticks>=2) begin energy_guard<=0;guard_reverse_ticks<=0;end
                else guard_reverse_ticks<=guard_reverse_ticks+1'b1;
            end else guard_reverse_ticks<=0;
        end
    end
    // Qualify catch with this interval, not the previous registered velocity.
    // Widen before subtracting so a large encoder discontinuity cannot wrap.
    wire signed [32:0] capture_cart_delta=$signed({position[31],position})-
                                         $signed({previous_position_tick[31],previous_position_tick});
    wire signed [32:0] abs_cart_velocity_tick=capture_cart_delta[32] ?
        -capture_cart_delta : capture_cart_delta;
    wire catch_ready=local_sample && (abs_error<=CAPTURE_WINDOW_ADC) &&
                     (abs_delta<=CAPTURE_SPEED_ADC) &&
                     (abs_cart_velocity_tick<=CAPTURE_CART_SPEED_COUNTS);
    // A reversal is observed on the sample after the physical extremum, so
    // qualify the excursion with previous_angle (the peak), not angle_raw.
    wire high_turn=local_sample && trend==1 && delta<=-TREND_STEP_ADC &&
                   previous_down_error>=EXTREMA_EXCURSION_ADC;
    wire low_turn=local_sample && trend==-1 && delta>=TREND_STEP_ADC &&
                  previous_down_error<=-EXTREMA_EXCURSION_ADC;
    wire signed [32:0] travel=$signed({position[31],position})-
                             $signed({run_origin[31],run_origin});
    // Predict roughly one pulse ahead.  Position alone is insufficient near
    // the origin: a fast cart can cross the entire safe area during the next
    // pulse.  Velocity is encoder counts per 5 ms controller tick.
    // Pipeline the soft swing-up prediction only. Raw hard travel protection
    // below remains combinational and immediate. Settles within80ns vs5ms tick.
    reg signed [32:0] soft_travel_sample;
    reg signed [63:0] soft_velocity_projection,projected_travel;
    reg force_negative,force_positive;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin soft_travel_sample<=0;soft_velocity_projection<=0;
            projected_travel<=0;force_negative<=0;force_positive<=0;end
        else begin
            soft_travel_sample<=travel;
            soft_velocity_projection<=$signed(cart_velocity_tick)*$signed(PULSE_TICKS);
            projected_travel<=$signed(soft_travel_sample)+soft_velocity_projection;
            force_negative<=projected_travel>SOFT_TRAVEL_COUNTS;
            force_positive<=projected_travel< -SOFT_TRAVEL_COUNTS;
        end
    end
    wire travel_exceeded=(travel>MAX_TRAVEL_COUNTS || travel< -MAX_TRAVEL_COUNTS);
    assign balance_enable=(state==BALANCE);
    assign swing_active=(state==KICK || state==MONITOR || state==PUSH || state==BRAKE);
    assign run_active=swing_active || balance_enable;
    wire balance_in_grace=(balance_timer<BALANCE_GRACE_TICKS);
    wire balance_angle_outside=balance_in_grace ?
        (abs_error>CAPTURE_TRIP_WINDOW_ADC) : (abs_error>TRIP_WINDOW_ADC);
    assign fault_pending=run_active && (!sensor_fresh || encoder_fault ||
        travel_exceeded || (balance_enable && balance_angle_outside));
    wire pulse_sign=pulse_positive ^ (SWING_POLARITY!=0);
    // Energy pumping sign: bang-bang sign(theta_dot*cos(theta)).  The first
    // hardware trial with the opposite sign reduced the oscillation, which
    // establishes this board's motor/ADC orientation directly.
    wire energy_positive_unoriented=
        ((trend== 1) && (abs_down<ENERGY_COS_SWITCH_ADC)) ||
        ((trend==-1) && (abs_down>=ENERGY_COS_SWITCH_ADC));
    wire monitor_positive_unoriented=force_negative ? 1'b0 :
                                     force_positive ? 1'b1 :
                                     energy_positive_unoriented;
    wire monitor_sign=monitor_positive_unoriented ^ (SWING_POLARITY!=0);
    wire moving_toward_upright=
        ((angle_error<0) && (trend==1)) ||
        ((angle_error>0) && (trend==-1));
    wire top_brake_positive_unoriented=(trend==1);
    wire top_brake_sign=top_brake_positive_unoriented ^ (SWING_POLARITY!=0);
    // Hold the measured 5 ms interval across the following control interval.
    // Combinational delta becomes zero when previous_angle is updated; using
    // it directly would remove departure braking immediately after the tick.
    // Brake only fast outward motion inside the existing top window. At low
    // speed or a reversal, return to the original pump/approach policy.
    wire fast_top_departure=(TOP_DEPARTURE_BRAKE_ENABLE!=0) && swing_delta_valid &&
        (abs_error<=TOP_BRAKE_WINDOW_ADC) &&
        (((angle_error>=0) && (swing_delta_sampled>CAPTURE_SPEED_ADC)) ||
         ((angle_error<=0) && (swing_delta_sampled< -CAPTURE_SPEED_ADC)));
    wire departure_brake_sign=(swing_delta_sampled>0) ^ (SWING_POLARITY!=0);
    // Fast approach must keep braking through the soft-PD mux. Otherwise the
    // position-proportional term can accelerate a fast pass inside +/-40 ADC.
    wire legacy_speed_brake=(TOP_SPEED_PRIORITY_ENABLE!=0) &&
        (state==MONITOR) && local_sample && swing_delta_valid &&
        (abs_error<=TOP_BRAKE_WINDOW_ADC) &&
        ((swing_delta_sampled>CAPTURE_SPEED_ADC) ||
         (swing_delta_sampled< -CAPTURE_SPEED_ADC));
    // Excess mode: keep priority over soft-PD whenever surplus energy remains,
    // so a fast arm is still slowed inside +/-SOFT_CAPTURE_WINDOW.
    wire excess_approach=excess_mode && (state==MONITOR) && local_sample &&
        swing_delta_valid && (abs_error<=TOP_BRAKE_WINDOW_ADC) &&
        moving_toward_upright && excess_brake!=0;
    assign top_speed_brake_active=excess_mode ? excess_approach : (state==MONITOR) &&
        ((ENERGY_GUARD_ENABLE!=0 && guard_brake) || legacy_speed_brake);
    wire signed [31:0] boosted_amplitude=$signed(amplitude)+SWING_DUTY_STEP;
    always @* begin
        swing_command=0;
        if(state==KICK || state==PUSH)
            swing_command=pulse_sign ? amplitude : -amplitude;
        else if(state==MONITOR && local_sample && trend!=0) begin
            // Energy pumping must stop before the upright point.  Driving in
            // the direction of angular travel moves the cart under the arm
            // and removes angular speed, giving the linear balance loop a
            // catchable hand-off instead of a full-speed fly-through.
            if(excess_mode) begin
                // Approach: brake only the surplus; coast when energy is short.
                if(abs_error<=TOP_BRAKE_WINDOW_ADC && moving_toward_upright)
                    swing_command=departure_brake_sign ? excess_brake_s : -excess_brake_s;
                else if(fast_top_departure)
                    swing_command=departure_brake_sign ? TOP_BRAKE_DUTY_16 : -TOP_BRAKE_DUTY_16;
                else
                    swing_command=monitor_sign ? amplitude : -amplitude;
            end else if(top_speed_brake_active || fast_top_departure)
                swing_command=departure_brake_sign ? TOP_BRAKE_DUTY_16 : -TOP_BRAKE_DUTY_16;
            else if(abs_error<=TOP_BRAKE_WINDOW_ADC && moving_toward_upright)
                swing_command=top_brake_sign ? TOP_BRAKE_DUTY_16 : -TOP_BRAKE_DUTY_16;
            else
                swing_command=monitor_sign ? amplitude : -amplitude;
            if(ENERGY_GUARD_ENABLE!=0 && !excess_mode) begin
                if(guard_brake) swing_command=guard_brake_command;
                else if(energy_guard) swing_command=0;
            end
        end
        // BRAKE is an all-off coast interval.  V3.1 immediately reversed the
        // motor here, cancelling the short acceleration before the arm could
        // move.  The signed PWM block drives both direction pins low at zero.
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE; fault_code<=0; balance_start<=0; run_origin<=0; capture_velocity_q8<=0;
            previous_angle<=0; history_valid<=0; trend<=0;
            pulse_timer<=0; total_timer<=0; monitor_timer<=0; pair_count<=0;
            capture_count<=0; balance_timer<=0;
            pulse_positive<=1; amplitude<=SWING_DUTY[15:0];
            previous_position_tick<=0; cart_velocity_tick<=0;
            swing_delta_sampled<=0; swing_delta_valid<=0;
        end else begin
            balance_start<=0;
            if(tick) begin
                swing_delta_sampled<=delta;
                swing_delta_valid<=local_sample && sensor_fresh && (state==MONITOR);
                previous_angle<=angle_raw;
                history_valid<=sensor_fresh;
                cart_velocity_tick<=position-previous_position_tick;
                previous_position_tick<=position;
                if(!local_sample) trend<=0;
                else if(delta>=TREND_STEP_ADC) trend<=2'sd1;
                else if(delta<=-TREND_STEP_ADC) trend<=-2'sd1;
            end
            // These protections do not wait for the 5 ms controller tick.
            if(emergency) begin state<=FAULT; fault_code<=1; end
            else if(state==FAULT) begin state<=FAULT; end
            else if(stop) begin state<=IDLE; capture_count<=0; balance_timer<=0; end
            else if(fault_pending) begin
                state<=FAULT;
                if(!sensor_fresh) fault_code<=2;
                else if(encoder_fault) fault_code<=5;
                else if(travel_exceeded) fault_code<=4;
                else fault_code<=3;
            end else if(state==IDLE) begin
                if(start && sensor_fresh && !encoder_fault && local_sample) begin
                    capture_velocity_q8<=0;
                    run_origin<=position;
                    previous_position_tick<=position;
                    cart_velocity_tick<=0;
                    capture_count<=0; balance_timer<=0;
                    total_timer<=0; monitor_timer<=0;
                    pair_count<=0; pulse_timer<=0; amplitude<=SWING_DUTY[15:0];
                    if(catch_ready) begin state<=BALANCE; balance_start<=1; balance_timer<=0; end
                    else if(TEST_MODE==2 && abs_down<=START_DOWN_WINDOW_ADC &&
                            abs_delta<=CAPTURE_SPEED_ADC) begin
                        state<=KICK; pulse_positive<=1;
                    end
                end
            end else if(tick && swing_active) begin
                total_timer<=total_timer+1'b1;
                if(catch_ready) capture_count<=capture_count+1'b1;
                else capture_count<=0;
                // Angle AND speed qualify takeover. Check every swing state.
                if(catch_ready && capture_count>=CAPTURE_HOLD_TICKS-1) begin
                    capture_velocity_q8<=capture_cart_delta <<< 8;
                    state<=BALANCE; balance_start<=1; capture_count<=0; balance_timer<=0;
                end else if(total_timer>=SWING_TIMEOUT_TICKS-1 || pair_count>=MAX_PULSE_PAIRS) begin
                    state<=FAULT; fault_code<=6;
                end else if(!local_sample) begin
                    // A potentiometer discontinuity must not masquerade as an extremum.
                    state<=MONITOR; pulse_timer<=0; monitor_timer<=0;
                end else begin
                    case(state)
                        KICK, PUSH: begin
                            if(pulse_timer>=PULSE_TICKS-1) begin state<=BRAKE; pulse_timer<=0; end
                            else pulse_timer<=pulse_timer+1'b1;
                        end
                        BRAKE: begin
                            if(pulse_timer>=BRAKE_TICKS-1) begin
                                state<=MONITOR; pulse_timer<=0; monitor_timer<=0;
                                pair_count<=pair_count+1'b1;
                                if(boosted_amplitude>SWING_MAX_DUTY) amplitude<=SWING_MAX_DUTY[15:0];
                                else amplitude<=boosted_amplitude[15:0];
                            end else pulse_timer<=pulse_timer+1'b1;
                        end
                        MONITOR: begin
                            if(high_turn || low_turn) begin
                                // Continuous energy pumping stays in MONITOR.
                                // Use each completed half-cycle to ramp the
                                // available drive and count the timeout limit.
                                state<=MONITOR;
                                pair_count<=pair_count+1'b1;
                                if(ENERGY_GUARD_ENABLE!=0 && energy_guard) amplitude<=amplitude;
                                else if(boosted_amplitude>SWING_MAX_DUTY)
                                    amplitude<=SWING_MAX_DUTY[15:0];
                                else
                                    amplitude<=boosted_amplitude[15:0];
                                pulse_timer<=0; monitor_timer<=0;
                            end else if(monitor_timer>=NO_EXTREMA_TICKS-1) begin
                                state<=FAULT; fault_code<=7;
                            end else monitor_timer<=monitor_timer+1'b1;
                        end
                        default: begin state<=FAULT; fault_code<=9; end
                    endcase
                end
            end else if(state==BALANCE && tick) begin
                if(BALANCE_TEST_TICKS>0 && balance_timer>=BALANCE_TEST_TICKS-1) begin
                    state<=IDLE; capture_count<=0; balance_timer<=0;
                end else if(balance_timer<BALANCE_GRACE_TICKS ||
                            (BALANCE_TEST_TICKS>0 && balance_timer<BALANCE_TEST_TICKS))
                    balance_timer<=balance_timer+1'b1;
            end
        end
    end
endmodule
