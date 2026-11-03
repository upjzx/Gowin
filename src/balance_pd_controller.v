// PID with selectable half-count/Q8 measurement precision and measured-velocity D.
// Position target always remains effective, even when angle P is suppressed.
module balance_pd_controller #(
    parameter integer UPRIGHT_ADC_X2 = 1527,
    // 1: take the upright point from the upright_adc_q8 port (K1 calibration).
    parameter integer RUNTIME_CAL = 0,
    parameter integer HIGH_RES_ANGLE = 0,
    parameter integer CONTINUOUS_ENABLE = 0,
    parameter integer RECOVERY_ENABLE = 0,
    parameter integer RECOVERY_ENTRY_ADC = 12, RECOVERY_EXIT_ADC = 4,
    parameter integer RECOVERY_QUIET_TICKS = 1000,
    parameter integer UPRIGHT_DITHER_ENABLE = 0,
    parameter integer ANGLE_DEADBAND_X2 = 1,
    parameter integer DEADBAND_CONFIRM_TICKS = 5,
    parameter integer CONTROL_DIV = 25000,
    parameter signed [15:0] KP = 16'sd250,
    parameter signed [15:0] STEADY_KP = 16'sd0,
    parameter signed [15:0] KD = 16'sd100,
    // Optional steady damping; preserve acquisition gains and D history.
    parameter signed [15:0] STEADY_KD = 16'sd0,
    parameter signed [15:0] REFERENCE_KP = 16'sd80, REFERENCE_KD = 16'sd600,
    parameter signed [15:0] KI_Q8 = 16'sd0,
    parameter integer INTEGRAL_LIMIT_COMMAND = 300,
    parameter integer INTEGRAL_WINDOW_ADC = 10,
    parameter integer MAX_D_COMMAND = 1000,
    parameter integer MAX_COMMAND = 2000,
    parameter integer DERIVATIVE_WINDOW_TICKS = 4,
    parameter integer DERIVATIVE_FILTER_SHIFT = 1,
    parameter integer MOTOR_POLARITY = 0
) (
    // track_only keeps measured D history warm without winding up I or
    // qualifying the balance deadband while the swing controller owns PWM.
    input wire clk, rst_n, enable, track_only, update,
    input wire [10:0] angle_x2,
    input wire [17:0] angle_q8,
    input wire [17:0] upright_adc_q8,
    input wire continuous_mode, reference_mode,
    input wire signed [31:0] target_offset_q8,
    output reg signed [15:0] command,
    output wire signed [31:0] measured_velocity_q8,
    output reg command_update,
    output reg inside_deadband, deadband_confirmed
);
    localparam integer N = DERIVATIVE_WINDOW_TICKS > 0 ? DERIVATIVE_WINDOW_TICKS : 1;
    // This design's clock is 50MHz: 50000 clocks = 1ms, full cycle = 2ms.
    localparam integer UPRIGHT_HALF_PERIOD_CYCLES = 50000;
    reg [15:0] upright_count;
    reg upright_high;
    wire signed [31:0] upright_base_x2 = RUNTIME_CAL != 0 ?
        $signed({14'b0,upright_adc_q8} + 32'sd64) >>> 7 : UPRIGHT_ADC_X2;
    wire signed [31:0] upright_base_q8 = RUNTIME_CAL != 0 ?
        $signed({14'b0,upright_adc_q8}) : UPRIGHT_ADC_X2 <<< 7;
    wire signed [31:0] live_upright_adc_x2 = upright_base_x2 +
        ((UPRIGHT_DITHER_ENABLE != 0 && upright_high) ? 32'sd1 : 32'sd0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin upright_count <= 0; upright_high <= 0; end
        else if (!enable || UPRIGHT_DITHER_ENABLE == 0) begin
            upright_count <= 0; upright_high <= 0;
        end else if (upright_count == UPRIGHT_HALF_PERIOD_CYCLES - 1) begin
            upright_count <= 0; upright_high <= !upright_high;
        end else upright_count <= upright_count + 1'b1;
    end
    // delta is in half-counts; D stays in whole ADC counts per 5ms reference.
    localparam integer VELOCITY_SCALE_Q8 = (250000 * 256) / (CONTROL_DIV * N * 2);
    reg signed [18:0] history [0:N-1];
    reg history_ready;
    reg [31:0] history_index, center_count;
    reg signed [31:0] velocity_q8, error_hold_q8;
    assign measured_velocity_q8=velocity_q8;
    reg signed [47:0] p_hold, d_hold;
    // Q16 preserves half-count/sub-count integration instead of discarding
    // every fractional increment in a Q8 right shift.
    reg signed [31:0] integral_q16, integral_candidate_q16;
    reg signed [48:0] raw_hold;
    reg [2:0] stage;
    integer i;
    wire signed [11:0] measurement = $signed({1'b0,angle_x2});
    wire signed [18:0] control_measurement = HIGH_RES_ANGLE ?
        $signed({1'b0,angle_q8}) : $signed({1'b0,angle_x2,7'b0});
    wire signed [31:0] measured_error_x2 = measurement - live_upright_adc_x2;
    wire center_now = measured_error_x2 <= ANGLE_DEADBAND_X2 && measured_error_x2 >= -ANGLE_DEADBAND_X2;
    wire qualified_now = !(CONTINUOUS_ENABLE!=0 && continuous_mode) && !track_only && center_now && (DEADBAND_CONFIRM_TICKS <= 1 || center_count >= DEADBAND_CONFIRM_TICKS - 1);
    wire signed [31:0] precise_error_q8 = $signed(control_measurement) - upright_base_q8 -
        ((UPRIGHT_DITHER_ENABLE != 0 && upright_high) ? 32'sd128 : 32'sd0);
    wire signed [31:0] angle_p_error_q8 = qualified_now ? 32'sd0 : precise_error_q8;
    wire signed [31:0] effective_error_q8 = angle_p_error_q8 - target_offset_q8;
    wire signed [18:0] delta_q8 = control_measurement - history[history_index];
    wire signed [31:0] delta_x2 = $signed(delta_q8) >>> 7;
    localparam integer PRECISE_VELOCITY_SCALE_Q8 = (250000 * 256) / (CONTROL_DIV * N);
    wire signed [63:0] precise_velocity_product = $signed(delta_q8) * PRECISE_VELOCITY_SCALE_Q8;
    wire signed [31:0] measured_velocity = HIGH_RES_ANGLE ?
        (precise_velocity_product >>> 8) :
        (delta_x2 * VELOCITY_SCALE_Q8);
    wire signed [31:0] filtered_velocity = velocity_q8 + ((measured_velocity - velocity_q8) >>> DERIVATIVE_FILTER_SHIFT);
    wire signed [31:0] velocity_next = (measured_velocity == 0 && velocity_q8 < (1 << DERIVATIVE_FILTER_SHIFT) &&
        velocity_q8 > -(1 << DERIVATIVE_FILTER_SHIFT)) ? 32'sd0 : filtered_velocity;
    // Large post-capture deviations use the proven acquisition P gain without
    // resetting D history or re-enabling the minimum-PWM acquisition boost.
    // Hysteresis and a stationary interval prevent gain chatter on the boundary.
    reg recovery_active;
    reg [31:0] recovery_quiet_count;
    wire recovery_outside=precise_error_q8>RECOVERY_ENTRY_ADC*256 ||
                          precise_error_q8< -RECOVERY_ENTRY_ADC*256;
    wire recovery_quiet=precise_error_q8<=RECOVERY_EXIT_ADC*256 &&
                        precise_error_q8>=-RECOVERY_EXIT_ADC*256 &&
                        velocity_q8<=64 && velocity_q8>=-64;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin recovery_active<=0; recovery_quiet_count<=0; end
        else if(!enable || track_only || RECOVERY_ENABLE==0 || !continuous_mode) begin
            recovery_active<=0; recovery_quiet_count<=0;
        end else if(update) begin
            if(recovery_outside) begin recovery_active<=1; recovery_quiet_count<=0; end
            else if(recovery_active && recovery_quiet) begin
                if(recovery_quiet_count>=RECOVERY_QUIET_TICKS-1) begin
                    recovery_active<=0; recovery_quiet_count<=0;
                end else recovery_quiet_count<=recovery_quiet_count+1'b1;
            end else recovery_quiet_count<=0;
        end
    end
    wire signed [15:0] active_kp = (CONTINUOUS_ENABLE!=0 && continuous_mode &&
        !recovery_active && STEADY_KP>0) ? STEADY_KP : KP;
    wire signed [47:0] baseline_p_product = error_hold_q8 * active_kp;
    wire signed [47:0] reference_p_product = error_hold_q8 * REFERENCE_KP;
    wire use_reference_gains=continuous_mode && reference_mode===1'b1;
    wire signed [47:0] p_product = use_reference_gains ? reference_p_product : baseline_p_product;
    wire signed [15:0] active_kd = (CONTINUOUS_ENABLE!=0 && continuous_mode && STEADY_KD>0) ? STEADY_KD : KD;
    wire signed [47:0] baseline_d_product = velocity_q8 * active_kd;
    wire signed [47:0] reference_d_product = velocity_q8 * REFERENCE_KD;
    wire signed [47:0] d_product = use_reference_gains ? reference_d_product : baseline_d_product;
    wire signed [47:0] d_limited = (d_product > MAX_D_COMMAND * 256) ? MAX_D_COMMAND * 256 :
        (d_product < -MAX_D_COMMAND * 256) ? -MAX_D_COMMAND * 256 : d_product;
    wire integral_allowed = INTEGRAL_WINDOW_ADC <= 0 ||
        (error_hold_q8 <= INTEGRAL_WINDOW_ADC * 256 && error_hold_q8 >= -INTEGRAL_WINDOW_ADC * 256);
    wire signed [47:0] integral_step_q16 = error_hold_q8 * KI_Q8;
    wire signed [48:0] integral_sum_q16 = $signed(integral_q16) + integral_step_q16;
    wire signed [31:0] integral_next_q16 = (KI_Q8 == 0 || !integral_allowed) ? 32'sd0 :
        (integral_sum_q16 > INTEGRAL_LIMIT_COMMAND * 65536) ? INTEGRAL_LIMIT_COMMAND * 65536 :
        (integral_sum_q16 < -INTEGRAL_LIMIT_COMMAND * 65536) ? -INTEGRAL_LIMIT_COMMAND * 65536 : integral_sum_q16[31:0];
    wire signed [31:0] integral_output_q8 = integral_candidate_q16[31] ?
        -((-$signed(integral_candidate_q16)) >>> 8) : (integral_candidate_q16 >>> 8);
    wire signed [48:0] sum_q8 = $signed(p_hold) + $signed(d_hold) + $signed(integral_output_q8);
    wire signed [48:0] raw_command = sum_q8[48] ? -((-sum_q8) >>> 8) : (sum_q8 >>> 8);
    wire signed [48:0] oriented = MOTOR_POLARITY ? -raw_hold : raw_hold;
    wire signed [31:0] limited_full = (oriented > MAX_COMMAND) ? MAX_COMMAND :
        (oriented < -MAX_COMMAND) ? -MAX_COMMAND : oriented[31:0];
    wire signed [15:0] limited = limited_full[15:0];
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            for(i=0;i<N;i=i+1) history[i] <= 0;
            history_index <= 0; history_ready <= 0; center_count <= 0; velocity_q8 <= 0; error_hold_q8 <= 0;
            p_hold <= 0; d_hold <= 0; raw_hold <= 0; stage <= 0;
            integral_q16 <= 0; integral_candidate_q16 <= 0;
            command <= 0; command_update <= 0; inside_deadband <= 0; deadband_confirmed <= 0;
        end else if(!enable) begin
            for(i=0;i<N;i=i+1) history[i] <= control_measurement;
            history_index <= 0; history_ready <= 0; center_count <= 0; velocity_q8 <= 0; stage <= 0;
            integral_q16 <= 0; integral_candidate_q16 <= 0;
            command <= 0; command_update <= 0; inside_deadband <= 0; deadband_confirmed <= 0;
        end else begin
            // Clear between update pulses too, including the catch edge.
            if(track_only) begin
                integral_q16 <= 0; integral_candidate_q16 <= 0;
                center_count <= 0; deadband_confirmed <= 0;
            end
            if(update) begin
            // Seed D from the first valid update, never from an inhibited ADC
            // value. P still acts immediately. This also covers enable and ADC
            // changing together when a transient angle inhibit clears.
            if (!history_ready) begin
                for(i=0;i<N;i=i+1) history[i] <= control_measurement;
                history_index <= 0; history_ready <= 1; velocity_q8 <= 0;
            end else begin
                history[history_index] <= control_measurement;
                history_index <= history_index == N-1 ? 0 : history_index + 1'b1;
                velocity_q8 <= velocity_next;
            end
            if(track_only || !center_now) center_count <= 0;
            else if(center_count < DEADBAND_CONFIRM_TICKS) center_count <= center_count + 1'b1;
            inside_deadband <= center_now; deadband_confirmed <= qualified_now;
            error_hold_q8 <= effective_error_q8;
            stage <= 1; command_update <= 0;
        end else begin
            command_update <= 0;
            case(stage)
                1: begin
                    p_hold <= p_product; d_hold <= d_limited;
                    integral_candidate_q16 <= track_only ? 32'sd0 : integral_next_q16; stage <= 2;
                end
                2: begin
                    raw_hold <= raw_command;
                    if(track_only || KI_Q8 == 0 || !integral_allowed) integral_q16 <= 0;
                    else if(!((sum_q8 > MAX_COMMAND * 256 && error_hold_q8 > 0) ||
                              (sum_q8 < -MAX_COMMAND * 256 && error_hold_q8 < 0)))
                        integral_q16 <= integral_candidate_q16;
                    stage <= 3;
                end
                3: begin command <= limited; command_update <= 1; stage <= 0; end
                default: stage <= 0;
            endcase
        end
        end
    end
endmodule
