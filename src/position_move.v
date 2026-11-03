// Fixed-endpoint smooth rotation, optional return leg and sine; one position loop.
// Units: counts and 5 ms outer ticks. Never drives PWM.
// 0 off, 1 settle, 3 ready, 4 CW turn, 7 CCW turn, 11 sine, 2 diverge/converge,
// 5 arrived/settling, 10 hold, 9 refused (one tick). Every motion is started by a key.
// Ordinary sine is unchanged; command4 selects legacy envelope or timed C2 peaks.
module position_move #(
    parameter integer STEP_COUNTS=270, DIRECTION=1,
    parameter integer SINE_DIRECTION=DIRECTION,
    parameter integer COUNTS_PER_REV=1080, WAYPOINT_TICKS_PER_30DEG=200,
    parameter integer WAYPOINT_ARRIVAL_COUNTS=12, WAYPOINT_DWELL_TICKS=40, WAYPOINT_TIMEOUT_TICKS=4000,
    parameter integer WAYPOINT_PASS_THROUGH=0, WAYPOINT_PASS_COUNTS=3,
    parameter integer FOLLOW_ON_SINE=0, STEP_PERIOD_TICKS=SINE_PERIOD_TICKS, SEQUENCE_DWELL_TICKS=4000,
    parameter integer SMOOTH_STEP_ENABLE=0, ROUNDTRIP_ENABLE=0, DIRECT_TURN_ENABLE=0,
    // 1: smooth step uses the minimum-jerk profile instead of the half cosine.
    parameter integer MINJERK_ENABLE=0,
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
    parameter integer AUTO_ONCE=1, AUTO_WAIT_TICKS=2000,
    // Only condition for accepting a key command: balanced this long (5 ms ticks).
    parameter integer READY_TICKS=100,
    parameter integer QUALIFY_TICKS=200, INITIAL_QUALIFY_TICKS=QUALIFY_TICKS, ARRIVAL_TOLERANCE=6,
    parameter integer ANGLE_WINDOW_ADC=8, SPEED_WINDOW_Q8=32,
    parameter integer MOVE_ANGLE_WINDOW_ADC=40, MAX_FOLLOW_ERROR=24,
    parameter integer MAX_SPEED_Q16=13107, SPEED_STEP_Q16=128,
    parameter integer MAX_TRAVEL_COUNTS=5000, TRAVEL_MARGIN=500
) (
    input wire clk, rst_n, enable, capture, update, target_locked, steady_ready,
    // Latched key command: 0 none, 1 CW turn, 2 CCW turn, 3 sine, 4 diverge/converge.
    input wire [2:0] command,
    input wire velocity_valid,
    output wire waypoint_settling,
    input wire signed [31:0] position, acquired_target, velocity_q8, angle_error_q8, run_origin,
    output reg override_enable,
    output reg signed [31:0] reference_position,
    output wire signed [31:0] reference_velocity_q8,
    // Second difference of the applied Q16 reference (counts/tick^2, Q16).
    output reg signed [31:0] reference_accel_q16,
    output wire sine_endpoint_active, sine_endpoint_positive,
    output wire signed [31:0] sine_endpoint_position,
    output reg [3:0] phase,
    output reg signed [31:0] goal, anchor,
    output reg [15:0] dwell_ticks
);
    reg signed [47:0] reference_q16;
    reg signed [31:0] speed_q16;
    reg [31:0] qualified, auto_wait;
    reg follow_used;
    reg return_leg;
    reg signed [31:0] sequence_origin;
    localparam integer ROUNDTRIP_CONFIG_OK=ROUNDTRIP_ENABLE==0 ||
        (FOLLOW_ON_SINE!=0 && SMOOTH_STEP_ENABLE!=0 && SINE_ENABLE==0 &&
         STEP_COUNTS>0 && (DIRECTION==1 || DIRECTION==-1));
    wire moving=phase==4 && SMOOTH_STEP_ENABLE==0 && DIRECT_TURN_ENABLE==0;
    wire direct_turn_running=DIRECT_TURN_ENABLE!=0 && (phase==4 || phase==7);
    wire smooth_step_running=DIRECT_TURN_ENABLE==0 && (phase==4 || (ROUNDTRIP_ENABLE!=0 && phase==7)) && SMOOTH_STEP_ENABLE!=0;
    wire diverge_running=WAVEPACKET_ENABLE!=0 && WAVEPACKET_PROFILE==0 && phase==2;
    wire peak_running=WAVEPACKET_ENABLE!=0 && WAVEPACKET_PROFILE==1 && phase==2;
    wire waypoint_running=WAVEPACKET_ENABLE==0 && phase==2;
    wire sine_running=phase==11;
    wire path_running=sine_running || diverge_running || smooth_step_running;
    wire minjerk_running=MINJERK_ENABLE!=0 && smooth_step_running;
    wire posture_ok=angle_error_q8<=ANGLE_WINDOW_ADC*256 && angle_error_q8>=-ANGLE_WINDOW_ADC*256;
    wire speed_ok=velocity_q8<=SPEED_WINDOW_Q8 && velocity_q8>=-SPEED_WINDOW_Q8;
    wire signed [32:0] final_error=$signed({goal[31],goal})-$signed({position[31],position});
    // Actual goal crossing ends the travel reference, but does not declare arrival.
    // Posture, measured speed and the full confirmation interval still apply.
    wire actual_goal_reached=(DIRECTION>0) ? final_error<=0 : final_error>=0;
    wire initial_ok=target_locked && steady_ready && posture_ok && speed_ok;
    wire final_ok=initial_ok && final_error<=ARRIVAL_TOLERANCE && final_error>=-ARRIVAL_TOLERANCE;
    wire ready_now=override_enable ? final_ok : initial_ok;
    wire signed [32:0] following_error=$signed({reference_position[31],reference_position})-$signed({position[31],position});
    // Pause only when the reference is pulling farther ahead of the plant.
    // If the arm leads an outbound reference, stopping that reference creates
    // an unnecessary return to an intermediate point, despite a farther goal.
    wire outbound_positive=DIRECTION>0;
    wire path_positive=outbound_positive;
    wire tracking_lag=path_positive ? following_error>MAX_FOLLOW_ERROR : following_error< -MAX_FOLLOW_ERROR;
    wire follow_ok=!tracking_lag &&
        angle_error_q8<=MOVE_ANGLE_WINDOW_ADC*256 && angle_error_q8>=-MOVE_ANGLE_WINDOW_ADC*256;
    reg signed [32:0] next_goal;
    reg signed [33:0] next_travel, anchor_travel;
    reg next_goal_ok;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin next_goal<=0; next_travel<=0; anchor_travel<=0; next_goal_ok<=0; end
        else begin
            next_goal<=FOLLOW_ON_SINE!=0 ?
                $signed({position[31],position})+DIRECTION*STEP_COUNTS :
                $signed({anchor[31],anchor})+DIRECTION*STEP_COUNTS;
            next_travel<=$signed({next_goal[32],next_goal})-$signed({{2{run_origin[31]}},run_origin});
            anchor_travel<=$signed({{2{anchor[31]}},anchor})-$signed({{2{run_origin[31]}},run_origin});
            next_goal_ok<=next_goal<=33'sd2147483647 && next_goal>=-33'sd2147483648 &&
                next_travel<=MAX_TRAVEL_COUNTS-TRAVEL_MARGIN && next_travel>=-(MAX_TRAVEL_COUNTS-TRAVEL_MARGIN) &&
                anchor_travel<=MAX_TRAVEL_COUNTS-TRAVEL_MARGIN && anchor_travel>=-(MAX_TRAVEL_COUNTS-TRAVEL_MARGIN);
        end
    end
    wire signed [47:0] rounded_reference=reference_q16+48'sd32768;
    assign reference_velocity_q8=(moving || path_running || waypoint_running || peak_running) ? speed_q16/256 : 32'sd0;

    // Phase is Q24 turns, time advances only on the existing 5 ms tick.
    // A remainder accumulator makes each period exactly PERIOD_TICKS long.
    localparam integer SINE_PERIOD=(SINE_PERIOD_TICKS>0) ? SINE_PERIOD_TICKS : 1;
    localparam integer STEP_PERIOD=(STEP_PERIOD_TICKS>0) ? STEP_PERIOD_TICKS : 1;
    localparam integer PACKET_PERIOD=WAVEPACKET_PERIOD_TICKS>0 && WAVEPACKET_PERIOD_TICKS<=120000 ? WAVEPACKET_PERIOD_TICKS : 1;
    localparam integer PACKET_CYCLES=WAVEPACKET_CYCLES>0 && WAVEPACKET_CYCLES<=100 ? WAVEPACKET_CYCLES : 1;
    localparam integer PACKET_TOTAL=PACKET_PERIOD*PACKET_CYCLES;
    localparam integer PACKET_CONFIG_OK=MAX_SPEED_Q16>0 && WAVEPACKET_AMPLITUDE>0 && WAVEPACKET_AMPLITUDE<=540 &&
        WAVEPACKET_PERIOD_TICKS>=1500 && WAVEPACKET_PERIOD_TICKS<=120000 &&
        WAVEPACKET_CYCLES>=2 && WAVEPACKET_CYCLES<=100 &&
        (WAVEPACKET_DIRECTION==1 || WAVEPACKET_DIRECTION==-1) &&
        (WAVEPACKET_ENVELOPE_POWER==1 || WAVEPACKET_ENVELOPE_POWER==2) &&
        // sin^4 envelope: |dx/dtick| <= A*(2*pi/P + 1.3*pi/(N*P)) < 9*A/P, N>=2.
        (64'd9*WAVEPACKET_AMPLITUDE*65536 <= 64'd1*MAX_SPEED_Q16*PACKET_PERIOD);
    wire [31:0] PERIOD=diverge_running ? PACKET_PERIOD : sine_running ? SINE_PERIOD : STEP_PERIOD;
    wire [31:0] PHASE_STEP=diverge_running ? 16777216/PACKET_PERIOD : sine_running ? 16777216/SINE_PERIOD : 8388608/STEP_PERIOD;
    wire [31:0] PHASE_REM=diverge_running ? 16777216%PACKET_PERIOD : sine_running ? 16777216%SINE_PERIOD : 8388608%STEP_PERIOD;
    wire signed [31:0] TABLE_AMPLITUDE=diverge_running ? WAVEPACKET_AMPLITUDE : sine_running ? SINE_AMPLITUDE : STEP_COUNTS;
    localparam integer SINE_CONFIG_OK=SINE_AMPLITUDE>0 && SINE_AMPLITUDE<=540 &&
        SINE_PERIOD_TICKS>=1500 && SINE_PERIOD_TICKS<=120000 &&
        SINE_CYCLES>=2 && SINE_CYCLES<=100 && (DIRECTION==1 || DIRECTION==-1) &&
        (SINE_DIRECTION==1 || SINE_DIRECTION==-1) &&
        // Bound peak speed including the first/last-cycle amplitude envelope.
        (64'd8*SINE_AMPLITUDE*65536 <= 64'd1*MAX_SPEED_Q16*SINE_PERIOD);
    reg [23:0] sine_phase;
    reg [31:0] sine_remainder, sine_tick, sine_cycle;
    wire [32:0] remainder_sum={1'b0,sine_remainder}+PHASE_REM;

    // Minimum-jerk unit step s(x)=10x^3-15x^4+6x^5, x=index/128, Q16.
    // Velocity and acceleration are zero at both endpoints.
    function signed [31:0] minjerk_position;
        input [7:0] index;
        begin case(index)
            8'd0: minjerk_position=32'sd0;
            8'd1: minjerk_position=32'sd0;
            8'd2: minjerk_position=32'sd2;
            8'd3: minjerk_position=32'sd8;
            8'd4: minjerk_position=32'sd19;
            8'd5: minjerk_position=32'sd37;
            8'd6: minjerk_position=32'sd63;
            8'd7: minjerk_position=32'sd99;
            8'd8: minjerk_position=32'sd145;
            8'd9: minjerk_position=32'sd204;
            8'd10: minjerk_position=32'sd277;
            8'd11: minjerk_position=32'sd364;
            8'd12: minjerk_position=32'sd467;
            8'd13: minjerk_position=32'sd586;
            8'd14: minjerk_position=32'sd723;
            8'd15: minjerk_position=32'sd878;
            8'd16: minjerk_position=32'sd1052;
            8'd17: minjerk_position=32'sd1246;
            8'd18: minjerk_position=32'sd1460;
            8'd19: minjerk_position=32'sd1695;
            8'd20: minjerk_position=32'sd1951;
            8'd21: minjerk_position=32'sd2229;
            8'd22: minjerk_position=32'sd2529;
            8'd23: minjerk_position=32'sd2851;
            8'd24: minjerk_position=32'sd3196;
            8'd25: minjerk_position=32'sd3564;
            8'd26: minjerk_position=32'sd3955;
            8'd27: minjerk_position=32'sd4369;
            8'd28: minjerk_position=32'sd4806;
            8'd29: minjerk_position=32'sd5266;
            8'd30: minjerk_position=32'sd5749;
            8'd31: minjerk_position=32'sd6255;
            8'd32: minjerk_position=32'sd6784;
            8'd33: minjerk_position=32'sd7335;
            8'd34: minjerk_position=32'sd7909;
            8'd35: minjerk_position=32'sd8504;
            8'd36: minjerk_position=32'sd9121;
            8'd37: minjerk_position=32'sd9759;
            8'd38: minjerk_position=32'sd10418;
            8'd39: minjerk_position=32'sd11098;
            8'd40: minjerk_position=32'sd11797;
            8'd41: minjerk_position=32'sd12515;
            8'd42: minjerk_position=32'sd13253;
            8'd43: minjerk_position=32'sd14008;
            8'd44: minjerk_position=32'sd14781;
            8'd45: minjerk_position=32'sd15571;
            8'd46: minjerk_position=32'sd16378;
            8'd47: minjerk_position=32'sd17199;
            8'd48: minjerk_position=32'sd18036;
            8'd49: minjerk_position=32'sd18887;
            8'd50: minjerk_position=32'sd19751;
            8'd51: minjerk_position=32'sd20627;
            8'd52: minjerk_position=32'sd21515;
            8'd53: minjerk_position=32'sd22414;
            8'd54: minjerk_position=32'sd23323;
            8'd55: minjerk_position=32'sd24241;
            8'd56: minjerk_position=32'sd25168;
            8'd57: minjerk_position=32'sd26101;
            8'd58: minjerk_position=32'sd27042;
            8'd59: minjerk_position=32'sd27987;
            8'd60: minjerk_position=32'sd28938;
            8'd61: minjerk_position=32'sd29892;
            8'd62: minjerk_position=32'sd30849;
            8'd63: minjerk_position=32'sd31808;
            8'd64: minjerk_position=32'sd32768;
            8'd65: minjerk_position=32'sd33728;
            8'd66: minjerk_position=32'sd34687;
            8'd67: minjerk_position=32'sd35644;
            8'd68: minjerk_position=32'sd36598;
            8'd69: minjerk_position=32'sd37549;
            8'd70: minjerk_position=32'sd38494;
            8'd71: minjerk_position=32'sd39435;
            8'd72: minjerk_position=32'sd40368;
            8'd73: minjerk_position=32'sd41295;
            8'd74: minjerk_position=32'sd42213;
            8'd75: minjerk_position=32'sd43122;
            8'd76: minjerk_position=32'sd44021;
            8'd77: minjerk_position=32'sd44909;
            8'd78: minjerk_position=32'sd45785;
            8'd79: minjerk_position=32'sd46649;
            8'd80: minjerk_position=32'sd47500;
            8'd81: minjerk_position=32'sd48337;
            8'd82: minjerk_position=32'sd49158;
            8'd83: minjerk_position=32'sd49965;
            8'd84: minjerk_position=32'sd50755;
            8'd85: minjerk_position=32'sd51528;
            8'd86: minjerk_position=32'sd52283;
            8'd87: minjerk_position=32'sd53021;
            8'd88: minjerk_position=32'sd53739;
            8'd89: minjerk_position=32'sd54438;
            8'd90: minjerk_position=32'sd55118;
            8'd91: minjerk_position=32'sd55777;
            8'd92: minjerk_position=32'sd56415;
            8'd93: minjerk_position=32'sd57032;
            8'd94: minjerk_position=32'sd57627;
            8'd95: minjerk_position=32'sd58201;
            8'd96: minjerk_position=32'sd58752;
            8'd97: minjerk_position=32'sd59281;
            8'd98: minjerk_position=32'sd59787;
            8'd99: minjerk_position=32'sd60270;
            8'd100: minjerk_position=32'sd60730;
            8'd101: minjerk_position=32'sd61167;
            8'd102: minjerk_position=32'sd61581;
            8'd103: minjerk_position=32'sd61972;
            8'd104: minjerk_position=32'sd62340;
            8'd105: minjerk_position=32'sd62685;
            8'd106: minjerk_position=32'sd63007;
            8'd107: minjerk_position=32'sd63307;
            8'd108: minjerk_position=32'sd63585;
            8'd109: minjerk_position=32'sd63841;
            8'd110: minjerk_position=32'sd64076;
            8'd111: minjerk_position=32'sd64290;
            8'd112: minjerk_position=32'sd64484;
            8'd113: minjerk_position=32'sd64658;
            8'd114: minjerk_position=32'sd64813;
            8'd115: minjerk_position=32'sd64950;
            8'd116: minjerk_position=32'sd65069;
            8'd117: minjerk_position=32'sd65172;
            8'd118: minjerk_position=32'sd65259;
            8'd119: minjerk_position=32'sd65332;
            8'd120: minjerk_position=32'sd65391;
            8'd121: minjerk_position=32'sd65437;
            8'd122: minjerk_position=32'sd65473;
            8'd123: minjerk_position=32'sd65499;
            8'd124: minjerk_position=32'sd65517;
            8'd125: minjerk_position=32'sd65528;
            8'd126: minjerk_position=32'sd65534;
            8'd127: minjerk_position=32'sd65536;
            8'd128: minjerk_position=32'sd65536;
            default: minjerk_position=32'sd65536;
        endcase end
    endfunction

        // First-quarter phase warp h(u)=3u^3-2u^4: zero initial velocity/acceleration,
    // unit phase slope at the peak. Mirror in the final quarter for smooth stop.
    function [31:0] start_phase_warp;
        input [7:0] index;
        begin case(index)
            8'd0: start_phase_warp=32'd0;
            8'd1: start_phase_warp=32'd6;
            8'd2: start_phase_warp=32'd48;
            8'd3: start_phase_warp=32'd159;
            8'd4: start_phase_warp=32'd376;
            8'd5: start_phase_warp=32'd730;
            8'd6: start_phase_warp=32'd1256;
            8'd7: start_phase_warp=32'd1983;
            8'd8: start_phase_warp=32'd2944;
            8'd9: start_phase_warp=32'd4169;
            8'd10: start_phase_warp=32'd5688;
            8'd11: start_phase_warp=32'd7528;
            8'd12: start_phase_warp=32'd9720;
            8'd13: start_phase_warp=32'd12289;
            8'd14: start_phase_warp=32'd15264;
            8'd15: start_phase_warp=32'd18668;
            8'd16: start_phase_warp=32'd22528;
            8'd17: start_phase_warp=32'd26868;
            8'd18: start_phase_warp=32'd31712;
            8'd19: start_phase_warp=32'd37081;
            8'd20: start_phase_warp=32'd43000;
            8'd21: start_phase_warp=32'd49488;
            8'd22: start_phase_warp=32'd56568;
            8'd23: start_phase_warp=32'd64257;
            8'd24: start_phase_warp=32'd72576;
            8'd25: start_phase_warp=32'd81543;
            8'd26: start_phase_warp=32'd91176;
            8'd27: start_phase_warp=32'd101490;
            8'd28: start_phase_warp=32'd112504;
            8'd29: start_phase_warp=32'd124231;
            8'd30: start_phase_warp=32'd136688;
            8'd31: start_phase_warp=32'd149886;
            8'd32: start_phase_warp=32'd163840;
            8'd33: start_phase_warp=32'd178562;
            8'd34: start_phase_warp=32'd194064;
            8'd35: start_phase_warp=32'd210355;
            8'd36: start_phase_warp=32'd227448;
            8'd37: start_phase_warp=32'd245350;
            8'd38: start_phase_warp=32'd264072;
            8'd39: start_phase_warp=32'd283619;
            8'd40: start_phase_warp=32'd304000;
            8'd41: start_phase_warp=32'd325221;
            8'd42: start_phase_warp=32'd347288;
            8'd43: start_phase_warp=32'd370204;
            8'd44: start_phase_warp=32'd393976;
            8'd45: start_phase_warp=32'd418605;
            8'd46: start_phase_warp=32'd444096;
            8'd47: start_phase_warp=32'd470448;
            8'd48: start_phase_warp=32'd497664;
            8'd49: start_phase_warp=32'd525744;
            8'd50: start_phase_warp=32'd554688;
            8'd51: start_phase_warp=32'd584493;
            8'd52: start_phase_warp=32'd615160;
            8'd53: start_phase_warp=32'd646684;
            8'd54: start_phase_warp=32'd679064;
            8'd55: start_phase_warp=32'd712293;
            8'd56: start_phase_warp=32'd746368;
            8'd57: start_phase_warp=32'd781283;
            8'd58: start_phase_warp=32'd817032;
            8'd59: start_phase_warp=32'd853606;
            8'd60: start_phase_warp=32'd891000;
            8'd61: start_phase_warp=32'd929203;
            8'd62: start_phase_warp=32'd968208;
            8'd63: start_phase_warp=32'd1008002;
            8'd64: start_phase_warp=32'd1048576;
            8'd65: start_phase_warp=32'd1089918;
            8'd66: start_phase_warp=32'd1132016;
            8'd67: start_phase_warp=32'd1174855;
            8'd68: start_phase_warp=32'd1218424;
            8'd69: start_phase_warp=32'd1262706;
            8'd70: start_phase_warp=32'd1307688;
            8'd71: start_phase_warp=32'd1353351;
            8'd72: start_phase_warp=32'd1399680;
            8'd73: start_phase_warp=32'd1446657;
            8'd74: start_phase_warp=32'd1494264;
            8'd75: start_phase_warp=32'd1542480;
            8'd76: start_phase_warp=32'd1591288;
            8'd77: start_phase_warp=32'd1640665;
            8'd78: start_phase_warp=32'd1690592;
            8'd79: start_phase_warp=32'd1741044;
            8'd80: start_phase_warp=32'd1792000;
            8'd81: start_phase_warp=32'd1843436;
            8'd82: start_phase_warp=32'd1895328;
            8'd83: start_phase_warp=32'd1947649;
            8'd84: start_phase_warp=32'd2000376;
            8'd85: start_phase_warp=32'd2053480;
            8'd86: start_phase_warp=32'd2106936;
            8'd87: start_phase_warp=32'd2160713;
            8'd88: start_phase_warp=32'd2214784;
            8'd89: start_phase_warp=32'd2269119;
            8'd90: start_phase_warp=32'd2323688;
            8'd91: start_phase_warp=32'd2378458;
            8'd92: start_phase_warp=32'd2433400;
            8'd93: start_phase_warp=32'd2488479;
            8'd94: start_phase_warp=32'd2543664;
            8'd95: start_phase_warp=32'd2598918;
            8'd96: start_phase_warp=32'd2654208;
            8'd97: start_phase_warp=32'd2709498;
            8'd98: start_phase_warp=32'd2764752;
            8'd99: start_phase_warp=32'd2819931;
            8'd100: start_phase_warp=32'd2875000;
            8'd101: start_phase_warp=32'd2929918;
            8'd102: start_phase_warp=32'd2984648;
            8'd103: start_phase_warp=32'd3039147;
            8'd104: start_phase_warp=32'd3093376;
            8'd105: start_phase_warp=32'd3147293;
            8'd106: start_phase_warp=32'd3200856;
            8'd107: start_phase_warp=32'd3254020;
            8'd108: start_phase_warp=32'd3306744;
            8'd109: start_phase_warp=32'd3358981;
            8'd110: start_phase_warp=32'd3410688;
            8'd111: start_phase_warp=32'd3461816;
            8'd112: start_phase_warp=32'd3512320;
            8'd113: start_phase_warp=32'd3562152;
            8'd114: start_phase_warp=32'd3611264;
            8'd115: start_phase_warp=32'd3659605;
            8'd116: start_phase_warp=32'd3707128;
            8'd117: start_phase_warp=32'd3753780;
            8'd118: start_phase_warp=32'd3799512;
            8'd119: start_phase_warp=32'd3844269;
            8'd120: start_phase_warp=32'd3888000;
            8'd121: start_phase_warp=32'd3930651;
            8'd122: start_phase_warp=32'd3972168;
            8'd123: start_phase_warp=32'd4012494;
            8'd124: start_phase_warp=32'd4051576;
            8'd125: start_phase_warp=32'd4089355;
            8'd126: start_phase_warp=32'd4125776;
            8'd127: start_phase_warp=32'd4160778;
            8'd128: start_phase_warp=32'd4194304;
            default: start_phase_warp=32'd4194304;
        endcase end
    endfunction
    reg [22:0] warp_coordinate;
    reg [31:0] warp_a,warp_b;
    reg [14:0] warp_fraction;
    reg signed [48:0] warp_interpolation;
    reg [31:0] warped_phase;
    reg [23:0] mapped_sine_phase;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin warp_coordinate<=0;warp_a<=0;warp_b<=0;warp_fraction<=0;warp_interpolation<=0;warped_phase<=0;mapped_sine_phase<=0;end
        else begin
            warp_coordinate<=sine_cycle==SINE_CYCLES-1 && sine_phase[23:22]==3 ? 25'd16777216-{1'b0,sine_phase} : {1'b0,sine_phase[21:0]};
            warp_a<=start_phase_warp(warp_coordinate[22:15]);
            warp_b<=start_phase_warp(warp_coordinate[22:15]+1'b1);
            warp_fraction<=warp_coordinate[14:0];
            warp_interpolation<=($signed({1'b0,warp_b})-$signed({1'b0,warp_a}))*$signed({1'b0,warp_fraction});
            warped_phase<=warp_a+(warp_interpolation >>> 15);
            mapped_sine_phase<=sine_cycle==0 && sine_phase[23:22]==0 ? warped_phase[23:0] :
                sine_cycle==SINE_CYCLES-1 && sine_phase[23:22]==3 ? 25'd16777216-warped_phase : sine_phase;
        end
    end
    wire [23:0] lookup_phase=sine_running ? mapped_sine_phase : sine_phase;
    wire [7:0] path_index=lookup_phase[23:16]+((smooth_step_running && !minjerk_running) ? 8'd64 : 8'd0);
    wire [7:0] sine_next_index=path_index+8'd1;
    wire [6:0] quarter_address_a=path_index[6] ? 7'd64-{1'b0,path_index[5:0]} : {1'b0,path_index[5:0]};
    wire [6:0] quarter_address_b=sine_next_index[6] ? 7'd64-{1'b0,sine_next_index[5:0]} : {1'b0,sine_next_index[5:0]};
    wire signed [31:0] quarter_a,quarter_b;
    sine_quarter_lut sine_table_a(.index(quarter_address_a),.value(quarter_a));
    sine_quarter_lut sine_table_b(.index(quarter_address_b),.value(quarter_b));
    reg signed [31:0] sine_raw_a, sine_raw_b;
    reg sine_negative_a, sine_negative_b;
    reg signed [31:0] sine_a, sine_b, sine_interpolated;
    reg [15:0] sine_fraction;
    reg signed [48:0] sine_interpolation;
    reg signed [17:0] envelope;
    // One envelope cycle spans N carrier cycles; both clocks pause together.
    reg [23:0] packet_phase;
    reg [31:0] packet_remainder;
    wire [32:0] packet_rem_sum={1'b0,packet_remainder}+(16777216%PACKET_TOTAL);
    wire signed [17:0] packet_envelope;
    wavepacket_envelope #(.POWER(WAVEPACKET_ENVELOPE_POWER)) envelope_lut(.clk(clk),.rst_n(rst_n),.phase(packet_phase),.value(packet_envelope));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin packet_phase<=0;packet_remainder<=0;end
        else if(!enable || capture || !diverge_running) begin packet_phase<=0;packet_remainder<=0;end
        else if(update && path_advancing) begin
            packet_phase<=packet_phase+16777216/PACKET_TOTAL+(packet_rem_sum>=PACKET_TOTAL);
            packet_remainder<=packet_rem_sum>=PACKET_TOTAL ? packet_rem_sum-PACKET_TOTAL : packet_rem_sum;
        end
    end
    reg signed [49:0] sine_product;
    reg signed [47:0] sine_reference, path_offset, oriented_path_offset;
    reg sine_bounds_ok;
    wire signed [33:0] sine_low=$signed({{2{anchor[31]}},anchor})-SINE_AMPLITUDE;
    wire signed [33:0] sine_high=$signed({{2{anchor[31]}},anchor})+SINE_AMPLITUDE;
    wire signed [34:0] sine_low_travel=sine_low-$signed({{3{run_origin[31]}},run_origin});
    wire signed [34:0] sine_high_travel=sine_high-$signed({{3{run_origin[31]}},run_origin});
    // Fixed-duration smooth step advances through soft position lag.
    // Posture gating remains; top-level120-count departure still latches motor stop.
    // Timed rotation and sine do not stretch their phase for soft position lag.
    // Posture can pause; the original120-count motor-stop guard remains active.
    wire sine_follow_ok=angle_error_q8<=MOVE_ANGLE_WINDOW_ADC*256 &&
        angle_error_q8>=-MOVE_ANGLE_WINDOW_ADC*256;
    // Pipeline settles long before the next outer tick. Reference velocity is
    // the finite difference of the exact Q16 reference applied to this loop.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sine_raw_a<=0; sine_raw_b<=0; sine_negative_a<=0; sine_negative_b<=0;
            sine_a<=0; sine_b<=0; sine_fraction<=0; sine_interpolation<=0;
            sine_interpolated<=0; envelope<=0; sine_product<=0; path_offset<=0; oriented_path_offset<=0; sine_reference<=0; sine_bounds_ok<=0;
        end else begin
            sine_raw_a<=(minjerk_running ? minjerk_position(path_index) : quarter_a)*TABLE_AMPLITUDE;
            sine_raw_b<=(minjerk_running ? minjerk_position(sine_next_index) : quarter_b)*TABLE_AMPLITUDE;
            // Minimum-jerk indices run 0..128 and are never negative.
            sine_negative_a<=path_index[7] && !minjerk_running; sine_negative_b<=sine_next_index[7] && !minjerk_running;
            sine_a<=sine_negative_a ? -sine_raw_a : sine_raw_a;
            sine_b<=sine_negative_b ? -sine_raw_b : sine_raw_b;
            sine_fraction<=lookup_phase[15:0];
            sine_interpolation<=($signed(sine_b)-$signed(sine_a))*$signed({1'b0,sine_fraction});
            sine_interpolated<=sine_a+(sine_interpolation >>> 16);
            // E(t)=sin(pi*t/T)^(2*POWER), Q16 in [0,65536]; carrier timing unchanged.
            envelope<=diverge_running ? packet_envelope : 18'sd65536;
            sine_product<=sine_interpolated*envelope;
            // Separate half-cosine offset, direction and anchor addition.
            // Each stage settles in clocks, well before the next5ms update.
            path_offset<=minjerk_running ? $signed(sine_interpolated) : smooth_step_running ?
                (($signed(STEP_COUNTS)*48'sd65536-$signed(sine_interpolated)) >>> 1) : (sine_product >>> 16);
            // Ordinary sine and wavepacket have independent initial directions.
            oriented_path_offset<=(diverge_running ? WAVEPACKET_DIRECTION : phase==11 ? SINE_DIRECTION :
                ((ROUNDTRIP_ENABLE!=0 && return_leg && smooth_step_running) ? -DIRECTION : DIRECTION))*path_offset;
            sine_reference<=$signed(anchor)*48'sd65536+oriented_path_offset;
            sine_bounds_ok<=SINE_CONFIG_OK && sine_low>=-34'sd2147483648 && sine_high<=34'sd2147483647 &&
                sine_low_travel>=-(MAX_TRAVEL_COUNTS-TRAVEL_MARGIN) &&
                sine_high_travel<=MAX_TRAVEL_COUNTS-TRAVEL_MARGIN;
        end
    end

    wire path_finish=(ROUNDTRIP_ENABLE==0 && smooth_step_running &&
        (DIRECTION>0 ? final_error<=15 : final_error>=-15)) ||
        sine_cycle>=(smooth_step_running ? 1 : diverge_running ? WAVEPACKET_CYCLES : SINE_CYCLES);
    // Acceleration only between two consecutive advancing path ticks:
    // pauses, starts and endpoint snaps report zero instead of a spike.
    wire path_advancing=path_running && !path_finish && sine_follow_ok;
    reg path_advanced;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin reference_accel_q16<=0; path_advanced<=0; end
        else if(!enable || capture) begin reference_accel_q16<=0; path_advanced<=0; end
        else if(update) begin
            path_advanced<=path_advancing;
            reference_accel_q16<=peak_running ? peak_accel_q16 : waypoint_running ? wp_accel_q16 : (path_advancing && path_advanced) ?
                (sine_reference-reference_q16)-speed_q16 : 32'sd0;
        end
    end

    wire sequence_in_band=target_locked && final_error<=45 && final_error>=-45 &&
        angle_error_q8<=MOVE_ANGLE_WINDOW_ADC*256 && angle_error_q8>=-MOVE_ANGLE_WINDOW_ADC*256;
    // Keep the return endpoint as the sine center: no reference jump to feedback.
    wire signed [31:0] sequence_center=ROUNDTRIP_ENABLE!=0 ? goal : position;
    wire signed [33:0] sequence_low=$signed({{2{sequence_center[31]}},sequence_center})-SINE_AMPLITUDE;
    wire signed [33:0] sequence_high=$signed({{2{sequence_center[31]}},sequence_center})+SINE_AMPLITUDE;
    wire signed [34:0] sequence_low_travel=sequence_low-$signed({{3{run_origin[31]}},run_origin});
    wire signed [34:0] sequence_high_travel=sequence_high-$signed({{3{run_origin[31]}},run_origin});
    reg sequence_sine_bounds_ok;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) sequence_sine_bounds_ok<=0;
        else sequence_sine_bounds_ok<=SINE_CONFIG_OK && SEQUENCE_DWELL_TICKS>0 && SEQUENCE_DWELL_TICKS<=65535 &&
            sequence_low>=-34'sd2147483648 && sequence_high<=34'sd2147483647 &&
            sequence_low_travel>=-(MAX_TRAVEL_COUNTS-TRAVEL_MARGIN) &&
            sequence_high_travel<=MAX_TRAVEL_COUNTS-TRAVEL_MARGIN;
    end
    reg signed [47:0] distance_q16, next_reference;
    reg follow_ok_hold;
    reg signed [31:0] desired_speed, next_speed;
    reg signed [32:0] speed_difference;
    // Brake before destination; bound acceleration also during tracking pauses.
    wire signed [47:0] requested_speed=distance_q16 >>> 7;
    wire signed [31:0] desired_request=!follow_ok_hold ? 32'sd0 :
        requested_speed>MAX_SPEED_Q16 ? MAX_SPEED_Q16 :
        requested_speed< -MAX_SPEED_Q16 ? -MAX_SPEED_Q16 : requested_speed[31:0];
    wire signed [31:0] next_speed_request=speed_difference>SPEED_STEP_Q16 ? speed_q16+SPEED_STEP_Q16 :
        speed_difference< -SPEED_STEP_Q16 ? speed_q16-SPEED_STEP_Q16 : desired_speed;
    wire reference_done=distance_q16<=256 && distance_q16>=-256 &&
        speed_q16<=SPEED_STEP_Q16 && speed_q16>=-SPEED_STEP_Q16;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            reference_position<=0; distance_q16<=0; follow_ok_hold<=0;
            desired_speed<=0; speed_difference<=0; next_speed<=0; next_reference<=0;
        end else begin
            reference_position<=rounded_reference[47:16];
            distance_q16<=$signed(goal)*48'sd65536-reference_q16;
            follow_ok_hold<=follow_ok;
            desired_speed<=desired_request;
            speed_difference<=$signed({desired_speed[31],desired_speed})-$signed({speed_q16[31],speed_q16});
            next_speed<=next_speed_request;
            next_reference<=reference_q16+next_speed;
        end
    end
    // A new motion starts from the held reference, so consecutive moves chain
    // without a jump; before the first move the measured position is used.
    // K4 paths always establish a fresh center at actual execution time.
    reg signed [31:0] command_base,turn_cw_goal,turn_ccw_goal,waypoint_first_goal;
    reg [2:0] base_command;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin command_base<=0;turn_cw_goal<=0;turn_ccw_goal<=0;waypoint_first_goal<=0;base_command<=0;end
        else begin
            command_base<=(command==3 || command==4) ? position : (override_enable ? reference_position : position);
            base_command<=command;
            turn_cw_goal<=command_base+DIRECTION*STEP_COUNTS;
            turn_ccw_goal<=command_base-DIRECTION*STEP_COUNTS;
            waypoint_first_goal<=command_base+COUNTS_PER_REV/12;
        end
    end
    wire signed [33:0] base_travel=$signed({{2{command_base[31]}},command_base})-$signed({{2{run_origin[31]}},run_origin});
    localparam integer TRAVEL_LIMIT=MAX_TRAVEL_COUNTS-TRAVEL_MARGIN;
    wire turn_cw_ok_live=base_travel+DIRECTION*STEP_COUNTS<=TRAVEL_LIMIT && base_travel+DIRECTION*STEP_COUNTS>=-TRAVEL_LIMIT;
    wire turn_ccw_ok_live=base_travel-DIRECTION*STEP_COUNTS<=TRAVEL_LIMIT && base_travel-DIRECTION*STEP_COUNTS>=-TRAVEL_LIMIT;
    wire sine_ok_live=SINE_CONFIG_OK && base_travel+SINE_AMPLITUDE<=TRAVEL_LIMIT && base_travel-SINE_AMPLITUDE>=-TRAVEL_LIMIT &&
        $signed({command_base[31],command_base})+SINE_AMPLITUDE<=33'sd2147483647 &&
        $signed({command_base[31],command_base})-SINE_AMPLITUDE>=-33'sd2147483648;
    wire packet_ok_live=PACKET_CONFIG_OK &&
        base_travel+WAVEPACKET_AMPLITUDE<=TRAVEL_LIMIT && base_travel-WAVEPACKET_AMPLITUDE>=-TRAVEL_LIMIT &&
        $signed({command_base[31],command_base})+WAVEPACKET_AMPLITUDE<=33'sd2147483647 &&
        $signed({command_base[31],command_base})-WAVEPACKET_AMPLITUDE>=-33'sd2147483648;
    localparam integer PEAK_CONFIG_OK=MAX_SPEED_Q16>0 && WAVEPACKET_PEAK_1>0 &&
        WAVEPACKET_PEAK_1<WAVEPACKET_PEAK_2 && WAVEPACKET_PEAK_2<WAVEPACKET_PEAK_3 &&
        WAVEPACKET_PEAK_3<WAVEPACKET_PEAK_4 && WAVEPACKET_PEAK_4<=2500 &&
        WAVEPACKET_PERIOD_TICKS>=1428 && WAVEPACKET_PERIOD_TICKS<=120000 && WAVEPACKET_PERIOD_TICKS%4==0 &&
        (WAVEPACKET_DIRECTION==1 || WAVEPACKET_DIRECTION==-1) &&
        64'd18*WAVEPACKET_PEAK_1*65536<=64'd1*MAX_SPEED_Q16*WAVEPACKET_PERIOD_TICKS &&
        64'd6*(WAVEPACKET_PEAK_3+WAVEPACKET_PEAK_4)*65536<=64'd1*MAX_SPEED_Q16*WAVEPACKET_PERIOD_TICKS;
    wire peak_ok_live=PEAK_CONFIG_OK && base_travel+WAVEPACKET_PEAK_4<=TRAVEL_LIMIT &&
        base_travel-WAVEPACKET_PEAK_4>=-TRAVEL_LIMIT &&
        $signed({command_base[31],command_base})+WAVEPACKET_PEAK_4<=33'sd2147483647 &&
        $signed({command_base[31],command_base})-WAVEPACKET_PEAK_4>=-33'sd2147483648;
    wire step_ok=SMOOTH_STEP_ENABLE!=0 && STEP_COUNTS>0 && MAX_SPEED_Q16>0 && STEP_PERIOD_TICKS>0 && STEP_PERIOD_TICKS<=120000 &&
        (MINJERK_ENABLE!=0 ? 64'd15*STEP_COUNTS*65536<=64'd8*MAX_SPEED_Q16*STEP_PERIOD :
        64'd2*STEP_COUNTS*65536<=64'd1*MAX_SPEED_Q16*STEP_PERIOD);
    wire command_ready=phase==3 || phase==5 || phase==8 || phase==10;
    wire waypoint_start=WAVEPACKET_ENABLE==0 && enable && !capture && update && command_ready && command==4 && checked_command==command && waypoint_ok;
    wire waypoint_ok_live=COUNTS_PER_REV>0 && COUNTS_PER_REV%12==0 && WAYPOINT_TICKS_PER_30DEG>0 &&
        (64'd15*(COUNTS_PER_REV/12)*65536<=64'd8*MAX_SPEED_Q16*WAYPOINT_TICKS_PER_30DEG) &&
        base_travel+COUNTS_PER_REV/2<=TRAVEL_LIMIT && base_travel-COUNTS_PER_REV/2>=-TRAVEL_LIMIT &&
        $signed({command_base[31],command_base})+COUNTS_PER_REV/2<=33'sd2147483647 &&
        $signed({command_base[31],command_base})-COUNTS_PER_REV/2>=-33'sd2147483648;
    // Check the command corridor before the state/goal write mux.
    // A just-changed command waits for its matching validation sample.
    reg turn_cw_ok,turn_ccw_ok,sine_ok,waypoint_ok,packet_ok,peak_ok;
    reg [2:0] checked_command;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin turn_cw_ok<=0;turn_ccw_ok<=0;sine_ok<=0;waypoint_ok<=0;packet_ok<=0;peak_ok<=0;checked_command<=0;end
        else begin
            turn_cw_ok<=turn_cw_ok_live;turn_ccw_ok<=turn_ccw_ok_live;
            sine_ok<=sine_ok_live;waypoint_ok<=waypoint_ok_live;packet_ok<=packet_ok_live;peak_ok<=peak_ok_live;checked_command<=base_command;
        end
    end
    wire peak_start=WAVEPACKET_ENABLE!=0 && WAVEPACKET_PROFILE==1 && enable && !capture && update &&
        command_ready && command==4 && checked_command==command && peak_ok;
    wire peak_done,peak_sample_ready;wire [2:0] peak_segment;
    wire signed [47:0] peak_reference_q16;
    wire signed [31:0] peak_speed_q16,peak_accel_q16;
    peak_wavepacket #(.PEAK_1(WAVEPACKET_PEAK_1),.PEAK_2(WAVEPACKET_PEAK_2),
        .PEAK_3(WAVEPACKET_PEAK_3),.PEAK_4(WAVEPACKET_PEAK_4),
        .PERIOD_TICKS(WAVEPACKET_PERIOD_TICKS),.DIRECTION(WAVEPACKET_DIRECTION)) peaks (
        .clk(clk),.rst_n(rst_n),.enable(enable && !capture && peak_running),.start(peak_start),.update(update),
        .allow_advance(sine_follow_ok),.center(command_base),.reference_q16(peak_reference_q16),
        .speed_q16(peak_speed_q16),.accel_q16(peak_accel_q16),.done(peak_done),.segment(peak_segment),.sample_ready(peak_sample_ready));
    wire wp_done,wp_timeout,wp_settling;
    wire signed [47:0] wp_reference_q16;
    wire signed [31:0] wp_speed_q16,wp_goal,wp_accel_q16;
    wire wp_endpoint_positive;
    // Next extremum follows applied reference direction; retain final center stop.
    wire sine_phase_positive=(mapped_sine_phase[23:22]==0 || mapped_sine_phase[23:22]==3)==(SINE_DIRECTION>0);
    // Q16/256 truncates toward zero: |speed|<256 has zero Q8 velocity.
    // Register this small comparison before endpoint routing; do not feed a
    // signed divider and its zero detector into the position controller.
    reg sine_velocity_positive;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) sine_velocity_positive<=SINE_DIRECTION>0;
        else sine_velocity_positive<=speed_q16>=256 ? 1'b1 : speed_q16<=-256 ? 1'b0 : sine_phase_positive;
    end
    assign sine_endpoint_active=(sine_running && !path_finish && !(sine_cycle==SINE_CYCLES-1 && sine_phase[23:22]==3));
    assign sine_endpoint_positive=waypoint_running ? wp_endpoint_positive : sine_velocity_positive;
    assign sine_endpoint_position=waypoint_running ? wp_goal :
        (sine_velocity_positive ? anchor+SINE_AMPLITUDE : anchor-SINE_AMPLITUDE);
    wire [15:0] wp_metadata;
    assign waypoint_settling=waypoint_running && wp_settling;
    waypoint_path #(.COUNTS_PER_REV(COUNTS_PER_REV),.TICKS_PER_30DEG(WAYPOINT_TICKS_PER_30DEG),
        .ARRIVAL_COUNTS(WAYPOINT_ARRIVAL_COUNTS),.DWELL_TICKS(WAYPOINT_DWELL_TICKS),
        .TIMEOUT_TICKS(WAYPOINT_TIMEOUT_TICKS),.PASS_THROUGH(WAYPOINT_PASS_THROUGH),.PASS_COUNTS(WAYPOINT_PASS_COUNTS)) waypoints (
        .clk(clk),.rst_n(rst_n),.enable(enable && !capture && waypoint_running),.start(waypoint_start),.update(update),
        .center(command_base),.position(position),.velocity_q8(velocity_q8),.velocity_valid(velocity_valid),
        .angle_error_q8(angle_error_q8),.reference_q16(wp_reference_q16),.speed_q16(wp_speed_q16),
        .goal(wp_goal),.reference_accel_q16(wp_accel_q16),.endpoint_positive(wp_endpoint_positive),.settling(wp_settling),.done(wp_done),.timed_out(wp_timeout),.metadata(wp_metadata));
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            return_leg<=0; sine_phase<=0; sine_remainder<=0; sine_tick<=0; sine_cycle<=0;
            reference_q16<=0; goal<=0; anchor<=0; speed_q16<=0; qualified<=0;
            phase<=0; override_enable<=0; dwell_ticks<=0;
        end else if(!enable || capture) begin
            return_leg<=0; sine_phase<=0; sine_remainder<=0; sine_tick<=0; sine_cycle<=0;
            reference_q16<=$signed(position)*48'sd65536; goal<=position; anchor<=position;
            speed_q16<=0; qualified<=0; phase<=enable ? 1 : 0;
            override_enable<=0; dwell_ticks<=0;
        end else if(update) begin
            dwell_ticks<=peak_running ? {1'b0,peak_segment,12'b0} : waypoint_running ? wp_metadata : qualified[15:0];
            if(phase==0 || phase==1) begin
                phase<=1; goal<=acquired_target; anchor<=position;
                reference_q16<=$signed(acquired_target)*48'sd65536;
                if(qualified>=READY_TICKS-1) begin phase<=3; qualified<=0; end
                else qualified<=qualified+1'b1;
            end else if(command_ready && command!=0 && checked_command==command) begin
                qualified<=0; speed_q16<=0;
                sine_phase<=0; sine_remainder<=0; sine_tick<=0; sine_cycle<=0;
                anchor<=command_base; reference_q16<=$signed(command_base)*48'sd65536;
                if((command==1 && step_ok && turn_cw_ok) || (command==2 && step_ok && turn_ccw_ok)) begin
                    override_enable<=1; return_leg<=command==2;
                    goal<=command==1 ? turn_cw_goal : turn_ccw_goal;
                    phase<=command==1 ? 4 : 7;
                end else if(command==3 && sine_ok) begin
                    override_enable<=1; goal<=command_base;phase<=11;
                end else if(command==4 && WAVEPACKET_ENABLE!=0 && ((WAVEPACKET_PROFILE==0 && packet_ok) || (WAVEPACKET_PROFILE==1 && peak_ok))) begin
                    override_enable<=1;goal<=command_base;phase<=2;
                end else if(command==4 && WAVEPACKET_ENABLE==0 && waypoint_ok) begin
                    override_enable<=1;goal<=waypoint_first_goal;phase<=2;
                end else begin
                    // Refused: hold where we are; one tick of phase 9 clears the request.
                    goal<=command_base; phase<=9;
                end
            end else if(phase==9) begin
                phase<=10;
            end else if(peak_running) begin
                reference_q16<=peak_reference_q16;speed_q16<=peak_speed_q16;goal<=anchor;
                if(peak_done)begin reference_q16<=$signed(anchor)*48'sd65536;speed_q16<=0;phase<=5;qualified<=0;end
            end else if(waypoint_running) begin
                reference_q16<=wp_reference_q16;speed_q16<=wp_speed_q16;goal<=wp_goal;
                if(wp_timeout) begin speed_q16<=0;phase<=8;end
                else if(wp_done) begin reference_q16<=$signed(anchor)*48'sd65536;speed_q16<=0;goal<=anchor;phase<=5;qualified<=0;end
            end else if(path_running) begin
                qualified<=0;
                if(path_finish) begin
                    reference_q16<=$signed(smooth_step_running ? goal : anchor)*48'sd65536; speed_q16<=0; phase<=5;
                end else if(sine_follow_ok) begin
                    reference_q16<=sine_reference;
                    speed_q16<=sine_reference-reference_q16;
                    if(sine_tick>=PERIOD-1) begin
                        sine_tick<=0; sine_phase<=0; sine_remainder<=0; sine_cycle<=sine_cycle+1'b1;
                    end else begin
                        sine_tick<=sine_tick+1'b1;
                        sine_phase<=sine_phase+PHASE_STEP+(remainder_sum>=PERIOD);
                        sine_remainder<=remainder_sum>=PERIOD ? remainder_sum-PERIOD : remainder_sum;
                    end
                end else speed_q16<=0;
            end else if(phase==5) begin
                // Settling at the endpoint with the reference held; then hold.
                if(qualified>=QUALIFY_TICKS-1) begin phase<=10; qualified<=0; end
                else qualified<=qualified+1'b1;
            end
        end
    end
endmodule
