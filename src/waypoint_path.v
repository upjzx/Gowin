// Command4 path: 0,+30,-60,+90,-120,+150,-180,+150,-120,+90,-60,+30,0 degrees.
// Every endpoint is relative to one fixed center; actual arrival qualifies each leg.
module waypoint_path #(
    parameter integer COUNTS_PER_REV=1080, TICKS_PER_30DEG=200,
    parameter integer ARRIVAL_COUNTS=12, DWELL_TICKS=40, TIMEOUT_TICKS=4000,
    parameter integer PASS_THROUGH=0, PASS_COUNTS=3,
    parameter integer MOVE_ANGLE_ADC=40, ARRIVAL_ANGLE_ADC=12, ARRIVAL_SPEED_Q8=64
)(
    input wire clk,rst_n,enable,start,update,velocity_valid,
    input wire signed [31:0] center,position,velocity_q8,angle_error_q8,
    output reg signed [47:0] reference_q16,
    output reg signed [31:0] speed_q16,goal,
    output reg signed [31:0] reference_accel_q16,
    output wire endpoint_positive,
    output reg settling,done,timed_out,
    output wire [15:0] metadata
);
    localparam integer UNIT=COUNTS_PER_REV/12;
    reg [3:0] segment;
    reg signed [31:0] origin,leg_start,leg_delta;
    reg [23:0] phase;
    reg [31:0] tick_count,remainder,qualified,wait_count;
    assign metadata={segment,settling,qualified[10:0]};
    function signed [31:0] point_units;
        input [3:0] index;
        begin case(index)
            0:point_units=1;1:point_units=-2;2:point_units=3;3:point_units=-4;
            4:point_units=5;5:point_units=-6;6:point_units=5;7:point_units=-4;
            8:point_units=3;9:point_units=-2;10:point_units=1;default:point_units=0;
        endcase end
    endfunction
    function [31:0] leg_period;
        input [3:0] index;
        begin case(index)
            0,11:leg_period=TICKS_PER_30DEG;1,10:leg_period=3*TICKS_PER_30DEG;
            2,9:leg_period=5*TICKS_PER_30DEG;3,8:leg_period=7*TICKS_PER_30DEG;
            4,7:leg_period=9*TICKS_PER_30DEG;default:leg_period=11*TICKS_PER_30DEG;
        endcase end
    endfunction
    function [31:0] phase_step;
        input [3:0] index;
        begin case(index)
            0,11:phase_step=8388608/TICKS_PER_30DEG;1,10:phase_step=8388608/(3*TICKS_PER_30DEG);
            2,9:phase_step=8388608/(5*TICKS_PER_30DEG);3,8:phase_step=8388608/(7*TICKS_PER_30DEG);
            4,7:phase_step=8388608/(9*TICKS_PER_30DEG);default:phase_step=8388608/(11*TICKS_PER_30DEG);
        endcase end
    endfunction
    function [31:0] phase_rem;
        input [3:0] index;
        begin case(index)
            0,11:phase_rem=8388608%TICKS_PER_30DEG;1,10:phase_rem=8388608%(3*TICKS_PER_30DEG);
            2,9:phase_rem=8388608%(5*TICKS_PER_30DEG);3,8:phase_rem=8388608%(7*TICKS_PER_30DEG);
            4,7:phase_rem=8388608%(9*TICKS_PER_30DEG);default:phase_rem=8388608%(11*TICKS_PER_30DEG);
        endcase end
    endfunction
    // Same sine LUT as position_move. Shift a half sine between each pair
    // of endpoints: x=start+delta*(1-cos(pi*t/T))/2.
    wire [7:0] cosine_index=phase[23:16]+8'd64;
    wire [7:0] cosine_next=cosine_index+8'd1;
    wire [6:0] address_a=cosine_index[6] ? 7'd64-{1'b0,cosine_index[5:0]} : {1'b0,cosine_index[5:0]};
    wire [6:0] address_b=cosine_next[6] ? 7'd64-{1'b0,cosine_next[5:0]} : {1'b0,cosine_next[5:0]};
    wire signed [31:0] quarter_a,quarter_b;
    sine_quarter_lut sine_table_a(.index(address_a),.value(quarter_a));
    sine_quarter_lut sine_table_b(.index(address_b),.value(quarter_b));
    assign endpoint_positive=leg_delta>0;

    wire [31:0] period=leg_period(segment);
    wire [32:0] remainder_sum={1'b0,remainder}+phase_rem(segment);
    reg signed [31:0] table_a,table_b,unit_position;
    reg [15:0] fraction;
    reg signed [48:0] interpolation;
    reg signed [63:0] displacement;
    reg signed [47:0] next_reference;
    // Constant lookup and staged multiplication settle before the next5ms tick.
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin table_a<=0;table_b<=0;fraction<=0;interpolation<=0;unit_position<=0;displacement<=0;next_reference<=0;end
        else begin
            table_a<=cosine_index[7] ? -quarter_a : quarter_a;
            table_b<=cosine_next[7] ? -quarter_b : quarter_b;fraction<=phase[15:0];
            interpolation<=($signed(table_b)-$signed(table_a))*$signed({1'b0,fraction});
            unit_position<=(32'sd65536-(table_a+(interpolation >>> 16))) >>> 1;
            displacement<=$signed(unit_position)*$signed(leg_delta);
            next_reference<=$signed(leg_start)*48'sd65536+displacement;
        end
    end
    wire angle_ok=angle_error_q8<=MOVE_ANGLE_ADC*256 && angle_error_q8>=-MOVE_ANGLE_ADC*256;
    wire signed [32:0] error=$signed({goal[31],goal})-$signed({position[31],position});
    wire arrived=velocity_valid && error<=ARRIVAL_COUNTS && error>=-ARRIVAL_COUNTS &&
        velocity_q8<=ARRIVAL_SPEED_Q8 && velocity_q8>=-ARRIVAL_SPEED_Q8 &&
        angle_error_q8<=ARRIVAL_ANGLE_ADC*256 && angle_error_q8>=-ARRIVAL_ANGLE_ADC*256;
    // Pass-through uses actual target crossing/1degree band, not low speed/dwell.
    // Encoder arrival alone switches a pass-through leg, even while posture
    // pauses reference advancement. Global FPGA fault gating remains upstream.
    reg point_passed;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) point_passed<=0;
        else point_passed<=(leg_delta>0 && error<=PASS_COUNTS) || (leg_delta<0 && error>=-PASS_COUNTS);
    end
    localparam signed [31:0] PASS_SPEED_Q16=(UNIT*65536)/TICKS_PER_30DEG;
    wire signed [31:0] pass_velocity=leg_delta>0 ? PASS_SPEED_Q16 : -PASS_SPEED_Q16;
    wire signed [47:0] pass_next=reference_q16+{{16{pass_velocity[31]}},pass_velocity};
    wire signed [47:0] pass_goal=$signed(goal)*48'sd65536;
    wire pass_reference_arrived=leg_delta>0 ? pass_next>=pass_goal : pass_next<=pass_goal;
    reg signed [31:0] next_goal,first_goal;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin next_goal<=0;first_goal<=0;end
        else begin next_goal<=origin+point_units(segment+1'b1)*UNIT;first_goal<=center+UNIT;end
    end
    reg path_advanced;
    wire path_advancing=PASS_THROUGH==0 && enable && !done && !timed_out && !settling &&
        !(PASS_THROUGH!=0 && point_passed) && tick_count<period && angle_ok;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin path_advanced<=0;reference_accel_q16<=0;end
        else if(start || !enable) begin path_advanced<=0;reference_accel_q16<=0;end
        else if(update) begin
            path_advanced<=path_advancing;
            reference_accel_q16<=(path_advancing && path_advanced) ?
                (next_reference-reference_q16)-speed_q16 : 32'sd0;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin origin<=0;leg_start<=0;leg_delta<=0;segment<=0;phase<=0;tick_count<=0;remainder<=0;qualified<=0;wait_count<=0;reference_q16<=0;speed_q16<=0;goal<=0;settling<=0;done<=0;timed_out<=0;end
        else if(start) begin
            origin<=center;leg_start<=center;leg_delta<=UNIT;goal<=first_goal;
            segment<=0;phase<=0;tick_count<=0;remainder<=0;qualified<=0;wait_count<=0;
            reference_q16<=$signed(center)*48'sd65536;speed_q16<=0;settling<=0;done<=0;timed_out<=0;
        end else if(!enable) begin
            done<=0;timed_out<=0;speed_q16<=0;qualified<=0;wait_count<=0;
        end else if(update && !done && !timed_out) begin
            if(PASS_THROUGH!=0 && point_passed) begin
                // Reverse the next reference immediately. Physical torque remains
                // under the balance loop; no forced motor polarity switch.
                reference_q16<=$signed(goal)*48'sd65536;speed_q16<=0;
                if(segment==11) begin done<=1;settling<=0;end
                else begin leg_start<=goal;leg_delta<=next_goal-goal;goal<=next_goal;segment<=segment+1'b1;
                    phase<=0;tick_count<=0;remainder<=0;qualified<=0;wait_count<=0;settling<=0;end
            end else if(PASS_THROUGH!=0) begin
                // No endpoint taper, zero-speed wait, or dwell. Keep a bounded
                // approach velocity until the encoder reaches the waypoint.
                // After the position reference reaches goal, this velocity is
                // a tracking request (not the derivative of the held reference).
                if(angle_ok) begin
                    reference_q16<=pass_reference_arrived ? pass_goal : pass_next;
                    speed_q16<=pass_velocity;settling<=pass_reference_arrived;
                    if(pass_reference_arrived) begin
                        if(TIMEOUT_TICKS>0 && wait_count>=TIMEOUT_TICKS-1) begin timed_out<=1;speed_q16<=0;end
                        else wait_count<=wait_count+1'b1;
                    end else wait_count<=0;
                end else speed_q16<=0;
            end else if(!settling) begin
                if(tick_count>=period) begin reference_q16<=$signed(goal)*48'sd65536;speed_q16<=0;settling<=1;qualified<=0;wait_count<=0;end
                else if(angle_ok) begin
                    reference_q16<=next_reference;speed_q16<=next_reference-reference_q16;
                    phase<=phase+phase_step(segment)+(remainder_sum>=period);
                    remainder<=remainder_sum>=period ? remainder_sum-period : remainder_sum;
                    tick_count<=tick_count+1'b1;
                end else speed_q16<=0;
            end else begin
                speed_q16<=0;
                if(PASS_THROUGH==0 && arrived) begin
                    if(qualified>=DWELL_TICKS-1) begin
                        if(segment==11) done<=1;
                        else begin leg_start<=goal;leg_delta<=next_goal-goal;goal<=next_goal;segment<=segment+1'b1;
                            phase<=0;tick_count<=0;remainder<=0;qualified<=0;wait_count<=0;settling<=0;end
                    end else qualified<=qualified+1'b1;
                end else qualified<=0;
                if(TIMEOUT_TICKS>0 && wait_count>=TIMEOUT_TICKS-1 && !(PASS_THROUGH==0 && arrived && qualified>=DWELL_TICKS-1)) timed_out<=1;
                else wait_count<=wait_count+1'b1;
            end
        end
    end
endmodule
