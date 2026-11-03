// Timed fixed-center peaks with C2 quintic joins and no endpoint dwell.
// Seven alternating extrema: +P1,-P2,+P3,-P4,+P3,-P2,+P1, then center.
// Horner arithmetic is time-multiplexed using split 32-bit adders; no DSPs.
// Q24 internal precision, Q16 applied position. A sample is ready in ~260 clocks.
module peak_wavepacket #(parameter integer PEAK_1=90,PEAK_2=240,PEAK_3=390,PEAK_4=540,
    PERIOD_TICKS=2500,DIRECTION=1)(
    input wire clk,rst_n,enable,start,update,allow_advance,
    input wire signed [31:0] center,
    output reg signed [47:0] reference_q16,
    output reg signed [31:0] speed_q16,accel_q16,
    output reg done,output reg [2:0] segment,output wire sample_ready);
    localparam integer SAFE_PERIOD=(PERIOD_TICKS>=4 && PERIOD_TICKS<=120000) ? PERIOD_TICKS : 4;
    localparam integer HALF=SAFE_PERIOD/2,QUARTER=SAFE_PERIOD/4;
    localparam integer HALF_STEP=16777216/HALF,HALF_REM=16777216%HALF;
    localparam integer QUARTER_STEP=16777216/QUARTER,QUARTER_REM=16777216%QUARTER;
    function signed [31:0] endpoint;input [2:0] s;begin case(s)
        0,6:endpoint=PEAK_1;1,5:endpoint=-PEAK_2;2,4:endpoint=PEAK_3;3:endpoint=-PEAK_4;default:endpoint=0;
    endcase end endfunction
    // Coefficients in counts, ascending powers c0..c5.
    // Interior endpoint curvature is -8*peak/(half-period)^2.
    // Start/end quarter-period coefficients preserve the same physical curvature.
    function signed [31:0] coefficient;input [2:0] s,k;begin case({s,k})
        6'd0:coefficient=0;
        6'd1:coefficient=0;
        6'd2:coefficient=0;
        6'd3:coefficient=9*(PEAK_1);
        6'd4:coefficient=-13*(PEAK_1);
        6'd5:coefficient=5*(PEAK_1);
        6'd8:coefficient=PEAK_1;
        6'd9:coefficient=0;
        6'd10:coefficient=-4*(PEAK_1);
        6'd11:coefficient=2*(PEAK_1)+6*(-PEAK_2);
        6'd12:coefficient=3*(PEAK_1)-7*(-PEAK_2);
        6'd13:coefficient=-2*(PEAK_1)+2*(-PEAK_2);
        6'd16:coefficient=-PEAK_2;
        6'd17:coefficient=0;
        6'd18:coefficient=-4*(-PEAK_2);
        6'd19:coefficient=2*(-PEAK_2)+6*(PEAK_3);
        6'd20:coefficient=3*(-PEAK_2)-7*(PEAK_3);
        6'd21:coefficient=-2*(-PEAK_2)+2*(PEAK_3);
        6'd24:coefficient=PEAK_3;
        6'd25:coefficient=0;
        6'd26:coefficient=-4*(PEAK_3);
        6'd27:coefficient=2*(PEAK_3)+6*(-PEAK_4);
        6'd28:coefficient=3*(PEAK_3)-7*(-PEAK_4);
        6'd29:coefficient=-2*(PEAK_3)+2*(-PEAK_4);
        6'd32:coefficient=-PEAK_4;
        6'd33:coefficient=0;
        6'd34:coefficient=-4*(-PEAK_4);
        6'd35:coefficient=2*(-PEAK_4)+6*(PEAK_3);
        6'd36:coefficient=3*(-PEAK_4)-7*(PEAK_3);
        6'd37:coefficient=-2*(-PEAK_4)+2*(PEAK_3);
        6'd40:coefficient=PEAK_3;
        6'd41:coefficient=0;
        6'd42:coefficient=-4*(PEAK_3);
        6'd43:coefficient=2*(PEAK_3)+6*(-PEAK_2);
        6'd44:coefficient=3*(PEAK_3)-7*(-PEAK_2);
        6'd45:coefficient=-2*(PEAK_3)+2*(-PEAK_2);
        6'd48:coefficient=-PEAK_2;
        6'd49:coefficient=0;
        6'd50:coefficient=-4*(-PEAK_2);
        6'd51:coefficient=2*(-PEAK_2)+6*(PEAK_1);
        6'd52:coefficient=3*(-PEAK_2)-7*(PEAK_1);
        6'd53:coefficient=-2*(-PEAK_2)+2*(PEAK_1);
        6'd56:coefficient=PEAK_1;
        6'd57:coefficient=0;
        6'd58:coefficient=-(PEAK_1);
        6'd59:coefficient=-7*(PEAK_1);
        6'd60:coefficient=12*(PEAK_1);
        6'd61:coefficient=-5*(PEAK_1);
        default:coefficient=0;endcase end endfunction
    reg active,request_toggle,was_advancing;
    reg signed [31:0] anchor;
    reg [31:0] tick_index,remainder;
    reg [23:0] phase;
    wire quarter_segment=segment==0 || segment==7;
    wire [31:0] period=quarter_segment ? QUARTER : HALF;
    wire [31:0] step=quarter_segment ? QUARTER_STEP : HALF_STEP;
    wire [31:0] rem_step=quarter_segment ? QUARTER_REM : HALF_REM;
    wire [32:0] rem_sum={1'b0,remainder}+rem_step;
    wire at_endpoint=tick_index>=period-1;

    // Iterative 40x24 signed-by-unsigned multiplication, two clocks per bit.
    reg [2:0] engine_state,work_segment,order;
    reg work_toggle,complete_toggle,result_valid;
    reg signed [39:0] work_value;
    reg signed [63:0] product,multiplicand;
    reg [23:0] work_phase,multiplier;
    reg [4:0] bit_index;
    reg [31:0] low_sum;reg carry;
    reg signed [31:0] relative_q16;
    wire [63:0] operand=multiplier[0] ? multiplicand : 64'd0;
    wire [32:0] mul_low={1'b0,product[31:0]}+{1'b0,operand[31:0]};
    wire [31:0] mul_high=product[63:32]+operand[63:32]+carry;
    wire signed [39:0] coefficient_q24=$signed(coefficient(work_segment,order))*40'sd16777216;
    wire [32:0] coefficient_low={1'b0,product[55:24]}+{1'b0,coefficient_q24[31:0]};
    wire [7:0] coefficient_high=product[63:56]+coefficient_q24[39:32]+carry;
    wire signed [39:0] next_work={coefficient_high,low_sum};
    wire engine_ready=result_valid && complete_toggle==request_toggle && engine_state==0;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin engine_state<=0;work_segment<=0;order<=0;work_toggle<=0;complete_toggle<=0;result_valid<=0;work_value<=0;product<=0;multiplicand<=0;work_phase<=0;multiplier<=0;bit_index<=0;low_sum<=0;carry<=0;relative_q16<=0;end
        else if(!enable || start) begin engine_state<=0;complete_toggle<=0;result_valid<=0;relative_q16<=0;end
        else case(engine_state)
            0:if(!result_valid || complete_toggle!=request_toggle) begin
                work_segment<=segment;work_toggle<=request_toggle;work_phase<=phase;
                work_value<=$signed(coefficient(segment,3'd5))*40'sd16777216;order<=4;engine_state<=1;
            end
            1:begin product<=0;multiplicand<={{24{work_value[39]}},work_value};multiplier<=work_phase;bit_index<=0;engine_state<=2;end
            2:begin low_sum<=mul_low[31:0];carry<=mul_low[32];engine_state<=3;end
            3:begin
                product<={mul_high,low_sum};multiplicand<=multiplicand<<<1;multiplier<=multiplier>>1;
                if(bit_index==23)engine_state<=4;else begin bit_index<=bit_index+1'b1;engine_state<=2;end
            end
            4:begin low_sum<=coefficient_low[31:0];carry<=coefficient_low[32];engine_state<=5;end
            5:begin
                work_value<=next_work;
                if(order==0)begin relative_q16<=next_work[39:8];complete_toggle<=work_toggle;result_valid<=1;engine_state<=0;end
                else begin order<=order-1'b1;engine_state<=1;end
            end
            default:engine_state<=0;
        endcase
    end
    // Keep long center/derivative adders separated from the evaluator.
    reg signed [47:0] oriented_offset,next_reference;
    reg signed [31:0] next_speed,next_accel;
    reg [3:0] ready_pipe;
    assign sample_ready=engine_ready && (&ready_pipe);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin oriented_offset<=0;next_reference<=0;next_speed<=0;next_accel<=0;ready_pipe<=0;end
        else begin
            oriented_offset<=DIRECTION*(at_endpoint ? $signed(endpoint(segment))*48'sd65536 : $signed(relative_q16));
            next_reference<=$signed(anchor)*48'sd65536+oriented_offset;
            next_speed<=next_reference-reference_q16;next_accel<=next_speed-speed_q16;
            if(!enable || start || !engine_ready)ready_pipe<=0;else ready_pipe<={ready_pipe[2:0],1'b1};
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n)begin active<=0;done<=0;segment<=0;anchor<=0;reference_q16<=0;speed_q16<=0;accel_q16<=0;tick_index<=0;phase<=0;remainder<=0;request_toggle<=0;was_advancing<=0;end
        else if(start)begin
            active<=1;done<=0;segment<=0;anchor<=center;reference_q16<=$signed(center)*48'sd65536;speed_q16<=0;accel_q16<=0;
            tick_index<=0;phase<=QUARTER_STEP;remainder<=QUARTER_REM;request_toggle<=1;was_advancing<=0;
        end else if(!enable)begin
            active<=0;done<=0;segment<=0;anchor<=center;reference_q16<=$signed(center)*48'sd65536;speed_q16<=0;accel_q16<=0;
            tick_index<=0;phase<=0;remainder<=0;request_toggle<=0;was_advancing<=0;
        end else if(update)begin
            if(active && !done && allow_advance && sample_ready)begin
                reference_q16<=next_reference;speed_q16<=next_speed;accel_q16<=was_advancing ? next_accel : 32'sd0;was_advancing<=1;
                if(at_endpoint)begin
                    tick_index<=0;
                    if(segment==7)begin done<=1;active<=0;reference_q16<=$signed(anchor)*48'sd65536;speed_q16<=0;accel_q16<=0;was_advancing<=0;end
                    else begin segment<=segment+1'b1;phase<=segment==6 ? QUARTER_STEP : HALF_STEP;remainder<=segment==6 ? QUARTER_REM : HALF_REM;request_toggle<=~request_toggle;end
                end else begin
                    tick_index<=tick_index+1'b1;phase<=phase+step+(rem_sum>=period);remainder<=rem_sum>=period ? rem_sum-period : rem_sum;request_toggle<=~request_toggle;
                end
            end else begin speed_q16<=0;accel_q16<=0;was_advancing<=0;end
        end
    end
endmodule
