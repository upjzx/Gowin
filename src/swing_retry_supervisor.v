// Outer scheduler only: the preserved V30 pumping/capture FSM is unchanged.
// Tick is 5 ms. Faults and K4 can never be cleared by automatic retries.
module swing_retry_supervisor #(
    parameter integer ENABLE=1, REPEAT_ENABLE=0, REPEAT_MAX_ROUNDS=0, TIMEOUT_TICKS=5000, MAX_ATTEMPTS=3,
    parameter integer SUCCESS_TICKS=200, REST_TICKS=400
) (
    input wire clk, rst_n, tick, run_active, balance_enable,
    input wire healthy, fault, cancelled, stable_balance, stationary_down,
    output wire inhibit, output reg retry_start,
    output reg [2:0] phase, output reg [7:0] attempt,
    output reg [15:0] elapsed_ticks,
    input wire cycle_complete
);
    // 0 idle, 1 attempting, 2 timeout/zero output, 3 waiting, 4 success,
    // 5 attempts exhausted, 6 protected/cancelled.
    reg [15:0] qualified;
    reg [15:0] completed_rounds;
    assign inhibit=ENABLE!=0 && (phase==2 || phase==3 || phase==5 || phase==6 || phase==7);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            phase<=0; attempt<=0; elapsed_ticks<=0; qualified<=0; retry_start<=0; completed_rounds<=0;
        end else begin
            retry_start<=0;
            if(ENABLE==0) begin phase<=0; attempt<=0; elapsed_ticks<=0; qualified<=0; end
            else if(fault || cancelled || (phase!=0 && !healthy)) begin
                phase<=6; qualified<=0;
            end else if(REPEAT_ENABLE!=0 && cycle_complete===1'b1 && phase==4) begin
                phase<=7;qualified<=0; if(completed_rounds<65535) completed_rounds<=completed_rounds+1'b1;
            end else if(phase==0 && run_active) begin
                phase<=1; attempt<=attempt+1'b1; elapsed_ticks<=0; qualified<=0;
            end else if(tick) begin
                case(phase)
                    1: begin
                        if(balance_enable && stable_balance && qualified>=SUCCESS_TICKS-1) begin
                            phase<=4; qualified<=0;
                        end else begin
                            if(balance_enable && stable_balance) qualified<=qualified+1'b1;
                            else qualified<=0;
                            if(elapsed_ticks>=TIMEOUT_TICKS-1) begin
                                elapsed_ticks<=TIMEOUT_TICKS; phase<=2; qualified<=0;
                            end else elapsed_ticks<=elapsed_ticks+1'b1;
                        end
                    end
                    2: begin phase<=attempt>=MAX_ATTEMPTS ? 5 : 3; qualified<=0; end
                    3: begin
                        if(!run_active && stationary_down) begin
                            if(qualified>=REST_TICKS-1) begin
                                phase<=0; retry_start<=1; qualified<=0;
                            end else qualified<=qualified+1'b1;
                        end else qualified<=0;
                    end
                    // Intentional end-of-mission coast, then require2s stationary
                    // hanging before a new swing-up. Faults still latch above.
                    7: begin
                        if((REPEAT_MAX_ROUNDS==0 || completed_rounds<REPEAT_MAX_ROUNDS) && !run_active && stationary_down) begin
                            if(qualified>=REST_TICKS-1) begin
                                phase<=0;retry_start<=1;qualified<=0;attempt<=0;elapsed_ticks<=0;
                            end else qualified<=qualified+1'b1;
                        end else qualified<=0;
                    end
                    // A successful balance never times out or restarts.
                    4: if(!run_active) phase<=6;
                    default: qualified<=0;
                endcase
            end
        end
    end
endmodule
