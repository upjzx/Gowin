// Endpoint/anti-stiction qualification only. The original damping observer is
// unchanged. 16 samples at 5 ms measures actual 80 ms encoder displacement;
// +/-2 counts corresponds to +/-25 counts/s, the existing Q8 threshold 32.
module encoder_quiet_window #(parameter integer WINDOW_LOG2=4)(
    input wire clk,rst_n,enable,capture,update,
    input wire signed [31:0] position,
    output reg signed [31:0] velocity_q8,
    output wire valid,stable_window
);
    localparam integer N=1<<WINDOW_LOG2;
    reg signed [31:0] history[0:N-1];
    reg [WINDOW_LOG2-1:0] index;
    reg [WINDOW_LOG2:0] samples;
    reg [WINDOW_LOG2:0] stationary_samples;
    reg signed [31:0] stationary_origin;
    wire signed [32:0] delta=$signed({position[31],position})-$signed({history[index][31],history[index]});
    wire signed [31:0] bounded=delta>32767 ? 32'sd32767 : delta< -32767 ? -32'sd32767 : delta[31:0];
    assign valid=samples>N;
    wire signed [32:0] excursion=$signed({position[31],position})-$signed({stationary_origin[31],stationary_origin});
    assign stable_window=valid && stationary_samples>=N;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin index<=0;samples<=0;velocity_q8<=0;stationary_origin<=0;stationary_samples<=0;end
        else if(!enable || capture) begin index<=0;samples<=0;velocity_q8<=0;stationary_samples<=0;end
        else if(update) begin
            history[index]<=position;index<=index+1'b1;
            if(samples==0 || excursion>2 || excursion< -2) begin
                stationary_origin<=position;stationary_samples<=0;
            end else if(stationary_samples<N) stationary_samples<=stationary_samples+1'b1;
            if(samples<N) begin samples<=samples+1'b1;velocity_q8<=0;end
            else begin velocity_q8<=bounded<<<(8-WINDOW_LOG2);samples<=N+1;end
        end
    end
endmodule
