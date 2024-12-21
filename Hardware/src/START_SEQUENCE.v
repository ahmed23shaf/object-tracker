module START_SEQUENCE(
    input clk,  // 40 MHz
    output reg resetn,   // active-low reset


    output reg state
);
    localparam [3:0]
        RESET_ON      = 4'd0,
        RESET_OFF     = 4'd1;


    reg [8:0] cycle_ctr;


    always @ (posedge clk) begin
   
        case (state)
            RESET_ON:  begin
                if (cycle_ctr == 9'd300) state <= RESET_OFF;
                cycle_ctr <= cycle_ctr+1'b1;
            end
            RESET_OFF: resetn <= 1'b1;
        endcase
    end


    initial begin
        cycle_ctr = 5'd0;
        resetn = 1'b0;
        state = RESET_ON;
    end


endmodule
