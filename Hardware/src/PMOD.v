module PMOD(
    input clk,


    output EN,
    output dir_out,


    input [31:0] PMOD_in
);
    reg start, dir_in;


    /*** STATE ENCODING ***/
    localparam [2:0]
        IDLE           = 3'd0,
        RUNNING        = 3'd1,
        STALL_START    = 3'd2;
    // IDLE -> RUNNING -> (after desired pulses) -> IDLE


    reg [27:0] pulse_counter;


    assign {start, dir_in, pulse_counter} = {PMOD_in[30], PMOD_in[28], PMOD_in[27:0]};


    assign DIR_OUT = dir_in;


    always @ (posedge clk) begin
        case (state)
            IDLE: begin
                if (start == 1'b1) begin
                    pulse_counter <= 8'd0;
                    state <= RUNNING;
                end
                else if (start == 1'b0) begin
                    state <= IDLE;
                end
            end
            RUNNING: begin
                EN <= ~EN;


                // condition to increment
                if (EN == 1'b0 && pulse_counter < pulse_num) pulse_counter <= pulse_counter + 1'b1;


                if (EN == 1'b0 && pulse_counter == pulse_num) begin
                    pulse_counter <= 8'd0;
                    EN <= 1'b0;
                    state <= STALL_START;
                end
            end
            STALL_START: begin
                if (start == 1'b0) state <= IDLE;
            end
        endcase
    end


    assign {EN, dir_out} = {EN, DIR_OUT};


    initial begin
        state = 3'd0;
        EN = 1'b0;
    end


endmodule
