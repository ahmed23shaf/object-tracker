module FrameCapture (
    input wire clk,
    input wire PC_image_req,


    output reg [3:0] state,
   
    output reg FRAME_REQ_OUT,
    output reg wr_reset,
    output reg rd_reset
);


    // based on BTPipeExample.v
    localparam [3:0]
        IDLE        = 4'd0,
        FIFO_RESET  = 4'd1,
        STALL_FIFO  = 4'd2, // waiting time for FIFO to reset
        FRAME_REQ   = 4'd3;


    // 0) Wait for IMAGE_REQUEST
    // 1) Reset the FIFO: reset all counters and enable reset bits
    // 2) Stall for some time for reset to process (inside hardware)
    // 3) Send the `FRAME_REQ` pulse (3.10.1)
    // 4) Wait out the exposure time, FOT          (inside hardware)
    // 5) Start writing to the FIFO from imager data during 'read-out time'
    // 6) Once FIFO has written 308*1024 pixels, return to an IDLE state


    reg [15:0] delay_ctr;


    always @ (posedge clk) begin
        case (state)
            IDLE: begin
                delay_ctr <= 16'd0;
                if (PC_image_req == 1'b1) state <= FIFO_RESET;
            end
            FIFO_RESET: begin
                delay_ctr <= 16'd0;
                {wr_reset, rd_reset} <= 2'b11;


                if (PC_image_req == 1'b0) state <= STALL_FIFO;
            end
            STALL_FIFO: begin
                {wr_reset, rd_reset} <= 2'b00;
                if (delay_ctr == 16'h7fff) begin
                    delay_ctr <= 16'd0;
                    state     <= FRAME_REQ;
                end
                else
                    delay_ctr <= delay_ctr + 1'b1;                
            end
            FRAME_REQ: begin
                FRAME_REQ_OUT <= 1'b1;


                if (delay_ctr == 16'h0003) begin
                    FRAME_REQ_OUT <= 1'b0;
                    delay_ctr <= 16'd0;
                    state <= IDLE;
                end
                else
                    delay_ctr <= delay_ctr + 1'b1;
            end
        endcase
    end


    initial begin
        FRAME_REQ_OUT = 1'b0;
        state      = IDLE;
        wr_reset   = 1'b0;
        rd_reset   = 1'b0;
    end


endmodule
