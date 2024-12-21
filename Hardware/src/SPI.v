// Master: Opal Kelley board
// Slave: Camera sensor
module SPI(
    input FSM_clk,  // 50 MHz


    //py
    input PC_control,
    input [6:0] reg_addr,
    input RW,                  // 1 if writing
    input [7:0] data_to_write,
    output reg [7:0] data_out,


    //spi signals
    output reg SPI_CLK,
    output reg SPI_IN,         // MOSI
    input wire SPI_OUT,        // MISO
    output reg SPI_EN,


    //debug
    output reg [3:0] state,
    output reg [7:0] led
);
    localparam [3:0]
        IDLE        = 4'd0,
        START       = 4'd1,
        ADDR        = 4'd2,
        WRITING     = 4'd3,
        READING     = 4'd4,
        FINISH      = 4'd5;
    // Writing transitions
    // 0, 1, 2, 3, 5, 0, ...
   
    // Reading
    // 0, 1, 2, 4, 5, 0, ...


    /*** FSM ***/
    reg [2:0] bit_counter;
    reg [4:0] clk_counter;  // Clock cycle = 2 ticks


    always @ (posedge FSM_clk) begin
        case (state)
            IDLE: begin
                if (PC_control == 1'b1) begin
                    state <= START;
                    {bit_counter, clk_counter} <= 8'd0;
                end
            end
            START: begin    // pulling SPI_EN high
                case (clk_counter)
                    5'd0: begin
                        SPI_EN <= 1'b1;
                        SPI_IN <= RW;
                    end
                    5'd1: begin
                        SPI_CLK <= 1'b1;
                        state <= ADDR;
                        {bit_counter, clk_counter} <= 8'd0;
                    end
                endcase


                if (clk_counter != 5'd1) clk_counter <= clk_counter + 1'b1;
            end
            ADDR: begin     // writing ADDR bits
                case (clk_counter)
                    5'd0: begin
                        SPI_CLK <= 1'b0;
                        SPI_IN <= reg_addr[6-bit_counter];
                    end
                    5'd1: begin
                        SPI_CLK <= 1'b1;
                        if (bit_counter == 6) begin
                            state <= RW ? WRITING:READING;
                            {bit_counter, clk_counter} <= 8'd0;
                        end
                        else begin
                            {bit_counter, clk_counter} <= {bit_counter + 1'b1, 5'd0};
                        end
                    end
                endcase


                if (clk_counter != 5'd1) clk_counter <= clk_counter + 1'b1;
            end
            WRITING: begin  // writing reg data
                case (clk_counter)
                    5'd0: begin
                        SPI_CLK <= 1'b0;
                        SPI_IN <= data_to_write[7-bit_counter];
                    end
                    5'd1: begin
                        SPI_CLK <= 1'b1;
                        if (bit_counter == 7) begin
                            state <= FINISH;
                            {bit_counter, clk_counter} <= 8'd0;
                        end
                        else begin
                            {bit_counter, clk_counter} <= {bit_counter + 1'b1, 5'd0};
                        end
                    end
                endcase


                if (clk_counter != 5'd1) clk_counter <= clk_counter + 1'b1;
            end
            READING: begin
                case (clk_counter)
                    5'd0: begin
                        {SPI_IN, SPI_CLK} <= 2'b00;
                    end
                    5'd1: begin
                        data_out[7-bit_counter] <= SPI_OUT; // read data on the rising edge
                       
                        SPI_CLK <= 1'b1;
                        if (bit_counter == 7) begin
                            state <= FINISH;
                            {bit_counter, clk_counter} <= 8'd0;
                        end
                        else begin
                            {bit_counter, clk_counter} <= {bit_counter + 1'b1, 5'd0};
                        end
                    end
                endcase


                if (clk_counter != 5'd1) clk_counter <= clk_counter + 1'b1;
            end
            FINISH: begin
                case (clk_counter)
                    5'd0: {SPI_IN, SPI_CLK} <= 2'b00;
                    5'd1: begin
                       SPI_EN <= 1'b0;
                       state <= IDLE;
                       {bit_counter, clk_counter} <= 8'd0;
                    end
                endcase
                if (clk_counter != 5'd1) clk_counter <= clk_counter + 1'b1;
            end
            default: led <= 8'hFF;
        endcase
    end


    /*** Variable init ***/
    initial begin
        state = IDLE;
        {SPI_EN, SPI_CLK, SPI_IN} = 3'd0;
        {bit_counter, clk_counter} = 8'd0;
        led = 8'h0A;
    end


endmodule
