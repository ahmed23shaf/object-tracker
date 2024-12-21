module I2C (
    input FSM_Clk,          // 2 MHz
    output I2C_SCL_1,
    inout  I2C_SDA_1,


    input  PC_control,      // GO/STOP! signal
    input [6:0] reg_addr,   // 7-bit register address
    input [6:0] SAD,        // 7-bit slave address 🤡🤡
    input  RW,              // 1 if reading
    input [3:0] number_bytes_to_read,
    input [7:0] data_to_write,
    output reg [31:0] data_0,
    output reg [31:0] data_1,
   
    //debug
    output reg [7:0] led,
    output reg ACK,
    output reg SCL,
    output reg SDA,
    output reg [3:0] state
);
    /*** BYTES TO WRITE ***/
    wire [7:0]  SAD_R_ADDR; assign SAD_R_ADDR = {SAD, 1'b1};   // LSB 1 for read
    wire [7:0]  SAD_W_ADDR; assign SAD_W_ADDR = {SAD, 1'b0};


    wire [7:0]  SUB_ADDR;   // MSB is 0 for single-byte operations=
    // Single-byte operations reserved for writing and reading only 1-byte
    assign      SUB_ADDR = (RW == 1'b0 || number_bytes_to_read == 4'd1) ? {1'b0, reg_addr}:{1'b1, reg_addr};


    /*** STATE ENCODING ***/
    localparam [3:0]
        IDLE          = 4'd0,
        ST            = 4'd1,
        SAD_W         = 4'd2,
        SAK           = 4'd3,
        SUB           = 4'd4,
        SAD_R         = 4'd5,
        WRITING_DATA  = 4'd6,
        READING_DATA  = 4'd7,
        NMAK          = 4'd8,
        MAK           = 4'd9,
        SP            = 4'd10,
        SR            = 4'd11;
   
    reg [3:0] prev_state;
    // Writing one byte transitions
    // 0, 1, 2, 3, 4, 3, 6, 3, 10, 0
   
    // Reading one byte
    // 0, 1, 2, 3, 4, 3, 11, 5, 3, 7, 8, 10, 0
   
    // Reading multiple bytes
    // 0, 1, 2, 3, 4, 3, 11, 5, 3, 7, 9, ..., 7, 9, 7, 8, 10, 0


    /*** FSM ***/
    reg [2:0] bit_counter;
    reg [4:0] clk_counter;
    reg [3:0] number_bytes_read;


    reg [7:0] data_o_byte;


    always @ (posedge FSM_Clk) begin
        case (state)
            IDLE: begin
                if (PC_control == 1'b1) begin
                    {prev_state, state} <= {state, ST};
                    {bit_counter, clk_counter} <= {3'd0, 5'd0};
                    number_bytes_read <= 4'd0;
                end
            end
            ST: begin   // establish a falling edge in SDA with SCLK high
                case (clk_counter)
                    5'd0: SCL <= 1'b1;
                    5'd2: SDA <= 1'b0;
                    5'd4: begin
                        {prev_state, state} <= {state, SAD_W};
                        {bit_counter, clk_counter} <= {3'd0, 5'd0};
                    end
                    default: state <= ST;
                endcase


                if (clk_counter != 5'd4) clk_counter <= clk_counter + 1'b1;


            end
            SAD_W: begin // SAD + W, writing 1 bit at a time starting with MSB
                case (clk_counter)
                    5'd0: SCL <= 1'b0;
                    5'd2: SDA <= SAD_W_ADDR[7 - bit_counter];
                    5'd4: SCL <= 1'b1;
                    5'd8: begin
                        if (bit_counter == 3'd7) begin
                            {prev_state, state} <= {state, SAK};
                            {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        end
                        else begin
                            {bit_counter, clk_counter} <= {bit_counter + 1'b1, 5'd0};
                        end
                    end
                    default: state <= SAD_W;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            SAK: begin
                case (clk_counter)
                    5'd0: {SCL, SDA} <= 2'b0z;
                    5'd4: {SCL, ACK} <= {1'b1, SDA};
                    5'd8: begin
                        {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        case (prev_state)
                            SAD_W:          {prev_state, state} <= {state, SUB};
                            SUB:            {prev_state, state} <= (RW == 1'b1) ? {state, SR}:{state, WRITING_DATA};
                            SAD_R:          {prev_state, state} <= {state, READING_DATA};
                            WRITING_DATA:   {prev_state, state} <= {state, SP};
                            default:        state <= SAK;
                        endcase
                    end
                    default: state <= SAK;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            SUB: begin
                case (clk_counter)
                    5'd0: SCL <= 1'b0;
                    5'd2: SDA <= SUB_ADDR[7 - bit_counter];
                    5'd4: SCL <= 1'b1;
                    5'd8: begin
                        if (bit_counter == 3'd7) begin
                            {prev_state, state} <= {state, SAK};
                            {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        end
                        else begin
                            {bit_counter, clk_counter} <= {bit_counter + 1'b1, 5'd0};
                        end
                    end
                    default: state <= SUB;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            SAD_R: begin
                case (clk_counter)
                    5'd0: SCL <= 1'b0;
                    5'd2: SDA <= SAD_R_ADDR[7 - bit_counter];
                    5'd4: SCL <= 1'b1;
                    5'd8: begin
                        if (bit_counter == 3'd7) begin
                            {prev_state, state} <= {state, SAK};
                            {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        end
                        else begin
                            {bit_counter, clk_counter} <= {bit_counter + 1'b1, 5'd0};
                        end
                    end
                    default: state <= SAD_R;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            WRITING_DATA: begin
                case (clk_counter)
                    5'd0: SCL <= 1'b0;
                    5'd2: SDA <= data_to_write[7 - bit_counter];
                    5'd4: SCL <= 1'b1;
                    5'd8: begin
                        if (bit_counter == 3'd7) begin
                            {prev_state, state} <= {state, SAK};
                            {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        end
                        else begin
                            {bit_counter, clk_counter} <= {bit_counter + 1'b1, 5'd0};
                        end
                    end
                    default: state <= WRITING_DATA;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            READING_DATA: begin // read only when SCL is high
               case (clk_counter)
                    5'd0: {SCL,SDA} <= 2'b0z;
                    5'd4: SCL <= 1'b1;
                    5'd5: data_o_byte[7-bit_counter] <= SDA;
                    5'd8: begin
                        if ((bit_counter == 3'd7)
                        && ((number_bytes_read+1'b1) == number_bytes_to_read)) begin // need to terminate
                            {prev_state, state} <= {state, NMAK};
                            {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        end
                        else if ((bit_counter == 3'd7)
                        && ((number_bytes_read+1'b1) < number_bytes_to_read)) begin // need to read more
                            number_bytes_read <= number_bytes_read + 1'b1;
                            {prev_state, state} <= {state, MAK};
                            {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        end
                        else begin
                            {bit_counter, clk_counter} <= {bit_counter + 1'b1, 5'd0};
                        end
                    end
                    default: state <= READING_DATA;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            NMAK: begin
                case (clk_counter)
                    5'd0: SCL <= 1'b0;
                    5'd2: SDA <= 1'b1;
                    5'd4: SCL <= 1'b1;
                    5'd8: begin
                        {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        {prev_state, state} <= {state, SP};
                    end
                    default: state <= NMAK;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            MAK: begin
                case (clk_counter)
                    5'd0: SCL <= 1'b0;
                    5'd2: SDA <= 1'b0;
                    5'd4: SCL <= 1'b1;
                    5'd8: begin
                        {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        {prev_state, state} <= {state, READING_DATA};
                    end
                    default: state <= MAK;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            SP: begin   // rising edge in SDA while SCL high
                case (clk_counter)
                    5'd0: SCL <= 1'b0; //{SCL, SDA} <= 2'b00;
                    5'd2: SDA <= 1'b0;
                    5'd4: SCL <= 1'b1;
                    5'd7: SDA <= 1'b1;
                    5'd10: begin    // wait a bit before reading again
                        {bit_counter, clk_counter} <= {3'd0, 5'd0};
                        {prev_state, state} <= {state, IDLE};
                    end
                    default: state <= SP;
                endcase


                if (clk_counter != 5'd10) clk_counter <= clk_counter + 1'b1;
            end
            SR: begin
                case (clk_counter)
                    5'd0: {SDA, SCL} <= 2'b00;
                    5'd4: {SDA, SCL} <= 2'b11;
                    5'd6: SDA <= 1'b0;
                    5'd8: begin
                        {prev_state, state} <= {state, SAD_R};
                        {bit_counter, clk_counter} <= {3'd0, 5'd0};
                    end
                    default: state <= SR;
                endcase


                if (clk_counter != 5'd8) clk_counter <= clk_counter + 1'b1;
            end
            default: led <= 8'hff; // turns LED off if some error occurs
        endcase
    end


    assign I2C_SCL_1 = SCL;
    assign I2C_SDA_1 = SDA;


    /*** Multi-byte read support ***/
    always @(posedge FSM_Clk) begin
        case ({number_bytes_read, state})
            {4'd0, READING_DATA}: data_0[31:24]     <= data_o_byte;
            {4'd1, READING_DATA}: data_0[23:16]     <= data_o_byte;
            {4'd2, READING_DATA}: data_0[15:8]      <= data_o_byte;
            {4'd3, READING_DATA}: data_0[7:0]       <= data_o_byte;
            {4'd4, READING_DATA}: data_1[31:24]     <= data_o_byte;
            {4'd5, READING_DATA}: data_1[23:16]     <= data_o_byte;
            {4'd6, READING_DATA}: data_1[15:8]      <= data_o_byte;
            {4'd7, READING_DATA}: data_1[7:0]       <= data_o_byte;
        endcase
    end


    /*** Variable init ***/
    initial begin
        {prev_state, state} = {2{IDLE}};
        {SCL, SDA, ACK} = 3'b111;
        {bit_counter, clk_counter} = {3'd0, 5'd0};
        number_bytes_read = 4'd0;
        {data_0, data_1} <= 64'd0;
        led = 8'h0F;
    end


endmodule
