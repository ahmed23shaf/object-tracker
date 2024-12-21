module top(
    // CLK
    input sys_clkn,
    input sys_clkp,
   
    // output [7:0] led,   //Debug


    // PYTHON
    input  [4:0] okUH,
    output [2:0] okHU,
    inout  [31:0] okUHU,
    inout  okAA,
 
    // CAMERA
    output CVM300_SYS_RES_N,
    output CVM300_CLK_IN,
    input  CVM300_CLK_OUT, // Parallel CMOS output clock
    output CVM300_FRAME_REQ,
    output CVM300_Enable_LVDS,
   
    output CVM300_SPI_EN,
    output CVM300_SPI_IN,
    input  CVM300_SPI_OUT,
    output CVM300_SPI_CLK,


    input [9:0] CVM300_D,
    input       CVM300_Line_valid,
    input       CVM300_Data_valid,


    // I2C
    output I2C_SCL_1,
    inout I2C_SDA_1,
   
    // PMOD
    output PMOD_A1,
    output PMOD_A2
);
    assign CVM300_Enable_LVDS = 1'b0;   // GND (parallel CMOS output)


    /****** Clock generator ******/
    wire ILA_Clk, CLK_MOTOR, I2C_FSM_CLK, SPI_FSM_CLK;
    ClockGenerator ClockGenerator1 (  .sys_clkn(sys_clkn),
                                      .sys_clkp(sys_clkp),
                                      .I2C_FSM_CLK(I2C_FSM_CLK),
                                      .SPI_FSM_CLK(SPI_FSM_CLK),
                                      .ILA_Clk(ILA_Clk),
                                      .CLK_INPUT(CVM300_CLK_IN),
                                      .CLK_MOTOR(CLK_MOTOR) );


    /****** OK Communication ******/
    wire [112:0]    okHE;    
    wire [64:0]     okEH;


    okHost hostIF (
        .okUH(okUH),
        .okHU(okHU),
        .okUHU(okUHU),
        .okClk(okClk),
        .okAA(okAA),
        .okHE(okHE),
        .okEH(okEH)
    );


    localparam  endPt_count = 4;    // 4 output
    wire [endPt_count*65-1:0] okEHx;  
    okWireOR # (.N(endPt_count)) wireOR (okEH, okEHx);


    wire [7:0] data_out;
    wire [31:0] data_0, data_1;


    wire PC_control, RW, RW_IMU, PC_Image_Req, PC_control_IMU;
    wire [6:0] reg_addr;
    wire [7:0] reg_addr_IMU, slave_addr;
    wire [7:0] data_to_write, data_to_write_IMU;
    wire [3:0] number_bytes_to_read;


    wire FIFO_read_enable, FIFO_BT_BlockSize_Full, FIFO_full, FIFO_empty, BT_Strobe;
    wire [31:0] FIFO_data_out;


    // ~~~~ Output (okWireOut) ~~~~ //
        // CMV300
    okWireOut wire20 (  .okHE(okHE),
                        .okEH(okEHx[ 0*65 +: 65 ]),
                        .ep_addr(8'h20),
                        .ep_datain({24'd0, data_out}));


        // BT PIPE
    okBTPipeOut CMV300ToPC (
        .okHE(okHE),
        .okEH(okEHx[ 1*65 +: 65 ]),
        .ep_addr(8'ha0),
        .ep_datain({FIFO_data_out[7:0],FIFO_data_out[15:8],FIFO_data_out[23:16],FIFO_data_out[31:24]}),
        .ep_read(FIFO_read_enable),
        .ep_blockstrobe(BT_Strobe),
        .ep_ready(FIFO_BT_BlockSize_Full)
    );


        // Accelerometer + Magnetometer
    okWireOut wire21 (  .okHE(okHE),
                        .okEH(okEHx[ 2*65 +: 65 ]),
                        .ep_addr(8'h21),
                        .ep_datain(data_0));
   
    okWireOut wire22 (  .okHE(okHE),
                        .okEH(okEHx[ 3*65 +: 65 ]),
                        .ep_addr(8'h22),
                        .ep_datain(data_1));




    // ~~~~ Input (okWireIn) ~~~~ //
        // CMV300 IMAGE SENSOR
    okWireIn wire00 (   .okHE(okHE),
                        .ep_addr(8'h00),
                        .ep_dataout({31'd0, PC_control}));


    okWireIn wire01 (   .okHE(okHE),
                        .ep_addr(8'h01),
                        .ep_dataout({24'd0, reg_addr, RW}));


    okWireIn wire02 (   .okHE(okHE),
                        .ep_addr(8'h02),
                        .ep_dataout({24'd0, data_to_write}));


    okWireIn wire03 (   .okHE(okHE),
                        .ep_addr(8'h03),
                        .ep_dataout({31'd0, PC_Image_Req}));


        // Accelerometer + Magnetometer
    okWireIn wire04 (   .okHE(okHE),
                        .ep_addr(8'h04),
                        .ep_dataout({31'd0, PC_control_IMU}));
   
    okWireIn wire05 (   .okHE(okHE),
                        .ep_addr(8'h05),
                        .ep_dataout({19'd0, data_to_write_IMU, number_bytes_to_read, RW_IMU}));


    okWireIn wire06 (   .okHE(okHE),
                        .ep_addr(8'h06),
                        .ep_dataout({16'd0, slave_addr, reg_addr_IMU}));


    wire motor_start, dir_in;
    wire [29:0] pulse_num;
    okWireIn wire07 (   .okHE(okHE),
                        .ep_addr(8'h07),
                        .ep_dataout({motor_start, dir_in, pulse_num}));


    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ CMV300 IMAGE SENSOR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    /****** SPI ******/
    wire [3:0] state;


    SPI spi (
        .FSM_clk(SPI_FSM_CLK),  // 50 MHz


        .PC_control(PC_control),
        .reg_addr(reg_addr),
        .RW(RW),                  // 1 if writing
        .data_to_write(data_to_write),
        .data_out(data_out),


        .SPI_CLK(CVM300_SPI_CLK),
        .SPI_IN(CVM300_SPI_IN),
        .SPI_OUT(CVM300_SPI_OUT),
        .SPI_EN(CVM300_SPI_EN),


        .state(state),
        .led()
    );




    /****** CAMERA ******/
    START_SEQUENCE start(
       .clk(CVM300_CLK_IN),
       .resetn(CVM300_SYS_RES_N),
       .state()
    );


    wire write_reset, read_reset;
    wire [3:0] img_state;   // debug
    reg [31:0] img_counter; // debug


    FrameCapture img_fsm (
        // inputs
        .clk(CVM300_CLK_OUT),
        .PC_image_req(PC_Image_Req),


        // outputs
        .state(img_state),
        .FRAME_REQ_OUT(CVM300_FRAME_REQ),
        .wr_reset(write_reset),
        .rd_reset(read_reset)
    );


    /****** FIFO ******/
    fifo_generator_0 fifo (
        .wr_clk(CVM300_CLK_OUT),
        .wr_rst(write_reset),   // from img fsm
        .rd_clk(okClk),
        .rd_rst(read_reset),    // from img fsm
        .din(CVM300_D[9:2]),
        .wr_en(CVM300_Data_valid),
        .rd_en(FIFO_read_enable),
        .dout(FIFO_data_out),
        .full(FIFO_full),
        .empty(FIFO_empty),      
        .prog_full(FIFO_BT_BlockSize_Full)        
    );


    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Accelerometer + Magnetometer ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
       /****** I2C ******/
    wire SCL, SDA, ACK_bit;


    I2C i2c (
        .FSM_Clk(I2C_FSM_CLK),
        .I2C_SCL_1(I2C_SCL_1),
        .I2C_SDA_1(I2C_SDA_1),
        .PC_control(PC_control_IMU),
        .reg_addr(reg_addr_IMU[6:0]),
        .SAD(slave_addr[6:0]),
        .RW(RW_IMU),
        .number_bytes_to_read(number_bytes_to_read),
        .data_to_write(data_to_write_IMU),
        .data_0(data_0),
        .data_1(data_1),
        .led(),
        .ACK(ACK_bit),
        .SCL(SCL),
        .SDA(SDA),
        .state()
    );


    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PMOD ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    PMOD obj (
        .clk(CLK_MOTOR),
       
        .EN(PMOD_A1),
        .dir_out(PMOD_A2),
       
        .PMOD_in({1'b0, motor_start, 1'b0, dir_in, pulse_num[27:0]})
    );
   


endmodule
