module ClockGenerator(
    input sys_clkn,
    input sys_clkp,         // clkn+clkp - differential 200 MHz
    output reg I2C_FSM_CLK, // 2 MHz
    output reg SPI_FSM_CLK, // 50 MHz
    output reg ILA_Clk,     // 20 MHz
    output reg CLK_INPUT,    // 40 MHz
    output wire CLK_MOTOR    // 200 MHz
);


    //Generate high speed main clock from two differential clock signals        
    wire clk;
    reg [23:0] ClkDivI2C = 24'd0;
    reg [23:0] ClkDivSPI = 24'd0;
    reg [23:0] ClkDivILA = 24'd0;
    reg [23:0] ClkDivINPUT = 24'd0;


    IBUFGDS osc_clk(
        .O(clk),    // 200 MHz
        .I(sys_clkp),
        .IB(sys_clkn)
    );    
    assign CLK_MOTOR = clk;
         
    // Initialize the two registers used in this module  
    initial begin
        {I2C_FSM_CLK, SPI_FSM_CLK} = 2'b00;        
        ILA_Clk = 1'b0;
        CLK_INPUT = 1'b0;
    end
 
    // We derive a clock signal that will be used for sampling signals for the ILA
    // This clock will be 3 times slower than the system clock.    
    always @(posedge clk) begin        
        // if (ClkDivILA == 10) begin
        //     ILA_Clk <= !ILA_Clk;                      
        //     ClkDivILA <= 0;
        // end else begin                        
        //     ClkDivILA <= ClkDivILA + 1'b1;
        // end
        ILA_Clk <= !ILA_Clk;
       
    end      


    // I2c - 2 MHz
    always @(posedge clk) begin        
       if (ClkDivI2C == 10) begin
         I2C_FSM_CLK <= !I2C_FSM_CLK;                  
         ClkDivI2C <= 0;
       end else begin
         ClkDivI2C <= ClkDivI2C + 1'b1;            
       end
    end


    // SPI - 50 Mhz
    always @(posedge clk) begin        
       if (ClkDivSPI == 4) begin
         SPI_FSM_CLK <= !SPI_FSM_CLK;                  
         ClkDivSPI <= 0;
       end else begin
         ClkDivSPI <= ClkDivSPI + 1'b1;            
       end
    end          
   
    always @(posedge clk) begin        
       if (ClkDivINPUT == 5) begin
         CLK_INPUT <= !CLK_INPUT;                  
         ClkDivINPUT <= 0;
       end else begin
         ClkDivINPUT <= ClkDivINPUT + 1'b1;            
       end
    end          
endmodule
