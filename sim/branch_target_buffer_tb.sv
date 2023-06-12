module branch_target_buffer_tb();

    logic btb_clk;
    logic btb_reset;
    logic btb_write; // allow data to be written
    logic btb_branch_taken; // Tells if branch was taken so write can be updated

    logic [31:0] btb_pc; // program counter in fetch
    logic [31:0] btb_new_pc; // program counter in decode/execute stage to be added
    logic [31:0] btb_data; // memory


    logic btb_valid_prediction;
    logic [31:0] btb_target; // PC branch will jump to


    // Clock generation
    always #5 CLK = ~CLK;

    // Test Stimulus
    initial begin

        
    end


    // Initialization of DUT
    branch_target_buffer BTB(.btb_clk(btb_clk), .btb_reset(), .btb_write(), .btb_branch_taken(), .btb_pc(), .btb_new_pc(), .btb_data(), .btb_valid_prediction(), .btb_target());

endmodule