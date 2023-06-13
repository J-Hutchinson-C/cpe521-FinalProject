module branch_target_buffer_tb();

    logic btb_clk = 1'b0;
    logic btb_reset;
    logic btb_write; // allow data to be written
    logic btb_branch_taken; // Tells if branch was taken so write can be updated

    logic [31:0] btb_pc; // program counter in fetch
    logic [31:0] btb_new_pc; // program counter in decode/execute stage to be added
    logic [31:0] btb_data; // memory


    logic btb_valid_prediction;
    logic [31:0] btb_target; // PC branch will jump to


    // Clock generation
    always #5 btb_clk = ~btb_clk;

    // Test Stimulus
    initial begin
        //Test writing to the btb
        // Test 1: Adding to new spots
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b1; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        btb_data = 32'hDEADBEEF; // Hex value to be stored in data section
        #20;
        btb_branch_taken = 1'b0; // Branch was taken this first time
        btb_new_pc = 32'h00000000; //PC where branch occurs
        btb_data = 32'hFEEDBEEF; // Hex value to be stored in data section
        
        #30;

        // Test 2: Changing the predictors at the spots
        // Should just change the predictor to 2
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b0; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        //btb_data = 32'h00000000; // Hex value to be stored in data section
        #10;
        // Should just change the predictor to 0
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b0; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        //btb_data = 32'h00000000; // Hex value to be stored in data section
        #10;
        // Should just change the predictor to 0 still
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b0; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        //btb_data = 32'h00000000; // Hex value to be stored in data section
        #10;
        // Should just change the predictor to 1
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b1; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        //btb_data = 32'h00000000; // Hex value to be stored in data section
        #10;
        // Should just change the predictor to 3
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b1; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        //btb_data = 32'h00000000; // Hex value to be stored in data section
        #10;
        // Should just change the predictor to 3 still
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b1; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        //btb_data = 32'h00000000; // Hex value to be stored in data section
        #10;
        // Should just change the predictor to 2
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b0; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        //btb_data = 32'h00000000; // Hex value to be stored in data section
        #10;
        // Should just change the predictor to 3
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b1; // Branch was taken this first time
        btb_new_pc = 32'h00000004; //PC where branch occurs
        //btb_data = 32'h00000000; // Hex value to be stored in data section
        #30;

        // Test 3: Replacing a spot with another branch that shares the same index
        // Replace 004 in the index
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b1; // Branch was taken this first time
        btb_new_pc = 32'h00001004; //PC where branch occurs
        btb_data = 32'hDEADFEED; // Hex value to be stored in data section
        #20;

        // Replace 000 in the index
        btb_write = 1'b1; // Enable writing
        btb_branch_taken = 1'b0; // Branch was not taken this first time
        btb_new_pc = 32'h00001000; //PC where branch occurs
        btb_data = 32'hDEADFEED; // Hex value to be stored in data section
        #20;

        btb_write = 1'b0; // Turn writing off

        #30; // Wait 50ns before moving onto reading tests


        // Test Reading from the BTB
        // Test 1: Read from the btb to see what target is stored
        btb_pc = 32'h00001004;
        #20
        btb_pc = 32'h00001000;
        #20;

        // Test 2: Read from the btb that is empty or has the incorrect value at the index
        btb_pc = 32'h00000004;
        #20
        btb_pc = 32'h00000000;
        #20;
 
    end

    // Initialization of DUT
    branch_target_buffer BTB(.btb_clk(btb_clk), .btb_reset(btb_reset), .btb_write(btb_write), .btb_branch_taken(btb_branch_taken), .btb_pc(btb_pc), .btb_new_pc(btb_new_pc), .btb_data(btb_data), .btb_valid_prediction(btb_valid_prediction), .btb_target(btb_target));

endmodule