module branch_target_buffer #(parameter btb_number_entries = 1024)(
    input btb_clk,
    input btb_reset,
    input btb_write, // allow data to be written
    input btb_branch_taken, // Tells if branch was taken so write can be updated

    input [31:0] btb_pc, // program counter in fetch
    input [31:0] btb_new_pc, // program counter in decode/execute stage to be added
    input [31:0] btb_data, // memory


    output btb_valid_prediction,
    output logic [31:0] btb_target // PC branch will jump to

    );

    // BTB
    typedef struct {
        logic [21:0] tag; // 32bit pc - 10bit index = 22bit tag
        logic [31:0] data; // The branch address
        logic [1:0] predictor;
        logic valid;
    } btb_entry_t;
    
    
    // Index variables used for the BTB
    logic [9:0] btb_index_read; // Get index which is first 10 bits of pc for fetch stage
    logic [9:0] btb_index_write; // Get index which is first 10 bits of pc for decode/execute stage

    // Variable for the output of the FSM
    logic [1:0] btb_fsm_predictor;

    // Declare BTB memory
    btb_entry_t btb[btb_number_entries];
    
    /////////////////////////////////////////////////////////
    // Initializing the BTB FSM
    // Need to check, prob some issues caused by the fact that input and output exist for the same variable
    branch_target_buffer_FSM BTB_FSM(.btb_fsm_clk(btb_clk), .btb_fsm_branch_taken(btb_branch_taken), .btb_fsm_current_prediction(btb[btb_index_write].predictor), .btb_fsm_new_prediction(btb_fsm_predictor));
    /////////////////////////////////////////////////////////


    //Initialize the BTB
    initial begin
        for (int i = 0; i < btb_number_entries; i++) begin
            btb[i].valid <= 0;
        end
    end

    always_ff @(posedge btb_clk) begin
        btb_index_read <= btb_pc[9:0]; // Get index which is first 10 bits of pc
        btb_index_write <= btb_new_pc[9:0]; // Get index which is first 10 bits of pc


        if (btb_reset) begin
            // Reset the BTB entries
            for (int i = 0; i < btb_number_entries; i++) begin
            btb[i].valid <= 0;
            end
        end

        // Fetch Stage
        // Read from btb
        // Does the branch pc already exist in the btb
        if ((btb[btb_index_read].valid) && (btb[btb_index_read].tag == btb_pc[31:10])) begin
            // Read Hit: Use the stored target address from BTB
            btb_target <= btb[btb_index_read].data;
            

        end
        else begin
            //Read Miss: Output the next PC value so pc = pc+4
            btb_target <= 0; // CHANGE THIS LATER just for testing

        end

        // Decode/Execute Stage
        // Write to btb so add new branch data
        if (btb_write) begin
            // if branch is put into completely new spot
            if (btb[btb_index_write].valid == 1'b0) begin
                btb[btb_index_write].tag <= btb_new_pc[31:10];
                btb[btb_index_write].data <= btb_data;
                // If branch was taken the set to taken strong
                if (btb_branch_taken) begin
                    btb[btb_index_write].predictor <= 3; // Initially set to "Taken Strong"
                end
                // else branch was was not taken so set to not taken strong
                else begin
                    btb[btb_index_write].predictor <= 0; // Initially set to "Not Taken Strong"
                end
                btb[btb_index_write].valid <= 1'b1;
            end

            // if branch is just updating the branch
            else if (btb[btb_index_write].tag == btb_new_pc[31:10]) begin
                // update the predictor from the FSM module output
                btb[btb_index_write].predictor <= btb_fsm_predictor;
            end

            // if branch is replacing a spot that contains an already existing branch
            else begin
                btb[btb_index_write].tag <= btb_new_pc[31:10];
                btb[btb_index_write].data <= btb_data;
                btb[btb_index_write].predictor <= 1; // Initially set to "Not Taken Weak"
                btb[btb_index_write].valid <= 1'b1;
            end
        end
    end
endmodule