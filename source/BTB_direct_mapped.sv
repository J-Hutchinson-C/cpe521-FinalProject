module BTB_direct_mapped #(parameter btb_number_entries = 1024);

typedef struct {
    logic [31:2] tag;
    logic [31:2] target;
    logic valid;
} btb_entry_t;

// Declare BTB memory
btb_entry_t btb[btb_number_entries];

//Read the BTB
fucntion logic [31:2] read_btb (input logic [31:2] pc);
    logic [31:2] target;
    int index = pc % btb_number_entries;

    if (btb[index].valid && btb[index].tag == pc)
        target = btb[index].target;
    
    return target;
endfunction