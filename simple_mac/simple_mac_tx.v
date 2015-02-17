`timescale 1ns / 1ps

/* It looks like a generic timer could become useful, so let's make one.
 * (Ok, it's not so useful: I just wanted to make a timer.)
 */

module timer #(
    parameter INTERVAL = 10,
    parameter DURATION = 1
    ) (
    input wire clk,
    input wire rst,
    input wire ena,
    output wire irq
    );
    
    function integer log2;
        input integer number;
        for (log2 = 0; number > 0; log2 = log2 + 1)
            number = number >> 1;
    endfunction
    
    localparam INTERVAL_WIDTH = log2(INTERVAL+1); // Why have I got +1 here?
    localparam DURATION_WIDTH = log2(DURATION+1); // Why have I got +1 here?
    
    reg [INTERVAL_WIDTH-1:0] interval_count = 0;
    reg [DURATION_WIDTH-1:0] duration_count = 0;
    
    assign irq = |duration_count;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            interval_count <= 0;
            duration_count <= 0;
        end else begin
            if (ena) begin
                if (interval_count == INTERVAL) begin
                    if (duration_count == DURATION-1) begin
                        interval_count <= 0;
                    end
                    duration_count <= duration_count + 1;
                end else begin
                    interval_count <= interval_count + 1;
                    duration_count <= 0;
                end
            end else begin
                interval_count <= 0;
                duration_count <= 0;
            end
        end
    end
endmodule

/* The PLS is responsible for converting the bit stream from the MAC to
 * the physical layer signal.
 *
 * When the MAC is transmitting the PLS encodes the data onto the wires
 * using Manchester encoding and 2.5 volt differential signalling.
 *
 * When it is not transmitting, the PLS is responsible for sending an IDL
 * signal. IDL starts with the end of transmission delimiter (ETD), which
 * is two bit-times of the differential signal asserted. That is followed
 * by a repeating pattern of 16ms of silence and the link test pulse. IDL
 * can be interrupted any time.
 * 
 * The PLS as described in 802.3 emits an output_next signal to request
 * the next bit. This is essentially a 10 MHz clock enable that is only
 * generated when data_complete is deasserted. This could be used drive a
 * rd_en on a FIFO. But in my case I want to use a simple shift register.
 * It adds complexity to synchronise the output of a clock enable driven
 * shift register and the PLS if the output_next signal is used to derive
 * the clock enable. So instead I will create use a 10 MHz synchronised
 * clock enable for driving both the shift register and the PLS. The FIFO
 * would be cool though and I may implement that later.
 */

module pls_tx (
    input wire clk,
    input wire rst,
    input wire ce,
    input wire data_complete,
    input wire output_unit,
    output wire td_p,
    output wire td_n
    );
    
    /* Timer for generating IDL pattern. */
    
    wire link_test_pulse;
    
    timer #(
        .INTERVAL(320000),
        .DURATION(2)
    ) idl_timer (
        .clk(clk),
        .rst(rst),
        .ena(data_complete),
        .irq(link_test_pulse)
    );
    
    /* Manchester encoded differential signalling. */
    
    reg td;
    
    always @(*) begin
        case ({data_complete, ce})
            2'b00: td = ~output_unit;
            2'b01: td = output_unit;
            default: td = 1;
        endcase
    end
    
    reg [3:0] efd = 0; // end of frame delimiter
    
    assign td_p = (~data_complete | link_test_pulse | (|efd)) ? td : 0;
    assign td_n = (~data_complete | link_test_pulse | (|efd)) ? ~td : 0;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            efd <= 0;
        end else begin
            if (data_complete) begin
                efd <= efd >> 1;
            end else begin
                efd <= 4'b1111;
            end
        end
    end
endmodule

/* Create a simple shift register with a clock enable to serialize data
 * to the PLS. Data presented on the in signal is automatically shifted
 * in on the next cycle when empty. The almost_empty signal can be used
 * to feed new data.
 */

module shift_register (
    input wire clk,
    input wire rst,
    input wire ce,
    input wire [7:0] in,
    output wire out,
    output wire empty,
    output wire almost_empty
    );
    
    reg [7:0] sreg = 0;
    reg [2:0] scnt = 0;
    assign out = sreg[0];
    assign empty = (ce & scnt == 0);
    assign almost_empty = (ce & scnt == 1);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
             sreg <= 0;
             scnt <= 0;
        end else begin
            if (ce) begin
                if (scnt == 0) begin
                    sreg <= in;
                    scnt <= 7;
                end else begin
                    sreg <= sreg >> 1;
                    scnt <= scnt - 1;
                end
            end
        end
    end
endmodule

/* We'll need to generate the CRC32 for the frame check sequence.
 */

module crc32 #(
    parameter MSB_FIRST = 1
    ) (
    input wire clk,
    input wire rst,
    input wire ena,
    input wire [7:0] data,
    input wire dvld,
    output wire [31:0] crc
    );
    
    reg [31:0] q = 0;
    reg [31:0] c = 0;
    
    wire [7:0] d;
    assign d = (MSB_FIRST) ? data : {data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]};
    assign crc = q;
    
    always @(*) begin
        c[0] = d[0] ^ d[6] ^ q[24] ^ q[30];
        c[1] = d[0] ^ d[1] ^ d[7] ^ q[25] ^ q[31] ^ d[6] ^ q[24] ^ q[30];
        c[2] = d[0] ^ d[1] ^ d[2] ^ q[26] ^ d[7] ^ q[25] ^ q[31] ^ d[6] ^ q[24] ^ q[30];
        c[3] = d[1] ^ d[2] ^ d[3] ^ q[27] ^ q[26] ^ d[7] ^ q[25] ^ q[31];
        c[4] = d[0] ^ d[2] ^ d[3] ^ d[4] ^ q[28] ^ q[27] ^ q[26] ^ d[6] ^ q[24] ^ q[30];
        c[5] = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[5] ^ q[29] ^ q[28] ^ q[27] ^ d[7] ^ q[25] ^ q[31] ^ d[6] ^ q[24] ^ q[30];
        c[6] = d[1] ^ d[2] ^ d[4] ^ d[5] ^ d[6] ^ q[30] ^ q[29] ^ q[28] ^ q[26] ^ d[7] ^ q[25] ^ q[31];
        c[7] = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[7] ^ q[31] ^ q[29] ^ q[27] ^ q[26] ^ q[24];
        c[8] = d[0] ^ d[1] ^ d[3] ^ d[4] ^ q[0] ^ q[28] ^ q[27] ^ q[25] ^ q[24];
        c[9] = d[1] ^ d[2] ^ d[4] ^ d[5] ^ q[1] ^ q[29] ^ q[28] ^ q[26] ^ q[25];
        c[10] = d[0] ^ d[2] ^ d[3] ^ d[5] ^ q[2] ^ q[29] ^ q[27] ^ q[26] ^ q[24];
        c[11] = d[0] ^ d[1] ^ d[3] ^ d[4] ^ q[3] ^ q[28] ^ q[27] ^ q[25] ^ q[24];
        c[12] = d[0] ^ d[1] ^ d[2] ^ d[4] ^ d[5] ^ q[4] ^ q[29] ^ q[28] ^ q[26] ^ q[25] ^ d[6] ^ q[24] ^ q[30];
        c[13] = d[1] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ q[5] ^ q[30] ^ q[29] ^ q[27] ^ q[26] ^ d[7] ^ q[25] ^ q[31];
        c[14] = d[2] ^ d[3] ^ d[4] ^ d[6] ^ d[7] ^ q[6] ^ q[31] ^ q[30] ^ q[28] ^ q[27] ^ q[26];
        c[15] = d[3] ^ d[4] ^ d[5] ^ d[7] ^ q[7] ^ q[31] ^ q[29] ^ q[28] ^ q[27];
        c[16] = d[0] ^ d[4] ^ d[5] ^ q[8] ^ q[29] ^ q[28] ^ q[24];
        c[17] = d[1] ^ d[5] ^ d[6] ^ q[9] ^ q[30] ^ q[29] ^ q[25];
        c[18] = d[2] ^ d[6] ^ d[7] ^ q[10] ^ q[31] ^ q[30] ^ q[26];
        c[19] = d[3] ^ d[7] ^ q[11] ^ q[31] ^ q[27];
        c[20] = d[4] ^ q[12] ^ q[28];
        c[21] = d[5] ^ q[13] ^ q[29];
        c[22] = d[0] ^ q[14] ^ q[24];
        c[23] = d[0] ^ d[1] ^ q[15] ^ q[25] ^ d[6] ^ q[24] ^ q[30];
        c[24] = d[1] ^ d[2] ^ q[16] ^ q[26] ^ d[7] ^ q[25] ^ q[31];
        c[25] = d[2] ^ d[3] ^ q[17] ^ q[27] ^ q[26];
        c[26] = d[0] ^ d[3] ^ d[4] ^ q[18] ^ q[28] ^ q[27] ^ d[6] ^ q[24] ^ q[30];
        c[27] = d[1] ^ d[4] ^ d[5] ^ q[19] ^ q[29] ^ q[28] ^ d[7] ^ q[25] ^ q[31];
        c[28] = d[2] ^ d[5] ^ d[6] ^ q[20] ^ q[30] ^ q[29] ^ q[26];
        c[29] = d[3] ^ d[6] ^ d[7] ^ q[21] ^ q[31] ^ q[30] ^ q[27];
        c[30] = d[4] ^ d[7] ^ q[22] ^ q[31] ^ q[28];
        c[31] = d[5] ^ q[23] ^ q[29];
    end
    
    always @(posedge clk or posedge rst) begin
        if(rst) begin
            q <= {32{1'b1}};
        end else begin
            if (ena) begin
                q <= (dvld) ? c : q;
            end else begin
                q <= {32{1'b1}};
            end
        end
    end
endmodule

/* The transmit engine of the MAC. To transmit a frame present the first
 * word on data and assert dvld. The MAC will then transmit the preamble
 * and assert dack when it is ready for the next word. Then when dvld is
 * deasserted the MAC will write the FCS and stop transmitting. Clock is
 * 20 MHz. Output is AUI interface.
 */
 
module mac_tx (
    input wire clk,
    input wire rst,
    input wire [7:0] data,
    input wire dvld,
    output wire dack,
    output wire led,
    output wire td_p,
    output wire td_n
    );
    
    /* Generate a 10 MHz clock enable that will be used by the PLS and
     * its attached shift register. */
    
    reg ce = 0;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ce <= 0;
        end else begin
            ce <= ~ce;
        end
    end
    
    /* Instantiate the PLS. The PLS will provide us with link up by
     * virtue of the IDL pattern it produces. */
    
    wire data_complete;
    wire output_unit;
    
    pls_tx pls_tx (
        .clk(clk),
        .rst(rst),
        .ce(ce),
        .data_complete(data_complete),
        .output_unit(output_unit),
        .td_p(td_p),
        .td_n(td_n)
    );
    
    /* Serialize data to the PLS using a shift register. I think it would
     * be nice to replace this with a small 8:1 ratio FWFT FIFO. We could
     * then use the output_next signal from the PLS to drive rd_en on the
     * FIFO, the empty signal from the FIFO to drive data_complete on the
     * PLS, and the the full signal from the FIFO for driving the state
     * machine. This would probably make things simpler. */
    
    reg [7:0] output_data = 0;
    wire empty;
    wire almost_empty;
    
    shift_register shift_register (
        .clk(clk),
        .rst(rst),
        .ce(ce),
        .in(output_data),
        .out(output_unit),
        .empty(empty),
        .almost_empty(almost_empty)
    );
    
    /* CRC32 generator. */
    
    reg crc_ena = 0;
    wire [31:0] crc;
    
    crc32 #(
        .MSB_FIRST(0)
    ) crc32 (
        .clk(clk),
        .rst(rst),
        .ena(crc_ena),
        .data(output_data),
        .dvld(crc_dvld),
        .crc(crc)
    );
    
    /* State machine to wrap a preamble, any necessary padding, and FCS
     * around a frame. */
    
    localparam idling = 0, sending_preamble = 1, sending_sfd = 2,
        sending_payload = 3, sending_padding = 4, sending_fcs = 5, waiting_ipg = 6;
    
    reg transmitting = 0;
    reg [15:0] count = 0;
    reg [2:0] state = 0;
    reg accept_data = 0;
    reg [7:0] buffered_data = 0;
    
    assign data_complete = ~transmitting;
    assign led = transmitting;
    assign dack = (empty & dvld & accept_data);
    assign crc_dvld = (empty & (state == sending_payload || state == sending_padding));
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            transmitting <= 0;
            count <= 0;
            state <= waiting_ipg;
            crc_ena <= 0;
            accept_data <= 0;
        end else begin
            case (state)
                idling: begin
                    transmitting <= 0; // redundant
                    count <= 0;
                    
                    if (dvld) begin
                        state <= sending_preamble;
                    end
                end
                sending_preamble: begin
                    if (empty) begin
                        output_data <= 8'h55;
                        count <= count + 1;
                        
                        if (count == 1) begin // Delay transmit enable because output_data lags a cycle
                            transmitting <= 1;
                        end
                        
                        if (count == 6) begin
                            accept_data <= 1;
                            count <= 0;
                            state <= sending_sfd;
                        end
                    end
                end
                sending_sfd: begin
                    if (empty) begin
                        output_data <= 8'hD5;
                        count <= 0;
                        
                        if (dvld) begin
                            buffered_data <= data;
                            crc_ena <= 1;
                            state <= sending_payload;
                        end else begin
                            state <= waiting_ipg;
                        end
                    end
                end
                sending_payload: begin
                    if (empty) begin
                        output_data <= buffered_data;
                        count <= count + 1;
                        buffered_data <= data;
                        
                        if (~dvld | ~accept_data) begin  
                            if (count < 59) begin
                                state <= sending_padding;
                            end else begin
                                count <= 0;
                                state <= sending_fcs;
                            end
                        end
                        
                        if (count == 2000) begin
                            // Okay buddy, enough's enough.
                            accept_data <= 0;
                            count <= 0;
                            state <= sending_fcs;
                        end
                    end
                    
                    if (empty) begin
                        accept_data <= 0;
                    end
                end
                sending_padding: begin
                    if (empty) begin
                        output_data <= 8'h00;
                        count <= count + 1;
                        
                        if (count == 60) begin
                            count <= 0;
                            state <= sending_fcs;
                        end
                    end
                end
                sending_fcs: begin
                    if (empty) begin
                        case (count)
                            0: output_data <= crc[7:0];
                            1: output_data <= crc[15:8];
                            2: output_data <= crc[23:16];
                            3: output_data <= crc[31:24];
                        endcase
                        count <= count + 1;
                        
                        if (count == 3) begin
                            count <= 0;
                            state <= waiting_ipg;
                        end
                    end
                end
                waiting_ipg: begin
                    if (empty) begin
                        transmitting <= 0;
                        crc_ena <= 0;
                        count <= count + 1;
                        
                        if (count == 191) begin
                            count <= 0;
                            state <= idling;
                        end
                    end
                end
                default: state <= idling;
            endcase
        end
    end
endmodule

/* Generate the transmit clock, instantiate the MAC, load a packet and
 * send it to the MAC.
 */

module top (
    input wire mclk,
    output wire led,
    output wire td_p,
    output wire td_n
    );
    
    reg rst = 0;
    initial #30 rst = 0;
    
    /* Generate the 20 MHz transmit clock. We'll drive everything from
     * this transmit clock to keep it simple. */
    
    wire clkdv;
    wire mclk0;
    wire locked;
    wire clk;
    
    DCM_SP #(
        .CLKIN_PERIOD(20.0),
        .CLKDV_DIVIDE(2.5),
        .CLK_FEEDBACK("1X")
    ) DCM_SP_inst (
        .CLKIN(mclk),
        .RST(rst),
        .CLK0(mclk0),
        .CLKFB(mclk0),
        .CLKDV(clkdv),
        .LOCKED(locked)
    );
    
    BUFG BUFG_inst (.I(clkdv), .O(clk));
    
    /* Instantiate the MAC. */
    
    reg [7:0] data = 0;
    reg dvld = 0;
    wire dack;
    
    mac_tx mac_tx (
        .clk(clk),
        .rst(rst),
        .data(data),
        .dvld(dvld),
        .dack(dack),
        .led(led),
        .td_p(td_p),
        .td_n(td_n)
    );
    
    /* Load our packet. */
    
    localparam num_lines = 128;
    reg [7:0] rom [num_lines-1:0]; // Blows up if 58 bytes, wtf?
    initial $readmemh("simple_mac_pkt.txt", rom, 0, num_lines-1);
    
    /* Use a timer to transmit a packet every second. */
    
    wire start_tx;
    
    timer #(
        .INTERVAL(20000000-1),
        .DURATION(1)
    ) tx_timer (
        .clk(clk),
        .rst(rst),
        .ena(locked),
        .irq(start_tx)
    );
    
    /* Transmit packet. */
    
    localparam last_addr = 58;
    reg [7:0] addr = 0;
    
    always @(posedge clk) begin
        if (dvld) begin
            if (dack) begin
                if (addr == last_addr) begin
                    dvld <= 0;
                end else begin
                    data <= rom[addr];
                    addr <= addr + 1;
                end
            end
        end else begin
            if (start_tx) begin
                data <= rom[0];
                addr <= 1;
                dvld <= 1;
            end
        end
    end
endmodule

/*module testbench;
    reg clk = 1;
    always #10 clk <= ~clk;
    top top(.mclk(clk));
endmodule*/
