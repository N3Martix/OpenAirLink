//
// Copyright 2024 Ettus Research, a National Instruments Brand
//
// SPDX-License-Identifier: LGPL-3.0-or-later
//
// Module: rfnoc_block_shiftright
//
// Description:
//
//   The Shiftright Block performs bit shift right operation
//
// Parameters:
//
//   THIS_PORTID : Control crossbar port to which this block is connected
//   CHDR_W      : AXIS-CHDR data bus width
//   MTU         : Maximum transmission unit (i.e., maximum packet size in
//                 CHDR words is 2**MTU).
//

`default_nettype none


module rfnoc_block_shiftright #(
  parameter [9:0] THIS_PORTID     = 10'd0,
  parameter       CHDR_W          = 64,
  parameter [5:0] MTU             = 10
)(
  // RFNoC Framework Clocks and Resets
  input  wire                   rfnoc_chdr_clk,
  input  wire                   rfnoc_ctrl_clk,
  input  wire                   ce_clk,
  // RFNoC Backend Interface
  input  wire [511:0]           rfnoc_core_config,
  output wire [511:0]           rfnoc_core_status,
  // AXIS-CHDR Input Ports (from framework)
  input  wire [(1)*CHDR_W-1:0] s_rfnoc_chdr_tdata,
  input  wire [(1)-1:0]        s_rfnoc_chdr_tlast,
  input  wire [(1)-1:0]        s_rfnoc_chdr_tvalid,
  output wire [(1)-1:0]        s_rfnoc_chdr_tready,
  // AXIS-CHDR Output Ports (to framework)
  output wire [(1)*CHDR_W-1:0] m_rfnoc_chdr_tdata,
  output wire [(1)-1:0]        m_rfnoc_chdr_tlast,
  output wire [(1)-1:0]        m_rfnoc_chdr_tvalid,
  input  wire [(1)-1:0]        m_rfnoc_chdr_tready,
  // AXIS-Ctrl Input Port (from framework)
  input  wire [31:0]            s_rfnoc_ctrl_tdata,
  input  wire                   s_rfnoc_ctrl_tlast,
  input  wire                   s_rfnoc_ctrl_tvalid,
  output wire                   s_rfnoc_ctrl_tready,
  // AXIS-Ctrl Output Port (to framework)
  output wire [31:0]            m_rfnoc_ctrl_tdata,
  output wire                   m_rfnoc_ctrl_tlast,
  output wire                   m_rfnoc_ctrl_tvalid,
  input  wire                   m_rfnoc_ctrl_tready
);

  `define RFNOC_CHDR_UTILS_PATH `"`UHD_FPGA_DIR/usrp3/lib/rfnoc/core/rfnoc_chdr_utils.vh`"
  `include `RFNOC_CHDR_UTILS_PATH

  //---------------------------------------------------------------------------
  // Signal Declarations
  //---------------------------------------------------------------------------

  // Clocks and Resets
  wire               ctrlport_clk;
  wire               ctrlport_rst;
  wire               axis_data_clk;
  wire               axis_data_rst;
  // CtrlPort Master
  wire               m_ctrlport_req_wr;
  wire               m_ctrlport_req_rd;
  wire [19:0]        m_ctrlport_req_addr;
  wire [31:0]        m_ctrlport_req_data;
  reg                m_ctrlport_resp_ack;
  reg  [31:0]        m_ctrlport_resp_data;
  // Payload Stream to User Logic: in
  wire [32*1-1:0]    m_in_payload_tdata;
  wire [1-1:0]       m_in_payload_tkeep;
  wire               m_in_payload_tlast;
  wire               m_in_payload_tvalid;
  wire               m_in_payload_tready;
  // Context Stream to User Logic: in
  wire [CHDR_W-1:0]  m_in_context_tdata;
  wire [3:0]         m_in_context_tuser;
  wire               m_in_context_tlast;
  wire               m_in_context_tvalid;
  wire               m_in_context_tready;
  // Payload Stream from User Logic: out
  wire [32*1-1:0]    s_out_payload_tdata;
  wire [0:0]         s_out_payload_tkeep;
  wire               s_out_payload_tlast;
  wire               s_out_payload_tvalid;
  wire               s_out_payload_tready;
  // Context Stream from User Logic: out
  wire [CHDR_W-1:0]  s_out_context_tdata;
  wire [3:0]         s_out_context_tuser;
  wire               s_out_context_tlast;
  wire               s_out_context_tvalid;
  wire               s_out_context_tready;

  //---------------------------------------------------------------------------
  // NoC Shell
  //---------------------------------------------------------------------------

  noc_shell_shiftright #(
    .CHDR_W              (CHDR_W),
    .THIS_PORTID         (THIS_PORTID),
    .MTU                 (MTU)
  ) noc_shell_shiftright_i (
    //---------------------
    // Framework Interface
    //---------------------

    // Clock Inputs
    .rfnoc_chdr_clk      (rfnoc_chdr_clk),
    .rfnoc_ctrl_clk      (rfnoc_ctrl_clk),
    .ce_clk              (ce_clk),
    // Reset Outputs
    .rfnoc_chdr_rst      (),
    .rfnoc_ctrl_rst      (),
    .ce_rst              (),
    // RFNoC Backend Interface
    .rfnoc_core_config   (rfnoc_core_config),
    .rfnoc_core_status   (rfnoc_core_status),
    // CHDR Input Ports  (from framework)
    .s_rfnoc_chdr_tdata  (s_rfnoc_chdr_tdata),
    .s_rfnoc_chdr_tlast  (s_rfnoc_chdr_tlast),
    .s_rfnoc_chdr_tvalid (s_rfnoc_chdr_tvalid),
    .s_rfnoc_chdr_tready (s_rfnoc_chdr_tready),
    // CHDR Output Ports (to framework)
    .m_rfnoc_chdr_tdata  (m_rfnoc_chdr_tdata),
    .m_rfnoc_chdr_tlast  (m_rfnoc_chdr_tlast),
    .m_rfnoc_chdr_tvalid (m_rfnoc_chdr_tvalid),
    .m_rfnoc_chdr_tready (m_rfnoc_chdr_tready),
    // AXIS-Ctrl Input Port (from framework)
    .s_rfnoc_ctrl_tdata  (s_rfnoc_ctrl_tdata),
    .s_rfnoc_ctrl_tlast  (s_rfnoc_ctrl_tlast),
    .s_rfnoc_ctrl_tvalid (s_rfnoc_ctrl_tvalid),
    .s_rfnoc_ctrl_tready (s_rfnoc_ctrl_tready),
    // AXIS-Ctrl Output Port (to framework)
    .m_rfnoc_ctrl_tdata  (m_rfnoc_ctrl_tdata),
    .m_rfnoc_ctrl_tlast  (m_rfnoc_ctrl_tlast),
    .m_rfnoc_ctrl_tvalid (m_rfnoc_ctrl_tvalid),
    .m_rfnoc_ctrl_tready (m_rfnoc_ctrl_tready),

    //---------------------
    // Client Interface
    //---------------------

    // CtrlPort Clock and Reset
    .ctrlport_clk              (ctrlport_clk),
    .ctrlport_rst              (ctrlport_rst),
    // CtrlPort Master
    .m_ctrlport_req_wr         (m_ctrlport_req_wr),
    .m_ctrlport_req_rd         (m_ctrlport_req_rd),
    .m_ctrlport_req_addr       (m_ctrlport_req_addr),
    .m_ctrlport_req_data       (m_ctrlport_req_data),
    .m_ctrlport_resp_ack       (m_ctrlport_resp_ack),
    .m_ctrlport_resp_data      (m_ctrlport_resp_data),

    // AXI-Stream Payload Context Clock and Reset
    .axis_data_clk (axis_data_clk),
    .axis_data_rst (axis_data_rst),
    // Payload Stream to User Logic: in
    .m_in_payload_tdata  (m_in_payload_tdata),
    .m_in_payload_tkeep  (m_in_payload_tkeep),
    .m_in_payload_tlast  (m_in_payload_tlast),
    .m_in_payload_tvalid (m_in_payload_tvalid),
    .m_in_payload_tready (m_in_payload_tready),
    // Context Stream to User Logic: in
    .m_in_context_tdata  (m_in_context_tdata),
    .m_in_context_tuser  (m_in_context_tuser),
    .m_in_context_tlast  (m_in_context_tlast),
    .m_in_context_tvalid (m_in_context_tvalid),
    .m_in_context_tready (m_in_context_tready),
    // Payload Stream from User Logic: out
    .s_out_payload_tdata  (s_out_payload_tdata),
    .s_out_payload_tkeep  (s_out_payload_tkeep),
    .s_out_payload_tlast  (s_out_payload_tlast),
    .s_out_payload_tvalid (s_out_payload_tvalid),
    .s_out_payload_tready (s_out_payload_tready),
    // Context Stream from User Logic: out
    .s_out_context_tdata  (s_out_context_tdata),
    .s_out_context_tuser  (s_out_context_tuser),
    .s_out_context_tlast  (s_out_context_tlast),
    .s_out_context_tvalid (s_out_context_tvalid),
    .s_out_context_tready (s_out_context_tready)
  );

  //---------------------------------------------------------------------------
  // User Registers
  //---------------------------------------------------------------------------
  //
  // There's only one register now, but we'll structure the register code to
  // make it easier to add more registers later.
  // Register use the ctrlport_clk clock.
  //
  //---------------------------------------------------------------------------

  localparam REG_SHIFT_ADDR    = 0; // Address shift right register
  localparam REG_SHIFT_DEFAULT = 0; // Default shift right value

  reg signed [15:0] reg_shift = REG_SHIFT_DEFAULT;

  always @(posedge ctrlport_clk) begin
    if (ctrlport_rst) begin
      reg_shift = REG_SHIFT_DEFAULT;
    end else begin
      // Default assignment
      m_ctrlport_resp_ack <= 0;

      // Read user register
      if (m_ctrlport_req_rd) begin // Read request
        case (m_ctrlport_req_addr)
          REG_SHIFT_ADDR: begin
            m_ctrlport_resp_ack  <= 1;
            m_ctrlport_resp_data <= { 16'b0, reg_shift };
          end
        endcase
      end

      // Write user register
      if (m_ctrlport_req_wr) begin // Write requst
        case (m_ctrlport_req_addr)
          REG_SHIFT_ADDR: begin
            m_ctrlport_resp_ack <= 1;
            reg_shift           <= m_ctrlport_req_data[15:0];
          end
        endcase
      end
    end
  end

  //---------------------------------------------------------------------------
  // User Logic
  //---------------------------------------------------------------------------
  //
  // User logic uses the axis_data_clk clock. While the registers above use the
  // ctrlport_clk clock, in the block YAML configuration file both the control
  // and data interfaces are specified to use the rfnoc_chdr clock. Therefore,
  // we do not need to cross clock domains when using user registers with
  // user logic.
  //
  //---------------------------------------------------------------------------



  wire [31:0] pipe_in_tdata;
  wire pipe_in_tvalid, pipe_in_tlast;
  wire pipe_in_tready;

  wire [31:0] pipe_out_tdata;
  wire pipe_out_tvalid, pipe_out_tlast;
  wire pipe_out_tready;

  // Adding FIFO to ensure Pipeline
  axi_fifo #(
    .WIDTH (32+1),
    .SIZE  (0)
  )
  pipeline0_axi_fifo (
    .clk      (ce_clk),
    .reset    (0),
    .clear    (0),
    .i_tdata  ({m_in_payload_tlast, m_in_payload_tdata}),
    .i_tvalid (m_in_payload_tvalid),
    .i_tready (m_in_payload_tready),
    .o_tdata  ({pipe_in_tlast, pipe_in_tdata}),
    .o_tvalid (pipe_in_tvalid),
    .o_tready (pipe_in_tready)
  );

  wire signed [15:0] shift_bits = reg_shift[15:0];
  wire signed [15:0] i = pipe_in_tdata[31:16];
  wire signed [15:0] q = pipe_in_tdata[15:0];

  wire signed [31:0] i_sr = i >>> shift_bits;
  wire signed [31:0] q_sr = q >>> shift_bits;

  wire signed [31:0] sr_data = {i_sr[15:0], q_sr[15:0]};

  axi_fifo #(
    .WIDTH (32+1),
    .SIZE  (0)
  )
  pipeline1_axi_fifo (
    .clk(ce_clk),
    .reset    (0),
    .clear    (0),
    .i_tdata  ({pipe_in_tlast, sr_data}),
    .i_tvalid (pipe_in_tvalid),
    .i_tready (pipe_in_tready),
    .o_tdata  ({pipe_out_tlast, pipe_out_tdata}),
    .o_tvalid (pipe_out_tvalid),
    .o_tready (pipe_out_tready)
  );


  // Sample data, pass through unchanged
  assign s_out_payload_tdata  = pipe_out_tdata;
  assign s_out_payload_tlast  = pipe_out_tlast;
  assign s_out_payload_tvalid = pipe_out_tvalid;
  assign pipe_out_tready      = s_out_payload_tready;

  // Context data, we are not doing anything with the context
  // (the CHDR header info) so we can simply pass through unchanged
  assign s_out_context_tdata  = m_in_context_tdata;
  assign s_out_context_tuser  = m_in_context_tuser;
  assign s_out_context_tlast  = m_in_context_tlast;
  assign s_out_context_tvalid = m_in_context_tvalid;
  assign m_in_context_tready  = s_out_context_tready;

  // Only 1-sample per clock, so tkeep should always be asserted
  assign s_out_payload_tkeep = 1'b1;

endmodule // rfnoc_block_shiftright


`default_nettype wire
