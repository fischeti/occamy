// Copyright 2020 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

`include "axi/typedef.svh"

module testharness import occamy_pkg::*; (
  input  logic        clk_i,
  input  logic        rst_ni
);

  // verilog_lint: waive explicit-parameter-storage-type
  localparam RTCTCK = 30.518us; // 32.768 kHz

  logic rtc_i;

  // Generate reset and clock.
  initial begin
    forever begin
      rtc_i = 1;
      #(RTCTCK/2);
      rtc_i = 0;
      #(RTCTCK/2);
    end
  end

  logic clk_periph_i, rst_periph_ni;
  assign clk_periph_i = clk_i;
  assign rst_periph_ni = rst_ni;

<%def name="tb_dramsys_ch(bus, name, base_addr)">
  ${bus.req_type()} ${name}_req;
  ${bus.rsp_type()} ${name}_rsp;

  axi_dram_sim #(
    .AxiAddrWidth( ${bus.aw}  ),
    .AxiDataWidth( ${bus.dw}  ),
    .AxiIdWidth  ( ${bus.iw}           ),
    .AxiUserWidth( ${bus.uw + 1}         ),
    .BASE        ( 64'h${format(base_addr, '012x')}),
    .DRAMType    ("HBM2"                ),
    .axi_req_t   ( ${bus.req_type()}    ),
    .axi_resp_t  ( ${bus.rsp_type()}    ),
    .axi_ar_t    ( ${bus.ar_chan_type()} ),
    .axi_r_t     ( ${bus.r_chan_type()}  ),
    .axi_aw_t    ( ${bus.aw_chan_type()} ),
    .axi_w_t     ( ${bus.w_chan_type()}  ),
    .axi_b_t     ( ${bus.b_chan_type()}  )
  ) i_${name}_channel (
    .clk_i,
    .rst_ni,
    .axi_req_i  ( ${name}_req  ),
    .axi_resp_o ( ${name}_rsp  )
  );
</%def>

<%def name="tb_axi_lite_mem(bus, name, fr_name)">
  `AXI_TYPEDEF_ALL_CT(${"axi_{}, axi_{}_req_t, axi_{}_resp_t,".format(name, name, name)}
    ${"logic [{}:0], logic [{}:0], logic [{}:0], logic [{}:0], logic [{}:0])\n".format(bus.aw-1, 0, bus.dw-1, bus.dw//8-1, 0)}

  axi_${name}_req_t axi_${name}_req;
  axi_${name}_resp_t axi_${name}_resp;

  axi_lite_to_axi #(
      .AxiDataWidth(${bus.dw}),
      .req_lite_t  (${bus.req_type()}),
      .resp_lite_t (${bus.rsp_type()}),
      .axi_req_t   (axi_${name}_req_t),
      .axi_resp_t  (axi_${name}_resp_t)
  ) i_axil2axi_${name}_pc (
      .slv_req_lite_i (${fr_name}_req),
      .slv_resp_lite_o(${fr_name}_rsp),
      .slv_aw_cache_i (axi_pkg::CACHE_MODIFIABLE),
      .slv_ar_cache_i (axi_pkg::CACHE_MODIFIABLE),
      .mst_req_o      (axi_${name}_req),
      .mst_resp_i     (axi_${name}_resp)
  );

  axi_sim_mem #(
    .AddrWidth (${bus.aw}),
    .DataWidth (${bus.dw}),
    .IdWidth (1),
    .UserWidth (1),
    .axi_req_t (axi_${name}_req_t),
    .axi_rsp_t (axi_${name}_resp_t)
  ) i_${name}_sim_mem (
    .clk_i,
    .rst_ni,
    .axi_req_i (axi_${name}_req),
    .axi_rsp_o (axi_${name}_rsp),
    .mon_w_valid_o (),
    .mon_w_addr_o (),
    .mon_w_data_o (),
    .mon_w_id_o (),
    .mon_w_user_o (),
    .mon_w_beat_count_o (),
    .mon_w_last_o (),
    .mon_r_valid_o (),
    .mon_r_addr_o (),
    .mon_r_data_o (),
    .mon_r_id_o (),
    .mon_r_user_o (),
    .mon_r_beat_count_o (),
    .mon_r_last_o ()
  );
</%def>

% for i in range(nr_hbm_channels):
  ${tb_dramsys_ch(hbm_xbar.__dict__["out_hbm_{}".format(i)], "hbm_channel_{}".format(i), 0x80000000 + i * 0x40000000)}
% endfor

  logic tx, rx;
  axi_lite_a48_d32_req_t axi_lite_bootrom_req;
  axi_lite_a48_d32_req_t axi_lite_fll_system_req;
  axi_lite_a48_d32_req_t axi_lite_fll_periph_req;
  axi_lite_a48_d32_req_t axi_lite_fll_hbm2e_req;

  axi_lite_a48_d32_rsp_t axi_lite_bootrom_rsp;
  axi_lite_a48_d32_rsp_t axi_lite_fll_system_rsp;
  axi_lite_a48_d32_rsp_t axi_lite_fll_periph_rsp;
  axi_lite_a48_d32_rsp_t axi_lite_fll_hbm2e_rsp;
  ${tb_axi_lite_mem(soc_axi_lite_narrow_periph_xbar.out_bootrom, "bootrom", "axi_lite_bootrom")}
  ${tb_axi_lite_mem(soc_axi_lite_narrow_periph_xbar.out_fll_system, "fll_system", "axi_lite_fll_system")}
  ${tb_axi_lite_mem(soc_axi_lite_narrow_periph_xbar.out_fll_periph, "fll_periph", "axi_lite_fll_periph")}
  ${tb_axi_lite_mem(soc_axi_lite_narrow_periph_xbar.out_fll_hbm2e, "fll_hbm2e", "axi_lite_fll_hbm2e")}
  occamy_top i_occamy (
    .clk_i,
    .rst_ni,
    .sram_cfgs_i ('0),
    .clk_periph_i,
    .rst_periph_ni,
    .rtc_i,
    .test_mode_i (1'b0),
    .chip_id_i ('0),
    .boot_mode_i ('0),
    .uart_tx_o (tx),
    .uart_rx_i (rx),
    .gpio_d_i ('0),
    .gpio_d_o (),
    .gpio_oe_o (),
    .jtag_trst_ni ('0),
    .jtag_tck_i ('0),
    .jtag_tms_i ('0),
    .jtag_tdi_i ('0),
    .jtag_tdo_o (),
    .i2c_sda_o (),
    .i2c_sda_i ('0),
    .i2c_sda_en_o (),
    .i2c_scl_o (),
    .i2c_scl_i ('0),
    .i2c_scl_en_o (),
    .spim_sck_o (),
    .spim_sck_en_o (),
    .spim_csb_o (),
    .spim_csb_en_o (),
    .spim_sd_o (),
    .spim_sd_en_o (),
    .spim_sd_i ('0),
    .bootrom_req_o (axi_lite_bootrom_req),
    .bootrom_rsp_i (axi_lite_bootrom_rsp),
    .fll_system_req_o (axi_lite_fll_system_req),
    .fll_system_rsp_i (axi_lite_fll_system_rsp),
    .fll_periph_req_o (axi_lite_fll_periph_req),
    .fll_periph_rsp_i (axi_lite_fll_periph_rsp),
    .fll_hbm2e_req_o (axi_lite_fll_hbm2e_req),
    .fll_hbm2e_rsp_i (axi_lite_fll_hbm2e_rsp),
    .hbi_wide_cfg_req_o (),
    .hbi_wide_cfg_rsp_i ('0),
    .hbi_narrow_cfg_req_o (),
    .hbi_narrow_cfg_rsp_i ('0),
    .hbm_cfg_req_o (),
    .hbm_cfg_rsp_i ('0),
    .pcie_cfg_req_o (),
    .pcie_cfg_rsp_i ('0),
    .chip_ctrl_req_o (),
    .chip_ctrl_rsp_i ('0),
    .ext_irq_i ('0),
% for i in range(nr_hbm_channels):
    .hbm_${i}_req_o (hbm_channel_${i}_req),
    .hbm_${i}_rsp_i (hbm_channel_${i}_rsp),
% endfor
% for s in ("wide", "narrow"):
    .hbi_${s}_req_i ('0),
    .hbi_${s}_rsp_o (),
    .hbi_${s}_req_o (),
    .hbi_${s}_rsp_i ('0),
% endfor
    .pcie_axi_req_o (),
    .pcie_axi_rsp_i ('0),
    .pcie_axi_req_i ('0),
    .pcie_axi_rsp_o ()
  );

  // Must be the frequency of i_uart0.clk_i in Hz
  localparam int unsigned UartDPIFreq = 1_000_000_000;

  uartdpi #(
    .BAUD ('d115_200),
    // Frequency shouldn't matter since we are sending with the same clock.
    .FREQ (UartDPIFreq),
    .NAME("uart0")
  ) i_uart0 (
    .clk_i (clk_i),
    .rst_ni (rst_ni),
    .tx_o (rx),
    .rx_i (tx)
  );

endmodule
