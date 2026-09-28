--------------------------------------------------------------------------------
-- AP040_PIPE - MC68040-style pipelined core (2026-09-25)
--
-- ap040_pipe_ram.vhd - the block RAMs of the instruction and data memory units
--
-- The caches' arrays (doc_AP040_PIPELINE_CACHES.md, "Storage"), instantiated
-- as altsyncram rather than inferred: inference into M10K is a judgement
-- Quartus renews on every compile, and its failure is silent -- the array
-- lands in logic cells and the fit stops closing (rtl/ap040/ap040_cache.v
-- says the same of the sequential core's caches). rtl/bram.vhd, shared with
-- the rest of the system, has no byte enables and no simple dual-port form;
-- these two do, and tests/ap040/sim_pipe_ram.v models them for Verilator.
--
-- ap040_pipe_tdpram_be  true dual-port, a byte enable per 8 bits on each port.
--                       Cyclone V M10K: 20 bits a port at most, so a 32-bit
--                       width is two blocks.
-- ap040_pipe_sdpram     simple dual-port: port A writes, port B reads. Up to
--                       40 bits a block.
--
-- Both: read address registered, output unregistered (the data is there
-- the cycle after the address, as rtl/bram.vhd's dpram). A read of an
-- address being written the same cycle, on either port, is undefined here
-- (DONT_CARE) and the model returns junk for it: the units never depend
-- on it.
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;

LIBRARY altera_mf;
USE altera_mf.altera_mf_components.all;

entity ap040_pipe_tdpram_be is
	generic (
		addr_width : integer := 8;
		data_width : integer := 32    -- a multiple of 8
	);
	port (
		clock     : in  std_logic;
		address_a : in  std_logic_vector(addr_width-1 downto 0);
		data_a    : in  std_logic_vector(data_width-1 downto 0);
		byteena_a : in  std_logic_vector(data_width/8-1 downto 0);
		wren_a    : in  std_logic;
		q_a       : out std_logic_vector(data_width-1 downto 0);
		address_b : in  std_logic_vector(addr_width-1 downto 0);
		data_b    : in  std_logic_vector(data_width-1 downto 0);
		byteena_b : in  std_logic_vector(data_width/8-1 downto 0);
		wren_b    : in  std_logic;
		q_b       : out std_logic_vector(data_width-1 downto 0)
	);
end entity;

architecture syn of ap040_pipe_tdpram_be is
begin
	ram : altsyncram
	generic map (
		address_reg_b                      => "CLOCK0",
		byteena_reg_b                      => "CLOCK0",
		byte_size                          => 8,
		clock_enable_input_a               => "BYPASS",
		clock_enable_input_b               => "BYPASS",
		clock_enable_output_a              => "BYPASS",
		clock_enable_output_b              => "BYPASS",
		indata_reg_b                       => "CLOCK0",
		intended_device_family             => "Cyclone V",
		lpm_type                           => "altsyncram",
		numwords_a                         => 2**addr_width,
		numwords_b                         => 2**addr_width,
		operation_mode                     => "BIDIR_DUAL_PORT",
		outdata_aclr_a                     => "NONE",
		outdata_aclr_b                     => "NONE",
		outdata_reg_a                      => "UNREGISTERED",
		outdata_reg_b                      => "UNREGISTERED",
		power_up_uninitialized             => "FALSE",
		ram_block_type                     => "M10K",
		read_during_write_mode_mixed_ports => "DONT_CARE",
		read_during_write_mode_port_a      => "DONT_CARE",
		read_during_write_mode_port_b      => "DONT_CARE",
		widthad_a                          => addr_width,
		widthad_b                          => addr_width,
		width_a                            => data_width,
		width_b                            => data_width,
		width_byteena_a                    => data_width/8,
		width_byteena_b                    => data_width/8,
		wrcontrol_wraddress_reg_b          => "CLOCK0"
	)
	port map (
		clock0    => clock,
		address_a => address_a,
		data_a    => data_a,
		byteena_a => byteena_a,
		wren_a    => wren_a,
		q_a       => q_a,
		address_b => address_b,
		data_b    => data_b,
		byteena_b => byteena_b,
		wren_b    => wren_b,
		q_b       => q_b
	);
end architecture;

LIBRARY ieee;
USE ieee.std_logic_1164.all;

LIBRARY altera_mf;
USE altera_mf.altera_mf_components.all;

entity ap040_pipe_sdpram is
	generic (
		addr_width : integer := 8;
		data_width : integer := 32
	);
	port (
		clock     : in  std_logic;
		wraddress : in  std_logic_vector(addr_width-1 downto 0);
		data      : in  std_logic_vector(data_width-1 downto 0);
		wren      : in  std_logic;
		rdaddress : in  std_logic_vector(addr_width-1 downto 0);
		q         : out std_logic_vector(data_width-1 downto 0)
	);
end entity;

architecture syn of ap040_pipe_sdpram is
begin
	ram : altsyncram
	generic map (
		address_aclr_b                     => "NONE",
		address_reg_b                      => "CLOCK0",
		clock_enable_input_a               => "BYPASS",
		clock_enable_input_b               => "BYPASS",
		clock_enable_output_b              => "BYPASS",
		intended_device_family             => "Cyclone V",
		lpm_type                           => "altsyncram",
		numwords_a                         => 2**addr_width,
		numwords_b                         => 2**addr_width,
		operation_mode                     => "DUAL_PORT",
		outdata_aclr_b                     => "NONE",
		outdata_reg_b                      => "UNREGISTERED",
		power_up_uninitialized             => "FALSE",
		ram_block_type                     => "M10K",
		read_during_write_mode_mixed_ports => "DONT_CARE",
		widthad_a                          => addr_width,
		widthad_b                          => addr_width,
		width_a                            => data_width,
		width_b                            => data_width,
		width_byteena_a                    => 1
	)
	port map (
		clock0    => clock,
		address_a => wraddress,
		data_a    => data,
		wren_a    => wren,
		address_b => rdaddress,
		q_b       => q
	);
end architecture;
