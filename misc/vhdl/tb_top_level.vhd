--------------------------------------------
-- Author: Mike Field <hamster@snap.net.nz>
--------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;
 
ENTITY tb_top_level IS
END tb_top_level;
 
ARCHITECTURE behavior OF tb_top_level IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT top_level
    PORT(
         mclk : IN  std_logic;
         gps_mag : IN  std_logic;
         gps_sgn : IN  std_logic;
         gps_clk : IN  std_logic;
         --tst_clk   : out   STD_LOGIC;
         led0 : OUT  std_logic;
         epp_astb : IN  std_logic;
         epp_dstb : IN  std_logic;
         epp_wait : OUT  std_logic;
         epp_wr : IN  std_logic;
         epp_data : INOUT  std_logic_vector(7 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal mclk : std_logic := '0';
   signal gps_mag : std_logic := '0';
   signal gps_sgn : std_logic := '0';
   signal gps_clk : std_logic := '0';
   signal epp_astb : std_logic := '1';
   signal epp_dstb : std_logic := '1';
   signal epp_wr : std_logic := '1';

   --BiDirs
   signal epp_data : std_logic_vector(7 downto 0);

    --Outputs
   signal led0 : std_logic;
   signal epp_wait : std_logic;
   signal tst_clk : std_logic;

   -- Clock period definitions
   constant mclk_period    : time := 20 ns;
   constant gps_clk_period : time := 61 ns;
 
BEGIN
 
   -- Instantiate the Unit Under Test (UUT)
   uut: top_level PORT MAP (
          mclk => mclk,
          --tst_clk  => tst_clk,
          gps_mag => gps_mag,
          gps_sgn => gps_sgn,
          gps_clk => gps_clk,
          led0 => led0,
          epp_astb => epp_astb,
          epp_dstb => epp_dstb,
          epp_wait => epp_wait,
          epp_wr => epp_wr,
          epp_data => epp_data
        );

   -- Clock process definitions
   mclk_process :process
   begin
   	mclk <= '0';
   	wait for mclk_period/2;
   	mclk <= '1';
   	wait for mclk_period/2;
   end process;
 
   gps_clk_process :process
   begin
   	gps_clk <= '0';
   	wait for gps_clk_period/2;
   	gps_clk <= '1';
   	wait for gps_clk_period/2;
      if gps_sgn = '0' and gps_mag = '0' then
         gps_sgn <= '0';
         gps_mag <= '1';
      elsif gps_sgn = '0' and gps_mag = '1' then
         gps_sgn <= '1';
         gps_mag <= '0';
      elsif gps_sgn = '1' and gps_mag = '0' then
         gps_sgn <= '1';
         gps_mag <= '1';
      else
         gps_sgn <= '0';
         gps_mag <= '0';
      end if;
   end process;
 

   -- Stimulus process
stim_proc: process
   begin   	
      -- hold reset state for 100 ns.
      wait for 100 ns;   
      
      epp_dstb <= '0';
      
      wait until epp_wait = '1';
      wait for 10 ns;
      epp_dstb <= '1';
      wait until epp_wait = '0';
      
   end process;

END;
