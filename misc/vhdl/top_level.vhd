--------------------------------------------
-- Author: Mike Field <hamster@snap.net.nz>
--------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity top_level is
    Port ( mclk      : in    STD_LOGIC;
           gps_mag   : in    STD_LOGIC;
           gps_sgn   : in    STD_LOGIC;
           gps_clk   : in    STD_LOGIC;
           led      : OUT  std_logic_vector(7 downto 0);
           epp_astb  : in    STD_LOGIC;
           epp_dstb  : in    STD_LOGIC;
           epp_wait  : out   STD_LOGIC;
           epp_wr    : in    STD_LOGIC;
           epp_data  : inout STD_LOGIC_VECTOR (7 downto 0));
end top_level;

architecture Behavioral of top_level is
   signal clk_100      : std_logic;
   
   component clocking is
      port (
         clk        : in  std_logic;
         clk_100    : out std_logic);
   end component;

   component gps_capture is
      port (
         clk        : in  std_logic;
         gps_mag    : in  std_logic;
         gps_sgn    : in  std_logic;
         gps_clk    : in  std_logic;
         led        : OUT  std_logic_vector(7 downto 0);
         fifo_data  : out std_logic_vector(7 downto 0) := (others => '0');
         fifo_we    : out std_logic;
         fifo_full  : in  std_logic;
         fifo_empty : in  std_logic;
         overrun    : out std_logic);
   end component;
   
   signal fifo_data_in : std_logic_vector(7 downto 0) := (others => '0');
   signal fifo_we      : std_logic;
   signal fifo_full    : std_logic;
   signal fifo_empty   : std_logic;

   component fifo_16k_x_8 is
      port ( 
         clk   : in  std_logic;
         wr    : in  std_logic;
         din   : in  std_logic_vector(7 downto 0) := (others => '0');
         
         rd    : in  std_logic;
         dout  : out std_logic_vector(7 downto 0) := (others => '0');

         full  : out std_logic;
         empty : out std_logic);
   end component;

   COMPONENT ip_fifo
   PORT (
      clk   : IN STD_LOGIC;
      din   : IN STD_LOGIC_VECTOR(7 DOWNTO 0);
      wr_en : IN STD_LOGIC;
      rd_en : IN STD_LOGIC;
      dout  : OUT STD_LOGIC_VECTOR(7 DOWNTO 0);
      full  : OUT STD_LOGIC;
      empty : OUT STD_LOGIC
      );
   END COMPONENT;

   signal fifo_data_out : std_logic_vector(7 downto 0) := (others => '0');
   signal fifo_rd       : std_logic;

   component epp_interface is
      port ( 
         clk        : in  std_logic;
         fifo_rd    : out   STD_LOGIC;
         fifo_data  : in    STD_LOGIC_VECTOR(7 downto 0);
         fifo_empty : in    STD_LOGIC;

         epp_astb   : in    STD_LOGIC;
         epp_dstb   : in    STD_LOGIC;
         epp_wait   : out   STD_LOGIC;
         epp_wr     : in    STD_LOGIC;
         epp_data   : inout STD_LOGIC_VECTOR (7 downto 0));
   end component;
   signal gen_test_clk : std_logic_vector(3 downto 0) := "0011";
begin

i_clocking: clocking port map (
         clk     => mclk,
         clk_100 => clk_100);

i_gps: gps_capture port map (
         clk        => clk_100,
         gps_mag    => gps_mag,
         gps_sgn    => gps_sgn,
         gps_clk    => gps_clk,
         led        => led,
         fifo_data  => fifo_data_in,
         fifo_we    => fifo_we,
         fifo_full  => fifo_full,
         fifo_empty => fifo_empty,
         overrun    => open);

i_ip_fifo: ip_fifo port map ( 
         clk   => clk_100,

         wr_en => fifo_we,
         din   => fifo_data_in,

         rd_en => fifo_rd,
         dout  => fifo_data_out,

         full  => fifo_full,
         empty => fifo_empty);
         
i_epp_interface: epp_interface port map (
         clk        => clk_100,
         
         fifo_rd    => fifo_rd,
         fifo_data  => fifo_data_out,
         fifo_empty => fifo_empty,
         
         epp_astb   => epp_astb,
         epp_dstb   => epp_dstb,
         epp_wait   => epp_wait,
         epp_wr     => epp_wr,
         epp_data   => epp_data);
end Behavioral;

