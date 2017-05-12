--------------------------------------------
-- Author: Mike Field <hamster@snap.net.nz>
--------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity gps_capture is
   port (
      clk       : in  std_logic;
 
      ---------------------------------------
      -- From the GPS front end
      ---------------------------------------
      gps_mag    : in  std_logic;
      gps_sgn    : in  std_logic;
      gps_clk    : in  std_logic;
      ---------------------------------------
      -- To the FIFO for transfer to the host
      ---------------------------------------
      fifo_data  : out std_logic_vector(7 downto 0) := (others => '0');
      fifo_we    : out std_logic;
      fifo_full  : in  std_logic;
      fifo_empty : in  std_logic;
      
      ---------------------------------------
      -- status for user
      ---------------------------------------       
      led        : OUT  std_logic_vector(7 downto 0);
      overrun    : out std_logic);
end gps_capture;

architecture Behavioral of gps_capture is
   signal sync_mag        : std_logic_vector(3 downto 0) := "0000";
   signal sync_sgn        : std_logic_vector(3 downto 0) := "0000";
   signal sync_clk        : std_logic_vector(3 downto 0) := "0000";
   signal overrun_counter : unsigned(26 downto 0) := (others => '1');
   signal data            : std_logic_vector(7 downto 0) := x"00";
   signal fifo_active     : std_logic := '0';
   signal one_in_six      : std_logic_vector(2 downto 0) := "001";
   signal gap_count       : unsigned(7 downto 0) := (others => '0');
   signal max_gap         : unsigned(7 downto 0) := (others => '0');
begin
   fifo_data <= data;
   led <= std_logic_vector(max_gap);
clk_proc: process(clk)
   begin
      if rising_edge(clk) then
         fifo_we <= '0';
         --------------------------------------------------
         -- Capture the data on the rising edge of sync_clk
         --------------------------------------------------
         if sync_clk(1) = '1' and sync_clk(0) = '0' then
            -- The upper two bits form a sequence counter to enable the 
            -- detection dropped data in the transfer to the host.
            if one_in_six(0) = '1' then
               data(7 downto 6) <= std_logic_vector(unsigned(data(7 downto 6))+1);
            end if;
            data(5 downto 0) <= sync_sgn(0) & data(5 downto 1);
            fifo_we     <= fifo_active and one_in_six(0);
            one_in_six  <= one_in_six(0) & one_in_six(5 downto 1);
            if max_gap < gap_count then
               max_gap <= gap_count;
            end if;
            gap_count <= (others => '0');
         else
            if gap_count /= x"FF" then
               gap_count <= gap_count+1;
            end if;
         end if;
 
         --------------------------------------------------------
         -- Displaying to the user when the FIFO is stalled. 
         -- This stretches the pulse on overrun for at least 
         -- 2^(overrun_counter'high) cycles if fifo_full gets set
         --------------------------------------------------------
         overrun <= std_logic(overrun_counter(overrun_counter'high));
         if fifo_full = '1' then
            overrun_counter <= (others => '1');
         else 
            overrun_counter <= overrun_counter - overrun_counter(overrun_counter'high downto overrun_counter'high);
         end if;

         -----------------------------------------------------------
         -- If the fifo becomes full, then assume that the host is 
         -- no longer reading new data. However, if the fifo becomes
         -- empty, then assume that the host is pulling data.
         -----------------------------------------------------------
         if fifo_empty = '1' then 
            fifo_active <= '1';
         elsif fifo_full = '1' then
            fifo_active <= '0';
            overrun_counter <= (others => '1');
         end if;
           
         -----------------------------------------
         -- Synchronise the incoming signals
         -----------------------------------------
         sync_mag <= gps_mag & sync_mag(3 downto 1);
         sync_sgn <= gps_sgn & sync_sgn(3 downto 1);
         sync_clk <= gps_clk & sync_clk(3 downto 1);
      end if;
   end process;
end Behavioral;
