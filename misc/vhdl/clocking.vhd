--------------------------------------------
-- Author: Mike Field <hamster@snap.net.nz>
--------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

library UNISIM;
use UNISIM.VComponents.all;

entity clocking is
    Port ( clk : in  STD_LOGIC;
           clk_100 : out  STD_LOGIC);
end clocking;

architecture Behavioral of clocking is
   signal clkfb : std_logic := '0';
begin

DCM_SP_inst : DCM_SP
   generic map (
      CLKDV_DIVIDE          => 2.0,    --  Divide by: 1.5,2.0,2.5,3.0,3.5,4.0,4.5,5.0,5.5,6.0,6.5
                                       --     7.0,7.5,8.0,9.0,10.0,11.0,12.0,13.0,14.0,15.0 or 16.0
      CLKFX_DIVIDE          => 2,      --  Can be any interger from 1 to 32
      CLKFX_MULTIPLY        => 6,      --  Can be any integer from 1 to 32
      CLKIN_DIVIDE_BY_2     => FALSE,  --  TRUE/FALSE to enable CLKIN divide by two feature
      CLKIN_PERIOD          => 20.0,   --  Specify period of input clock
      CLKOUT_PHASE_SHIFT    => "NONE", --  Specify phase shift of "NONE", "FIXED" or "VARIABLE" 
      CLK_FEEDBACK          => "1X",   --  Specify clock feedback of "NONE", "1X" or "2X" 
      DESKEW_ADJUST         => "SYSTEM_SYNCHRONOUS", -- "SOURCE_SYNCHRONOUS", "SYSTEM_SYNCHRONOUS" or
                                       --     an integer from 0 to 15
      DLL_FREQUENCY_MODE    => "LOW",  -- "HIGH" or "LOW" frequency mode for DLL
      DUTY_CYCLE_CORRECTION => TRUE,   --  Duty cycle correction, TRUE or FALSE
      PHASE_SHIFT           => 0,      --  Amount of fixed phase shift from -255 to 255
      STARTUP_WAIT          => FALSE)  --  Delay configuration DONE until DCM_SP LOCK, TRUE/FALSE
   port map (
      CLK0     => clkfb,
      CLK180   => open,
      CLK270   => open,
      CLK2X    => open,
      CLK2X180 => open,
      CLK90    => open,
      CLKDV    => open,
      CLKFX    => clk_100,
      CLKFX180 => open,
      LOCKED   => open,
      PSDONE   => open,
      STATUS   => open,
      CLKFB    => clkfb,
      CLKIN    => clk,   -- Clock input (from IBUFG, BUFG or DCM)
      PSCLK    => open,
      PSEN     => '0',
      PSINCDEC => '0',
      RST      => '0'
   );


end Behavioral;

