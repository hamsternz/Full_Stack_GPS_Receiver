--------------------------------------------
-- Author: Mike Field <hamster@snap.net.nz>
--------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity epp_interface is
      port ( 
         clk        : in  std_logic;
         fifo_rd    : out   STD_LOGIC := '0';
         fifo_data  : in    STD_LOGIC_VECTOR(7 downto 0);
         fifo_empty : in    STD_LOGIC;

         epp_astb   : in    STD_LOGIC;
         epp_dstb   : in    STD_LOGIC;
         epp_wait   : out   STD_LOGIC := '0';
         epp_wr     : in    STD_LOGIC;
         epp_data   : inout STD_LOGIC_VECTOR (7 downto 0));
end epp_interface;

architecture Behavioral of epp_interface is
   type  t_state is (s_idle, s_astb_read, s_astb_write, s_dstb_read, s_dstb_write);
   signal state : t_state := s_idle;
   signal sync_astb : std_logic_vector(1 downto 0) := "00";
   signal sync_dstb : std_logic_vector(1 downto 0) := "00";
begin

   with state select epp_data <= fifo_data       when s_dstb_read,
                                 x"AA"           when s_astb_read,
                                 (others => 'Z') when others;
                                 
      
process(clk) 
   begin
      if rising_edge(clk) then
         fifo_rd <= '0';
         case state is       
            when s_idle    =>
               if sync_astb(0) = '0' then
                  --------------
                  -- address strobe
                  --------------
                  if epp_wr = '0' then
                     state <= s_astb_write;
                  else
                     state <= s_astb_read;
                  end if;                  
                  epp_wait <= '1';
               elsif sync_dstb(0) = '0' then
                  --------------
                  -- data strobe
                  --------------
                  if epp_wr = '0' then
                     state <= s_dstb_write;
                     epp_wait <= '1';
                  else
                     if fifo_empty = '0' then
                        state <= s_dstb_read;
                        epp_wait <= '1';
                     end if;
                  end if;                  
               end if;
            when s_astb_read => 
               if sync_astb(0) = '1' then
                  epp_wait <= '0';
                  state <= s_idle;
               end if;
            when s_astb_write => 
               if sync_astb(0) = '1' then
                  epp_wait <= '0';
                  state <= s_idle;
               end if;
            when s_dstb_read => 
               if sync_dstb(0) = '1' then
                  epp_wait <= '0';
                  state <= s_idle;
                  fifo_rd <= '1';
               end if;
            when s_dstb_write => 
               if sync_dstb(0) = '1' then
                  epp_wait <= '0';
                  state <= s_idle;
               end if;
            when others    => 
               state <= s_idle;
         end case;

         ---------------------------
         -- Sync the strobe signals
         ---------------------------
         sync_astb <= epp_astb & sync_astb(sync_astb'high downto 1);
         sync_dstb <= epp_dstb & sync_dstb(sync_dstb'high downto 1);
      end if;
   end process;

end Behavioral;

