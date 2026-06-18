library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity TestSensores is
    port (
        clk         : in  std_logic;  -- PIN_17, 50 MHz
        reset       : in  std_logic;  -- PIN_144, activo bajo

        led_1       : out std_logic;  -- vida (1 Hz)
        led_2       : out std_logic;  -- sensor_izq
        led_3       : out std_logic;  -- sensor_der

        sensor_izq  : in  std_logic;
        sensor_der  : in  std_logic
    );
end TestSensores;

architecture rtl of TestSensores is

    -- 50 MHz -> toggle cada 25 millones de ciclos = 1 Hz
    signal cnt_1hz : integer range 0 to 24_999_999 := 0;
    signal led1_r  : std_logic := '0';

begin

    --------------------------------------------------------------------------
    -- LED de vida (1 Hz)
    --------------------------------------------------------------------------
    process(clk, reset)
    begin
        if reset = '0' then
            cnt_1hz <= 0;
            led1_r  <= '0';

        elsif rising_edge(clk) then

            if cnt_1hz = 24_999_999 then
                cnt_1hz <= 0;
                led1_r  <= not led1_r;
            else
                cnt_1hz <= cnt_1hz + 1;
            end if;

        end if;
    end process;

    --------------------------------------------------------------------------
    -- Salidas
    --------------------------------------------------------------------------
    led_1 <= led1_r;

    -- Reflejan directamente el estado de los sensores
    led_2 <= sensor_izq;
    led_3 <= sensor_der;

end rtl;