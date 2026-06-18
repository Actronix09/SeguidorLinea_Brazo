library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity encoder_servo is
    Port (
        clk   : in  STD_LOGIC;
        sig_a : in  STD_LOGIC;
        sig_b : in  STD_LOGIC;
        pwm   : out STD_LOGIC
    );
end encoder_servo;

architecture contar of encoder_servo is

    -- Sincronización y debounce
    signal a_sync, b_sync : std_logic_vector(1 downto 0);
    signal a_deb, b_deb   : std_logic;
    signal last_a         : std_logic;

    -- PWM 50 Hz → 20 ms = 1_000_000 ciclos a 50 MHz
    constant PERIOD    : integer := 1_000_000;
    constant PULSE_MIN : integer := 25_000;    -- 0.5 ms → 0°
    constant PULSE_MAX : integer := 125_000;   -- 2.5 ms → 180°

    -- 1° = (125_000 - 25_000) / 180 ≈ 556 ciclos
    constant PASO      : integer := 556;

    signal pwm_counter : integer range 0 to PERIOD   := 0;
    signal pulse_width : integer range PULSE_MIN to PULSE_MAX := 75_000; -- centro 90°

begin

-- Sincronización y debounce de 2 etapas
SR: process(clk)
begin
    if rising_edge(clk) then
        a_sync <= a_sync(0) & sig_a;
        b_sync <= b_sync(0) & sig_b;
    end if;
end process;

a_deb <= a_sync(1);
b_deb <= b_sync(1);

-- Decodificación del encoder → incremento/decremento de 1° por clic
rotar: process(clk)
begin
    if rising_edge(clk) then
        last_a <= a_deb;

        if (a_deb /= last_a) then
            if (a_deb = b_deb) then
                -- Horario: +1°, respetando el límite máximo
                if pulse_width <= PULSE_MAX - PASO then
                    pulse_width <= pulse_width + PASO;
                else
                    pulse_width <= PULSE_MAX;
                end if;
            else
                -- Antihorario: -1°, respetando el límite mínimo
                if pulse_width >= PULSE_MIN + PASO then
                    pulse_width <= pulse_width - PASO;
                else
                    pulse_width <= PULSE_MIN;
                end if;
            end if;
        end if;

    end if;
end process;

-- Generador PWM 50 Hz
pwm_gen: process(clk)
begin
    if rising_edge(clk) then
        if pwm_counter = PERIOD - 1 then
            pwm_counter <= 0;
        else
            pwm_counter <= pwm_counter + 1;
        end if;

        if pwm_counter < pulse_width then
            pwm <= '1';
        else
            pwm <= '0';
        end if;
    end if;
end process;

end contar;