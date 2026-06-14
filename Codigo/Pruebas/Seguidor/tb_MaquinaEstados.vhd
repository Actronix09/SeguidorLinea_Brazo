-- ============================================================================
-- tb_MaquinaEstados - Verifica la TABLA del seguidor de LÍNEA FINA (sensores fuera).
--   s_*='1' = ese sensor ve NEGRO (sobre la línea). Tabla (s_der, s_izq) -> giro:
--     (0,0) NINGUNA -> recto            (ambas ruedas ~igual)
--     (1,0) DER sobre línea -> IZQUIERDA (rueda DER empuja  -> cb1 > ca1)
--     (0,1) IZQ sobre línea -> DERECHA   (rueda IZQ empuja  -> ca1 > cb1)
--     (1,1) CASO ESPECIAL -> recto       (ambas ruedas ~igual)
--   Mide el ciclo de trabajo de cada rueda (cuenta a1/b1 durante > 1 periodo PWM)
--   y comprueba la asimetría esperada. USAR_ZONA=false (seguidor puro).
--
--   vlib work
--   vcom -2008 MaquinaEstados.vhd tb_MaquinaEstados.vhd
--   vsim tb_MaquinaEstados -do "run -all"
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_MaquinaEstados is
end tb_MaquinaEstados;

architecture sim of tb_MaquinaEstados is

    component MaquinaEstados
        generic (
            USAR_ZONA : boolean := false; MODO_ZONA : integer := 0;
            FILTRO_CYCLES : integer := 50_000; LINE_LVL : std_logic := '1'
        );
        port (
            clk, rst : in std_logic;
            sensor_izq, sensor_der : in std_logic;
            motor_a1, motor_a2, motor_b1, motor_b2 : out std_logic;
            start_scan, trigger_drop : out std_logic;
            scan_active, arm_ready, has_object : in std_logic;
            led_estado, led_error : out std_logic
        );
    end component;

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal sizq, sder : std_logic := '0';
    signal a1, a2, b1, b2 : std_logic;
    signal start_scan, trigger_drop : std_logic;
    signal scan_active, arm_ready, has_object : std_logic := '0';
    signal led_estado, led_error : std_logic;
    signal simdone : boolean := false;

    -- Medición de duty: cuenta ciclos con a1/b1 en alto mientras 'meas'.
    signal meas : boolean := false;
    signal ca1, cb1 : integer := 0;

begin

    -- LINE_LVL='1': en el tb sizq/sder='1' = "sobre la línea" (NEGRO). USAR_ZONA=false.
    dut : MaquinaEstados
        generic map (USAR_ZONA => false, MODO_ZONA => 0, FILTRO_CYCLES => 4, LINE_LVL => '1')
        port map (
            clk => clk, rst => rst, sensor_izq => sizq, sensor_der => sder,
            motor_a1 => a1, motor_a2 => a2, motor_b1 => b1, motor_b2 => b2,
            start_scan => start_scan, trigger_drop => trigger_drop,
            scan_active => scan_active, arm_ready => arm_ready, has_object => has_object,
            led_estado => led_estado, led_error => led_error
        );

    clk_proc : process
    begin
        while not simdone loop
            clk <= '0'; wait for 10 ns; clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    -- a1 = motor IZQ adelante, b1 = motor DER adelante (PWM). Integramos su duty.
    count : process(clk)
    begin
        if rising_edge(clk) then
            if meas then
                if a1 = '1' then ca1 <= ca1 + 1; end if;
                if b1 = '1' then cb1 <= cb1 + 1; end if;
            else
                ca1 <= 0; cb1 <= 0;
            end if;
        end if;
    end process;

    stim : process
    begin
        rst <= '1'; wait for 200 ns; rst <= '0';
        wait until rising_edge(clk);

        -- ---- (0,0) NINGUNA -> recto: ambas ruedas ~iguales ----
        sder <= '0'; sizq <= '0'; wait for 2 us;
        meas <= true; wait for 1.6 ms; meas <= false; wait for 1 us;
        assert ca1 > 5000 and cb1 > 5000
            report "FALLO (0,0): el robot no avanza (motores parados)" severity error;
        assert (ca1 - cb1) < 8000 and (cb1 - ca1) < 8000
            report "FALLO (0,0): debería ir RECTO (ruedas desbalanceadas)" severity error;
        report "(0,0) NINGUNA -> recto OK" severity note;

        -- ---- (1,0) DER sobre línea -> IZQUIERDA: rueda DER (cb1) más fuerte ----
        sder <= '1'; sizq <= '0'; wait for 2 us;
        meas <= true; wait for 1.6 ms; meas <= false; wait for 1 us;
        assert cb1 > ca1 + 5000
            report "FALLO (1,0): debería girar IZQUIERDA (rueda DER no domina)" severity error;
        report "(1,0) DER sobre linea -> IZQUIERDA OK" severity note;

        -- ---- (0,1) IZQ sobre línea -> DERECHA: rueda IZQ (ca1) más fuerte ----
        sder <= '0'; sizq <= '1'; wait for 2 us;
        meas <= true; wait for 1.6 ms; meas <= false; wait for 1 us;
        assert ca1 > cb1 + 5000
            report "FALLO (0,1): debería girar DERECHA (rueda IZQ no domina)" severity error;
        report "(0,1) IZQ sobre linea -> DERECHA OK" severity note;

        -- ---- (1,1) CASO ESPECIAL -> recto: ambas ruedas ~iguales ----
        sder <= '1'; sizq <= '1'; wait for 2 us;
        meas <= true; wait for 1.6 ms; meas <= false; wait for 1 us;
        assert ca1 > 5000 and cb1 > 5000
            report "FALLO (1,1): el robot no avanza" severity error;
        assert (ca1 - cb1) < 8000 and (cb1 - ca1) < 8000
            report "FALLO (1,1): debería ir RECTO" severity error;
        report "(1,1) CASO ESPECIAL -> recto OK" severity note;

        report "OK: el seguidor sigue la tabla (0,0)recto (1,0)izq (0,1)der (1,1)especial" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
