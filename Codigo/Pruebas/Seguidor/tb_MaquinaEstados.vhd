-- ============================================================================
-- tb_MaquinaEstados - Verifica el seguidor SIMPLE con los 2 sensores DENTRO de la
--   línea (línea ancha). s_*='1' = ese sensor está SOBRE el negro.
--   Tabla (s_izq=I, s_der=D):
--     (0,0) -> DETENERSE                 (ambas ruedas en 0)
--     (1,0) -> GIRAR IZQUIERDA           (DER empuja -> cb1>ca1)
--     (0,1) -> GIRAR DERECHA             (IZQ empuja -> ca1>cb1)
--     (1,1) -> AVANZAR                   (ambas ruedas ~igual)
--   Se prueba con MODO_PIVOTE=true: en los giros la rueda INTERIOR va en REVERSA
--   (se activa motor_a2/b2). Mide el duty de cada salida (cuenta > 1 periodo PWM).
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
            DUTY_RECTO    : integer := 30000;
            DUTY_GIRO_EXT : integer := 45000;
            DUTY_GIRO_INT : integer := 15000;
            MODO_PIVOTE   : boolean := false;
            FILTRO_CYCLES : integer := 15000;
            LINE_LVL      : std_logic := '0'
        );
        port (
            clk, rst : in std_logic;
            sensor_izq, sensor_der : in std_logic;
            motor_a1, motor_a2, motor_b1, motor_b2 : out std_logic;
            led_estado : out std_logic
        );
    end component;

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal sizq, sder : std_logic := '0';
    signal a1, a2, b1, b2 : std_logic;
    signal led_estado : std_logic;
    signal simdone : boolean := false;

    -- Medición de duty: cuenta ciclos en alto de cada salida mientras 'meas'.
    -- a1/b1 = adelante (IZQ/DER); a2/b2 = reversa (IZQ/DER).
    signal meas : boolean := false;
    signal ca1, cb1, ca2, cb2 : integer := 0;

begin

    -- LINE_LVL='1' en el tb: sizq/sder='1' = "sobre la línea". MODO_PIVOTE=true para
    -- ejercitar el pívot (rueda interior en reversa).
    dut : MaquinaEstados
        generic map (MODO_PIVOTE => true, FILTRO_CYCLES => 4, LINE_LVL => '1')
        port map (
            clk => clk, rst => rst, sensor_izq => sizq, sensor_der => sder,
            motor_a1 => a1, motor_a2 => a2, motor_b1 => b1, motor_b2 => b2,
            led_estado => led_estado
        );

    clk_proc : process
    begin
        while not simdone loop
            clk <= '0'; wait for 10 ns; clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    -- Integra el duty de las 4 salidas (a1/b1=adelante, a2/b2=reversa).
    count : process(clk)
    begin
        if rising_edge(clk) then
            if meas then
                if a1 = '1' then ca1 <= ca1 + 1; end if;
                if b1 = '1' then cb1 <= cb1 + 1; end if;
                if a2 = '1' then ca2 <= ca2 + 1; end if;
                if b2 = '1' then cb2 <= cb2 + 1; end if;
            else
                ca1 <= 0; cb1 <= 0; ca2 <= 0; cb2 <= 0;
            end if;
        end if;
    end process;

    stim : process
    begin
        rst <= '1'; wait for 200 ns; rst <= '0';
        wait until rising_edge(clk);

        -- ---- (0,0) -> DETENERSE: ambas ruedas paradas ----
        sizq <= '0'; sder <= '0'; wait for 2 us;
        meas <= false; wait for 1 us;                 -- limpia contadores
        meas <= true;  wait for 1.6 ms;               -- mide (> 1 periodo PWM); evalúa con meas activo
        assert ca1 = 0 and cb1 = 0
            report "FALLO (0,0): deberia DETENERSE (motores activos)" severity error;
        report "(0,0) -> detenerse OK" severity note;

        -- ---- (1,0) -> GIRAR IZQUIERDA: DER adelante, IZQ en REVERSA (pívot) ----
        sizq <= '1'; sder <= '0'; wait for 2 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert cb1 > 5000
            report "FALLO (1,0): la rueda DER deberia empujar adelante" severity error;
        assert ca2 > 5000 and ca1 = 0
            report "FALLO (1,0): la rueda IZQ deberia ir en REVERSA (pivot)" severity error;
        report "(1,0) -> girar izquierda (pivot: IZQ en reversa) OK" severity note;

        -- ---- (0,1) -> GIRAR DERECHA: IZQ adelante, DER en REVERSA (pívot) ----
        sizq <= '0'; sder <= '1'; wait for 2 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert ca1 > 5000
            report "FALLO (0,1): la rueda IZQ deberia empujar adelante" severity error;
        assert cb2 > 5000 and cb1 = 0
            report "FALLO (0,1): la rueda DER deberia ir en REVERSA (pivot)" severity error;
        report "(0,1) -> girar derecha (pivot: DER en reversa) OK" severity note;

        -- ---- (1,1) -> AVANZAR: ambas ruedas ~iguales y en marcha ----
        sizq <= '1'; sder <= '1'; wait for 2 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert ca1 > 5000 and cb1 > 5000
            report "FALLO (1,1): el robot no avanza" severity error;
        assert (ca1 - cb1) < 8000 and (cb1 - ca1) < 8000
            report "FALLO (1,1): deberia AVANZAR recto (ruedas desbalanceadas)" severity error;
        report "(1,1) -> avanzar OK" severity note;

        meas <= false;
        report "OK: seguidor simple (2 sensores dentro) sigue la tabla" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
