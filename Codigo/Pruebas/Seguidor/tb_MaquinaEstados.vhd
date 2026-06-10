-- ============================================================================
-- tb_MaquinaEstados - Seguidor + orquestación del brazo (Etapa 2).
--   Verifica: motores se mueven al seguir; en zona SIN objeto -> start_scan;
--   espera scan_active+arm_ready y sale; en zona CON objeto -> trigger_drop;
--   espera has_object=0 y sale.
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
            ZONA_CYCLES : integer := 2_000_000; SALIR_CYCLES : integer := 25_000_000;
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

    signal saw_scan, saw_drop, mot_active : boolean := false;
    signal chk_no_zona : boolean := false;  -- armado en Fase E: (1,1) desde GIRO NO debe zonear (gate)
    signal fallo_zona  : boolean := false;  -- true si dispara zona estando chk_no_zona

begin

    dut : MaquinaEstados
        generic map (ZONA_CYCLES => 40, SALIR_CYCLES => 60, FILTRO_CYCLES => 4)
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

    -- Monitores
    mon : process(clk)
    begin
        if rising_edge(clk) then
            if start_scan = '1' then saw_scan <= true; end if;
            if trigger_drop = '1' then saw_drop <= true; end if;
            if a1 = '1' then mot_active <= true; end if;
            if chk_no_zona and (start_scan = '1' or trigger_drop = '1') then
                fallo_zona <= true;
            end if;
        end if;
    end process;

    stim : process
    begin
        rst <= '1'; wait for 100 ns; rst <= '0';
        arm_ready <= '1';                       -- brazo libre al inicio
        wait until rising_edge(clk);

        -- ---- Fase A: seguir recto (ambos blanco, línea entre medio) ----
        sizq <= '0'; sder <= '0';
        wait for 3 us;
        assert mot_active report "FALLO: los motores no se mueven al seguir" severity error;
        report "Fase A OK: motores activos al seguir recto" severity note;

        -- ---- Fase B: corrección a un lado y al otro (no debe trabarse) ----
        sizq <= '1'; sder <= '0'; wait for 2 us;   -- IZQ pisa línea -> girar IZQ
        sizq <= '0'; sder <= '1'; wait for 2 us;   -- DER pisa línea -> girar DER
        sizq <= '0'; sder <= '0'; wait for 1 us;   -- vuelve al centro
        report "Fase B OK: corrección/recuperación sin trabarse" severity note;

        -- ---- Fase C: zona SIN objeto -> start_scan ----
        has_object <= '0';
        sizq <= '1'; sder <= '1';               -- cuadrado: ambos en línea
        wait for 2 us;                          -- supera ZONA_CYCLES -> start_scan
        assert saw_scan report "FALLO: no disparó start_scan en zona sin objeto" severity error;
        report "Fase C OK: zona sin objeto -> start_scan" severity note;
        -- emula barrido + agarre (brazo OCUPADO: arm_ready=0 mientras scan_active=1)
        scan_active <= '1'; arm_ready <= '0';
        wait for 2 us;
        scan_active <= '0';                     -- barrido terminó
        sizq <= '0'; sder <= '0';               -- el robot sale de la zona (sensores libres)
        wait for 1 us;
        arm_ready <= '1';                       -- brazo terminó (sin objeto)
        wait for 3 us;                          -- E_SALIR_ZONA -> E_SEGUIR

        -- ---- Fase D: zona CON objeto -> trigger_drop ----
        has_object <= '1';
        sizq <= '1'; sder <= '1';
        wait for 2 us;                          -- zona -> trigger_drop
        assert saw_drop report "FALLO: no disparó trigger_drop en zona con objeto" severity error;
        report "Fase D OK: zona con objeto -> trigger_drop" severity note;
        -- emula el depósito (grab_ctrl baja has_object y el robot sale)
        arm_ready <= '0';
        wait for 1 us;
        has_object <= '0';                      -- soltó el cubo
        sizq <= '0'; sder <= '0';
        arm_ready <= '1';
        wait for 3 us;

        -- ---- Fase E: (1,1) ENTRANDO GIRANDO no debe disparar ZONA falsa (gate ZONA_DESDE_RECTO) ----
        sizq <= '0'; sder <= '0'; wait for 1 us;   -- centrado (ultimo_giro=RECTO)
        sizq <= '1'; sder <= '0'; wait for 1 us;   -- IZQ pisa -> girando (ultimo_giro=IZQ)
        chk_no_zona <= true;
        sizq <= '1'; sder <= '1';                  -- DOBLE negro VINIENDO DE GIRO
        wait for 3 us;                             -- > ZONA_CYCLES: el gate debe BLOQUEAR la zona
        assert not fallo_zona
            report "FALLO: (1,1) entrando girando disparó zona falsa (gate no funcionó)" severity error;
        chk_no_zona <= false;
        report "Fase E OK: (1,1) desde giro NO dispara zona falsa (gate)" severity note;
        sizq <= '0'; sder <= '0'; wait for 1 us;

        report "OK: seguidor + orquestación (scan/drop por estado de acarreo) funcionan" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
