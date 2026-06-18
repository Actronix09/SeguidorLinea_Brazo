-- ============================================================================
-- tb_MaquinaEstados - Verifica el seguidor SIMPLE (2 sensores DENTRO de la línea)
--   + la ZONA DE RECOGIDA por UNA línea blanca ancha.
--
--   s_*='1' = ese sensor está SOBRE la línea (LINE_LVL='1' en este tb).
--   Tabla (s_izq=I, s_der=D):
--     (1,1) -> AVANZAR            (ambas ruedas ~igual)
--     (1,0) -> GIRAR IZQUIERDA    (DER empuja, IZQ en reversa por pívot)
--     (0,1) -> GIRAR DERECHA      (IZQ empuja, DER en reversa por pívot)
--     (0,0) -> doble blanco: AVANZA para cruzar; >=W_ARM arma; >W_STOP detiene
--
--   ZONA: doble blanco (0,0) >= W_ARM -> ARMA. Al volver a la línea (esperando que
--   ambos sensores estén en (1,1)) se DISPARA start_scan. Doble blanco > W_STOP =
--   pérdida -> DETENER. Si el barrido falla (sensor_err) -> E_FALLO (alto+zona_fallo).
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

    -- Umbrales chicos para simular rápido pero > 1 periodo PWM (65536) para medir duty.
    constant C_FILTRO : integer := 4;
    constant C_WARM   : integer := 200_000;   -- 4 ms @50MHz: arma la zona
    constant C_WSTOP  : integer := 500_000;   -- 10 ms @50MHz: detiene (pérdida)
    constant C_ARR    : integer := 2_000;     -- 40 us @50MHz: empujón de arranque (corto para el tb)

    signal clk : std_logic := '0';
    signal rst : std_logic := '1';
    signal sizq, sder : std_logic := '0';
    signal a1, a2, b1, b2 : std_logic;
    signal led_estado : std_logic;
    signal simdone : boolean := false;

    -- Handshake con el brazo (los manda el tb).
    signal start_scan   : std_logic;
    signal trigger_drop : std_logic;
    signal scan_active  : std_logic := '0';
    signal arm_ready    : std_logic := '0';
    signal has_object   : std_logic := '0';
    signal sensor_err   : std_logic := '0';
    signal zona_fallo   : std_logic;

    -- Medición de duty: cuenta ciclos en alto de cada salida mientras 'meas'.
    signal meas : boolean := false;
    signal ca1, cb1, ca2, cb2 : integer := 0;

    -- Cuenta de pulsos de start_scan / trigger_drop (la stim los lee).
    signal n_scan : integer := 0;
    signal n_drop : integer := 0;

begin

    dut : entity work.MaquinaEstados
        generic map (
            MODO_PIVOTE   => true,
            FILTRO_CYCLES => C_FILTRO,
            LINE_LVL      => '1',
            DUTY_ARRANQUE => 40000,
            T_ARRANQUE    => C_ARR,
            W_ARM_CYCLES  => C_WARM,
            W_STOP_CYCLES => C_WSTOP
        )
        port map (
            clk => clk, rst => rst, sensor_izq => sizq, sensor_der => sder,
            motor_a1 => a1, motor_a2 => a2, motor_b1 => b1, motor_b2 => b2,
            led_estado => led_estado,
            start_scan => start_scan, trigger_drop => trigger_drop,
            scan_active => scan_active,
            arm_ready => arm_ready, has_object => has_object,
            sensor_err => sensor_err, zona_fallo => zona_fallo
        );

    clk_proc : process
    begin
        while not simdone loop
            clk <= '0'; wait for 10 ns; clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

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

    scancount : process(clk)
    begin
        if rising_edge(clk) then
            if start_scan   = '1' then n_scan <= n_scan + 1; end if;
            if trigger_drop = '1' then n_drop <= n_drop + 1; end if;
        end if;
    end process;

    stim : process
        variable s0 : integer;
        variable d0 : integer;
    begin
        rst <= '1'; wait for 200 ns; rst <= '0';
        wait until rising_edge(clk);

        -- =====================================================================
        -- FASE 0: ARRANQUE -> empujón recto al ENCENDER (anti-atasco)
        -- =====================================================================
        sizq <= '0'; sder <= '0';                 -- sensores irrelevantes: el empujón es ciego
        meas <= true; wait for 25 us;             -- mide dentro del empujón (T_ARRANQUE=40us)
        assert ca1 > 100 and cb1 > 100
            report "FALLO arranque: el empujon deberia mover AMBOS motores adelante" severity error;
        assert ca2 = 0 and cb2 = 0
            report "FALLO arranque: el empujon NO deberia ir en reversa" severity error;
        meas <= false;
        report "arranque: empujon recto al encender OK" severity note;
        wait for 50 us;                           -- deja terminar el empujón -> E_SEGUIR

        -- =====================================================================
        -- FASE A: tabla normal (estado estable)
        -- =====================================================================
        sizq <= '1'; sder <= '1'; wait for 2 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert ca1 > 5000 and cb1 > 5000
            report "FALLO (1,1): el robot no avanza" severity error;
        assert (ca1 - cb1) < 8000 and (cb1 - ca1) < 8000
            report "FALLO (1,1): deberia AVANZAR recto" severity error;
        report "(1,1) -> avanzar OK" severity note;

        sizq <= '1'; sder <= '0'; wait for 2 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert cb1 > 5000
            report "FALLO (1,0): la rueda DER deberia empujar adelante" severity error;
        assert ca2 > 5000 and ca1 = 0
            report "FALLO (1,0): la rueda IZQ deberia ir en REVERSA (pivot)" severity error;
        report "(1,0) -> girar izquierda OK" severity note;

        sizq <= '0'; sder <= '1'; wait for 2 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert ca1 > 5000
            report "FALLO (0,1): la rueda IZQ deberia empujar adelante" severity error;
        assert cb2 > 5000 and cb1 = 0
            report "FALLO (0,1): la rueda DER deberia ir en REVERSA (pivot)" severity error;
        report "(0,1) -> girar derecha OK" severity note;

        -- =====================================================================
        -- FASE B: blanco BREVE (< W_ARM) -> AVANZA y NO dispara la zona
        -- =====================================================================
        s0 := n_scan;
        sizq <= '0'; sder <= '0';                 -- doble blanco
        wait for 1 us;
        meas <= true;  wait for 1.6 ms;           -- aún < W_ARM(4ms): debe AVANZAR
        assert ca1 > 5000 and cb1 > 5000
            report "FALLO blanco breve: deberia AVANZAR para cruzar" severity error;
        meas <= false;
        sizq <= '1'; sder <= '1'; wait for 200 us;  -- vuelve a la linea
        assert n_scan = s0
            report "FALLO: blanco breve (<W_ARM) NO deberia disparar la zona" severity error;
        report "blanco breve (<W_ARM) -> avanza, sin zona OK" severity note;

        -- =====================================================================
        -- FASE C: zona -> doble blanco >= W_ARM, espera a AMBOS sensores, dispara
        -- =====================================================================
        s0 := n_scan;
        sizq <= '0'; sder <= '0'; wait for 6 ms;    -- doble blanco 6ms: ARMA (>=4ms, <10ms)
        -- un sensor vuelve primero (0,1): aún NO debe disparar (espera al otro)
        sizq <= '0'; sder <= '1'; wait for 1 ms;
        assert n_scan = s0
            report "FALLO zona: no debe disparar con un solo sensor (espera al otro)" severity error;
        -- ahora vuelven AMBOS (1,1) -> dispara
        sizq <= '1'; sder <= '1'; wait for 200 us;
        assert n_scan > s0
            report "FALLO zona: no disparo start_scan al volver ambos sensores" severity error;
        report "zona: doble blanco >=0.5s + retorno a (1,1) -> start_scan OK" severity note;

        -- Handshake del brazo: barrido y fin.
        scan_active <= '1'; wait for 200 us;
        scan_active <= '0'; arm_ready <= '1'; wait for 200 us;
        arm_ready <= '0';

        sizq <= '1'; sder <= '1'; wait for 50 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert ca1 > 5000 and cb1 > 5000
            report "FALLO zona: no reanudo el seguimiento tras arm_ready" severity error;
        meas <= false;
        report "zona: reanuda seguimiento tras arm_ready OK" severity note;

        -- =====================================================================
        -- FASE C2: zona CON cubo -> DEPÓSITO (trigger_drop, NO start_scan)
        -- =====================================================================
        s0 := n_scan; d0 := n_drop;
        has_object <= '1'; arm_ready <= '1';        -- lleva cubo, brazo en HOLD
        sizq <= '0'; sder <= '0'; wait for 6 ms;    -- doble blanco: ARMA
        sizq <= '1'; sder <= '1'; wait for 100 us;  -- vuelven ambos -> dispara la zona
        assert n_drop > d0
            report "FALLO deposito: no disparo trigger_drop al llegar con cubo" severity error;
        assert n_scan = s0
            report "FALLO deposito: con cubo NO debe escanear (start_scan)" severity error;
        report "zona con cubo -> trigger_drop (deposita) OK" severity note;

        -- Handshake del depósito: el brazo sale de HOLD (arm_ready=0), suelta y vuelve a REST.
        arm_ready <= '0'; wait for 200 us;          -- depósito en curso
        has_object <= '0';                          -- soltó el cubo
        arm_ready <= '1'; wait for 200 us;          -- brazo de vuelta en reposo
        arm_ready <= '0';

        sizq <= '1'; sder <= '1'; wait for 50 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert ca1 > 5000 and cb1 > 5000
            report "FALLO deposito: no reanudo el seguimiento tras depositar" severity error;
        meas <= false;
        report "zona con cubo: deposita y reanuda seguimiento OK" severity note;

        -- =====================================================================
        -- FASE D: doble blanco SOSTENIDO (> W_STOP) -> ALTO FIJO (solo reset)
        -- =====================================================================
        rst <= '1'; sizq <= '1'; sder <= '1'; wait for 200 ns;
        rst <= '0'; wait until rising_edge(clk);
        wait for 1 us;

        sizq <= '0'; sder <= '0'; wait for 11 ms;   -- > W_STOP(10ms): pérdida
        meas <= true;  wait for 1.6 ms;
        assert ca1 = 0 and cb1 = 0 and ca2 = 0 and cb2 = 0
            report "FALLO perdida: doble blanco > 1s deberia DETENERSE" severity error;
        meas <= false;
        -- Alto FIJO: aunque vuelva la línea (1,1), NO debe reanudar.
        sizq <= '1'; sder <= '1'; wait for 50 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert ca1 = 0 and cb1 = 0 and ca2 = 0 and cb2 = 0
            report "FALLO perdida: deberia quedar en ALTO FIJO (no reanuda con la linea)" severity error;
        meas <= false;
        report "doble blanco > W_STOP -> alto fijo (solo sale con reset) OK" severity note;

        -- =====================================================================
        -- FASE E: FALLO DE SENSOR -> E_FALLO (detiene + zona_fallo/LED error)
        -- =====================================================================
        rst <= '1'; sizq <= '1'; sder <= '1'; sensor_err <= '0'; wait for 200 ns;
        rst <= '0'; wait until rising_edge(clk);
        wait for 1 us;

        sizq <= '0'; sder <= '0'; wait for 6 ms;    -- arma
        sizq <= '1'; sder <= '1'; wait for 200 us;  -- dispara
        scan_active <= '1'; wait for 200 us;
        sensor_err  <= '1';                          -- el sensor no respondió
        scan_active <= '0'; arm_ready <= '1'; wait for 200 us;
        arm_ready <= '0';
        assert zona_fallo = '1'
            report "FALLO sensor: deberia entrar en E_FALLO (zona_fallo=1)" severity error;

        sizq <= '1'; sder <= '1'; wait for 50 us;
        meas <= false; wait for 1 us;
        meas <= true;  wait for 1.6 ms;
        assert ca1 = 0 and cb1 = 0 and ca2 = 0 and cb2 = 0
            report "FALLO sensor: el robot deberia quedar DETENIDO en E_FALLO" severity error;
        meas <= false;
        assert zona_fallo = '1'
            report "FALLO sensor: zona_fallo deberia seguir encendido" severity error;
        report "fallo de sensor: robot detenido + zona_fallo (LED error) OK" severity note;

        report "OK: seguidor + zona (1 linea blanca ancha) sigue la especificacion" severity note;
        simdone <= true;
        wait;
    end process;

end sim;
