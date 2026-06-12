-- ============================================================================
-- pivote - Generador de PÍVOT PULSADO POR PASOS (módulo auxiliar de Sísifo).
--   Ejecuta el pívot de las maniobras de zona en PASOS discretos: cada paso =
--   un KICK corto (duty PIVOT_DUTY, duración PASO_CYCLES) + un SETTLE (motores
--   en freno, PIV_SETTLE_CYCLES) tras el cual pulsa 'paso_tick' (ahí el robot
--   está asentado y los sensores filtrados son válidos). Acota el rango: corre
--   hasta PASOS_ZONA (modo='0') o PASOS_LINEA (modo='1') pasos y levanta
--   'fin_pasos'. Pívot lento y repetible sin perder torque (el problema del
--   pívot continuo era que se pasaba de largo y no detectaba la línea fina).
-- ============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity pivote is
    generic (
        PASO_CYCLES       : integer := 60_000;   -- 1) duración del KICK de cada paso (~1.2 ms)
        PIVOT_DUTY        : integer := 42_000;   -- 2) velocidad del pívot (>35000 torque)
        PASOS_ZONA        : integer := 24;       -- 3) cap de pasos en fase ZONA
        PASOS_LINEA       : integer := 40;       -- 4) cap de pasos en fase LÍNEA
        PIV_SETTLE_CYCLES : integer := 60_000    -- settle tras kick (>= FILTRO_CYCLES)
    );
    port (
        clk       : in  std_logic;
        rst       : in  std_logic;                       -- activo alto
        ena       : in  std_logic;                       -- '1' pivotea, '0' inactivo (tgt=0)
        modo      : in  std_logic;                       -- '0'=ZONA, '1'=LÍNEA (selecciona cap)
        dir       : in  std_logic;                       -- '0'=DER, '1'=IZQ
        tgt_l     : out integer range -65535 to 65535;
        tgt_r     : out integer range -65535 to 65535;
        paso_tick : out std_logic;                       -- pulso 1 ciclo al COMPLETAR un paso
        fin_pasos : out std_logic                        -- '1' al alcanzar el cap de pasos
    );
end pivote;

architecture rtl of pivote is
    function imax(a, b : integer) return integer is
    begin
        if a > b then return a; else return b; end if;
    end function;
    constant PASOS_MAX : integer := imax(PASOS_ZONA, PASOS_LINEA);
    constant DUR_MAX   : integer := imax(PASO_CYCLES, PIV_SETTLE_CYCLES);

    type pst_t is (P_IDLE, P_KICK, P_SETTLE, P_DONE);
    signal pst      : pst_t := P_IDLE;
    signal dur_cnt  : integer range 0 to DUR_MAX := 0;       -- kick Y settle (estados exclusivos)
    signal step_cnt : integer range 0 to PASOS_MAX := 0;
    signal tick_r, fin_r : std_logic := '0';
begin
    -- Duty COMBINACIONAL (sin registros): solo hay kick en P_KICK (ahorro de LEs).
    tgt_l <=  PIVOT_DUTY when (pst = P_KICK and dir = '0') else
             -PIVOT_DUTY when (pst = P_KICK and dir = '1') else 0;
    tgt_r <= -PIVOT_DUTY when (pst = P_KICK and dir = '0') else
              PIVOT_DUTY when (pst = P_KICK and dir = '1') else 0;
    paso_tick <= tick_r; fin_pasos <= fin_r;

    process(clk, rst)
        variable cap : integer range 0 to PASOS_MAX;
    begin
        if rst = '1' then
            pst <= P_IDLE; dur_cnt <= 0; step_cnt <= 0; tick_r <= '0'; fin_r <= '0';
        elsif rising_edge(clk) then
            tick_r <= '0';                                   -- pulso por defecto
            if modo = '1' then cap := PASOS_LINEA; else cap := PASOS_ZONA; end if;
            case pst is
                when P_IDLE =>
                    fin_r <= '0'; step_cnt <= 0;
                    if ena = '1' then dur_cnt <= 0; pst <= P_KICK; end if;

                when P_KICK =>
                    if ena = '0' then
                        pst <= P_IDLE;
                    elsif dur_cnt >= PASO_CYCLES-1 then
                        dur_cnt <= 0; pst <= P_SETTLE;
                    else
                        dur_cnt <= dur_cnt + 1;
                    end if;

                when P_SETTLE =>
                    if ena = '0' then
                        pst <= P_IDLE;
                    elsif dur_cnt >= PIV_SETTLE_CYCLES-1 then
                        tick_r <= '1';                       -- paso completado (sensores válidos)
                        if step_cnt >= cap-1 then
                            step_cnt <= step_cnt + 1;
                            fin_r <= '1'; pst <= P_DONE;
                        else
                            step_cnt <= step_cnt + 1;
                            dur_cnt <= 0; pst <= P_KICK;
                        end if;
                    else
                        dur_cnt <= dur_cnt + 1;
                    end if;

                when P_DONE =>
                    fin_r <= '1';
                    if ena = '0' then pst <= P_IDLE; end if;
            end case;
        end if;
    end process;
end rtl;

-- ============================================================================
-- MaquinaEstados - Cerebro de Sísifo (Etapa 2): seguidor de línea de DOS MODOS
--                  + maniobra de ZONA con orquestación del brazo (LIDAR/agarre/
--                  depósito). FPGA Cyclone II EP2C5T144C7 | QRD1114 x2 | L293 | 50 MHz
-- ----------------------------------------------------------------------------
-- PROBLEMA QUE RESUELVE: la navegación de dos modos y la detección de zona
--   comparten el MISMO disparador (doble-negro 1,1). Antes se separaban solo por
--   TIEMPO, pero al pivotear para "confirmar" la zona, los sensores (a 103 mm del
--   eje) salen de la zona (6 cm) con ~16° de giro y el doble-negro se rompe -> la
--   zona nunca se confirmaba. Ahora se discrimina con una SONDA ACTIVA.
--
-- DISCRIMINACIÓN CURVA vs ZONA (E_SONDA): al ver (1,1) se hace un pívot PEQUEÑO
--   configurable (CURVA_PROBE_CYCLES) hacia el último giro:
--     - si aparece BLANCO en ambos sensores (línea delgada de 20 mm: el pívot
--       destapó las orillas) => era CURVA: se descarta zona y se arma un LOCKOUT
--       (ZONA_LOCKOUT_CYCLES) para no re-sondear en una curva sostenida.
--     - si la sonda termina y SIGUE el doble-negro (no hay orillas blancas) =>
--       es ZONA => entra la maniobra.
--
-- MANIOBRA DE ZONA: los PÍVOTS se ejecutan con el módulo 'pivote' (pulsado por
--   PASOS, asentando cada paso para leer sensores limpio; antes el pívot continuo
--   se pasaba de largo y no centraba ni detectaba la línea fina).
--   1) Centrar por BORDES (E_Z_BORDE_A/B, E_Z_CENTRO): pivota a un lado hasta
--      destapar blanco (borde A), al otro midiendo el ancho A..B en pasos y vuelve
--      la mitad. Cap = PASOS_ZONA.
--   2) AVANZA recto hasta que al menos un sensor deje el negro; +OFFSET_CYCLES.
--   3) BUSCA la línea de salida pivotando por pasos a un lado y al otro
--      (E_Z_BUSCA_A/B); RECENTRA (E_Z_RECENTRO) hasta que la línea quede ENTRE
--      ambos sensores. Cap = PASOS_LINEA.
--   4) ACCIÓN al final: sin objeto -> start_scan (LIDAR+agarre); con objeto ->
--      trigger_drop (depósito). Espera a que el brazo termine y reanuda.
--
-- HANDSHAKE BRAZO (grab_ctrl/LIDAR): start_scan/trigger_drop son PULSOS de 1
--   ciclo. arm_ready YA está en '1' al disparar el escaneo, y scan_active sube 1
--   ciclo DESPUÉS -> por eso E_SCAN_INI espera scan_active='1' antes de que
--   E_SCAN_FIN espere arm_ready='1' (evita saltarse el escaneo). El depósito
--   espera has_object='0' (único "terminé de soltar").
--
-- Motores (L293): tgt con signo -> + adelante (INx1=PWM), - reversa (INx2=PWM),
--   0 freno. PWM 16 bits (~763 Hz), duty = |tgt_*| / 65536.
-- ANTI-RUIDO: filtro de histéresis (FILTRO_CYCLES) -> s_izq/s_der ('1'=NEGRO).
-- NOTA: nivel del sensor SOBRE la línea = LINE_LVL (en este robot '0').
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity MaquinaEstados is
    generic (
        FILTRO_CYCLES : integer := 50_000;      -- antirrebote de sensores (~1 ms @50MHz)
        LINE_LVL      : std_logic := '0'        -- nivel del sensor SOBRE la línea
    );
    port (
        clk          : in  std_logic;
        rst          : in  std_logic;          -- activo alto
        sensor_izq   : in  std_logic;
        sensor_der   : in  std_logic;
        -- motores (PWM directo en las entradas del L293)
        motor_a1     : out std_logic;          -- motor IZQ adelante
        motor_a2     : out std_logic;          -- motor IZQ reversa
        motor_b1     : out std_logic;          -- motor DER adelante
        motor_b2     : out std_logic;          -- motor DER reversa
        -- interfaz con el brazo/LIDAR
        start_scan   : out std_logic;          -- pulso: dispara el barrido
        trigger_drop : out std_logic;          -- pulso: deposita el objeto
        scan_active  : in  std_logic;          -- '1' mientras el LIDAR barre
        arm_ready    : in  std_logic;          -- '1' cuando el brazo está libre/HOLD
        has_object   : in  std_logic;          -- '1' si lleva el cubo
        -- LEDs
        led_estado   : out std_logic;          -- parpadeo 1 Hz
        led_error    : out std_logic
    );
end MaquinaEstados;

architecture rtl of MaquinaEstados is

    -- ===== CALIBRACIÓN NAVEGACIÓN (duty sobre 65536) ==========================
    -- Geometría: vía b=70 mm, sensores 103 mm ADELANTE del eje, straddle (línea
    -- de 20 mm ENTRE los QRD). recto-recto: piso de torque (no baja de ~35000).
    constant DUTY_IZQ_RECTO  : integer := 35000;
    constant DUTY_DER_RECTO  : integer := 35000;
    -- MODO RECTA (un solo sensor en negro): exterior empuja, interior se APAGA.
    constant DUTY_RECTA_EXT  : integer := 45000;
    constant DUTY_RECTA_INT  : integer := 0;
    -- MODO CURVA (doble negro): exterior adelante + interior en REVERSA (pívot).
    constant DUTY_CURVA_EXT  : integer := 49000;
    constant DUTY_CURVA_INT  : integer := 49000;
    -- Pívot PULSADO de modo curva (kick reversa + descanso arco).
    constant CURVA_PULSO_CYCLES : integer := 1_000_000;   -- KICK   (~20 ms)
    constant CURVA_PAUSA_CYCLES : integer := 500_000;     -- descanso (~10 ms)
    constant CURVA_PERIODO      : integer := CURVA_PULSO_CYCLES + CURVA_PAUSA_CYCLES;

    -- Habilita la maniobra de zona. false = solo sigue la línea (probar el
    -- seguidor aislado: en (1,1) hace pívot de curva y NUNCA entra a la maniobra).
    constant USAR_ZONA       : boolean := true;
    -- Estilo de (1,1) cuando NO se sondea (lockout/zona off): true = pívot pulsado.
    constant USAR_MODO_CURVA : boolean := true;

    -- ===== DISCRIMINACIÓN CURVA vs ZONA (sonda) ===============================
    -- PERILLA CRÍTICA: corto para no pasar de ~16° en una zona (se perdería el
    -- cuadro y se leería como curva); largo para que en una línea de 20 mm el
    -- pívot SÍ destape blanco. ~120 ms de arranque. (50_000 ciclos = 1 ms.)
    constant CURVA_PROBE_CYCLES  : integer := 6_000_000;
    -- Tras confirmar CURVA, no re-sondear durante una curva sostenida (~0.8 s).
    constant ZONA_LOCKOUT_CYCLES : integer := 25_000_000;

    -- ===== MANIOBRA DE ZONA ===================================================
    -- Avances rectos (>= 35000 piso de torque); van por TIEMPO.
    constant DUTY_Z_AVANCE : integer := 45000;            -- avance recto en zona
    constant MAX_AVANCE_CYCLES   : integer := 16_000_000; -- ~0.32 s watchdog de avance
    constant OFFSET_CYCLES       : integer := 4_000_000;  -- ~80 ms avance extra
    constant MAX_BUSCA_TRIES     : integer := 3;          -- reintentos del barrido de línea
    constant REARME_CYCLES       : integer := 25_000_000; -- ~0.5 s recto despejando el cuadro

    -- ===== 4 PERILLAS DEL PÍVOT POR PASOS (módulo 'pivote') ===================
    constant PIV_PASO_CYCLES   : integer := 60_000;   -- 1) duración de cada paso (kick, ~1.2 ms)
    constant PIV_DUTY          : integer := 20_000;   -- 2) velocidad del pívot
    constant PIV_PASOS_ZONA    : integer := 16;       -- 3) nº de pasos en detección de ZONA
    constant PIV_PASOS_LINEA   : integer := 20;       -- 4) nº de pasos en detección de LÍNEA
    constant PIV_SETTLE_CYCLES : integer := 60_000;   -- asentamiento por paso (>= FILTRO_CYCLES)

    function imax(a, b : integer) return integer is
    begin
        if a > b then return a; else return b; end if;
    end function;
    constant LOCKOUT_MAX : integer := imax(ZONA_LOCKOUT_CYCLES, REARME_CYCLES);
    constant EDGE_MAX    : integer := imax(MAX_AVANCE_CYCLES, OFFSET_CYCLES);
    constant PASOS_MAX   : integer := imax(PIV_PASOS_ZONA, PIV_PASOS_LINEA);

    -- Componente: pívot pulsado por pasos
    component pivote
        generic (
            PASO_CYCLES       : integer;
            PIVOT_DUTY        : integer;
            PASOS_ZONA        : integer;
            PASOS_LINEA       : integer;
            PIV_SETTLE_CYCLES : integer
        );
        port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            ena       : in  std_logic;
            modo      : in  std_logic;
            dir       : in  std_logic;
            tgt_l     : out integer range -65535 to 65535;
            tgt_r     : out integer range -65535 to 65535;
            paso_tick : out std_logic;
            fin_pasos : out std_logic
        );
    end component;

    -- Sensores: filtro de histéresis -> '1' = sobre línea (NEGRO). REGISTRADOS
    -- (1 ciclo de retraso); usar s_izq/s_der en condiciones, no derivados.
    signal flt_izq, flt_der : integer range 0 to FILTRO_CYCLES := 0;
    signal s_izq, s_der     : std_logic := '0';

    -- Duty objetivo por rueda CON SIGNO: + adelante, - reversa, 0 freno.
    signal tgt_l, tgt_r : integer range -65535 to 65535 := 0;

    -- Último giro single-sensor: da la dirección del pívot/keep-last y de la sonda.
    type giro_t is (G_RECTO, G_IZQ, G_DER);
    signal ultimo_giro : giro_t := G_RECTO;

    -- Dirección CONCRETA de pívot durante la maniobra (nunca "recto").
    type dir_t is (D_IZQ, D_DER);
    signal probe_dir : dir_t := D_DER;   -- "lado A" del centrado y del barrido de re-enganche

    -- Fase del pívot pulsado de modo curva (corre mientras dure el doble-negro).
    signal curva_cnt : integer range 0 to CURVA_PERIODO := 0;

    -- PWM 16 bits libre (~763 Hz).
    signal pwm16 : unsigned(15 downto 0) := (others => '0');

    -- Contadores de discriminación / maniobra (por tiempo)
    signal lockout_cnt : integer range 0 to LOCKOUT_MAX := 0;  -- cuenta-abajo
    -- tcnt: temporizador COMPARTIDO. La sonda (probe) y AVANZA/OFFSET (avance) nunca
    -- corren a la vez -> un solo contador ancho en vez de dos (ahorro de LEs).
    signal tcnt        : integer range 0 to EDGE_MAX := 0;
    signal busca_tries : integer range 0 to MAX_BUSCA_TRIES := 0;
    signal reentro     : std_logic := '0';   -- en BORDE_B: ya se re-vio el negro doble

    -- Contadores de PASOS de la maniobra (pívot por pasos). paso_n se reusa para el
    -- ancho A..B en BORDE_B (no se solapa con su uso en CENTRO/RECENTRO).
    signal paso_n   : integer range 0 to PASOS_MAX := 0;
    signal centro_n : integer range 0 to PASOS_MAX := 0;

    -- Interfaz con el módulo 'pivote'
    signal piv_ena, piv_modo, piv_dir : std_logic := '0';   -- driven SOLO por el proceso fsm
    signal piv_tgt_l, piv_tgt_r       : integer range -65535 to 65535 := 0;
    signal piv_paso_tick, piv_fin     : std_logic := '0';

    -- FSM
    type est_t is (E_INICIO, E_SEGUIR, E_SONDA,
                   E_Z_BORDE_A, E_Z_BORDE_B, E_Z_CENTRO,
                   E_Z_AVANZA, E_Z_OFFSET,
                   E_Z_BUSCA_A, E_Z_BUSCA_B, E_Z_RECENTRO,
                   E_Z_ACCION, E_SCAN_INI, E_SCAN_FIN, E_DROP_WAIT, E_REARME);
    signal est : est_t := E_INICIO;

    signal start_scan_r   : std_logic := '0';
    signal trigger_drop_r : std_logic := '0';

    -- 1 Hz
    signal clk_1s : std_logic := '0';
    signal cnt_1s : integer range 0 to 16_666_665 := 0;   -- LED de vida ~1.5 Hz (24 bits, ahorro LEs)

begin

    -- =========================================================================
    -- Módulo de pívot pulsado por pasos (las 4 perillas)
    -- =========================================================================
    u_piv : pivote
        generic map (
            PASO_CYCLES       => PIV_PASO_CYCLES,
            PIVOT_DUTY        => PIV_DUTY,
            PASOS_ZONA        => PIV_PASOS_ZONA,
            PASOS_LINEA       => PIV_PASOS_LINEA,
            PIV_SETTLE_CYCLES => PIV_SETTLE_CYCLES
        )
        port map (
            clk => clk, rst => rst, ena => piv_ena, modo => piv_modo, dir => piv_dir,
            tgt_l => piv_tgt_l, tgt_r => piv_tgt_r,
            paso_tick => piv_paso_tick, fin_pasos => piv_fin
        );

    -- =========================================================================
    -- Generador 1 Hz (LED de vida)
    -- =========================================================================
    p_1hz : process(clk, rst)
    begin
        if rst = '1' then
            cnt_1s <= 0; clk_1s <= '0';
        elsif rising_edge(clk) then
            if cnt_1s = 16_666_665 then
                cnt_1s <= 0; clk_1s <= not clk_1s;
            else
                cnt_1s <= cnt_1s + 1;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Generador PWM (16 bits, libre). Fija el duty de marcha = |tgt_*| / 65536.
    -- =========================================================================
    p_pwm : process(clk, rst)
    begin
        if rst = '1' then
            pwm16 <= (others => '0');
        elsif rising_edge(clk) then
            pwm16 <= pwm16 + 1;
        end if;
    end process;

    -- tgt > 0: adelante (INx1=PWM); tgt < 0: reversa (INx2=PWM); tgt = 0: freno
    motor_a1 <= '1' when (tgt_l > 0 and to_integer(pwm16) <  tgt_l) else '0';
    motor_a2 <= '1' when (tgt_l < 0 and to_integer(pwm16) < -tgt_l) else '0';
    motor_b1 <= '1' when (tgt_r > 0 and to_integer(pwm16) <  tgt_r) else '0';
    motor_b2 <= '1' when (tgt_r < 0 and to_integer(pwm16) < -tgt_r) else '0';

    -- =========================================================================
    -- FSM principal: seguidor 2 modos + sonda + maniobra de zona + brazo
    -- =========================================================================
    fsm : process(clk, rst)
    begin
        if rst = '1' then
            est <= E_INICIO;
            tgt_l <= 0; tgt_r <= 0;
            ultimo_giro <= G_RECTO;
            curva_cnt <= 0;
            flt_izq <= 0; flt_der <= 0;
            s_izq <= '0'; s_der <= '0';
            probe_dir <= D_DER;
            tcnt <= 0; lockout_cnt <= 0;
            busca_tries <= 0; reentro <= '0';
            paso_n <= 0; paso_n <= 0; centro_n <= 0;
            piv_ena <= '0'; piv_modo <= '0'; piv_dir <= '0';
            start_scan_r <= '0'; trigger_drop_r <= '0';
        elsif rising_edge(clk) then
            -- pulsos por defecto a '0' (solo E_Z_ACCION los pone en '1' por 1 ciclo)
            start_scan_r   <= '0';
            trigger_drop_r <= '0';
            -- pívot APAGADO por defecto; los estados de pívot lo levantan a '1'
            piv_ena        <= '0';

            -- -------- Filtro de histéresis de sensores (antirrebote) ----------
            if sensor_izq = LINE_LVL then
                if flt_izq < FILTRO_CYCLES then flt_izq <= flt_izq + 1; end if;
            else
                if flt_izq > 0 then flt_izq <= flt_izq - 1; end if;
            end if;
            if flt_izq = FILTRO_CYCLES then s_izq <= '1';
            elsif flt_izq = 0       then s_izq <= '0'; end if;

            if sensor_der = LINE_LVL then
                if flt_der < FILTRO_CYCLES then flt_der <= flt_der + 1; end if;
            else
                if flt_der > 0 then flt_der <= flt_der - 1; end if;
            end if;
            if flt_der = FILTRO_CYCLES then s_der <= '1';
            elsif flt_der = 0       then s_der <= '0'; end if;

            -- fase del pívot pulsado de curva, mientras dure el doble-negro
            if s_izq = '1' and s_der = '1' then
                if curva_cnt >= CURVA_PERIODO - 1 then curva_cnt <= 0;
                else curva_cnt <= curva_cnt + 1; end if;
            else
                curva_cnt <= 0;
            end if;

            -- lockout (curva sostenida / re-arme de zona): cuenta-abajo. Si un
            -- estado lo re-arma el mismo ciclo, esa asignación (posterior) gana
            -- porque aquí lockout_cnt vale 0 y este 'if' no asigna.
            if lockout_cnt > 0 then
                lockout_cnt <= lockout_cnt - 1;
            end if;

            case est is

                when E_INICIO =>
                    tgt_l <= 0; tgt_r <= 0;
                    est <= E_SEGUIR;

                -- ---- Seguir la línea (2 modos) + gate de sonda en doble-negro ----
                when E_SEGUIR =>
                    if s_izq = '0' and s_der = '0' then        -- ambos blanco -> recto
                        ultimo_giro <= G_RECTO;
                        tgt_l <= DUTY_IZQ_RECTO; tgt_r <= DUTY_DER_RECTO;

                    elsif s_izq = '0' and s_der = '1' then     -- DER único -> recta der
                        ultimo_giro <= G_DER;
                        tgt_l <= DUTY_RECTA_EXT; tgt_r <= DUTY_RECTA_INT;

                    elsif s_izq = '1' and s_der = '0' then     -- IZQ único -> recta izq
                        ultimo_giro <= G_IZQ;
                        tgt_l <= DUTY_RECTA_INT; tgt_r <= DUTY_RECTA_EXT;

                    else
                        -- (1,1) DOBLE NEGRO: ¿sonda de zona o pívot de curva?
                        if USAR_ZONA and lockout_cnt = 0 then
                            -- arranca la SONDA hacia el último giro (mantiene motion)
                            case ultimo_giro is
                                when G_DER  => probe_dir <= D_DER;
                                               tgt_l <= DUTY_CURVA_EXT;  tgt_r <= -DUTY_CURVA_INT;
                                when G_IZQ  => probe_dir <= D_IZQ;
                                               tgt_l <= -DUTY_CURVA_INT; tgt_r <= DUTY_CURVA_EXT;
                                when others => probe_dir <= D_DER;       -- zona de frente
                                               tgt_l <= DUTY_CURVA_EXT;  tgt_r <= -DUTY_CURVA_INT;
                            end case;
                            tcnt <= 0;
                            est <= E_SONDA;
                        else
                            -- lockout activo (curva sostenida) o zona off: pívot keep-last
                            if USAR_MODO_CURVA then
                                case ultimo_giro is
                                    when G_RECTO =>
                                        tgt_l <= DUTY_IZQ_RECTO; tgt_r <= DUTY_DER_RECTO;
                                    when G_DER =>
                                        if curva_cnt < CURVA_PULSO_CYCLES then
                                            tgt_l <= DUTY_CURVA_EXT; tgt_r <= -DUTY_CURVA_INT;
                                        else
                                            tgt_l <= DUTY_RECTA_EXT; tgt_r <= DUTY_RECTA_INT;
                                        end if;
                                    when G_IZQ =>
                                        if curva_cnt < CURVA_PULSO_CYCLES then
                                            tgt_l <= -DUTY_CURVA_INT; tgt_r <= DUTY_CURVA_EXT;
                                        else
                                            tgt_l <= DUTY_RECTA_INT; tgt_r <= DUTY_RECTA_EXT;
                                        end if;
                                end case;
                            else
                                case ultimo_giro is
                                    when G_RECTO => tgt_l <= DUTY_IZQ_RECTO; tgt_r <= DUTY_DER_RECTO;
                                    when G_DER   => tgt_l <= DUTY_RECTA_EXT; tgt_r <= DUTY_RECTA_INT;
                                    when G_IZQ   => tgt_l <= DUTY_RECTA_INT; tgt_r <= DUTY_RECTA_EXT;
                                end case;
                            end if;
                        end if;
                    end if;

                -- ---- SONDA: pívot pequeño que mide el ancho del negro ----
                when E_SONDA =>
                    if probe_dir = D_DER then
                        tgt_l <= DUTY_CURVA_EXT;  tgt_r <= -DUTY_CURVA_INT;
                    else
                        tgt_l <= -DUTY_CURVA_INT; tgt_r <= DUTY_CURVA_EXT;
                    end if;
                    if s_izq = '0' and s_der = '0' then
                        -- apareció blanco => línea delgada => era CURVA
                        if probe_dir = D_DER then ultimo_giro <= G_DER;
                        else ultimo_giro <= G_IZQ; end if;
                        lockout_cnt <= ZONA_LOCKOUT_CYCLES;
                        est <= E_SEGUIR;
                    elsif tcnt >= CURVA_PROBE_CYCLES then
                        -- sigue doble-negro => es ZONA
                        paso_n <= 0; reentro <= '0';
                        est <= E_Z_BORDE_A;
                    else
                        tcnt <= tcnt + 1;
                    end if;

                -- ---- Centrado por bordes: borde A (lado del último giro) ----
                when E_Z_BORDE_A =>
                    piv_ena <= '1'; piv_modo <= '0';
                    if probe_dir = D_DER then piv_dir <= '0'; else piv_dir <= '1'; end if;
                    tgt_l <= piv_tgt_l; tgt_r <= piv_tgt_r;
                    if piv_paso_tick = '1' then
                        if (s_izq = '0' and s_der = '0') then           -- borde A
                            piv_ena <= '0'; paso_n <= 0; paso_n <= 0; reentro <= '0';
                            est <= E_Z_BORDE_B;
                        elsif piv_fin = '1' then                        -- fallback: no halló borde
                            piv_ena <= '0'; paso_n <= 0; paso_n <= 0; reentro <= '0';
                            est <= E_Z_BORDE_B;
                        end if;
                    end if;

                -- ---- Centrado por bordes: borde B (opuesto), mide el ancho en pasos ----
                when E_Z_BORDE_B =>
                    piv_ena <= '1'; piv_modo <= '0';
                    if probe_dir = D_DER then piv_dir <= '1'; else piv_dir <= '0'; end if;
                    tgt_l <= piv_tgt_l; tgt_r <= piv_tgt_r;
                    if piv_paso_tick = '1' then
                        if reentro = '0' then
                            if (s_izq = '1' and s_der = '1') then       -- re-entró al doble-negro
                                reentro <= '1'; paso_n <= 0;
                            elsif piv_fin = '1' then                    -- nunca re-vio negro: fallback
                                piv_ena <= '0'; centro_n <= paso_n / 2; paso_n <= 0;
                                est <= E_Z_CENTRO;
                            end if;
                        else
                            if (s_izq = '0' and s_der = '0') then       -- borde B
                                piv_ena <= '0'; centro_n <= paso_n / 2; paso_n <= 0;
                                est <= E_Z_CENTRO;
                            elsif piv_fin = '1' then
                                piv_ena <= '0'; centro_n <= paso_n / 2; paso_n <= 0;
                                est <= E_Z_CENTRO;
                            else
                                paso_n <= paso_n + 1;
                            end if;
                        end if;
                    end if;

                -- ---- Volver al centro (de regreso hacia el lado A) ----
                when E_Z_CENTRO =>
                    piv_ena <= '1'; piv_modo <= '0';
                    if probe_dir = D_DER then piv_dir <= '0'; else piv_dir <= '1'; end if;
                    tgt_l <= piv_tgt_l; tgt_r <= piv_tgt_r;
                    if piv_paso_tick = '1' then
                        if paso_n >= centro_n then
                            piv_ena <= '0'; paso_n <= 0; tcnt <= 0; est <= E_Z_AVANZA;
                        elsif piv_fin = '1' then
                            piv_ena <= '0'; paso_n <= 0; tcnt <= 0; est <= E_Z_AVANZA;
                        else
                            paso_n <= paso_n + 1;
                        end if;
                    end if;

                -- ---- Avanzar recto hasta que al menos un sensor deje el negro ----
                when E_Z_AVANZA =>
                    tgt_l <= DUTY_Z_AVANCE; tgt_r <= DUTY_Z_AVANCE;
                    if (s_izq = '0' or s_der = '0') or tcnt >= MAX_AVANCE_CYCLES then
                        tcnt <= 0; est <= E_Z_OFFSET;
                    else
                        tcnt <= tcnt + 1;
                    end if;

                -- ---- Avance extra (offset) ----
                when E_Z_OFFSET =>
                    tgt_l <= DUTY_Z_AVANCE; tgt_r <= DUTY_Z_AVANCE;
                    if tcnt >= OFFSET_CYCLES then
                        tcnt <= 0; busca_tries <= 0; paso_n <= 0;
                        est <= E_Z_BUSCA_A;
                    else
                        tcnt <= tcnt + 1;
                    end if;

                -- ---- Buscar la línea de salida: barrido lado A (pívot por pasos) ----
                when E_Z_BUSCA_A =>
                    piv_ena <= '1'; piv_modo <= '1';
                    if probe_dir = D_DER then piv_dir <= '0'; else piv_dir <= '1'; end if;
                    tgt_l <= piv_tgt_l; tgt_r <= piv_tgt_r;
                    if piv_paso_tick = '1' then
                        if (s_izq = '1' or s_der = '1') then            -- vio la línea
                            piv_ena <= '0'; paso_n <= 0; est <= E_Z_RECENTRO;
                        elsif piv_fin = '1' then
                            piv_ena <= '0'; paso_n <= 0; est <= E_Z_BUSCA_B;
                        end if;
                    end if;

                -- ---- Buscar la línea de salida: barrido lado B ----
                when E_Z_BUSCA_B =>
                    piv_ena <= '1'; piv_modo <= '1';
                    if probe_dir = D_DER then piv_dir <= '1'; else piv_dir <= '0'; end if;
                    tgt_l <= piv_tgt_l; tgt_r <= piv_tgt_r;
                    if piv_paso_tick = '1' then
                        if (s_izq = '1' or s_der = '1') then
                            piv_ena <= '0'; paso_n <= 0; est <= E_Z_RECENTRO;
                        elsif piv_fin = '1' then
                            piv_ena <= '0'; paso_n <= 0;
                            if busca_tries >= MAX_BUSCA_TRIES then
                                est <= E_Z_RECENTRO;                   -- se rinde, asume centrado
                            else
                                busca_tries <= busca_tries + 1;
                                est <= E_Z_BUSCA_A;
                            end if;
                        end if;
                    end if;

                -- ---- Recentrar: línea entre ambos sensores otra vez ----
                when E_Z_RECENTRO =>
                    piv_ena <= '1'; piv_modo <= '1';
                    tgt_l <= piv_tgt_l; tgt_r <= piv_tgt_r;
                    if piv_paso_tick = '1' then
                        if (s_izq = '0' and s_der = '0') then           -- línea ENTRE ambos
                            piv_ena <= '0'; paso_n <= 0; tcnt <= 0; est <= E_Z_ACCION;
                        elsif paso_n >= PIV_PASOS_LINEA - 1 then         -- tope de pasos
                            piv_ena <= '0'; paso_n <= 0; tcnt <= 0; est <= E_Z_ACCION;
                        else
                            paso_n <= paso_n + 1;
                            if s_der = '1' and s_izq = '0' then piv_dir <= '0';      -- corrige der
                            elsif s_izq = '1' and s_der = '0' then piv_dir <= '1';   -- corrige izq
                            end if;
                        end if;
                    end if;

                -- ---- Acción del brazo (al final de la maniobra) ----
                when E_Z_ACCION =>
                    tgt_l <= 0; tgt_r <= 0;
                    if has_object = '0' then
                        start_scan_r <= '1';              -- busca + agarra (LIDAR)
                        est <= E_SCAN_INI;
                    else
                        trigger_drop_r <= '1';            -- deposita
                        est <= E_DROP_WAIT;
                    end if;

                when E_SCAN_INI =>                         -- espera que ARRANQUE el barrido
                    tgt_l <= 0; tgt_r <= 0;
                    if scan_active = '1' then
                        est <= E_SCAN_FIN;
                    end if;

                when E_SCAN_FIN =>                         -- espera que el brazo TERMINE
                    tgt_l <= 0; tgt_r <= 0;
                    if arm_ready = '1' and scan_active = '0' then
                        lockout_cnt <= REARME_CYCLES;
                        est <= E_REARME;
                    end if;

                when E_DROP_WAIT =>                        -- espera a que suelte el cubo
                    tgt_l <= 0; tgt_r <= 0;
                    if has_object = '0' then
                        lockout_cnt <= REARME_CYCLES;
                        est <= E_REARME;
                    end if;

                -- ---- Re-arme: avanza recto despejando el cuadro, sin re-sondear ----
                when E_REARME =>
                    tgt_l <= DUTY_Z_AVANCE; tgt_r <= DUTY_Z_AVANCE;
                    if lockout_cnt = 0 then
                        ultimo_giro <= G_RECTO;
                        est <= E_SEGUIR;
                    end if;

            end case;
        end if;
    end process;

    start_scan   <= start_scan_r;
    trigger_drop <= trigger_drop_r;
    led_estado   <= clk_1s;
    led_error    <= '0';

end rtl;
