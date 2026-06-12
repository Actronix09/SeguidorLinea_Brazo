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
-- MANIOBRA DE ZONA (todo en lazo abierto, por tiempo/pulsos, sin encoders):
--   1) Centrar por BORDES con rango limitado: pivota a un lado hasta salir del
--      negro (borde A), luego al otro midiendo el ancho (borde B) y vuelve al
--      punto medio. Topes de tiempo (MAX_BORDE_CYCLES) por si no halla un borde.
--   2) AVANZA recto hasta que al menos un sensor deje el negro; +OFFSET_CYCLES.
--   3) BUSCA la línea de salida pivotando un cap a cada lado; RECENTRA (línea
--      entre ambos sensores).
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
    constant ZONA_LOCKOUT_CYCLES : integer := 40_000_000;

    -- ===== MANIOBRA DE ZONA (lazo abierto) ====================================
    -- Duties >= 35000 (piso de torque). Reutiliza la perilla de curva para pivotar.
    constant DUTY_Z_PIVOT  : integer := DUTY_CURVA_EXT;   -- pívot de maniobra (56000)
    constant DUTY_Z_AVANCE : integer := 45000;            -- avance recto en zona
    constant MAX_BORDE_CYCLES    : integer := 8_000_000;  -- ~160 ms tope por borde
    constant MAX_AVANCE_CYCLES   : integer := 30_000_000; -- ~0.6 s watchdog de avance
    constant OFFSET_CYCLES       : integer := 4_000_000;  -- ~80 ms avance extra
    constant BUSCA_CAP_CYCLES    : integer := 5_000_000;  -- ~100 ms por lado del barrido
    constant RECENTRO_CAP_CYCLES : integer := 6_000_000;  -- ~120 ms tope recentrado
    constant MAX_BUSCA_TRIES     : integer := 3;          -- reintentos del barrido
    constant REARME_CYCLES       : integer := 25_000_000; -- ~0.5 s recto despejando el cuadro

    function imax(a, b : integer) return integer is
    begin
        if a > b then return a; else return b; end if;
    end function;
    constant LOCKOUT_MAX : integer := imax(ZONA_LOCKOUT_CYCLES, REARME_CYCLES);
    constant EDGE_MAX    : integer := imax(imax(imax(2*MAX_BORDE_CYCLES, MAX_AVANCE_CYCLES),
                                                imax(OFFSET_CYCLES, 2*BUSCA_CAP_CYCLES)),
                                           RECENTRO_CAP_CYCLES);

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
    signal probe_dir : dir_t := D_DER;   -- "lado A" del centrado
    signal busca_dir : dir_t := D_DER;   -- primer lado del barrido de re-enganche

    -- Fase del pívot pulsado de modo curva (corre mientras dure el doble-negro).
    signal curva_cnt : integer range 0 to CURVA_PERIODO := 0;

    -- PWM 16 bits libre (~763 Hz).
    signal pwm16 : unsigned(15 downto 0) := (others => '0');

    -- Contadores de discriminación / maniobra
    signal probe_cnt   : integer range 0 to CURVA_PROBE_CYCLES := 0;
    signal lockout_cnt : integer range 0 to LOCKOUT_MAX := 0;  -- cuenta-abajo
    signal edge_cnt    : integer range 0 to EDGE_MAX := 0;     -- reusado en la maniobra
    signal centro_cnt  : integer range 0 to MAX_BORDE_CYCLES := 0;
    signal busca_tries : integer range 0 to MAX_BUSCA_TRIES := 0;
    signal reentro     : std_logic := '0';   -- en BORDE_B: ya se re-vio el negro doble

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
    signal cnt_1s : integer range 0 to 24_999_999 := 0;

begin

    -- =========================================================================
    -- Generador 1 Hz (LED de vida)
    -- =========================================================================
    p_1hz : process(clk, rst)
    begin
        if rst = '1' then
            cnt_1s <= 0; clk_1s <= '0';
        elsif rising_edge(clk) then
            if cnt_1s = 24_999_999 then
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
            probe_dir <= D_DER; busca_dir <= D_DER;
            probe_cnt <= 0; lockout_cnt <= 0; edge_cnt <= 0;
            centro_cnt <= 0; busca_tries <= 0; reentro <= '0';
            start_scan_r <= '0'; trigger_drop_r <= '0';
        elsif rising_edge(clk) then
            -- pulsos por defecto a '0' (solo E_Z_ACCION los pone en '1' por 1 ciclo)
            start_scan_r   <= '0';
            trigger_drop_r <= '0';

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
                            probe_cnt <= 0;
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
                    elsif probe_cnt >= CURVA_PROBE_CYCLES then
                        -- sigue doble-negro => es ZONA
                        edge_cnt <= 0; reentro <= '0';
                        est <= E_Z_BORDE_A;
                    else
                        probe_cnt <= probe_cnt + 1;
                    end if;

                -- ---- Centrado por bordes: borde A (lado del último giro) ----
                when E_Z_BORDE_A =>
                    if probe_dir = D_DER then
                        tgt_l <= DUTY_Z_PIVOT;  tgt_r <= -DUTY_Z_PIVOT;
                    else
                        tgt_l <= -DUTY_Z_PIVOT; tgt_r <= DUTY_Z_PIVOT;
                    end if;
                    if (s_izq = '0' and s_der = '0') or edge_cnt >= MAX_BORDE_CYCLES then
                        edge_cnt <= 0; reentro <= '0'; est <= E_Z_BORDE_B;
                    else
                        edge_cnt <= edge_cnt + 1;
                    end if;

                -- ---- Centrado por bordes: borde B (lado opuesto), mide el ancho ----
                when E_Z_BORDE_B =>
                    if probe_dir = D_DER then               -- B = izquierda
                        tgt_l <= -DUTY_Z_PIVOT; tgt_r <= DUTY_Z_PIVOT;
                    else                                    -- B = derecha
                        tgt_l <= DUTY_Z_PIVOT;  tgt_r <= -DUTY_Z_PIVOT;
                    end if;
                    if reentro = '0' then
                        -- aún saliendo del borde A: espera re-entrar al negro doble
                        if s_izq = '1' and s_der = '1' then reentro <= '1'; end if;
                        if edge_cnt >= 2*MAX_BORDE_CYCLES then  -- nunca re-vio negro: fallback
                            centro_cnt <= edge_cnt / 2; edge_cnt <= 0; est <= E_Z_CENTRO;
                        else
                            edge_cnt <= edge_cnt + 1;
                        end if;
                    else
                        -- ya cruzando el negro hacia el borde B
                        if (s_izq = '0' and s_der = '0') or edge_cnt >= 2*MAX_BORDE_CYCLES then
                            centro_cnt <= edge_cnt / 2;   -- punto medio borde A..B
                            edge_cnt <= 0; est <= E_Z_CENTRO;
                        else
                            edge_cnt <= edge_cnt + 1;
                        end if;
                    end if;

                -- ---- Volver al centro (de regreso hacia el lado A) ----
                when E_Z_CENTRO =>
                    if probe_dir = D_DER then
                        tgt_l <= DUTY_Z_PIVOT;  tgt_r <= -DUTY_Z_PIVOT;
                    else
                        tgt_l <= -DUTY_Z_PIVOT; tgt_r <= DUTY_Z_PIVOT;
                    end if;
                    if edge_cnt >= centro_cnt then
                        edge_cnt <= 0; est <= E_Z_AVANZA;
                    else
                        edge_cnt <= edge_cnt + 1;
                    end if;

                -- ---- Avanzar recto hasta que al menos un sensor deje el negro ----
                when E_Z_AVANZA =>
                    tgt_l <= DUTY_Z_AVANCE; tgt_r <= DUTY_Z_AVANCE;
                    if (s_izq = '0' or s_der = '0') or edge_cnt >= MAX_AVANCE_CYCLES then
                        edge_cnt <= 0; est <= E_Z_OFFSET;
                    else
                        edge_cnt <= edge_cnt + 1;
                    end if;

                -- ---- Avance extra (offset) ----
                when E_Z_OFFSET =>
                    tgt_l <= DUTY_Z_AVANCE; tgt_r <= DUTY_Z_AVANCE;
                    if edge_cnt >= OFFSET_CYCLES then
                        edge_cnt <= 0; busca_tries <= 0; busca_dir <= probe_dir;
                        est <= E_Z_BUSCA_A;
                    else
                        edge_cnt <= edge_cnt + 1;
                    end if;

                -- ---- Buscar la línea de salida: barrido lado A ----
                when E_Z_BUSCA_A =>
                    if busca_dir = D_DER then
                        tgt_l <= DUTY_Z_PIVOT;  tgt_r <= -DUTY_Z_PIVOT;
                    else
                        tgt_l <= -DUTY_Z_PIVOT; tgt_r <= DUTY_Z_PIVOT;
                    end if;
                    if s_izq = '1' or s_der = '1' then        -- vio la línea
                        edge_cnt <= 0; est <= E_Z_RECENTRO;
                    elsif edge_cnt >= BUSCA_CAP_CYCLES then
                        edge_cnt <= 0; est <= E_Z_BUSCA_B;
                    else
                        edge_cnt <= edge_cnt + 1;
                    end if;

                -- ---- Buscar la línea de salida: barrido lado B (el doble) ----
                when E_Z_BUSCA_B =>
                    if busca_dir = D_DER then                 -- opuesto = izquierda
                        tgt_l <= -DUTY_Z_PIVOT; tgt_r <= DUTY_Z_PIVOT;
                    else
                        tgt_l <= DUTY_Z_PIVOT;  tgt_r <= -DUTY_Z_PIVOT;
                    end if;
                    if s_izq = '1' or s_der = '1' then
                        edge_cnt <= 0; est <= E_Z_RECENTRO;
                    elsif edge_cnt >= 2*BUSCA_CAP_CYCLES then
                        edge_cnt <= 0;
                        if busca_tries >= MAX_BUSCA_TRIES then
                            est <= E_Z_RECENTRO;              -- se rinde, asume centrado
                        else
                            busca_tries <= busca_tries + 1;
                            est <= E_Z_BUSCA_A;
                        end if;
                    else
                        edge_cnt <= edge_cnt + 1;
                    end if;

                -- ---- Recentrar: línea entre ambos sensores otra vez ----
                when E_Z_RECENTRO =>
                    if s_der = '1' and s_izq = '0' then
                        tgt_l <= DUTY_Z_PIVOT;  tgt_r <= -DUTY_Z_PIVOT;   -- corrige a der
                    elsif s_izq = '1' and s_der = '0' then
                        tgt_l <= -DUTY_Z_PIVOT; tgt_r <= DUTY_Z_PIVOT;    -- corrige a izq
                    else
                        tgt_l <= 0; tgt_r <= 0;                           -- centrado o (1,1): detente
                    end if;
                    if (s_izq = '0' and s_der = '0') or edge_cnt >= RECENTRO_CAP_CYCLES then
                        edge_cnt <= 0; est <= E_Z_ACCION;
                    else
                        edge_cnt <= edge_cnt + 1;
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
