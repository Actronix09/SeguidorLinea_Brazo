-- ============================================================================
-- MaquinaEstados - Cerebro de Sísifo (Etapa 2): seguidor de línea + orquestación
--                  del brazo (buscar/agarrar/depositar) en pista cerrada en loop.
-- FPGA: Cyclone II EP2C5T144C7 | Sensores: QRD1114 x2 (LM393) | Puente H: L293 | 50 MHz
-- ----------------------------------------------------------------------------
-- Seguidor STRADDLE de DOS MODOS (la línea negra pasa ENTRE los 2 QRD;
--   s_*='1' = ese sensor ve NEGRO = está SOBRE la línea):
--
--   MODO RECTA (UN solo sensor en negro): enciende la rueda EXTERIOR y APAGA la
--     interior (arco suave). Duties DUTY_RECTA_EXT / DUTY_RECTA_INT(=0 freno).
--   MODO CURVA (DOBLE negro 1,1): pívot con la rueda interior en REVERSA, en la dirección
--     del ÚLTIMO giro, en PULSOS cortos (kick CURVA_PULSO_CYCLES / descanso CURVA_PAUSA_CYCLES)
--     para no girar en el sitio y pasarse. Perilla USAR_MODO_CURVA. Duties DUTY_CURVA_EXT/INT.
--
--   ZONA (cuadro negro de entrega/recogida): negro-doble (1,1) SOSTENIDO ZONA_CYCLES. El
--     gate ZONA_DESDE_RECTO exige venir de RECTO (no de un giro) para NO disparar zona falsa
--     en curvas (que también dan (1,1)). En la zona se detiene y, según 'has_object': sin
--     objeto dispara el LIDAR (start_scan); con objeto deposita (trigger_drop). Luego AVANZA
--     (escape) hasta dejar el cuadro y reanuda.
--
--   Mapa de estados de sensores:
--   - (0,0) ambos BLANCO -> centrados -> RECTO (DUTY_*_RECTO).
--   - (0,1) DER sobre línea -> derivó IZQ -> MODO RECTA girar DERECHA (arco).
--   - (1,0) IZQ sobre línea -> derivó DER -> MODO RECTA girar IZQUIERDA (arco).
--   - (1,1) ambos NEGRO -> MODO CURVA: PÍVOT pulsado en dir. del último giro (G_RECTO=recto).
--           A la vez, si venía de RECTO, cuenta zona_cnt; si dura ZONA_CYCLES -> ZONA.
--
-- Motores (L293): tgt con signo -> + adelante (INx1=PWM), - reversa (INx2=PWM), 0 freno.
--   PWM de 16 bits (periodo 65536 ~763 Hz). Duty de marcha = |tgt_*| / 65536.
--
-- ANTI-RUIDO: los sensores pasan por un filtro de histéresis (FILTRO_CYCLES) para que
--   el parpadeo del comparador LM393 en el borde blanco/negro NO dispare correcciones
--   falsas (era una de las causas de "se sale random en recta").
--
-- NOTA: el nivel del sensor sobre la línea se fija con LINE_LVL (en este robot='0').
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity MaquinaEstados is
    generic (
        ZONA_CYCLES   : integer := 2_000_000;   -- negro-doble (desde recto) sostenido => zona
        SALIR_CYCLES  : integer := 10_000_000;  -- margen tras DEJAR el cuadro negro (0.2 s)
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

    -- ===== CALIBRACIÓN (duty sobre 65536) =====================================
    -- Geometría: vía b=70 mm (b/2=35 mm), sensores 103 mm ADELANTE del eje.
    --
    -- recto-recto: piso de torque de Sísifo (no baja sin chillar). Por rueda: si en
    -- recto se va torcido, sube el lado lento / baja el rápido.
    constant DUTY_IZQ_RECTO  : integer := 48000;
    constant DUTY_DER_RECTO  : integer := 48000;

    -- ---- 4 PERILLAS DE GIRO --------------------------------------------------
    -- MODO RECTA (un solo sensor en negro): exterior empuja, interior se APAGA (freno).
    constant DUTY_RECTA_EXT  : integer := 65000;   -- rueda exterior (la que empuja)
    constant DUTY_RECTA_INT  : integer := 0;       -- rueda interior (0 = freno; subir = arco)
    -- MODO CURVA (doble negro): exterior adelante + interior en REVERSA (pívot).
    constant DUTY_CURVA_EXT  : integer := 56000;   -- rueda exterior adelante
    constant DUTY_CURVA_INT  : integer := 56000;   -- rueda interior en REVERSA (magnitud)
    -- --------------------------------------------------------------------------

    -- escape: al salir de la zona (lento, controlado).
    constant DUTY_IZQ_ESCAPE : integer := 35000;
    constant DUTY_DER_ESCAPE : integer := 35000;

    -- Tiempos del pívot PULSADO de MODO CURVA: el pívot va a PULSOS (no continuo) para no
    -- girar en el sitio y pasarse -> pega CURVA_PULSO en reversa, descansa CURVA_PAUSA en
    -- arco, y repite mientras dure el doble-negro. (a 50 MHz: 50_000 ciclos = 1 ms)
    constant CURVA_PULSO_CYCLES : integer := 1_000_000;  -- KICK pívot reversa     (~20 ms)
    constant CURVA_PAUSA_CYCLES : integer :=   500_000;  -- descanso arco entre kicks (~10 ms)
    constant CURVA_PERIODO      : integer := CURVA_PULSO_CYCLES + CURVA_PAUSA_CYCLES;
    -- ==========================================================================

    -- Habilita la detección de ZONA de entrega/recogida (cuadro negro ancho). 'true' = al
    -- CONFIRMAR zona (negro-doble sostenido) entra a E_ZONA (brazo/LIDAR). 'false' = solo
    -- sigue la línea y pasa sobre los cuadros (probar el seguidor aislado en toda la pista).
    constant USAR_ZONA : boolean := true;

    -- MODO CURVA (pívot en reversa) en doble-negro. true = en (1,1) pivota en reversa hacia
    -- el último giro (toma curvas cerradas). false = en (1,1) mantiene el último giro en modo
    -- recta (arco, sin reversa).
    constant USAR_MODO_CURVA : boolean := true;

    -- Discriminador curva-vs-zona. true = la ZONA solo cuenta si el negro-doble se entró
    -- viniendo de RECTO (ultimo_giro=G_RECTO) => una curva (se entra girando) NO dispara zona
    -- falsa (evita los espasmos). El pívot de curva SÍ actúa siempre en (1,1); esto solo
    -- gatea el CONTADOR de zona. false = cualquier (1,1) sostenido cuenta (puede dar falsos).
    constant ZONA_DESDE_RECTO : boolean := true;

    -- Sensores: filtro de histéresis (cuenta arriba/abajo) -> '1' = sobre línea (NEGRO)
    signal flt_izq, flt_der : integer range 0 to FILTRO_CYCLES := 0;
    signal s_izq, s_der     : std_logic := '0';
    signal ambos_linea      : std_logic := '0';   -- ambos sobre NEGRO (candidato a zona)

    -- Duty objetivo por rueda CON SIGNO: + adelante, - reversa, 0 freno.
    signal tgt_l, tgt_r : integer range -65535 to 65535 := 0;

    -- Último giro (single-sensor): da la dirección del pívot/keep-last en (1,1).
    type giro_t is (G_RECTO, G_IZQ, G_DER);
    signal ultimo_giro : giro_t := G_RECTO;

    -- Fase del pívot pulsado de MODO CURVA (corre solo mientras hay doble-negro).
    signal curva_cnt : integer range 0 to CURVA_PERIODO := 0;

    -- PWM 16 bits libre (~763 Hz). El duty de cada rueda = |tgt_*| sobre 65536.
    signal pwm16 : unsigned(15 downto 0) := (others => '0');

    -- Detección de zona / salida
    signal zona_cnt  : integer range 0 to ZONA_CYCLES := 0;
    signal salir_cnt : integer range 0 to SALIR_CYCLES := 0;

    -- FSM
    type est_t is (E_INICIO, E_SEGUIR, E_ZONA, E_SCAN_INI, E_SCAN_FIN,
                   E_DROP_WAIT, E_SALIR_ZONA);
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

    -- tgt > 0: adelante (INx1=PWM); tgt < 0: reversa (INx2=PWM); tgt = 0: freno (ambas 0)
    motor_a1 <= '1' when (tgt_l > 0 and to_integer(pwm16) <  tgt_l) else '0';
    motor_a2 <= '1' when (tgt_l < 0 and to_integer(pwm16) < -tgt_l) else '0';
    motor_b1 <= '1' when (tgt_r > 0 and to_integer(pwm16) <  tgt_r) else '0';
    motor_b2 <= '1' when (tgt_r < 0 and to_integer(pwm16) < -tgt_r) else '0';

    -- =========================================================================
    -- FSM principal: seguidor (recta + curva) + zonas + orquestación del brazo
    -- =========================================================================
    fsm : process(clk, rst)
    begin
        if rst = '1' then
            est <= E_INICIO;
            tgt_l <= 0; tgt_r <= 0;
            ultimo_giro <= G_RECTO;
            curva_cnt <= 0;
            flt_izq <= 0; flt_der <= 0;
            s_izq <= '0'; s_der <= '0'; ambos_linea <= '0';
            zona_cnt <= 0; salir_cnt <= 0;
            start_scan_r <= '0'; trigger_drop_r <= '0';
        elsif rising_edge(clk) then
            -- pulsos por defecto a '0'
            start_scan_r   <= '0';
            trigger_drop_r <= '0';

            -- -------- Filtro de histéresis de sensores (antirrebote) ----------
            -- flt_* sube si el QRD ve la línea (=LINE_LVL), baja si no. s_* solo cambia
            -- cuando flt_* llega a un extremo => parpadeo del comparador no pasa.
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

            ambos_linea <= s_izq and s_der;

            -- contadores en doble-negro: el pívot pulsado (curva_cnt) corre SIEMPRE; el de ZONA
            -- (zona_cnt) está gateado por ZONA_DESDE_RECTO para no disparar zona falsa en curvas.
            if s_izq = '1' and s_der = '1' then
                if curva_cnt >= CURVA_PERIODO - 1 then curva_cnt <= 0;
                else curva_cnt <= curva_cnt + 1; end if;
                if (not ZONA_DESDE_RECTO) or ultimo_giro = G_RECTO then
                    if zona_cnt < ZONA_CYCLES then zona_cnt <= zona_cnt + 1; end if;
                else
                    zona_cnt <= 0;
                end if;
            else
                zona_cnt  <= 0;
                curva_cnt <= 0;
            end if;

            case est is

                when E_INICIO =>
                    tgt_l <= 0; tgt_r <= 0;
                    est <= E_SEGUIR;

                -- ---- Seguir la línea (modo recta + pívot de curva en doble-negro) ----
                -- s_*='1' = ese sensor ve NEGRO (sobre la línea).
                when E_SEGUIR =>
                    if USAR_ZONA and zona_cnt >= ZONA_CYCLES then
                        est <= E_ZONA;
                    else
                        if s_izq = '0' and s_der = '0' then        -- ambos blanco -> recto
                            ultimo_giro <= G_RECTO;
                            tgt_l <= DUTY_IZQ_RECTO;
                            tgt_r <= DUTY_DER_RECTO;

                        elsif s_izq = '0' and s_der = '1' then     -- DER único -> MODO RECTA der
                            ultimo_giro <= G_DER;
                            tgt_l <= DUTY_RECTA_EXT;               -- izq (exterior) empuja
                            tgt_r <= DUTY_RECTA_INT;               -- der (interior) freno

                        elsif s_izq = '1' and s_der = '0' then     -- IZQ único -> MODO RECTA izq
                            ultimo_giro <= G_IZQ;
                            tgt_l <= DUTY_RECTA_INT;               -- izq (interior) freno
                            tgt_r <= DUTY_RECTA_EXT;               -- der (exterior) empuja

                        else
                            -- (1,1) DOBLE NEGRO -> MODO CURVA: pívot PULSADO (kick reversa /
                            -- descanso arco) en dir. del último giro. G_RECTO -> sigue recto.
                            if USAR_MODO_CURVA then
                                case ultimo_giro is
                                    when G_RECTO =>
                                        tgt_l <= DUTY_IZQ_RECTO; tgt_r <= DUTY_DER_RECTO;
                                    when G_DER =>
                                        if curva_cnt < CURVA_PULSO_CYCLES then   -- KICK pívot der
                                            tgt_l <= DUTY_CURVA_EXT; tgt_r <= -DUTY_CURVA_INT;
                                        else                                     -- descanso (arco)
                                            tgt_l <= DUTY_RECTA_EXT; tgt_r <= DUTY_RECTA_INT;
                                        end if;
                                    when G_IZQ =>
                                        if curva_cnt < CURVA_PULSO_CYCLES then   -- KICK pívot izq
                                            tgt_l <= -DUTY_CURVA_INT; tgt_r <= DUTY_CURVA_EXT;
                                        else                                     -- descanso (arco)
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

                -- ---- En la zona: detener y decidir según acarreo ----
                when E_ZONA =>
                    tgt_l <= 0; tgt_r <= 0;
                    if has_object = '0' then
                        start_scan_r <= '1';            -- busca+agarra
                        est <= E_SCAN_INI;
                    else
                        trigger_drop_r <= '1';          -- deposita
                        est <= E_DROP_WAIT;
                    end if;

                when E_SCAN_INI =>                       -- espera que arranque el barrido
                    tgt_l <= 0; tgt_r <= 0;
                    if scan_active = '1' then
                        est <= E_SCAN_FIN;
                    end if;

                when E_SCAN_FIN =>                       -- espera que el brazo termine
                    tgt_l <= 0; tgt_r <= 0;
                    if arm_ready = '1' then
                        salir_cnt <= 0;
                        est <= E_SALIR_ZONA;
                    end if;

                when E_DROP_WAIT =>                      -- espera a que suelte el cubo
                    tgt_l <= 0; tgt_r <= 0;
                    if has_object = '0' then
                        salir_cnt <= 0;
                        est <= E_SALIR_ZONA;
                    end if;

                -- ---- Salir de la zona: avanza recto (escape) hasta DEJAR el cuadro negro ----
                -- Una vez fuera del negro (ambos_linea='0') espera un margen y reanuda;
                -- así no vuelve a disparar la MISMA zona.
                when E_SALIR_ZONA =>
                    ultimo_giro <= G_RECTO;                 -- al reanudar arranca como recto
                    tgt_l <= DUTY_IZQ_ESCAPE; tgt_r <= DUTY_DER_ESCAPE;
                    if ambos_linea = '0' then               -- ya salió del cuadro negro
                        if salir_cnt >= SALIR_CYCLES-1 then
                            est <= E_SEGUIR;
                        else
                            salir_cnt <= salir_cnt + 1;     -- margen tras salir
                        end if;
                    else
                        salir_cnt <= 0;                     -- aún sobre el negro: sigue avanzando
                    end if;

            end case;
        end if;
    end process;

    start_scan   <= start_scan_r;
    trigger_drop <= trigger_drop_r;
    led_estado   <= clk_1s;
    led_error    <= '0';

end rtl;
