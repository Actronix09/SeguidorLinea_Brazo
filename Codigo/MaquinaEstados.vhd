-- ============================================================================
-- MaquinaEstados - Cerebro de Sísifo (Etapa 2): seguidor de línea + orquestación
--                  del brazo (buscar/agarrar/depositar) en pista cerrada en loop.
-- FPGA: Cyclone II EP2C5T144C7 | Sensores: QRD1114 x2 (LM393) | Puente H: L293 | 50 MHz
-- ----------------------------------------------------------------------------
-- Seguidor de LÍNEA FINA (los 2 QRD van normalmente FUERA, sobre BLANCO; s_*='1' = ese
--   sensor ve NEGRO = sobre la línea). Tabla de giro según (s_der, s_izq):
--
--   NINGUNA (ambos blanco 0,0): va RECTO (DUTY_*_RECTO).
--   CORRECCIÓN (un solo sensor sobre la línea): la rueda EXTERIOR empuja y la INTERIOR se
--     corrige con UN solo estilo, elegido por la perilla MODO_PIVOTE:
--       MODO_PIVOTE=false -> interior ADELANTE pero más lenta (arco suave).  [default]
--       MODO_PIVOTE=true  -> interior en REVERSA (pívot, giro cerrado).
--     Duties DUTY_GIRO_EXT (exterior) y DUTY_GIRO_INT (magnitud de la interior).
--   CASO ESPECIAL (ambos negro 1,1): zona / marcador (ver MODO_ZONA).
--
--   ZONA (entrega/recogida) -- OPCIONAL, perilla USAR_ZONA (default false). El MARCADOR que
--   anuncia la zona se elige con MODO_ZONA (solo el detector elegido sobrevive a síntesis):
--     0 = TIEMPO  : (1,1) SOSTENIDO ZONA_CYCLES (ambos sobre negro = caso especial sostenido).
--     1 = AJEDREZ : alterna IZQ / DER sobre la línea; dispara al volver a blanco tras N_ALTERN.
--     2 = RAYAS   : franjas NEGRAS transversales (ambos sensores a la vez); dispara al volver
--                   a blanco tras N_RAYAS franjas.  [recomendado / más robusto]
--     3 = CUADRO  : un cuadro NEGRO acotado; dispara al llegar al blanco de salida.
--   ENMARCADO (los 3 marcadores): un BLANCO de entrada (LEAD_CYCLES) ARMA el conteo y el
--   BLANCO de salida es el DISPARO; ventanas de negro (W_MIN/W_MAX) y de hueco (T_GAP) filtran
--   ruido/curvas. En la zona se detiene y, según 'has_object': sin objeto dispara el LIDAR
--   (start_scan); con objeto deposita (trigger_drop). Al terminar REANUDA la línea
--   directamente (sin fase de escape). Con USAR_ZONA=false, toda esta rama se elimina.
--
--   Mapa de estados de sensores (s_der, s_izq) -- línea fina:
--   - (0,0) ambos BLANCO -> NINGUNA -> RECTO (DUTY_*_RECTO).
--   - (1,0) DER sobre línea -> IZQUIERDA.
--   - (0,1) IZQ sobre línea -> DERECHA.
--   - (1,1) ambos NEGRO -> CASO ESPECIAL (zona / marcador).
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
        USAR_ZONA     : boolean := false;       -- brazo/zona: false = seguidor puro (default)
        -- MARCADOR de zona: 0=TIEMPO (fallback), 1=AJEDREZ, 2=RAYAS (rec.), 3=CUADRO blanco.
        -- MODO_ZONA es constante => el detector NO elegido se elimina por síntesis (sin área).
        MODO_ZONA     : integer range 0 to 3 := 2; -- 0 - Zona Blanca | 1 Chekboard | 2 - Rayas
        N_RAYAS       : integer := 2;           -- (modo 2) franjas blancas a contar
        N_ALTERN      : integer := 4;           -- (modo 1) alternancias IZQ/DER a contar
        ZONA_CYCLES   : integer := 15_000_000;  -- (modo 0) (1,1) sostenido p/ confirmar zona (~0.3 s)
        -- Enmarcado con negro (modos 1/2/3). *_CYCLES dependen de velocidad y tamaño del
        -- marcador (ciclos ~ distancia/velocidad * 50MHz): son puntos de partida, calibrar.
        LEAD_CYCLES   : integer := 2_000_000;   -- negro sólido de entrada que ARMA (~40 ms)
        W_MIN_CYCLES  : integer := 500_000;     -- blanco mínimo válido (~10 ms; > FILTRO)
        W_MAX_CYCLES  : integer := 6_000_000;   -- (modo 3) blanco máx; si excede = perdido (~120 ms)
        T_GAP_CYCLES  : integer := 8_000_000;   -- máx entre rayas/alternancias antes de resetear (~160 ms)
        FILTRO_CYCLES : integer := 25_000;      -- antirrebote de sensores (~1 ms @50MHz)
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
    constant DUTY_IZQ_RECTO  : integer := 30000;
    constant DUTY_DER_RECTO  : integer := 30000;

    -- ---- PERILLAS DE CORRECCIÓN (un solo modo, elegido por MODO_PIVOTE) -------
    -- Al salir un sensor a blanco: la rueda EXTERIOR empuja y la INTERIOR se corrige.
    constant DUTY_GIRO_EXT   : integer := 30000;   -- rueda exterior (la que empuja)
    constant DUTY_GIRO_INT   : integer := 30000;   -- rueda interior (magnitud)
    -- false = interior ADELANTE pero más lenta (arco suave, default); true = interior en
    -- REVERSA (pívot, giro cerrado). Sube DUTY_GIRO_INT para corregir más fuerte.
    constant MODO_PIVOTE     : boolean := true;
    -- --------------------------------------------------------------------------

    -- ==========================================================================

    -- Rueda interior CON SIGNO según el modo de corrección (se pliega en elaboración):
    -- adelante (arco) si MODO_PIVOTE=false, en reversa (pívot) si true.
    function f_int(piv : boolean; mag : integer) return integer is
    begin
        if piv then return -mag; else return mag; end if;
    end function;
    constant TGT_GIRO_INT : integer := f_int(MODO_PIVOTE, DUTY_GIRO_INT);

    -- Sensores: filtro de histéresis (cuenta arriba/abajo) -> '1' = sobre línea (NEGRO)
    signal flt_izq, flt_der : integer range 0 to FILTRO_CYCLES := 0;
    signal s_izq, s_der     : std_logic := '0';

    -- Duty objetivo por rueda CON SIGNO: + adelante, - reversa, 0 freno.
    signal tgt_l, tgt_r : integer range -65535 to 65535 := 0;

    -- Último giro: da la dirección de recuperación cuando se PIERDE la línea (0,0).
    type giro_t is (G_RECTO, G_IZQ, G_DER);
    signal ultimo_giro : giro_t := G_RECTO;

    -- PWM 16 bits libre (~763 Hz). El duty de cada rueda = |tgt_*| sobre 65536.
    signal pwm16 : unsigned(15 downto 0) := (others => '0');

    -- Detección de zona / salida
    signal zona_cnt  : integer range 0 to ZONA_CYCLES := 0;   -- (modo 0) (1,1) sostenido
    signal salido    : std_logic := '1';   -- (modo 0) ya dejó el bloque negro anterior (anti re-disparo)

    -- ---- Detección de zona por MARCADOR (modos 1/2/3) ------------------------
    signal armed     : std_logic := '0';   -- conteo armado tras negro de entrada (LEAD)
    signal lost_real : std_logic := '0';   -- (modo 3) blanco demasiado largo = perdió línea
    signal zona_trig : std_logic := '0';   -- disparo de zona -> E_SEGUIR pasa a E_ZONA
    signal run_white : std_logic := '0';   -- dentro de un tramo blanco (raya/cuadro)
    signal saw_both  : std_logic := '0';   -- durante el tramo se vio AMBOS blanco
    signal lead_cnt  : integer range 0 to LEAD_CYCLES  := 0;  -- negro de entrada
    signal gap_cnt   : integer range 0 to T_GAP_CYCLES := 0;  -- hueco entre eventos
    signal white_len : integer range 0 to W_MAX_CYCLES := 0;  -- longitud del blanco actual
    signal stripe_cnt: integer range 0 to N_RAYAS  := 0;      -- (modo 2) rayas contadas
    signal alt_cnt   : integer range 0 to N_ALTERN := 0;      -- (modo 1) alternancias contadas
    signal last_side : integer range 0 to 2 := 0;  -- (modo 1) último lado confirmado (1=izq,2=der)
    signal prev_side : integer range 0 to 2 := 0;  -- (modo 1) lado instantáneo previo (estabilidad)

    -- FSM
    type est_t is (E_INICIO, E_SEGUIR, E_ZONA, E_SCAN_INI, E_SCAN_FIN,
                   E_DROP_WAIT);
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
        -- Atajos combinacionales de los sensores (NEGRO=línea): ambos blanco, ambos negro,
        -- un solo blanco; y cruce_marca = "ir RECTO porque estoy cruzando un marcador".
        variable v_bw, v_bb, v_ow, v_cruce : std_logic;
        variable v_side : integer range 0 to 2;
    begin
        if rst = '1' then
            est <= E_INICIO;
            tgt_l <= 0; tgt_r <= 0;
            ultimo_giro <= G_RECTO;
            flt_izq <= 0; flt_der <= 0;
            s_izq <= '0'; s_der <= '0';
            zona_cnt <= 0; salido <= '1';
            start_scan_r <= '0'; trigger_drop_r <= '0';
            armed <= '0'; lost_real <= '0'; zona_trig <= '0';
            run_white <= '0'; saw_both <= '0';
            lead_cnt <= 0; gap_cnt <= 0; white_len <= 0;
            stripe_cnt <= 0; alt_cnt <= 0; last_side <= 0; prev_side <= 0;
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

            -- Contador de ZONA (sólo relevante si USAR_ZONA). Con "ambos dentro", (1,1) es ir
            -- centrado; una ZONA = (1,1) SOSTENIDO mucho más que una recta (que se rompe al
            -- micro-corregir a (1,0)/(0,1)). Cualquier salida a blanco resetea el conteo.
            if s_izq = '1' and s_der = '1' then
                if zona_cnt < ZONA_CYCLES then zona_cnt <= zona_cnt + 1; end if;
            else
                zona_cnt <= 0;
            end if;

            -- 'salido' (modo 0): el robot ya dejó el bloque negro de la zona anterior (vio algo
            -- distinto de (1,1)). Como ya NO hay fase de escape, esto evita que el MISMO bloque
            -- se vuelva a disparar al reanudar: el modo 0 sólo redispara tras haber salido.
            if not (s_izq = '1' and s_der = '1') then
                salido <= '1';
            end if;

            -- ===== Atajos de sensores (NEGRO = sobre la línea) ====================
            if s_izq = '0' and s_der = '0' then v_bw := '1'; else v_bw := '0'; end if;  -- ambos blanco
            if s_izq = '1' and s_der = '1' then v_bb := '1'; else v_bb := '0'; end if;  -- ambos negro
            v_ow := s_izq xor s_der;                                                    -- un solo blanco

            -- cruce_marca: '1' => estoy cruzando un marcador NEGRO -> ir RECTO (no corregir).
            -- Evita que el (1,1) de una raya/cuadro negro se interprete como caso especial.
            v_cruce := '0';
            if USAR_ZONA then
                case MODO_ZONA is
                    when 2      => if run_white = '1' or v_bb = '1' then v_cruce := '1'; end if;
                    when 3      => if (run_white = '1' or v_bb = '1') and lost_real = '0' then v_cruce := '1'; end if;
                    when 1      => if armed = '1' and v_ow = '1' then v_cruce := '1'; end if;
                    when others => null;   -- modo 0: sin gracia
                end case;
            end if;

            -- ===== Detección de ZONA por MARCADOR =================================
            -- Solo cuenta mientras SEGUIMOS (est=E_SEGUIR). Fuera de E_SEGUIR los detectores
            -- quedan en reposo (hay que ver un NUEVO negro de entrada para volver a armar) ->
            -- así no se re-dispara el mismo marcador al reanudar tras la zona.
            if USAR_ZONA and est = E_SEGUIR then

                -- Enmarcado: armar tras LEAD_CYCLES de BLANCO continuo (fondo normal de la pista).
                if v_bw = '1' then
                    if lead_cnt < LEAD_CYCLES then lead_cnt <= lead_cnt + 1; end if;
                    if lead_cnt = LEAD_CYCLES-1 then armed <= '1'; end if;
                else
                    lead_cnt <= 0;
                end if;

                case MODO_ZONA is

                    -- ---- Modo 0: TIEMPO (fallback): usa zona_cnt de arriba ----
                    when 0 =>
                        if salido = '1' and zona_cnt >= ZONA_CYCLES then zona_trig <= '1'; end if;

                    -- ---- Modo 2: RAYAS transversales (ambos sensores cruzan a la vez) ----
                    when 2 =>
                        -- NOTA: run_white/saw_both = "dentro de un tramo NEGRO / vio ambos negro"
                        -- (nombres heredados; la pista es blanca y las rayas son NEGRAS).
                        if armed = '1' then
                            if v_bb = '1' then                  -- dentro de una raya (NEGRA)
                                run_white <= '1'; saw_both <= '1';
                                if white_len < W_MAX_CYCLES then white_len <= white_len + 1; end if;
                                gap_cnt <= 0;
                            elsif v_bw = '1' then               -- blanco (fondo)
                                if run_white = '1' then         -- terminó un tramo negro
                                    if saw_both = '1' and white_len >= W_MIN_CYCLES
                                       and stripe_cnt < N_RAYAS then
                                        stripe_cnt <= stripe_cnt + 1;
                                    end if;
                                    run_white <= '0'; saw_both <= '0'; white_len <= 0;
                                end if;
                                if stripe_cnt >= N_RAYAS then
                                    zona_trig <= '1';           -- blanco de salida tras N rayas
                                else
                                    if gap_cnt < T_GAP_CYCLES then gap_cnt <= gap_cnt + 1; end if;
                                    if gap_cnt = T_GAP_CYCLES-1 then
                                        stripe_cnt <= 0; armed <= '0'; gap_cnt <= 0;
                                    end if;
                                end if;
                            else                                -- un solo sensor (borde de raya)
                                run_white <= '1';
                                if white_len < W_MAX_CYCLES then white_len <= white_len + 1; end if;
                            end if;
                        end if;

                    -- ---- Modo 1: AJEDREZ (alterna IZQ-sobre-línea / DER-sobre-línea) ----
                    when 1 =>
                        if armed = '1' then
                            if v_bb = '1' then                  -- ambos NEGRO: desalineado -> reset
                                armed <= '0'; alt_cnt <= 0; last_side <= 0;
                                prev_side <= 0; white_len <= 0; gap_cnt <= 0;
                            elsif v_ow = '1' then               -- exactamente un sensor sobre la línea
                                if s_izq = '1' then v_side := 1; else v_side := 2; end if;  -- 1=izq,2=der
                                if prev_side = v_side then
                                    if white_len < W_MIN_CYCLES then white_len <= white_len + 1; end if;
                                else
                                    white_len <= 0;
                                end if;
                                prev_side <= v_side;
                                if white_len = W_MIN_CYCLES-1 and last_side /= v_side then
                                    if last_side = 0 then
                                        last_side <= v_side;        -- primera marca: no cuenta
                                    else
                                        if alt_cnt < N_ALTERN then alt_cnt <= alt_cnt + 1; end if;
                                        last_side <= v_side;
                                    end if;
                                end if;
                                gap_cnt <= 0;
                            else                                -- ambos blanco (fondo) -> salida
                                prev_side <= 0; white_len <= 0;
                                if alt_cnt >= N_ALTERN then
                                    zona_trig <= '1';           -- blanco de salida tras N alternancias
                                else
                                    if gap_cnt < T_GAP_CYCLES then gap_cnt <= gap_cnt + 1; end if;
                                    if gap_cnt = T_GAP_CYCLES-1 then
                                        alt_cnt <= 0; last_side <= 0; armed <= '0'; gap_cnt <= 0;
                                    end if;
                                end if;
                            end if;
                        end if;

                    -- ---- Modo 3: CUADRO negro acotado ----
                    when 3 =>
                        -- (cuadro NEGRO sobre fondo blanco). run_white/saw_both = tramo negro.
                        if armed = '1' then
                            if v_bw = '1' then                  -- blanco (fondo)
                                if run_white = '1' then         -- salió del negro -> evalúa
                                    if saw_both = '1' and white_len >= W_MIN_CYCLES
                                       and lost_real = '0' then
                                        zona_trig <= '1';       -- cuadro válido: blanco de salida
                                    end if;
                                    run_white <= '0'; saw_both <= '0'; white_len <= 0; lost_real <= '0';
                                end if;
                            else                                -- negro (uno o ambos)
                                run_white <= '1';
                                if v_bb = '1' then saw_both <= '1'; end if;
                                if white_len < W_MAX_CYCLES then
                                    white_len <= white_len + 1;
                                else
                                    lost_real <= '1';           -- negro demasiado largo = perdió línea
                                end if;
                                gap_cnt <= 0;
                            end if;
                        end if;

                    when others => null;
                end case;

            else
                -- Fuera de E_SEGUIR: detectores en reposo.
                armed <= '0'; lost_real <= '0'; zona_trig <= '0';
                run_white <= '0'; saw_both <= '0';
                lead_cnt <= 0; gap_cnt <= 0; white_len <= 0;
                stripe_cnt <= 0; alt_cnt <= 0; last_side <= 0; prev_side <= 0;
            end if;

            case est is

                when E_INICIO =>
                    tgt_l <= 0; tgt_r <= 0;
                    est <= E_SEGUIR;

                -- ---- Seguir la línea (línea FINA; sensores normalmente FUERA, sobre BLANCO) ----
                -- s_*='1' = ese sensor ve NEGRO (sobre la línea). Tabla (s_der, s_izq):
                --   (0,0) ninguno sobre línea -> NINGUNA (recto)
                --   (1,0) DER sobre línea     -> IZQUIERDA
                --   (0,1) IZQ sobre línea     -> DERECHA
                --   (1,1) ambos sobre línea   -> CASO ESPECIAL (zona / marcador)
                when E_SEGUIR =>
                    if USAR_ZONA and zona_trig = '1' then
                        est <= E_ZONA;
                    elsif USAR_ZONA and v_cruce = '1' then
                        -- GRACIA: cruzando un marcador NEGRO -> ir RECTO (no corregir)
                        ultimo_giro <= G_RECTO;
                        tgt_l <= DUTY_IZQ_RECTO;
                        tgt_r <= DUTY_DER_RECTO;
                    else
                        if s_der = '0' and s_izq = '0' then        -- (0,0) NINGUNA -> recto
                            ultimo_giro <= G_RECTO;
                            tgt_l <= DUTY_IZQ_RECTO;
                            tgt_r <= DUTY_DER_RECTO;

                        elsif s_der = '1' and s_izq = '0' then     -- DER sobre línea -> IZQUIERDA
                            ultimo_giro <= G_IZQ;
                            tgt_l <= TGT_GIRO_INT;                 -- izq (interior) lenta
                            tgt_r <= DUTY_GIRO_EXT;                -- der (exterior) empuja

                        elsif s_der = '0' and s_izq = '1' then     -- IZQ sobre línea -> DERECHA
                            ultimo_giro <= G_DER;
                            tgt_l <= DUTY_GIRO_EXT;                -- izq (exterior) empuja
                            tgt_r <= TGT_GIRO_INT;                 -- der (interior) lenta

                        else                                       -- (1,1) CASO ESPECIAL -> recto
                            ultimo_giro <= G_RECTO;                -- (la zona la dispara zona_trig)
                            tgt_l <= DUTY_IZQ_RECTO;
                            tgt_r <= DUTY_DER_RECTO;
                        end if;
                    end if;

                -- ---- En la zona: detener y decidir según acarreo ----
                when E_ZONA =>
                    tgt_l <= 0; tgt_r <= 0;
                    salido <= '0';                      -- (modo 0) hasta dejar de nuevo el negro
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
                        est <= E_SEGUIR;                 -- reanuda la línea directamente
                    end if;

                when E_DROP_WAIT =>                      -- espera a que suelte el cubo
                    tgt_l <= 0; tgt_r <= 0;
                    if has_object = '0' then
                        est <= E_SEGUIR;                 -- reanuda la línea directamente
                    end if;

            end case;
        end if;
    end process;

    start_scan   <= start_scan_r;
    trigger_drop <= trigger_drop_r;
    led_estado   <= clk_1s;
    led_error    <= '0';

end rtl;
