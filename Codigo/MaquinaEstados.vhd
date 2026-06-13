-- ============================================================================
-- MaquinaEstados - Cerebro de Sísifo (Etapa 2): seguidor de línea + orquestación
--                  del brazo (buscar/agarrar/depositar) en pista cerrada en loop.
-- FPGA: Cyclone II EP2C5T144C7 | Sensores: QRD1114 x2 (LM393) | Puente H: L293 | 50 MHz
-- ----------------------------------------------------------------------------
-- Seguidor con AMBOS SENSORES DENTRO de la línea (la línea negra es ANCHA y los 2
--   QRD van normalmente SOBRE el negro; s_*='1' = ese sensor ve NEGRO = sobre línea):
--
--   CENTRADO (ambos negro 1,1): va RECTO (DUTY_*_RECTO).
--   CORRECCIÓN (un solo sensor sale a blanco): la rueda EXTERIOR empuja y la INTERIOR se
--     corrige con UN solo estilo, elegido por la perilla MODO_PIVOTE:
--       MODO_PIVOTE=false -> interior ADELANTE pero más lenta (arco suave).  [default]
--       MODO_PIVOTE=true  -> interior en REVERSA (pívot, giro cerrado).
--     Duties DUTY_GIRO_EXT (exterior) y DUTY_GIRO_INT (magnitud de la interior).
--   PERDIDA (ambos blanco 0,0): perdió la línea -> recupera girando en la dir. del ÚLTIMO giro.
--
--   ZONA (cuadro negro de entrega/recogida) -- OPCIONAL, perilla USAR_ZONA (default false):
--     OJO: con "ambos dentro", ir centrado YA es (1,1), la MISMA señal que un cuadro. Una ZONA
--     se distingue solo por TIEMPO: (1,1) SOSTENIDO ZONA_CYCLES (mucho más que una recta, que se
--     rompe al micro-corregir a (1,0)/(0,1)). En la zona se detiene y, según 'has_object': sin
--     objeto dispara el LIDAR (start_scan); con objeto deposita (trigger_drop). Luego AVANZA
--     (escape) y reanuda. Con USAR_ZONA=false, toda esta rama se elimina por síntesis.
--
--   Mapa de estados de sensores (ambos dentro de la línea):
--   - (1,1) ambos NEGRO -> CENTRADO -> RECTO (DUTY_*_RECTO).
--   - (1,0) IZQ negro, DER blanco -> derivó DER -> corrige IZQUIERDA.
--   - (0,1) IZQ blanco, DER negro -> derivó IZQ -> corrige DERECHA.
--   - (0,0) ambos BLANCO -> PERDIÓ la línea -> recupera en dir. del último giro.
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
        ZONA_CYCLES   : integer := 15_000_000;  -- (1,1) sostenido p/ confirmar zona (~0.3 s)
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

    -- ---- PERILLAS DE CORRECCIÓN (un solo modo, elegido por MODO_PIVOTE) -------
    -- Al salir un sensor a blanco: la rueda EXTERIOR empuja y la INTERIOR se corrige.
    constant DUTY_GIRO_EXT   : integer := 60000;   -- rueda exterior (la que empuja)
    constant DUTY_GIRO_INT   : integer := 20000;   -- rueda interior (magnitud)
    -- false = interior ADELANTE pero más lenta (arco suave, default); true = interior en
    -- REVERSA (pívot, giro cerrado). Sube DUTY_GIRO_INT para corregir más fuerte.
    constant MODO_PIVOTE     : boolean := false;
    -- --------------------------------------------------------------------------

    -- escape: al salir de la zona (lento, controlado).
    constant DUTY_IZQ_ESCAPE : integer := 35000;
    constant DUTY_DER_ESCAPE : integer := 35000;
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
    signal ambos_linea      : std_logic := '0';   -- ambos sobre NEGRO (candidato a zona)

    -- Duty objetivo por rueda CON SIGNO: + adelante, - reversa, 0 freno.
    signal tgt_l, tgt_r : integer range -65535 to 65535 := 0;

    -- Último giro: da la dirección de recuperación cuando se PIERDE la línea (0,0).
    type giro_t is (G_RECTO, G_IZQ, G_DER);
    signal ultimo_giro : giro_t := G_RECTO;

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

            -- Contador de ZONA (sólo relevante si USAR_ZONA). Con "ambos dentro", (1,1) es ir
            -- centrado; una ZONA = (1,1) SOSTENIDO mucho más que una recta (que se rompe al
            -- micro-corregir a (1,0)/(0,1)). Cualquier salida a blanco resetea el conteo.
            if s_izq = '1' and s_der = '1' then
                if zona_cnt < ZONA_CYCLES then zona_cnt <= zona_cnt + 1; end if;
            else
                zona_cnt <= 0;
            end if;

            case est is

                when E_INICIO =>
                    tgt_l <= 0; tgt_r <= 0;
                    est <= E_SEGUIR;

                -- ---- Seguir la línea (ambos sensores DENTRO del negro) ----
                -- s_*='1' = ese sensor ve NEGRO (sobre la línea). Centrado = (1,1).
                when E_SEGUIR =>
                    if USAR_ZONA and zona_cnt >= ZONA_CYCLES then
                        est <= E_ZONA;
                    else
                        if s_izq = '1' and s_der = '1' then        -- CENTRADO -> recto
                            ultimo_giro <= G_RECTO;
                            tgt_l <= DUTY_IZQ_RECTO;
                            tgt_r <= DUTY_DER_RECTO;

                        elsif s_izq = '1' and s_der = '0' then     -- DER salió -> corrige IZQUIERDA
                            ultimo_giro <= G_IZQ;
                            tgt_l <= TGT_GIRO_INT;                 -- izq (interior) lenta/reversa
                            tgt_r <= DUTY_GIRO_EXT;                -- der (exterior) empuja

                        elsif s_izq = '0' and s_der = '1' then     -- IZQ salió -> corrige DERECHA
                            ultimo_giro <= G_DER;
                            tgt_l <= DUTY_GIRO_EXT;                -- izq (exterior) empuja
                            tgt_r <= TGT_GIRO_INT;                 -- der (interior) lenta/reversa

                        else
                            -- (0,0) AMBOS BLANCO -> PERDIÓ la línea: recupera girando en la
                            -- dirección del último giro (la línea quedó hacia ese lado).
                            case ultimo_giro is
                                when G_RECTO => tgt_l <= DUTY_IZQ_RECTO; tgt_r <= DUTY_DER_RECTO;
                                when G_IZQ   => tgt_l <= TGT_GIRO_INT;   tgt_r <= DUTY_GIRO_EXT;
                                when G_DER   => tgt_l <= DUTY_GIRO_EXT;  tgt_r <= TGT_GIRO_INT;
                            end case;
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
                -- TODO zonas (esquema ambos-dentro): el centro de la línea también es (1,1), así
                --   que "ambos_linea=0" para detectar la salida del cuadro habrá que revisarlo al
                --   reactivar USAR_ZONA (hoy esta rama está dormida por defecto).
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
