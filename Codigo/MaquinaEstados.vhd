-- ============================================================================
-- MaquinaEstados - Seguidor de línea SIMPLE con los DOS sensores DENTRO de la
--   línea ANCHA + ZONA DE RECOGIDA por UNA línea blanca ancha transversal.
--
--   s_*='1' = ese sensor está SOBRE la línea (color = LINE_LVL:
--   '0' = línea negra, '1' = línea blanca). La comparación con LINE_LVL se hace
--   en el filtro antirrebote, así que la tabla no depende del color.
--
--   Tabla de verdad (s_izq=I, s_der=D; s='1' cuando VAL_Sen == LINE_LVL):
--     (1,0) -> GIRAR IZQUIERDA
--     (0,1) -> GIRAR DERECHA
--     (1,1) -> AVANZAR (recto)
--     (0,0) -> doble blanco: AVANZA para cruzar; si dura >= W_ARM (0.5 s) ARMA la
--              zona; si dura > W_STOP (1 s) = pérdida real -> DETENER.
--
--   ZONA (1 línea blanca ancha): el doble blanco (0,0) sostenido >= W_ARM_CYCLES
--   (~0.5 s) ARMA la zona. Cuando los sensores vuelven a la línea (al volver UNO se
--   espera al OTRO hasta tener (1,1)) se DISPARA y ALTERNA según el acarreo:
--     - SIN cubo  -> start_scan (LIDAR busca + agarra, queda en HOLD).
--     - CON cubo  -> trigger_drop (deposita: gira la base, extiende, abre garra).
--   Si el doble blanco supera W_STOP_CYCLES (~1 s) = pérdida de línea -> DETIENE.
--   Handshake con grab_ctrl: start_scan/trigger_drop(out) / scan_active / arm_ready /
--   has_object.
--   Si el barrido falla (sensor_err) -> E_FALLO: brazo en reposo, robot detenido,
--   zona_fallo='1' (LED de error). Sale con reset.
--
--   Motores L293 (PWM con signo): + adelante (INx1=PWM), - reversa (INx2=PWM), 0 = freno.
--   Duty de marcha de cada rueda = |tgt_*| / 65536.
-- ============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity MaquinaEstados is
    generic (
        DUTY_RECTO    : integer := 29000;  -- duty en recta (0..65535)
        DUTY_GIRO_EXT : integer := 28000;  -- rueda exterior en curva (la que empuja)
        DUTY_GIRO_INT : integer := 28000;  -- rueda interior en curva (magnitud)
        MODO_PIVOTE   : boolean := true;   -- false = arco suave; true = pivote (interior en reversa)
        FILTRO_CYCLES : integer := 0;      -- antirrebote del sensor (~0.3 ms @50MHz)
        LINE_LVL      : std_logic := '0';  -- valor del QRD SOBRE la línea: '0'=línea NEGRA, '1'=línea BLANCA
        -- Zona de recogida (1 línea blanca ancha). Calibrar a velocidad y ancho reales.
        W_ARM_CYCLES  : integer := 12_500_000; -- doble blanco MÍNIMO para ARMAR la zona (~0.5 s @50MHz)
        W_STOP_CYCLES : integer := 25_000_000  -- doble blanco que DETIENE el robot (pérdida real, ~1 s)
    );
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;        -- activo alto
        sensor_izq : in  std_logic;
        sensor_der : in  std_logic;
        motor_a1   : out std_logic;        -- IZQ adelante
        motor_a2   : out std_logic;        -- IZQ reversa
        motor_b1   : out std_logic;        -- DER adelante
        motor_b2   : out std_logic;        -- DER reversa
        led_estado : out std_logic;        -- LED de vida (1 Hz)
        -- Handshake con el brazo (grab_ctrl)
        start_scan  : out std_logic;       -- pulso: inicia barrido LIDAR + agarre
        trigger_drop: out std_logic;       -- pulso: deposita el objeto (zona con cubo)
        scan_active : in  std_logic;       -- '1' mientras el LIDAR barre
        arm_ready   : in  std_logic;       -- '1' cuando el brazo terminó (HOLD o REST)
        has_object  : in  std_logic;       -- '1' mientras el brazo acarrea un objeto
        sensor_err  : in  std_logic;       -- '1' si el barrido falló (sensor no responde)
        zona_fallo  : out std_logic        -- '1' = detenido por fallo de sensor (LED de error)
    );
end MaquinaEstados;

architecture rtl of MaquinaEstados is

    -- Rueda interior con signo: reversa si pivote, adelante (lenta) si arco.
    function f_int(piv : boolean; mag : integer) return integer is
    begin
        if piv then return -mag; else return mag; end if;
    end function;
    constant TGT_GIRO_INT : integer := f_int(MODO_PIVOTE, DUTY_GIRO_INT);

    -- Antirrebote de sensores (cuenta arriba/abajo) -> s_*='1' = SOBRE la línea (=LINE_LVL).
    signal flt_izq, flt_der : integer range 0 to FILTRO_CYCLES := 0;
    signal s_izq, s_der     : std_logic := '0';

    -- Duty objetivo por rueda (con signo: + adelante, - reversa, 0 freno).
    signal tgt_l, tgt_r : integer range -65535 to 65535 := 0;

    -- PWM 16 bits libre (~763 Hz).
    signal pwm16 : unsigned(15 downto 0) := (others => '0');

    -- LED de vida 1 Hz.
    signal clk_1s : std_logic := '0';
    signal cnt_1s : integer range 0 to 24_999_999 := 0;

    -- Zona de recogida (1 línea blanca ancha) -------------------------------
    --   E_PERDIDA: alto FIJO por pérdida de línea (doble blanco > W_STOP). Sale con reset.
    --   E_FALLO  : alto FIJO por fallo de sensor durante el barrido.        Sale con reset.
    --   E_DROP_INI/E_DROP_FIN: con cubo, la zona DEPOSITA (alterna recogida/depósito).
    type est_t is (E_SEGUIR, E_ZONA, E_SCAN_INI, E_SCAN_FIN,
                   E_DROP_INI, E_DROP_FIN, E_PERDIDA, E_FALLO);
    signal est : est_t := E_SEGUIR;

    signal white_cnt    : integer range 0 to W_STOP_CYCLES := 0;  -- ciclos de doble blanco (0,0)
    signal armed        : std_logic := '0';                      -- '1' = doble blanco >= 0.5 s, zona armada
    signal start_scan_r : std_logic := '0';
    signal trigger_drop_r : std_logic := '0';

begin

    -- ------------------------------------------------------------------------
    -- LED de vida (parpadeo 1 Hz)
    -- ------------------------------------------------------------------------
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
    led_estado <= clk_1s;

    -- ------------------------------------------------------------------------
    -- PWM libre de 16 bits
    -- ------------------------------------------------------------------------
    p_pwm : process(clk, rst)
    begin
        if rst = '1' then
            pwm16 <= (others => '0');
        elsif rising_edge(clk) then
            pwm16 <= pwm16 + 1;
        end if;
    end process;

    -- Salidas de motor: + adelante (INx1=PWM), - reversa (INx2=PWM), 0 = freno.
    motor_a1 <= '1' when (tgt_l > 0 and to_integer(pwm16) <  tgt_l) else '0';
    motor_a2 <= '1' when (tgt_l < 0 and to_integer(pwm16) < -tgt_l) else '0';
    motor_b1 <= '1' when (tgt_r > 0 and to_integer(pwm16) <  tgt_r) else '0';
    motor_b2 <= '1' when (tgt_r < 0 and to_integer(pwm16) < -tgt_r) else '0';

    start_scan   <= start_scan_r;
    trigger_drop <= trigger_drop_r;
    -- LED de fallo (LED 3): SÓLIDO = fallo de sensor; PARPADEO (1 Hz) = línea perdida.
    zona_fallo <= '1'    when est = E_FALLO   else
                  clk_1s when est = E_PERDIDA else
                  '0';

    -- ------------------------------------------------------------------------
    -- Seguidor + detección de zona (1 línea blanca ancha) + mini-FSM del brazo
    -- ------------------------------------------------------------------------
    p_seguidor : process(clk, rst)
        variable both_line : boolean;   -- (1,1) ambos sobre la línea negra
        variable both_off  : boolean;   -- (0,0) ambos fuera (línea blanca / pérdida)
    begin
        if rst = '1' then
            flt_izq <= 0; flt_der <= 0;
            s_izq <= '0'; s_der <= '0';
            tgt_l <= 0; tgt_r <= 0;
            est <= E_SEGUIR;
            white_cnt <= 0; armed <= '0'; start_scan_r <= '0'; trigger_drop_r <= '0';
        elsif rising_edge(clk) then

            start_scan_r   <= '0';   -- por defecto: sin pulso
            trigger_drop_r <= '0';

            -- ---- Antirrebote: s_* sube a '1' tras FILTRO_CYCLES viendo la línea (=LINE_LVL),
            --      y baja a '0' tras FILTRO_CYCLES sin verla. Filtra el parpadeo del LM393.
            if sensor_izq = LINE_LVL then
                if flt_izq < FILTRO_CYCLES then flt_izq <= flt_izq + 1; end if;
                if flt_izq >= FILTRO_CYCLES then s_izq <= '1'; end if;   -- '1' al saturar (FILTRO=0 => directo)
            else
                if flt_izq > 0 then flt_izq <= flt_izq - 1; end if;
                if flt_izq = 0 then s_izq <= '0'; end if;
            end if;

            if sensor_der = LINE_LVL then
                if flt_der < FILTRO_CYCLES then flt_der <= flt_der + 1; end if;
                if flt_der >= FILTRO_CYCLES then s_der <= '1'; end if;
            else
                if flt_der > 0 then flt_der <= flt_der - 1; end if;
                if flt_der = 0 then s_der <= '0'; end if;
            end if;

            both_line := (s_izq = '1' and s_der = '1');
            both_off  := (s_izq = '0' and s_der = '0');

            -- ---- FSM de zona -------------------------------------------------
            case est is

                -- ============ SEGUIR: tabla normal + detección de zona ============
                when E_SEGUIR =>
                    if both_off then
                        -- Doble blanco: cuenta su duración.
                        if white_cnt < W_STOP_CYCLES then
                            white_cnt <= white_cnt + 1;
                            tgt_l <= DUTY_RECTO; tgt_r <= DUTY_RECTO;   -- avanza para cruzar la línea blanca
                            if white_cnt >= W_ARM_CYCLES then
                                armed <= '1';                           -- >= 0.5 s de doble blanco: zona ARMADA
                            end if;
                        else
                            est <= E_PERDIDA;                           -- > 1 s en doble blanco: pérdida -> ALTO FIJO
                            tgt_l <= 0; tgt_r <= 0;
                            armed <= '0';
                        end if;
                    else
                        -- Al menos un sensor sobre la línea.
                        white_cnt <= 0;
                        if armed = '1' and both_line then
                            -- Ambos sensores de vuelta en la línea -> ejecutar zona.
                            est <= E_ZONA;
                            tgt_l <= 0; tgt_r <= 0;
                            armed <= '0';
                        else
                            -- Seguimiento normal (también mientras, armado, espera al otro sensor:
                            -- si va a (1,0)/(0,1) gira para recentrar hasta tener (1,1)).
                            if both_line then                           -- (1,1) AVANZAR
                                tgt_l <= DUTY_RECTO; tgt_r <= DUTY_RECTO;
                            elsif s_izq = '1' and s_der = '0' then      -- (1,0) GIRAR IZQUIERDA
                                tgt_l <= TGT_GIRO_INT; tgt_r <= DUTY_GIRO_EXT;
                            else                                        -- (0,1) GIRAR DERECHA
                                tgt_l <= DUTY_GIRO_EXT; tgt_r <= TGT_GIRO_INT;
                            end if;
                        end if;
                    end if;

                -- ============ ZONA: detener y alternar recogida / depósito ============
                when E_ZONA =>
                    tgt_l <= 0; tgt_r <= 0;
                    if has_object = '0' then
                        start_scan_r <= '1';        -- SIN cubo: barrido + agarre
                        est <= E_SCAN_INI;
                    else
                        trigger_drop_r <= '1';      -- CON cubo: depositar
                        est <= E_DROP_INI;
                    end if;

                -- ============ Espera a que el LIDAR confirme el barrido ============
                when E_SCAN_INI =>
                    tgt_l <= 0; tgt_r <= 0;
                    if scan_active = '1' then
                        est <= E_SCAN_FIN;
                    end if;

                -- ============ Espera a que el brazo termine (HOLD o REST) ============
                when E_SCAN_FIN =>
                    tgt_l <= 0; tgt_r <= 0;
                    if arm_ready = '1' then
                        if sensor_err = '1' then
                            est <= E_FALLO;                 -- el sensor falló: detener y avisar
                        else
                            est <= E_SEGUIR;                -- todo bien: reanuda el seguimiento
                            white_cnt <= 0; armed <= '0';
                        end if;
                    end if;

                -- ============ DEPÓSITO: espera a que el brazo ARRANQUE el drop ============
                when E_DROP_INI =>
                    tgt_l <= 0; tgt_r <= 0;
                    if arm_ready = '0' then              -- salió de HOLD: el depósito comenzó
                        est <= E_DROP_FIN;
                    end if;

                -- ============ Espera a que el brazo TERMINE de depositar (vuelve a REST) ====
                when E_DROP_FIN =>
                    tgt_l <= 0; tgt_r <= 0;
                    if arm_ready = '1' then              -- depósito completo: brazo en reposo
                        est <= E_SEGUIR;
                        white_cnt <= 0; armed <= '0';
                    end if;

                -- ============ PÉRDIDA de línea: alto FIJO (sale con reset) ============
                when E_PERDIDA =>
                    tgt_l <= 0; tgt_r <= 0;                 -- LED 3 PARPADEA (línea perdida)

                -- ============ FALLO de sensor: brazo en reposo, robot detenido ============
                when E_FALLO =>
                    tgt_l <= 0; tgt_r <= 0;                 -- alto permanente (sale con reset); LED 3 SÓLIDO
            end case;

        end if;
    end process;

end rtl;
