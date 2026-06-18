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
--   ARRANQUE (anti-atasco): al ENCENDER y al SALIR de zona el robot venía PARADO; un
--   empujón recto `DUTY_ARRANQUE` durante `T_ARRANQUE` lo despega antes de seguir la línea.
--
--   Motores L293 (PWM con signo): + adelante (INx1=PWM), - reversa (INx2=PWM), 0 = freno.
--   Duty de marcha de cada rueda = |tgt_*| / 65536.
-- ============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity MaquinaEstados is
    generic (
        DUTY_RECTO    : integer := 30000;
        DUTY_GIRO_EXT : integer := 29000;
        DUTY_GIRO_INT : integer := 29000;
        MODO_PIVOTE   : boolean := true;       -- true=pivote (interior reversa); false=arco suave
        FILTRO_CYCLES : integer := 0;          -- antirrebote sensor (~0.3 ms @50 MHz)
        LINE_LVL      : std_logic := '0';      -- nivel del QRD sobre la línea: '0'=negra, '1'=blanca
        DUTY_ARRANQUE : integer := 40000;      -- empujón anti-atasco; 0=desactiva
        T_ARRANQUE    : integer := 12_500_000;
        W_ARM_CYCLES  : integer := 12_500_000; -- doble-blanco mínimo para armar zona (~0.5 s @50 MHz)
        W_STOP_CYCLES : integer := 25_000_000  -- doble-blanco que detiene por pérdida de línea (~1 s)
    );
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        sensor_izq : in  std_logic;
        sensor_der : in  std_logic;
        motor_a1   : out std_logic;            -- IZQ adelante
        motor_a2   : out std_logic;            -- IZQ reversa
        motor_b1   : out std_logic;            -- DER adelante
        motor_b2   : out std_logic;            -- DER reversa
        led_estado : out std_logic;            -- 1 Hz
        start_scan  : out std_logic;
        trigger_drop: out std_logic;
        scan_active : in  std_logic;
        arm_ready   : in  std_logic;
        has_object  : in  std_logic;
        sensor_err  : in  std_logic;
        zona_fallo  : out std_logic
    );
end MaquinaEstados;

architecture rtl of MaquinaEstados is

    -- tgt_l/r con signo: + adelante (INx1=PWM), - reversa (INx2=PWM), 0=freno
    function f_int(piv : boolean; mag : integer) return integer is
    begin
        if piv then return -mag; else return mag; end if;
    end function;
    constant TGT_GIRO_INT : integer := f_int(MODO_PIVOTE, DUTY_GIRO_INT);

    signal flt_izq, flt_der : integer range 0 to FILTRO_CYCLES := 0;
    signal s_izq, s_der     : std_logic := '0';

    signal tgt_l, tgt_r : integer range -65535 to 65535 := 0;

    signal pwm16 : unsigned(15 downto 0) := (others => '0');

    signal clk_1s : std_logic := '0';
    signal cnt_1s : integer range 0 to 24_999_999 := 0;

    type est_t is (E_ARRANQUE, E_SEGUIR, E_ZONA, E_SCAN_INI, E_SCAN_FIN,
                   E_DROP_INI, E_DROP_FIN, E_PERDIDA, E_FALLO);
    signal est : est_t := E_ARRANQUE;

    signal white_cnt      : integer range 0 to W_STOP_CYCLES := 0;
    signal arr_cnt        : integer range 0 to T_ARRANQUE := 0;
    signal armed          : std_logic := '0';
    signal start_scan_r   : std_logic := '0';
    signal trigger_drop_r : std_logic := '0';

begin

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

    p_pwm : process(clk, rst)
    begin
        if rst = '1' then
            pwm16 <= (others => '0');
        elsif rising_edge(clk) then
            pwm16 <= pwm16 + 1;
        end if;
    end process;

    motor_a1 <= '1' when (tgt_l > 0 and to_integer(pwm16) <  tgt_l) else '0';
    motor_a2 <= '1' when (tgt_l < 0 and to_integer(pwm16) < -tgt_l) else '0';
    motor_b1 <= '1' when (tgt_r > 0 and to_integer(pwm16) <  tgt_r) else '0';
    motor_b2 <= '1' when (tgt_r < 0 and to_integer(pwm16) < -tgt_r) else '0';

    start_scan   <= start_scan_r;
    trigger_drop <= trigger_drop_r;
    -- LED 3: solido=fallo sensor; parpadeo=linea perdida
    zona_fallo <= '1'    when est = E_FALLO   else
                  clk_1s when est = E_PERDIDA else
                  '0';

    p_seguidor : process(clk, rst)
        variable both_line : boolean;
        variable both_off  : boolean;
    begin
        if rst = '1' then
            flt_izq <= 0; flt_der <= 0;
            s_izq <= '0'; s_der <= '0';
            tgt_l <= 0; tgt_r <= 0;
            est <= E_ARRANQUE;
            white_cnt <= 0; arr_cnt <= 0; armed <= '0';
            start_scan_r <= '0'; trigger_drop_r <= '0';
        elsif rising_edge(clk) then

            start_scan_r   <= '0';
            trigger_drop_r <= '0';

            if sensor_izq = LINE_LVL then
                if flt_izq < FILTRO_CYCLES then flt_izq <= flt_izq + 1; end if;
                if flt_izq >= FILTRO_CYCLES then s_izq <= '1'; end if;
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

            case est is

                when E_ARRANQUE =>
                    tgt_l <= DUTY_ARRANQUE; tgt_r <= DUTY_ARRANQUE;
                    if arr_cnt >= T_ARRANQUE-1 then
                        arr_cnt <= 0;
                        white_cnt <= 0; armed <= '0';
                        est <= E_SEGUIR;
                    else
                        arr_cnt <= arr_cnt + 1;
                    end if;

                when E_SEGUIR =>
                    if both_off then
                        if white_cnt < W_STOP_CYCLES then
                            white_cnt <= white_cnt + 1;
                            tgt_l <= DUTY_RECTO; tgt_r <= DUTY_RECTO;
                            if white_cnt >= W_ARM_CYCLES then
                                armed <= '1';
                            end if;
                        else
                            est <= E_PERDIDA;
                            tgt_l <= 0; tgt_r <= 0;
                            armed <= '0';
                        end if;
                    else
                        white_cnt <= 0;
                        if armed = '1' and both_line then
                            est <= E_ZONA;
                            tgt_l <= 0; tgt_r <= 0;
                            armed <= '0';
                        else
                            if both_line then                           -- (1,1) recto
                                tgt_l <= DUTY_RECTO; tgt_r <= DUTY_RECTO;
                            elsif s_izq = '1' and s_der = '0' then     -- (1,0) izquierda
                                tgt_l <= TGT_GIRO_INT; tgt_r <= DUTY_GIRO_EXT;
                            else                                        -- (0,1) derecha
                                tgt_l <= DUTY_GIRO_EXT; tgt_r <= TGT_GIRO_INT;
                            end if;
                        end if;
                    end if;

                when E_ZONA =>
                    tgt_l <= 0; tgt_r <= 0;
                    if has_object = '0' then
                        start_scan_r <= '1';
                        est <= E_SCAN_INI;
                    else
                        trigger_drop_r <= '1';
                        est <= E_DROP_INI;
                    end if;

                when E_SCAN_INI =>
                    tgt_l <= 0; tgt_r <= 0;
                    if scan_active = '1' then
                        est <= E_SCAN_FIN;
                    end if;

                when E_SCAN_FIN =>
                    tgt_l <= 0; tgt_r <= 0;
                    if arm_ready = '1' then
                        if sensor_err = '1' then
                            est <= E_FALLO;
                        else
                            est <= E_ARRANQUE;
                            arr_cnt <= 0; white_cnt <= 0; armed <= '0';
                        end if;
                    end if;

                when E_DROP_INI =>
                    tgt_l <= 0; tgt_r <= 0;
                    if arm_ready = '0' then
                        est <= E_DROP_FIN;
                    end if;

                when E_DROP_FIN =>
                    tgt_l <= 0; tgt_r <= 0;
                    if arm_ready = '1' then
                        est <= E_ARRANQUE;
                        arr_cnt <= 0; white_cnt <= 0; armed <= '0';
                    end if;

                when E_PERDIDA =>
                    tgt_l <= 0; tgt_r <= 0;

                when E_FALLO =>
                    tgt_l <= 0; tgt_r <= 0;

            end case;

        end if;
    end process;

end rtl;
