-- ============================================================================
-- MaquinaEstados - Seguidor de línea SIMPLE con los DOS sensores DENTRO de la
--   línea ANCHA. s_*='1' = ese sensor está SOBRE la línea (color = LINE_LVL:
--   '0' = línea negra, '1' = línea blanca). La comparación con LINE_LVL se hace
--   en el filtro antirrebote, así que la tabla no depende del color.
--
--   Tabla de verdad (s_izq=I, s_der=D; s='1' cuando VAL_Sen == LINE_LVL):
--     (0,0) -> DETENERSE (freno)
--     (1,0) -> GIRAR IZQUIERDA
--     (0,1) -> GIRAR DERECHA
--     (1,1) -> AVANZAR (recto)
--
--   Motores L293 (PWM con signo): + adelante (INx1=PWM), - reversa (INx2=PWM), 0 = freno.
--   Duty de marcha de cada rueda = |tgt_*| / 65536.
-- ============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity MaquinaEstados is
    generic (
        DUTY_RECTO    : integer := 30000;  -- duty en recta (0..65535)
        DUTY_GIRO_EXT : integer := 30000;  -- rueda exterior en curva (la que empuja)
        DUTY_GIRO_INT : integer := 30000;  -- rueda interior en curva (magnitud)
        MODO_PIVOTE   : boolean := true;  -- false = arco suave; true = pivote (interior en reversa)
        FILTRO_CYCLES : integer := 10000;  -- antirrebote del sensor (~0.3 ms @50MHz)
        LINE_LVL      : std_logic := '0'   -- valor del QRD SOBRE la línea: '0'=línea NEGRA, '1'=línea BLANCA
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
        led_estado : out std_logic         -- LED de vida (1 Hz)
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

    -- ------------------------------------------------------------------------
    -- Seguidor: antirrebote de sensores + tabla de seguimiento
    -- ------------------------------------------------------------------------
    p_seguidor : process(clk, rst)
    begin
        if rst = '1' then
            flt_izq <= 0; flt_der <= 0;
            s_izq <= '0'; s_der <= '0';
            tgt_l <= 0; tgt_r <= 0;
        elsif rising_edge(clk) then

            -- Antirrebote: s_* sube a '1' tras FILTRO_CYCLES viendo la línea (=LINE_LVL),
            -- y baja a '0' tras FILTRO_CYCLES sin verla. Filtra el parpadeo del LM393.
            if sensor_izq = LINE_LVL then
                if flt_izq < FILTRO_CYCLES then flt_izq <= flt_izq + 1; end if;
            elsif flt_izq > 0 then
                flt_izq <= flt_izq - 1;
            end if;
            if flt_izq = FILTRO_CYCLES then s_izq <= '1';
            elsif flt_izq = 0          then s_izq <= '0'; end if;

            if sensor_der = LINE_LVL then
                if flt_der < FILTRO_CYCLES then flt_der <= flt_der + 1; end if;
            elsif flt_der > 0 then
                flt_der <= flt_der - 1;
            end if;
            if flt_der = FILTRO_CYCLES then s_der <= '1';
            elsif flt_der = 0          then s_der <= '0'; end if;

            -- Tabla de verdad (s_izq=I, s_der=D; s='1' cuando VAL_Sen == LINE_LVL):
            if s_izq = '1' and s_der = '1' then          -- (1,1) AVANZAR (recto)
                tgt_l <= DUTY_RECTO;
                tgt_r <= DUTY_RECTO;
            elsif s_izq = '1' and s_der = '0' then       -- (1,0) GIRAR IZQUIERDA
                tgt_l <= TGT_GIRO_INT;                   -- rueda IZQ (interior) lenta/reversa
                tgt_r <= DUTY_GIRO_EXT;                  -- rueda DER (exterior) empuja
            elsif s_izq = '0' and s_der = '1' then       -- (0,1) GIRAR DERECHA
                tgt_l <= DUTY_GIRO_EXT;                  -- rueda IZQ (exterior) empuja
                tgt_r <= TGT_GIRO_INT;                   -- rueda DER (interior) lenta/reversa
            else                                         -- (0,0) DETENERSE (freno)
                tgt_l <= 0;
                tgt_r <= 0;
            end if;

        end if;
    end process;

end rtl;
