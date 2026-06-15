library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity MaquinaEstados is
    generic (
        DUTY_RECTO    : integer := 30000; -- Duty de marcha en recta (0..65535)
        DUTY_GIRO_EXT : integer := 30000; -- Duty de la rueda exterior en curva
        DUTY_GIRO_INT : integer := 30000; -- Duty de la rueda interior en curva
        MODO_PIVOTE   : boolean := true; -- false=arco suave, true=girar sobre sí mismo
        FILTRO_CYCLES : integer := 25000; -- Ciclos de antirrebote del sensor (~0.5ms a 50MHz)
        LINE_LVL      : std_logic := '1'  -- Valor del sensor cuando está sobre la línea negra
    );
    port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        sensor_izq : in  std_logic;
        sensor_der : in  std_logic;
        -- Salidas de motores (formato L293 con signo)
        motor_a1   : out std_logic; -- IZQ adelante (PWM when tgt_l > 0)
        motor_a2   : out std_logic; -- IZQ reversa  (PWM when tgt_l < 0)
        motor_b1   : out std_logic; -- DER adelante (PWM when tgt_r > 0)
        motor_b2   : out std_logic; -- DER reversa  (PWM when tgt_r < 0)
        -- Interfaz de zona (deshabilitada por ahora; conectar a '0')
        start_scan   : out std_logic;
        trigger_drop : out std_logic;
        scan_active  : in  std_logic;
        arm_ready    : in  std_logic;
        has_object   : in  std_logic;
        led_estado   : out std_logic;
        led_error    : out std_logic
    );
end MaquinaEstados;

architecture rtl of MaquinaEstados is

    -- Duty interior con signo según el modo de corrección (-DUTY_GIRO_INT o +DUTY_GIRO_INT)
    function f_int(piv : boolean; mag : integer) return integer is
    begin
        if piv then return -mag; else return mag; end if;
    end function;

    constant TGT_GIRO_INT : integer := f_int(MODO_PIVOTE, DUTY_GIRO_INT);

    -- Filtro de histéresis (antirrebote)
    signal flt_izq, flt_der : integer range 0 to FILTRO_CYCLES := 0;
    signal s_izq, s_der     : std_logic := '0';

    -- Targets de cada rueda (con signo: +adelante, -reversa, 0=freno)
    signal tgt_l, tgt_r : integer range -65535 to 65535 := 0;

    -- Generador de PWM libre de 16 bits (free running counter)
    signal pwm16 : unsigned(15 downto 0) := (others => '0');

    -- Generador de 1 Hz (LED de vida)
    signal clk_1s : std_logic := '0';
    signal cnt_1s : integer range 0 to 24_999_999 := 0;

begin
    -- ========================================================================
    -- Generador 1 Hz (LED de vida)
    -- ========================================================================
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

    -- ========================================================================
    -- Generador PWM 16 bits (duty de cada rueda = |tgt_*| sobre 65536)
    -- ========================================================================
    p_pwm : process(clk, rst)
    begin
        if rst = '1' then
            pwm16 <= (others => '0');
        elsif rising_edge(clk) then
            pwm16 <= pwm16 + 1;
        end if;
    end process;

    -- ========================================================================
    -- Señales de zona inactivas (conexión mecánica con SeguidorLinea_Brazo)
    -- ========================================================================
    start_scan   <= '0';
    trigger_drop <= '0';
    led_error    <= '0';
    led_estado   <= clk_1s;

    -- Salidas de motor: con signo -> adelante (INx1=PWM), reversa (INx2=PWM), cero=freno
    motor_a1 <= '1' when (tgt_l > 0 and to_integer(pwm16) <  tgt_l) else '0';
    motor_a2 <= '1' when (tgt_l < 0 and to_integer(pwm16) < -tgt_l) else '0';
    motor_b1 <= '1' when (tgt_r > 0 and to_integer(pwm16) <  tgt_r) else '0';
    motor_b2 <= '1' when (tgt_r < 0 and to_integer(pwm16) < -tgt_r) else '0';

    -- ========================================================================
    -- FSM principal: solo seguimiento de línea (sin zonas/brazo)
    -- ========================================================================
    fsm : process(clk, rst)
    begin
        if rst = '1' then
            tgt_l <= 0; tgt_r <= 0;
            flt_izq <= 0; flt_der <= 0;
            s_izq <= '0'; s_der <= '0';
        elsif rising_edge(clk) then

            -- ---- Filtro de histéresis de sensores (antirrebote) ----
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

            -- Tabla de seguimiento (s_der, s_izq) = (1 si sobre línea negra, 0 si blanco)
            --   (0,0) ninguno sobre línea => RECTO
            --   (1,0) derecha sobre línea  => GIRAR IZQUIERDA
            --   (0,1) izquierda sobre línea => GIRAR DERECHA
            --   (1,1) ambos sobre línea    => RECTO (salvaguarda)
            if s_der = '0' and s_izq = '0' then
                -- Recto
                tgt_l <= DUTY_RECTO;
                tgt_r <= DUTY_RECTO;
            elsif s_der = '1' and s_izq = '0' then
                -- Sensor derecho sobre línea => corregir hacia la izquierda
                tgt_l <= TGT_GIRO_INT;
                tgt_r <= DUTY_GIRO_EXT;
            elsif s_der = '0' and s_izq = '1' then
                -- Sensor izquierdo sobre línea => corregir hacia la derecha
                tgt_l <= DUTY_GIRO_EXT;
                tgt_r <= TGT_GIRO_INT;
            else
                -- (1,1) ambos sobre línea (falso en línea fina; seguro ir recto)
                tgt_l <= DUTY_RECTO;
                tgt_r <= DUTY_RECTO;
            end if;

        end if;
    end process;

end rtl;
