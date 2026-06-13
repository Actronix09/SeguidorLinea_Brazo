-- ============================================================================
-- polarPWM - Control de 5 Servomotores
-- FPGA: Cyclone IV EP4CE6E22C8 | Reloj: 50 MHz
-- ============================================================================
-- CORRECCIONES:
--   1. rst activo alto consistente en todos los procesos
--   2. cur_* inicializan en HOME real (phi=0, t1=90, t2=0, t3=0, grip=cerrado)
--      que coincide con los valores que manda el TestBrazo al arrancar
--   3. Compensación cinemática separada del módulo:
--      polarPWM recibe ángulos ABSOLUTOS de servo directamente.
--      La compensación se aplica en quien llama (SeguidorLinea_Brazo).
--      Esto evita que la compensación congele servos cuando los ángulos
--      relativos suman exactamente la posición actual.
--   4. Pulso PWM corregido: 1.0 ms = 0°, 2.0 ms = 180° (estándar hobby)
-- ============================================================================
--
-- INTERFAZ:
--   phi_in     → ángulo ABSOLUTO servo base        0-180°
--   theta1_in  → ángulo ABSOLUTO servo hombro      0-180°
--   theta2_in  → ángulo ABSOLUTO servo codo        0-180°
--   theta3_in  → ángulo ABSOLUTO servo muñeca      0-180°
--   grip_cmd   → 0=abierto (0°), 1=cerrar (99° = 55%)
--
-- MOVIMIENTO PROGRESIVO:
--   RAMP_STEP = 8_000_000 ciclos → ~6 °/s → 180° en ~28 s (torque suave)
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity polarPWM is
    Port (
        clk         : in  std_logic;
        rst         : in  std_logic;                     -- activo ALTO
        phi_in      : in  std_logic_vector(7 downto 0);
        theta1_in   : in  std_logic_vector(7 downto 0);
        theta2_in   : in  std_logic_vector(7 downto 0);
        theta3_in   : in  std_logic_vector(7 downto 0);
        grip_cmd    : in  std_logic;                     -- 0=abierto, 1=cerrar
        pwm_phi     : out std_logic;
        pwm_theta1  : out std_logic;
        pwm_theta2  : out std_logic;
        pwm_theta3  : out std_logic;
        pwm_gripper : out std_logic
    );
end polarPWM;

architecture Behavioral of polarPWM is

    -- -------------------------------------------------------------------------
    -- Constantes PWM (50 MHz)
    --   Periodo : 20 ms  = 1_000_000 ciclos
    --   0°      : 0.5 ms =    25_000 ciclos   ← estándar hobby real
    --   180°    : 2.5 ms =   125_000 ciclos   ← estándar hobby real
    --   Paso    : (125_000 - 25_000) / 180 = 556 ciclos/°
    -- -------------------------------------------------------------------------
    constant PWM_PERIOD : integer := 1_000_000;
    constant PWM_MIN    : integer :=    25_000;   -- 0.5 ms → 0°
    constant PWM_MAX    : integer :=   125_000;   -- 2.5 ms → 180°
    constant PWM_STEP   : integer :=       556;   -- ciclos por grado

    -- Gripper: normalmente abierto, cierra al 55% (99°)
    constant GRIP_OPEN  : integer :=   0;
    constant GRIP_CLOSE : integer :=  99;

    -- Rampa: ciclos entre incrementos de 1°
    --   8_000_000 ciclos → ~6 °/s → 0-180° en ~28 s  (torque suave)
    --   Ajuste: bajar para más velocidad, subir para menos torque
    constant RAMP_STEP  : integer := 500_000;

    -- -------------------------------------------------------------------------
    -- LUT ángulo (0-180) → ciclos de pulso
    -- -------------------------------------------------------------------------
    type angle_pwm_t is array (0 to 180) of integer range PWM_MIN to PWM_MAX;

    function build_lut return angle_pwm_t is
        variable lut : angle_pwm_t;
        variable v   : integer;
    begin
        for i in 0 to 180 loop
            v := PWM_MIN + i * PWM_STEP;
            if v > PWM_MAX then v := PWM_MAX; end if;
            lut(i) := v;
        end loop;
        return lut;
    end function;

    constant ANGLE_PWM : angle_pwm_t := build_lut;

    -- -------------------------------------------------------------------------
    -- Función clamping
    -- -------------------------------------------------------------------------
    function clamp180(x : integer) return integer is
    begin
        if    x < 0   then return 0;
        elsif x > 180 then return 180;
        else               return x;
        end if;
    end function;

    -- -------------------------------------------------------------------------
    -- Función rampa: avanza 1° hacia el objetivo
    -- -------------------------------------------------------------------------
    function ramp1(cur : integer; tgt : integer) return integer is
    begin
        if    cur < tgt then return cur + 1;
        elsif cur > tgt then return cur - 1;
        else                 return cur;
        end if;
    end function;

    -- -------------------------------------------------------------------------
    -- Ángulos OBJETIVO (leídos de las entradas, sin compensación)
    -- -------------------------------------------------------------------------
    signal tgt_phi  : integer range 0 to 180 := 0;
    signal tgt_t1   : integer range 0 to 180 := 90;
    signal tgt_t2   : integer range 0 to 180 := 0;
    signal tgt_t3   : integer range 0 to 180 := 0;
    signal tgt_grip : integer range 0 to 180 := GRIP_CLOSE;

    -- cur_* inicializan en HOME para que al arrancar no haya salto de rampa
    -- theta3 está invertida: HOME externo=0° → interno=180°
    signal cur_phi  : integer range 0 to 180 := 0;
    signal cur_t1   : integer range 0 to 180 := 90;
    signal cur_t2   : integer range 0 to 180 := 0;
    signal cur_t3   : integer range 0 to 180 := 180;  -- invertido: 180-0=180
    signal cur_grip : integer range 0 to 180 := GRIP_CLOSE;

    -- Tick de rampa
    signal ramp_cnt  : integer range 0 to RAMP_STEP-1 := 0;
    signal ramp_tick : std_logic := '0';

    -- Contador PWM
    signal cuenta : integer range 0 to PWM_PERIOD-1 := 0;

    -- Registros de salida
    signal r_phi  : std_logic := '0';
    signal r_t1   : std_logic := '0';
    signal r_t2   : std_logic := '0';
    signal r_t3   : std_logic := '0';
    signal r_grip : std_logic := '0';

begin

    -- =========================================================================
    -- Proceso 1: Leer entradas → ángulos objetivo
    -- =========================================================================
    read_targets : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                tgt_phi  <= 0;
                tgt_t1   <= 90;
                tgt_t2   <= 0;
                tgt_t3   <= 180;   -- invertido: HOME externo=0° → interno=180°
                tgt_grip <= GRIP_CLOSE;
            else
                tgt_phi  <= clamp180(to_integer(unsigned(phi_in)));
                tgt_t1   <= clamp180(to_integer(unsigned(theta1_in)));
                tgt_t2   <= clamp180(to_integer(unsigned(theta2_in)));
                -- theta3 montado invertido mecánicamente: se invierte aquí
                tgt_t3   <= clamp180(180 - to_integer(unsigned(theta3_in)));
                if grip_cmd = '1' then
                    tgt_grip <= GRIP_CLOSE;
                else
                    tgt_grip <= GRIP_OPEN;
                end if;
            end if;
        end if;
    end process read_targets;

    -- =========================================================================
    -- Proceso 2: Generador de tick de rampa
    -- =========================================================================
    gen_ramp_tick : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                ramp_cnt  <= 0;
                ramp_tick <= '0';
            else
                if ramp_cnt = RAMP_STEP - 1 then
                    ramp_cnt  <= 0;
                    ramp_tick <= '1';
                else
                    ramp_cnt  <= ramp_cnt + 1;
                    ramp_tick <= '0';
                end if;
            end if;
        end if;
    end process gen_ramp_tick;

    -- =========================================================================
    -- Proceso 3: Interpolación progresiva
    -- =========================================================================
    interpolate : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                cur_phi  <= 0;
                cur_t1   <= 90;
                cur_t2   <= 0;
                cur_t3   <= 180;   -- invertido: HOME externo=0° → interno=180°
                cur_grip <= GRIP_CLOSE;
            elsif ramp_tick = '1' then
                cur_phi  <= ramp1(cur_phi,  tgt_phi);
                cur_t1   <= ramp1(cur_t1,   tgt_t1);
                cur_t2   <= ramp1(cur_t2,   tgt_t2);
                cur_t3   <= ramp1(cur_t3,   tgt_t3);
                cur_grip <= ramp1(cur_grip, tgt_grip);
            end if;
        end if;
    end process interpolate;

    -- =========================================================================
    -- Proceso 4: Generador PWM — 5 canales, 20 ms periodo
    -- =========================================================================
    gen_pwm : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                cuenta <= 0;
                r_phi  <= '0'; r_t1 <= '0'; r_t2 <= '0';
                r_t3   <= '0'; r_grip <= '0';
            else
                if cuenta = PWM_PERIOD - 1 then
                    cuenta <= 0;
                else
                    cuenta <= cuenta + 1;
                end if;

                if cuenta < ANGLE_PWM(cur_phi)  then r_phi  <= '1'; else r_phi  <= '0'; end if;
                if cuenta < ANGLE_PWM(cur_t1)   then r_t1   <= '1'; else r_t1   <= '0'; end if;
                if cuenta < ANGLE_PWM(cur_t2)   then r_t2   <= '1'; else r_t2   <= '0'; end if;
                if cuenta < ANGLE_PWM(cur_t3)   then r_t3   <= '1'; else r_t3   <= '0'; end if;
                if cuenta < ANGLE_PWM(cur_grip) then r_grip <= '1'; else r_grip <= '0'; end if;
            end if;
        end if;
    end process gen_pwm;

    -- =========================================================================
    -- Salidas
    -- =========================================================================
    pwm_phi     <= r_phi;
    pwm_theta1  <= r_t1;
    pwm_theta2  <= r_t2;
    pwm_theta3  <= r_t3;
    pwm_gripper <= r_grip;

end Behavioral;