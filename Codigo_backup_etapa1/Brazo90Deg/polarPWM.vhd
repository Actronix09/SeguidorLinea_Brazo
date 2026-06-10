-- ============================================================================
-- polarPWM - Conversión de Coordenadas Polares a PWM
-- FPGA: Cyclone IV EP4CE6E22C8 | Brazo Robótico 5 Ejes
-- ============================================================================
-- CAMBIO: Se agregó el puerto pwm_gripper como 5.ª salida PWM independiente,
--         generada a partir de gripper_in. Ahora los 5 servos tienen cada uno
--         su propia señal PWM sin necesidad de instanciar el módulo dos veces.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity polarPWM is
    Port (
        clk         : in  std_logic;
        rst         : in  std_logic;                      -- activo bajo
        phi_in      : in  std_logic_vector(7 downto 0);  -- 0-180 grados
        theta_in    : in  std_logic_vector(7 downto 0);  -- 0-180 grados
        radio_in    : in  std_logic_vector(7 downto 0);  -- reservado
        gripper_in  : in  std_logic_vector(7 downto 0);  -- 0-180 grados
        pwm_phi     : out std_logic;
        pwm_theta1  : out std_logic;
        pwm_theta2  : out std_logic;
        pwm_theta3  : out std_logic;
        pwm_gripper : out std_logic                       -- ← NUEVO puerto
    );
end polarPWM;

architecture Behavioral of polarPWM is

    -- -------------------------------------------------------------------------
    -- Constantes PWM  (reloj 50 MHz)
    --   Periodo : 20 ms  = 1_000_000 ciclos
    --   Mínimo  : 0.5 ms =    25_000 ciclos  →  0°
    --   Máximo  : 2.0 ms =   100_000 ciclos  → 180°
    --   Paso    : (100_000 − 25_000) / 180   ≈  417 ciclos/°
    -- -------------------------------------------------------------------------
    constant PWM_PERIOD : integer := 1_000_000;
    constant PWM_MIN    : integer :=    25_000;
    constant PWM_MAX    : integer :=   100_000;
    constant PWM_STEP   : integer :=       417;

    -- -------------------------------------------------------------------------
    -- LUT ángulo → ancho de pulso  (generada en elaboración)
    -- -------------------------------------------------------------------------
    type angle_to_pwm_t is array (0 to 180) of integer range PWM_MIN to PWM_MAX;

    function build_lut return angle_to_pwm_t is
        variable lut : angle_to_pwm_t;
        variable v   : integer;
    begin
        for i in 0 to 180 loop
            v := PWM_MIN + i * PWM_STEP;
            if v > PWM_MAX then v := PWM_MAX; end if;
            lut(i) := v;
        end loop;
        return lut;
    end function;

    constant ANGLE_PWM : angle_to_pwm_t := build_lut;

    -- -------------------------------------------------------------------------
    -- Clamp a [0, 180]
    -- -------------------------------------------------------------------------
    function clamp180(x : integer) return integer is
    begin
        if    x < 0   then return 0;
        elsif x > 180 then return 180;
        else               return x;
        end if;
    end function;

    -- -------------------------------------------------------------------------
    -- Ángulos registrados (síncronos)
    -- -------------------------------------------------------------------------
    signal ang_phi  : integer range 0 to 180 := 90;
    signal ang_t1   : integer range 0 to 180 := 90;
    signal ang_t2   : integer range 0 to 180 := 90;
    signal ang_t3   : integer range 0 to 180 := 90;
    signal ang_grip : integer range 0 to 180 := 90;   -- ← NUEVO

    -- Contador PWM compartido
    signal cuenta   : integer range 0 to PWM_PERIOD-1 := 0;

    -- Registros de salida
    signal r_phi    : std_logic := '0';
    signal r_t1     : std_logic := '0';
    signal r_t2     : std_logic := '0';
    signal r_t3     : std_logic := '0';
    signal r_grip   : std_logic := '0';               -- ← NUEVO

begin

    -- =========================================================================
    -- Registro síncrono de ángulos
    -- =========================================================================
    reg_ang : process(clk)
        variable phi_v, tht_v, grp_v : integer;
    begin
        if rising_edge(clk) then
            if rst = '0' then
                ang_phi  <= 90;
                ang_t1   <= 90;
                ang_t2   <= 90;
                ang_t3   <= 90;
                ang_grip <= 90;
            else
                phi_v := to_integer(unsigned(phi_in));
                tht_v := to_integer(unsigned(theta_in));
                grp_v := to_integer(unsigned(gripper_in));

                ang_phi  <= clamp180(phi_v);
                ang_t1   <= clamp180(tht_v);
                ang_t2   <= clamp180(180 - tht_v);   -- complemento mecánico
                ang_t3   <= clamp180(grp_v);          -- muñeca / theta3
                ang_grip <= clamp180(grp_v);          -- gripper físico
            end if;
        end if;
    end process reg_ang;

    -- =========================================================================
    -- Generador PWM — 5 canales, contador compartido
    -- =========================================================================
    gen_pwm : process(clk)
    begin
        if rising_edge(clk) then
            if rst = '0' then
                cuenta <= 0;
                r_phi  <= '0';
                r_t1   <= '0';
                r_t2   <= '0';
                r_t3   <= '0';
                r_grip <= '0';
            else
                -- Contador de periodo (20 ms)
                if cuenta = PWM_PERIOD - 1 then
                    cuenta <= 0;
                else
                    cuenta <= cuenta + 1;
                end if;

                -- Canal phi
                if cuenta < ANGLE_PWM(ang_phi)  then r_phi  <= '1'; else r_phi  <= '0'; end if;
                -- Canal theta1
                if cuenta < ANGLE_PWM(ang_t1)   then r_t1   <= '1'; else r_t1   <= '0'; end if;
                -- Canal theta2 (complemento mecánico)
                if cuenta < ANGLE_PWM(ang_t2)   then r_t2   <= '1'; else r_t2   <= '0'; end if;
                -- Canal theta3 / muñeca
                if cuenta < ANGLE_PWM(ang_t3)   then r_t3   <= '1'; else r_t3   <= '0'; end if;
                -- Canal gripper físico  ← NUEVO
                if cuenta < ANGLE_PWM(ang_grip) then r_grip <= '1'; else r_grip <= '0'; end if;
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
    pwm_gripper <= r_grip;    -- ← NUEVO

end Behavioral;