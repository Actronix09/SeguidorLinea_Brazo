-- ============================================================================
-- SeguidorLinea_BrazoTestBrazo - Prueba autÃƒÆ’Ã‚Â³noma del brazo robÃƒÆ’Ã‚Â³tico
-- FPGA: Cyclone IV EP4CE6E22C8 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ============================================================================
-- Comportamiento:
--   Al salir de reset el brazo va a HOME y comienza la secuencia en bucle.
--   Sin switches, sin sensores externos.
--
-- ÃƒÆ’Ã‚Ângulos enviados a polarPWM son ABSOLUTOS (ya no relativos).
-- La compensaciÃƒÆ’Ã‚Â³n cinemÃƒÆ’Ã‚Â¡tica se calcula aquÃƒÆ’Ã‚Â­ antes de enviar.
--
-- SECUENCIA (STEP_MS = 2500 ms por paso):
--   Paso  0 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ HOME          : phi=  0, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso  1 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Giro base+90  : phi= 90, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso  2 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Giro base 180 : phi=180, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso  3 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Retorno base  : phi=  0, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso  4 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Hombro bajo   : phi=  0, t1= 45, t2=  0, t3=  0, grip=cerrado
--   Paso  5 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Hombro alto   : phi=  0, t1=135, t2=  0, t3=  0, grip=cerrado
--   Paso  6 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ HOME          : phi=  0, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso  7 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Codo 45Ãƒâ€šÃ‚Â°      : phi=  0, t1= 90, t2= 45, t3=  0, grip=cerrado
--   Paso  8 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Codo 90Ãƒâ€šÃ‚Â°      : phi=  0, t1= 90, t2= 90, t3=  0, grip=cerrado
--   Paso  9 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ HOME          : phi=  0, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso 10 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ MuÃƒÆ’Ã‚Â±eca 45Ãƒâ€šÃ‚Â°    : phi=  0, t1= 90, t2=  0, t3= 45, grip=cerrado
--   Paso 11 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ MuÃƒÆ’Ã‚Â±eca 135Ãƒâ€šÃ‚Â°   : phi=  0, t1= 90, t2=  0, t3=135, grip=cerrado
--   Paso 12 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ HOME          : phi=  0, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso 13 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Abre gripper  : phi=  0, t1= 90, t2=  0, t3=  0, grip=abierto
--   Paso 14 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Cierra gripper: phi=  0, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso 15 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Pick posiciÃƒÆ’Ã‚Â³n : phi= 45, t1= 60, t2= 30, t3= 15, grip=abierto
--   Paso 16 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Agarre        : phi= 45, t1= 60, t2= 30, t3= 15, grip=cerrado
--   Paso 17 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Retira        : phi= 45, t1= 90, t2=  0, t3=  0, grip=cerrado
--   Paso 18 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ Deposita      : phi=135, t1= 90, t2= 30, t3=  0, grip=abierto
--   Paso 19 ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Å“ HOME final    : phi=  0, t1= 90, t2=  0, t3=  0, grip=cerrado
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity SeguidorLinea_BrazoTestBrazo is
    Port (
        clk          : in  std_logic;
        reset        : in  std_logic;     -- activo bajo
        servo_phi    : out std_logic;
        servo_theta1 : out std_logic;
        servo_theta2 : out std_logic;
        servo_theta3 : out std_logic;
        servo_gripper: out std_logic;
        led_estado   : out std_logic;     -- parpadea 1 Hz
        led_error    : out std_logic      -- siempre '0'
    );
end SeguidorLinea_BrazoTestBrazo;

architecture Behavioral of SeguidorLinea_BrazoTestBrazo is

    component polarPWM
        Port (
            clk         : in  std_logic;
            rst         : in  std_logic;                    -- activo ALTO
            phi_in      : in  std_logic_vector(7 downto 0);
            theta1_in   : in  std_logic_vector(7 downto 0);
            theta2_in   : in  std_logic_vector(7 downto 0);
            theta3_in   : in  std_logic_vector(7 downto 0);
            grip_cmd    : in  std_logic;
            pwm_phi     : out std_logic;
            pwm_theta1  : out std_logic;
            pwm_theta2  : out std_logic;
            pwm_theta3  : out std_logic;
            pwm_gripper : out std_logic
        );
    end component;

    -- -------------------------------------------------------------------------
    -- Tabla de posiciones ÃƒÂ¢Ã¢â€šÂ¬Ã¢â‚¬Â ÃƒÆ’Ã‚Â¡ngulos ABSOLUTOS de cada servo
    -- -------------------------------------------------------------------------
    constant N_PASOS : integer := 6;

    type pos_t is record
        phi  : integer range 0 to 180;
        t1   : integer range 0 to 180;
        t2   : integer range 0 to 180;
        t3   : integer range 0 to 180;
        grip : std_logic;
    end record;

    type seq_t is array (0 to N_PASOS-1) of pos_t;

    constant SECUENCIA : seq_t := (
        --  phi   t1   t2   t3   grip
        ( 180,  90,   0,   0, '0'),
        (  45,  90,   0,   0, '1'),
        (  45, 135,  45,   0, '1'), 
        ( 135, 135,  45,   0, '1'),
        ( 135,  90,   0,   0, '1'),
        ( 180,  90,   0,   0, '0')
        );

    -- -------------------------------------------------------------------------
    -- TemporizaciÃƒÆ’Ã‚Â³n
    --   STEP_MS = 2500 ms: 900 ms rampa + 1600 ms pausa visual
    -- -------------------------------------------------------------------------
    constant STEP_MS  : integer := 2500;
    constant CNT_STEP : integer := STEP_MS * 50_000 - 1;  -- ciclos por paso

    -- -------------------------------------------------------------------------
    -- SeÃƒÆ’Ã‚Â±ales internas
    -- -------------------------------------------------------------------------
    signal rst_int    : std_logic;   -- activo alto para polarPWM

    -- Secuenciador
    signal step_cnt   : integer range 0 to CNT_STEP := 0;
    signal paso_reg   : integer range 0 to N_PASOS-1 := 0;  -- paso CARGADO
    signal paso_next  : integer range 0 to N_PASOS-1 := 0;  -- prÃƒÆ’Ã‚Â³ximo paso

    -- Comandos registrados hacia polarPWM (se actualizan al inicio de cada paso)
    signal phi_cmd    : std_logic_vector(7 downto 0) := x"00";
    signal t1_cmd     : std_logic_vector(7 downto 0) := x"5A";
    signal t2_cmd     : std_logic_vector(7 downto 0) := x"00";
    signal t3_cmd     : std_logic_vector(7 downto 0) := x"00";
    signal grip_cmd   : std_logic := '1';

    -- 1 Hz
    signal clk_1s     : std_logic := '0';
    signal cnt_1s     : integer range 0 to 49_999_999 := 0;

begin

    -- reset bajo ÃƒÂ¢Ã¢â‚¬Â Ã¢â‚¬â„¢ rst_int alto (activo alto para polarPWM)
    rst_int <= not reset;

    -- =========================================================================
    -- 1 Hz para led_estado
    -- =========================================================================
    gen_1hz : process(clk)
    begin
        if rising_edge(clk) then
            if reset = '0' then
                cnt_1s <= 0;
                clk_1s <= '0';
            else
                if cnt_1s = 49_999_999 then
                    cnt_1s <= 0;
                    clk_1s <= not clk_1s;
                else
                    cnt_1s <= cnt_1s + 1;
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Secuenciador: cuenta ciclos y calcula el nÃƒÆ’Ã‚Âºmero del siguiente paso
    -- SEPARADO del proceso de carga para evitar race conditions
    -- =========================================================================
    seq_timer : process(clk)
    begin
        if rising_edge(clk) then
            if reset = '0' then
                step_cnt  <= 0;
                paso_next <= 0;   -- arranca cargando HOME en el primer ciclo
            else
                if step_cnt = CNT_STEP then
                    step_cnt <= 0;
                    -- Avanzar ÃƒÆ’Ã‚Â­ndice para el SIGUIENTE tick
                    if paso_next = N_PASOS - 1 then
                        paso_next <= 0;
                    else
                        paso_next <= paso_next + 1;
                    end if;
                else
                    step_cnt <= step_cnt + 1;
                end if;
            end if;
        end if;
    end process seq_timer;

    -- =========================================================================
    -- Registro de comandos: se carga al inicio de cada paso (step_cnt=0)
    -- En ese momento paso_next ya tiene el ÃƒÆ’Ã‚Â­ndice correcto del paso actual
    -- porque se actualizÃƒÆ’Ã‚Â³ en el ciclo anterior cuando step_cnt llegÃƒÆ’Ã‚Â³ a CNT_STEP
    -- =========================================================================
    reg_cmd : process(clk)
        variable p : pos_t;
    begin
        if rising_edge(clk) then
            if reset = '0' then
                -- HOME en reset
                phi_cmd  <= x"00";
                t1_cmd   <= x"5A";
                t2_cmd   <= x"00";
                t3_cmd   <= x"00";
                grip_cmd <= '1';
                paso_reg <= 0;
            elsif step_cnt = 0 then
                -- Cargar posiciÃƒÆ’Ã‚Â³n correspondiente al paso actual
                -- paso_next contiene el ÃƒÆ’Ã‚Â­ndice que acaba de actualizarse
                p := SECUENCIA(paso_next);
                phi_cmd  <= std_logic_vector(to_unsigned(p.phi, 8));
                t1_cmd   <= std_logic_vector(to_unsigned(p.t1,  8));
                t2_cmd   <= std_logic_vector(to_unsigned(p.t2,  8));
                t3_cmd   <= std_logic_vector(to_unsigned(p.t3,  8));
                grip_cmd <= p.grip;
                paso_reg <= paso_next;
            end if;
        end if;
    end process reg_cmd;

    -- =========================================================================
    -- Instancia polarPWM
    -- =========================================================================
    u_polarPWM : polarPWM
        Port Map (
            clk         => clk,
            rst         => rst_int,
            phi_in      => phi_cmd,
            theta1_in   => t1_cmd,
            theta2_in   => t2_cmd,
            theta3_in   => t3_cmd,
            grip_cmd    => grip_cmd,
            pwm_phi     => servo_phi,
            pwm_theta1  => servo_theta1,
            pwm_theta2  => servo_theta2,
            pwm_theta3  => servo_theta3,
            pwm_gripper => servo_gripper
        );

    -- =========================================================================
    -- LEDs
    -- =========================================================================
    led_estado <= clk_1s;
    led_error  <= '0';

end Behavioral;