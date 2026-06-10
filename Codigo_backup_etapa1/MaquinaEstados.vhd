-- ============================================================================
-- MaquinaEstados - Control de Seguidor de Línea
-- FPGA: Cyclone IV EP4CE6E22C8 | Sensores: QRD1114 (x2) | Puente H: L293
-- ============================================================================
-- Hardware real:
--   - 2 sensores de línea: izquierdo y derecho (sin sensor centro)
--   - Motor PWM directo a IN del L293 (sin pin enable separado)
--   - Detección de zona negra: ambos sensores activos simultáneamente
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity MaquinaEstados is
    Port (
        clk                : in  std_logic;
        rst                : in  std_logic;
        -- Solo 2 sensores de línea
        sensor_izq         : in  std_logic;
        sensor_der         : in  std_logic;
        -- Motores (PWM directo en IN, sin enable)
        motor1_in1         : out std_logic;
        motor1_in2         : out std_logic;
        motor2_in1         : out std_logic;
        motor2_in2         : out std_logic;
        motor1_pwm         : out std_logic;
        motor2_pwm         : out std_logic;
        -- LIDAR
        lidar_start        : out std_logic;
        lidar_complete     : in  std_logic;
        lidar_phi          : in  std_logic_vector(7 downto 0);
        lidar_theta        : in  std_logic_vector(7 downto 0);
        lidar_dist         : in  std_logic_vector(7 downto 0);
        -- Control de brazo
        brazo_garra_abrir  : out std_logic;
        brazo_garra_cerrar : out std_logic;
        brazo_mover        : out std_logic;
        brazo_home         : out std_logic;
        -- Debug
        estado_actual      : out std_logic_vector(3 downto 0);
        error_flag         : out std_logic
    );
end MaquinaEstados;

architecture Behavioral of MaquinaEstados is

    -- Constantes de velocidad para control gradual con PWM en INx
    -- Utilizamos velocidades mas diferenciadas para una correccion
    -- de trayectoria mas suave (control "diferencial" simplificado).
    constant VELOCIDAD_ALTA  : integer := 240;
    constant VELOCIDAD_MEDIA : integer := 170;
    constant VELOCIDAD_BAJA  : integer := 100;

    -- ------------------------------------------------------------------------
    -- Velocidades del modo DIFERENCIAL (PWM maximo motor externo,
    -- PWM reducido motor interno) para curvas suaves.
    -- ------------------------------------------------------------------------
    constant VEL_DER_ALTA  : integer := 220;  -- motor derecho = rapido en curva izquierda
    constant VEL_DER_BAJA  : integer := 100;  -- motor derecho = lento en curva derecha
    constant VEL_IZQ_ALTA  : integer := 220;  -- motor izquierdo = rapido en curva derecha
    constant VEL_IZQ_BAJA  : integer := 100;  -- motor izquierdo = lento en curva izquierda

    type estado_type is (INICIO, SEGUIR_LINEA, DETECTA_ZONA_NEGRA, EXPLORAR_ZONA,
                         BUSCAR_OBJETO, AGARRAR_OBJETO, CALCULAR_RETORNO,
                         RETORNAR_INICIO, DEJAR_OBJETO, MEMORIZAR_ZONA,
                         CONTINUAR_PISTA, ZONA_BLANCA, ERROR);
    signal estado : estado_type := INICIO;

    signal motor1_in1_sig : std_logic := '0';
    signal motor1_in2_sig : std_logic := '0';
    signal motor2_in1_sig : std_logic := '0';
    signal motor2_in2_sig : std_logic := '0';
    signal pwm_motor1     : std_logic_vector(7 downto 0) := x"00";
    signal pwm_motor2     : std_logic_vector(7 downto 0) := x"00";

    signal cnt_1ms        : integer range 0 to 49999 := 0;
    signal timer_ms       : integer range 0 to 65535 := 0;
    signal timer_preset   : integer range 0 to 65535 := 0;
    signal timer_load_req : std_logic := '0';

    signal objeto_en_pinza : std_logic := '0';
    signal girando_derecha : std_logic := '0';
    signal error_sig       : std_logic := '0';
    signal brazo_abrir     : std_logic := '0';
    signal brazo_cerrar    : std_logic := '0';
    signal brazo_home_sig  : std_logic := '0';

    signal sensor_izq_reg  : std_logic := '0';
    signal sensor_der_reg  : std_logic := '0';

    -- Detección de zona negra: ambos sensores activos a la vez
    signal zona_negra      : std_logic := '0';

begin

    -- =========================================================================
    -- Temporizador 1 ms
    -- =========================================================================
    contador_1ms : process(clk, rst)
    begin
        if rst = '0' then
            cnt_1ms <= 0;
        elsif rising_edge(clk) then
            if cnt_1ms = 49999 then
                cnt_1ms <= 0;
            else
                cnt_1ms <= cnt_1ms + 1;
            end if;
        end if;
    end process;

    decrementa_timer : process(clk, rst)
    begin
        if rst = '0' then
            timer_ms <= 0;
        elsif rising_edge(clk) then
            if timer_load_req = '1' then
                timer_ms <= timer_preset;
            elsif cnt_1ms = 49999 then
                if timer_ms > 0 then
                    timer_ms <= timer_ms - 1;
                end if;
            end if;
        end if;
    end process;

    -- =========================================================================
    -- Generador PWM de motores
    --   El pulso PWM se aplica directamente sobre motor_in1/in2 del L293.
    --   Cuando IN1='1' e IN2='0' el motor gira adelante; PWM modula IN1
    --   para controlar la velocidad sin necesidad del pin ENABLE.
    -- =========================================================================
    gen_pwm : process(clk)
        variable cnt_pwm : integer range 0 to 255 := 0;
    begin
        if rising_edge(clk) then
            cnt_pwm := (cnt_pwm + 1) mod 256;
            if cnt_pwm < to_integer(unsigned(pwm_motor1)) then
                motor1_pwm <= '1';
            else
                motor1_pwm <= '0';
            end if;
            if cnt_pwm < to_integer(unsigned(pwm_motor2)) then
                motor2_pwm <= '1';
            else
                motor2_pwm <= '0';
            end if;
        end if;
    end process;

    -- Detección de zona negra con solo 2 sensores
    zona_negra <= sensor_izq_reg and sensor_der_reg;

    -- =========================================================================
    -- Máquina de estados
    -- =========================================================================
    fsm : process(clk, rst)
    begin
        if rst = '0' then
            estado         <= INICIO;
            motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
            motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
            pwm_motor1     <= x"00";
            pwm_motor2     <= x"00";
            objeto_en_pinza <= '0';
            error_sig      <= '0';
            timer_load_req <= '0';
        elsif rising_edge(clk) then
            estado_actual  <= std_logic_vector(to_unsigned(estado_type'pos(estado), 4));
            timer_load_req <= '0';   -- pulso de un ciclo

            -- Registro de sensores (anti-rebote de 1 ciclo)
            sensor_izq_reg <= sensor_izq;
            sensor_der_reg <= sensor_der;

            case estado is

                -- --------------------------------------------------------------
                when INICIO =>
                    error_sig      <= '0';
                    objeto_en_pinza <= '0';
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                    pwm_motor1     <= x"00";
                    pwm_motor2     <= x"00";
                    estado         <= SEGUIR_LINEA;

                -- --------------------------------------------------------------
                when SEGUIR_LINEA =>
                    -- Deteccion de zona negra (ambos sensores activos)
                    if zona_negra = '1' then
                        estado <= DETECTA_ZONA_NEGRA;

                    -- --------------------------------------------------------------
                    -- Sin linea (ambos sensores en blanco): busca segun ultimo giro
                    --   Se mantiene la ultima direccion, pero se reduce la
                    --   velocidad para evitar perder completamente la pista.
                    -- --------------------------------------------------------------
                    elsif sensor_izq_reg = '0' and sensor_der_reg = '0' then
                        if girando_derecha = '1' then
                            -- Ultimo giro a derecha -> pivotar a derecha buscando
                            motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                            motor2_in1_sig <= '0'; motor2_in2_sig <= '1';
                        else
                            -- Ultimo giro a izquierda -> pivotar a izquierda buscando
                            motor1_in1_sig <= '0'; motor1_in2_sig <= '1';
                            motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        end if;
                        pwm_motor1 <= std_logic_vector(to_unsigned(VELOCIDAD_BAJA, 8));
                        pwm_motor2 <= std_logic_vector(to_unsigned(VELOCIDAD_BAJA, 8));

                    -- --------------------------------------------------------------
                    -- Sensor izquierdo activo (negro) -> robot esta desviado a la izquierda.
                    -- Corregimos hacia la derecha: velocidad alta en motor derecho,
                    -- velocidad reducida en motor izquierdo.
                    -- --------------------------------------------------------------
                    elsif sensor_izq_reg = '1' then
                        -- Motor 1 (derecho): rapido hacia adelante
                        motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                        pwm_motor1     <= std_logic_vector(to_unsigned(VEL_DER_ALTA, 8));
                        -- Motor 2 (izquierda): lento hacia adelante
                        motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        pwm_motor2     <= std_logic_vector(to_unsigned(VEL_IZQ_BAJA, 8));
                        girando_derecha <= '1';

                    -- --------------------------------------------------------------
                    -- Sensor derecho activo (negro) -> robot esta desviado a la derecha.
                    -- Corregimos hacia la izquierda: velocidad alta en motor izquierdo,
                    -- velocidad reducida en motor derecho.
                    -- --------------------------------------------------------------
                    elsif sensor_der_reg = '1' then
                        -- Motor 1 (derecho): lento hacia adelante
                        motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                        pwm_motor1     <= std_logic_vector(to_unsigned(VEL_DER_BAJA, 8));
                        -- Motor 2 (izquierda): rapido hacia adelante
                        motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        pwm_motor2     <= std_logic_vector(to_unsigned(VEL_IZQ_ALTA, 8));
                        girando_derecha <= '0';

                    -- --------------------------------------------------------------
                    -- Ambos sensores en blanco (pero != zona negra): avanzar recto
                    -- a maxima velocidad.
                    -- --------------------------------------------------------------
                    else
                        motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                        motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        pwm_motor1     <= std_logic_vector(to_unsigned(VELOCIDAD_ALTA, 8));
                        pwm_motor2     <= std_logic_vector(to_unsigned(VELOCIDAD_ALTA, 8));
                    end if;

                -- --------------------------------------------------------------
                when DETECTA_ZONA_NEGRA =>
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                    pwm_motor1     <= x"00";
                    pwm_motor2     <= x"00";
                    estado         <= EXPLORAR_ZONA;

                -- --------------------------------------------------------------
                when EXPLORAR_ZONA =>
                    if lidar_complete = '1' then
                        if to_integer(unsigned(lidar_dist)) < 200 then
                            objeto_en_pinza <= '1';
                            estado          <= BUSCAR_OBJETO;
                        else
                            timer_preset   <= 1000;
                            timer_load_req <= '1';
                            estado         <= MEMORIZAR_ZONA;
                        end if;
                    end if;

                -- --------------------------------------------------------------
                when BUSCAR_OBJETO =>
                    if to_integer(unsigned(lidar_dist)) < 50 then
                        estado <= AGARRAR_OBJETO;
                    else
                        motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                        motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        pwm_motor1     <= std_logic_vector(to_unsigned(VELOCIDAD_BAJA, 8));
                        pwm_motor2     <= std_logic_vector(to_unsigned(VELOCIDAD_BAJA, 8));
                    end if;

                -- --------------------------------------------------------------
                when AGARRAR_OBJETO =>
                    brazo_cerrar    <= '1';
                    objeto_en_pinza <= '1';
                    estado          <= CALCULAR_RETORNO;

                -- --------------------------------------------------------------
                when CALCULAR_RETORNO =>
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '1';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '1';
                    pwm_motor1     <= std_logic_vector(to_unsigned(VELOCIDAD_MEDIA, 8));
                    pwm_motor2     <= std_logic_vector(to_unsigned(VELOCIDAD_MEDIA, 8));
                    timer_preset   <= 1000;
                    timer_load_req <= '1';
                    estado         <= RETORNAR_INICIO;

                -- --------------------------------------------------------------
                when RETORNAR_INICIO =>
                    -- Llegó al inicio: ambos sensores activos = zona negra de inicio
                    if zona_negra = '1' then
                        motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                        motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                        estado         <= DEJAR_OBJETO;
                    elsif timer_ms = 0 then
                        error_sig <= '1';
                        estado    <= ERROR;
                    end if;

                -- --------------------------------------------------------------
                when DEJAR_OBJETO =>
                    brazo_abrir     <= '1';
                    objeto_en_pinza <= '0';
                    timer_preset    <= 1000;
                    timer_load_req  <= '1';
                    estado          <= MEMORIZAR_ZONA;

                -- --------------------------------------------------------------
                when MEMORIZAR_ZONA =>
                    estado <= CONTINUAR_PISTA;

                -- --------------------------------------------------------------
                when CONTINUAR_PISTA =>
                    estado <= SEGUIR_LINEA;

                -- --------------------------------------------------------------
                when ZONA_BLANCA =>
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                    pwm_motor1     <= x"00";
                    pwm_motor2     <= x"00";
                    if objeto_en_pinza = '1' then
                        brazo_abrir     <= '1';
                        objeto_en_pinza <= '0';
                    end if;

                -- --------------------------------------------------------------
                when ERROR =>
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                    pwm_motor1     <= x"00";
                    pwm_motor2     <= x"00";
                    error_sig      <= '1';

                when others =>
                    estado <= INICIO;
            end case;
        end if;
    end process;

    -- =========================================================================
    -- Salidas
    -- =========================================================================
    motor1_in1 <= motor1_in1_sig;
    motor1_in2 <= motor1_in2_sig;
    motor2_in1 <= motor2_in1_sig;
    motor2_in2 <= motor2_in2_sig;
    brazo_mover <= '0';
    brazo_home  <= brazo_home_sig;
    brazo_garra_abrir  <= brazo_abrir;
    brazo_garra_cerrar <= brazo_cerrar;
    error_flag  <= error_sig;

end Behavioral;