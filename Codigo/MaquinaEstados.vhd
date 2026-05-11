-- ============================================================================
-- MaquinaEstados - Control de Seguidor de Línea
-- FPGA: Cyclone IV EP4CE6E22C8 | Sensores: QRD1114 | Puente H: L293
-- ============================================================================
-- Descripción: Controla un robot seguidor de línea que navega por una pista
-- negra, detecta zonas de búsqueda, localiza objetos con LIDAR, y regresa
-- al punto de inicio para depositar el objeto.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================================
-- ENTITY: MaquinaEstados
-- ============================================================================
entity MaquinaEstados is
    Port (
        clk                : in  std_logic;
        rst                : in  std_logic;
        sensor_izq         : in  std_logic;
        sensor_der         : in  std_logic;
        sensor_centro      : in  std_logic;
        motor1_in1         : out std_logic;
        motor1_in2         : out std_logic;
        motor2_in1         : out std_logic;
        motor2_in2         : out std_logic;
        motor1_pwm         : out std_logic;
        motor2_pwm         : out std_logic;
        lidar_start        : out std_logic;
        lidar_complete     : in  std_logic;
        lidar_phi          : in  std_logic_vector(7 downto 0);
        lidar_theta        : in  std_logic_vector(7 downto 0);
        lidar_dist         : in  std_logic_vector(7 downto 0);
        brazo_garra_abrir  : out std_logic;
        brazo_garra_cerrar : out std_logic;
        brazo_mover        : out std_logic;
        brazo_home         : out std_logic;
        estado_actual      : out std_logic_vector(3 downto 0);
        error_flag         : out std_logic
    );
end MaquinaEstados;

-- ============================================================================
-- ARCHITECTURE: Behavioral
-- ============================================================================
architecture Behavioral of MaquinaEstados is

    -- Constantes de velocidad
    constant VELOCIDAD_ALTA  : integer := 200;
    constant VELOCIDAD_MEDIA : integer := 150;
    constant VELOCIDAD_BAJA  : integer := 100;
    
    -- Máquina de estados
    type estado_type is (INICIO, SEGUIR_LINEA, DETECTA_ZONA_NEGRA, EXPLORAR_ZONA,
                         BUSCAR_OBJETO, AGARRAR_OBJETO, CALCULAR_RETORNO,
                         RETORNAR_INICIO, DEJAR_OBJETO, MEMORIZAR_ZONA,
                         CONTINUAR_PISTA, ZONA_BLANCA, ERROR);
    signal estado : estado_type := INICIO;
    
    -- Control de motores
    signal motor1_in1_sig : std_logic := '0';
    signal motor1_in2_sig : std_logic := '0';
    signal motor2_in1_sig : std_logic := '0';
    signal motor2_in2_sig : std_logic := '0';
    signal pwm_motor1     : std_logic_vector(7 downto 0) := x"00";
    signal pwm_motor2     : std_logic_vector(7 downto 0) := x"00";
    
    -- Temporizadores
    signal cnt_1ms    : integer range 0 to 49999 := 0;
    signal timer_ms   : integer range 0 to 65535 := 0;
    
    -- Variables de control
    signal objeto_en_pinza : std_logic := '0';
    signal girando_derecha : std_logic := '0';
    signal error_sig       : std_logic := '0';
    signal brazo_abrir     : std_logic := '0';
    signal brazo_cerrar    : std_logic := '0';
    signal brazo_home_sig  : std_logic := '0';
    
    -- Sensores
    signal sensor_izq_reg  : std_logic := '0';
    signal sensor_der_reg  : std_logic := '0';

begin

    -- Temporizador 1ms
    contador_1ms : process(clk, rst)
    begin
        if rst = '0' then
            cnt_1ms <= 0;
        elsif rising_edge(clk) then
            if cnt_1ms = 49999 then
                cnt_1ms <= 0;
                if timer_ms > 0 then
                    timer_ms <= timer_ms - 1;
                end if;
            else
                cnt_1ms <= cnt_1ms + 1;
            end if;
        end if;
    end process;

    -- PWM de motores
    gen_pwm : process(clk)
        variable cnt_pwm : integer range 0 to 255 := 0;
    begin
        if rising_edge(clk) then
            cnt_pwm := (cnt_pwm + 1) mod 256;
            motor1_pwm <= '1' when cnt_pwm < to_integer(unsigned(pwm_motor1)) else '0';
            motor2_pwm <= '1' when cnt_pwm < to_integer(unsigned(pwm_motor2)) else '0';
        end if;
    end process;

    -- Máquina de estados principal
    fsm : process(clk, rst)
    begin
        if rst = '0' then
            estado <= INICIO;
            motor1_in1_sig <= '0';
            motor1_in2_sig <= '0';
            motor2_in1_sig <= '0';
            motor2_in2_sig <= '0';
            pwm_motor1 <= x"00";
            pwm_motor2 <= x"00";
            objeto_en_pinza <= '0';
            error_sig <= '0';
        elsif rising_edge(clk) then
            estado_actual <= std_logic_vector(to_unsigned(estado_type'pos(estado), 4));
            
            -- Registro de sensores
            sensor_izq_reg <= sensor_izq;
            sensor_der_reg <= sensor_der;
            
            case estado is
                -- INICIO: inicialización
                when INICIO =>
                    error_sig <= '0';
                    objeto_en_pinza <= '0';
                    motor1_in1_sig <= '0';
                    motor1_in2_sig <= '0';
                    motor2_in1_sig <= '0';
                    motor2_in2_sig <= '0';
                    pwm_motor1 <= x"00";
                    pwm_motor2 <= x"00";
                    estado <= SEGUIR_LINEA;
                
                -- SEGUIR_LINEA: seguimiento normal
                when SEGUIR_LINEA =>
                    if sensor_izq_reg = '0' and sensor_der_reg = '0' then
                        -- Perdió línea: buscar última posición
                        if girando_derecha = '1' then
                            motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                            motor2_in1_sig <= '0'; motor2_in2_sig <= '1';
                        else
                            motor1_in1_sig <= '0'; motor1_in2_sig <= '1';
                            motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        end if;
                        pwm_motor1 <= std_logic_vector(to_unsigned(VELOCIDAD_BAJA, 8));
                        pwm_motor2 <= std_logic_vector(to_unsigned(VELOCIDAD_BAJA, 8));
                    elsif sensor_izq_reg = '1' then
                        -- Girar izquierda
                        motor1_in1_sig <= '0'; motor1_in2_sig <= '1';
                        motor2_in1_sig <= '0'; motor2_in2_sig <= '1';
                        pwm_motor1 <= std_logic_vector(to_unsigned(VELOCIDAD_MEDIA, 8));
                        pwm_motor2 <= std_logic_vector(to_unsigned(VELOCIDAD_MEDIA, 8));
                        girando_derecha <= '0';
                    elsif sensor_der_reg = '1' then
                        -- Girar derecha
                        motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                        motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        pwm_motor1 <= std_logic_vector(to_unsigned(VELOCIDAD_MEDIA, 8));
                        pwm_motor2 <= std_logic_vector(to_unsigned(VELOCIDAD_MEDIA, 8));
                        girando_derecha <= '1';
                    else
                        -- Recto
                        motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                        motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        pwm_motor1 <= std_logic_vector(to_unsigned(VELOCIDAD_ALTA, 8));
                        pwm_motor2 <= std_logic_vector(to_unsigned(VELOCIDAD_ALTA, 8));
                    end if;
                    
                    -- Detectar zona negra (búsqueda)
                    if sensor_izq_reg = '1' and sensor_der_reg = '1' and sensor_centro = '1' then
                        estado <= DETECTA_ZONA_NEGRA;
                    end if;
                
                -- DETECTA_ZONA_NEGRA: iniciar escaneo
                when DETECTA_ZONA_NEGRA =>
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                    pwm_motor1 <= x"00";
                    pwm_motor2 <= x"00";
                    estado <= EXPLORAR_ZONA;
                
                -- EXPLORAR_ZONA: escanear con LIDAR
                when EXPLORAR_ZONA =>
                    if lidar_complete = '1' then
                        if to_integer(unsigned(lidar_dist)) < 200 then
                            objeto_en_pinza <= '1';
                            estado <= BUSCAR_OBJETO;
                        else
                            timer_ms <= 1000;
                            estado <= MEMORIZAR_ZONA;
                        end if;
                    end if;
                
                -- BUSCAR_OBJETO: mover hacia objeto
                when BUSCAR_OBJETO =>
                    if to_integer(unsigned(lidar_dist)) < 50 then
                        estado <= AGARRAR_OBJETO;
                    else
                        motor1_in1_sig <= '1'; motor1_in2_sig <= '0';
                        motor2_in1_sig <= '1'; motor2_in2_sig <= '0';
                        pwm_motor1 <= std_logic_vector(to_unsigned(VELOCIDAD_BAJA, 8));
                        pwm_motor2 <= std_logic_vector(to_unsigned(VELOCIDAD_BAJA, 8));
                    end if;
                
                -- AGARRAR_OBJETO: cerrar pinza
                when AGARRAR_OBJETO =>
                    brazo_cerrar <= '1';
                    objeto_en_pinza <= '1';
                    estado <= CALCULAR_RETORNO;
                
                -- CALCULAR_RETORNO: invertir dirección
                when CALCULAR_RETORNO =>
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '1';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '1';
                    pwm_motor1 <= std_logic_vector(to_unsigned(VELOCIDAD_MEDIA, 8));
                    pwm_motor2 <= std_logic_vector(to_unsigned(VELOCIDAD_MEDIA, 8));
                    timer_ms <= 1000;
                    estado <= RETORNAR_INICIO;
                
                -- RETORNAR_INICIO: volver al inicio
                when RETORNAR_INICIO =>
                    if sensor_izq_reg = '1' and sensor_der_reg = '1' then
                        motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                        motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                        estado <= DEJAR_OBJETO;
                    elsif timer_ms = 0 then
                        error_sig <= '1';
                        estado <= ERROR;
                    end if;
                
                -- DEJAR_OBJETO: soltar objeto
                when DEJAR_OBJETO =>
                    brazo_abrir <= '1';
                    objeto_en_pinza <= '0';
                    timer_ms <= 1000;
                    estado <= MEMORIZAR_ZONA;
                
                -- MEMORIZAR_ZONA: guardar zona
                when MEMORIZAR_ZONA =>
                    estado <= CONTINUAR_PISTA;
                
                -- CONTINUAR_PISTA: reanudar
                when CONTINUAR_PISTA =>
                    estado <= SEGUIR_LINEA;
                
                -- ZONA_BLANCA: final de pista
                when ZONA_BLANCA =>
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                    pwm_motor1 <= x"00";
                    pwm_motor2 <= x"00";
                    if objeto_en_pinza = '1' then
                        brazo_abrir <= '1';
                        objeto_en_pinza <= '0';
                    end if;
                
                -- ERROR: manejo de errores
                when ERROR =>
                    motor1_in1_sig <= '0'; motor1_in2_sig <= '0';
                    motor2_in1_sig <= '0'; motor2_in2_sig <= '0';
                    pwm_motor1 <= x"00";
                    pwm_motor2 <= x"00";
                    error_sig <= '1';
                
                -- Por defecto
                when others =>
                    estado <= INICIO;
            end case;
        end if;
    end process;

    -- Salidas
    motor1_in1 <= motor1_in1_sig;
    motor1_in2 <= motor1_in2_sig;
    motor2_in1 <= motor2_in1_sig;
    motor2_in2 <= motor2_in2_sig;
    brazo_mover <= '0';
    brazo_home <= brazo_home_sig;
    error_flag <= error_sig;

end Behavioral;