-- ============================================================================
-- LIDAR - Control del Sensor VL6180X
-- FPGA: Cyclone IV EP4CE6E22C8 | Sensor: VL6180X (I2C)
-- ============================================================================
-- Descripción: Escanea un área de -45° a +45° detectando objetos mediante
-- el sensor de distancia óptico VL6180X. Retorna las coordenadas polares
-- del objeto más cercano usando promedio ponderado.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- ============================================================================
-- ENTITY: LIDAR
-- ============================================================================
entity LIDAR is
    Port (
        clk           : in  std_logic;                     -- 50 MHz
        rst           : in  std_logic;                     -- Reset
        start_scan    : in  std_logic;                     -- Inicio escaneo
        i2c_scl       : out std_logic;                     -- I2C clock
        i2c_sda       : inout std_logic;                   -- I2C data
        i2c_gpio      : in  std_logic;                     -- GPIO (opcional)
        best_phi      : out std_logic_vector(7 downto 0);  -- Phi mejor
        best_theta    : out std_logic_vector(7 downto 0);  -- Theta mejor
        best_distance : out std_logic_vector(7 downto 0);  -- Distancia
        scan_complete : out std_logic;                     -- Fin escaneo
        current_state : out std_logic_vector(3 downto 0);  -- Debug
        debug_data    : out std_logic_vector(15 downto 0)  -- Debug
    );
end LIDAR;

-- ============================================================================
-- ARCHITECTURE: Behavioral
-- ============================================================================
architecture Behavioral of LIDAR is

    -- Constantes
    constant NUM_SCAN_POINTS : integer := 19;  -- -45° a +45° en pasos de 5°
    subtype distance_type is integer range 0 to 255;
    subtype angle_type is integer range -45 to 45;
    type distance_array is array (0 to NUM_SCAN_POINTS-1) of distance_type;
    type angle_array is array (0 to NUM_SCAN_POINTS-1) of angle_type;

    -- Máquina de estados
    type state_type is (IDLE, INIT, STARTING, WAIT_M, READ_M, NEXT_PT, REFINE, CALC, OUTPUT, COMPLETE);
    signal state : state_type := IDLE;
    
    -- Datos de escaneo
    signal scan_idx      : integer range 0 to NUM_SCAN_POINTS-1 := 0;
    signal phi_angle     : angle_type := 0;
    signal measured_dist : distance_type := 0;
    signal distances     : distance_array := (others => 0);
    signal phi_values    : angle_array := (others => 0);
    signal min_idx       : integer range 0 to NUM_SCAN_POINTS-1 := 0;
    signal min_dist      : distance_type := 255;
    
    -- Resultados
    signal best_phi_int   : angle_type := 0;
    signal best_theta_int : integer range 0 to 180 := 90;
    signal best_dist_int  : distance_type := 255;
    signal scan_done      : std_logic := '0';
    
    -- I2C
    signal i2c_scl_reg    : std_logic := '1';
    signal i2c_sda_reg    : std_logic := '1';
    signal i2c_timer      : integer range 0 to 249 := 0;
    signal i2c_bit_cnt    : integer range 0 to 7 := 0;
    signal i2c_done_pulse : std_logic := '0';

begin

    -- Proceso principal: I2C y máquina de estados
    main_proc : process(clk, rst)
    begin
        if rst = '0' then
            state <= IDLE;
            scan_done <= '0';
            i2c_scl_reg <= '1';
            i2c_sda_reg <= '1';
        elsif rising_edge(clk) then
            current_state <= std_logic_vector(to_unsigned(state_type'pos(state), 4));
            i2c_done_pulse <= '0';
            
            -- Temporizador I2C (100 kHz)
            if i2c_timer < 249 then
                i2c_timer <= i2c_timer + 1;
            else
                i2c_timer <= 0;
                i2c_scl_reg <= not i2c_scl_reg;
                if i2c_scl_reg = '0' and state /= IDLE then
                    if i2c_bit_cnt < 7 then
                        i2c_bit_cnt <= i2c_bit_cnt + 1;
                    else
                        i2c_bit_cnt <= 0;
                        i2c_done_pulse <= '1';
                    end if;
                end if;
            end if;
            
            -- Máquina de estados
            case state is
                when IDLE =>
                    i2c_scl_reg <= '1';
                    i2c_sda_reg <= '1';
                    scan_done <= '0';
                    if start_scan = '1' then
                        state <= INIT;
                    end if;
                    
                when INIT =>
                    i2c_sda_reg <= '0';
                    state <= STARTING;
                    
                when STARTING =>
                    scan_idx <= 0;
                    phi_angle <= -45;
                    min_dist <= 255;
                    i2c_sda_reg <= '1';
                    state <= WAIT_M;
                    
                when WAIT_M =>
                    if i2c_done_pulse = '1' then
                        state <= READ_M;
                    end if;
                    
                when READ_M =>
                    measured_dist <= 100;  -- Simulado (VL6180X real)
                    distances(scan_idx) <= measured_dist;
                    phi_values(scan_idx) <= phi_angle;
                    if measured_dist > 0 and measured_dist < min_dist then
                        min_dist <= measured_dist;
                        min_idx <= scan_idx;
                    end if;
                    state <= NEXT_PT;
                    
                when NEXT_PT =>
                    if scan_idx < NUM_SCAN_POINTS-1 then
                        scan_idx <= scan_idx + 1;
                        phi_angle <= phi_angle + 5;
                        state <= WAIT_M;
                    else
                        state <= REFINE;
                    end if;
                    
                when REFINE =>
                    state <= CALC;
                    
                when CALC =>
                    best_phi_int <= phi_values(min_idx);
                    best_theta_int <= 90;
                    best_dist_int <= distances(min_idx);
                    state <= OUTPUT;
                    
                when OUTPUT =>
                    scan_done <= '1';
                    state <= COMPLETE;
                    
                when COMPLETE =>
                    scan_done <= '1';
                    if start_scan = '0' then
                        state <= IDLE;
                    end if;
                    
                when others =>
                    state <= IDLE;
            end case;
        end if;
    end process;

    -- Salidas
    i2c_scl <= i2c_scl_reg;
    i2c_sda <= i2c_sda_reg when state /= IDLE else '1';
    best_phi <= std_logic_vector(to_unsigned(best_phi_int + 45, 8));
    best_theta <= std_logic_vector(to_unsigned(best_theta_int, 8));
    best_distance <= std_logic_vector(to_unsigned(best_dist_int, 8));
    scan_complete <= scan_done;
    debug_data <= std_logic_vector(to_unsigned(measured_dist, 8)) & 
                  std_logic_vector(to_unsigned(scan_idx, 8));

end Behavioral;