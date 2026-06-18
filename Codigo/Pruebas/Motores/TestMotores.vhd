-- ============================================================================
-- TestMotores - Prueba aislada de los 2 motores DC (diagnóstico de cableado).
-- FPGA: Cyclone II EP2C5T144C7 | Puente H: L293 (enables fijos en HW) | 50 MHz
-- ----------------------------------------------------------------------------
-- Top AUTÓNOMO (sin sensores ni brazo). Recorre 8 fases en loop para verificar
-- en placa QUÉ motor es cuál y si el sentido (adelante/atrás) es el correcto:
--
--   M1 = motor IZQ (motor_a1 adelante / motor_a2 reversa)
--   M2 = motor DER (motor_b1 adelante / motor_b2 reversa)
--
--   Fase 1: M1 full ADELANTE,  M2 apagado
--   Fase 2: M1 freno,          M2 apagado
--   Fase 3: M1 full ATRÁS,     M2 apagado
--   Fase 4: M1 freno,          M2 apagado
--   Fase 5: M1 apagado,        M2 full ADELANTE
--   Fase 6: M1 apagado,        M2 freno
--   Fase 7: M1 apagado,        M2 full ATRÁS
--   Fase 8: M1 apagado,        M2 freno
--   (vuelve a la Fase 1)
--
-- "full" = entrada en '1' fijo (100% duty) para que se vea claro y con torque.
--
-- NOTA HW: en el L293 con los ENABLES FIJOS en hardware, "freno" y "apagado" son
-- ELÉCTRICAMENTE LO MISMO: ambas entradas (INx1, INx2) en '0' => el motor queda
-- FRENADO (no hay "coast"/libre sin controlar el enable). Por eso las fases de
-- freno y los motores "apagados" se ven igual: el motor simplemente NO gira.
--
-- LEDs (activo-bajo: '0' enciende):
--   led_1 = vida (parpadeo 1 Hz)   -> confirma que el diseño corre.
--   led_2 = encendido en fases de M1 (1-4) -> debería moverse el motor IZQ.
--   led_3 = encendido en fases de M2 (5-8) -> debería moverse el motor DER.
--
-- Si se mueve el motor EQUIVOCADO, o al REVÉS, el cableado/pines están cruzados.
--
-- Para construir: en SeguidorLinea_Brazo.qsf poner TOP_LEVEL_ENTITY = TestMotores
-- (se reusan las asignaciones de pin por NOMBRE). Revertir a SeguidorLinea_Brazo
-- después de la prueba.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity TestMotores is
    generic (
        PHASE_CYCLES : integer := 75_000_000   -- duración de cada fase (1.5 s @ 50 MHz)
    );
    port (
        clk      : in  std_logic;              -- PIN_17, 50 MHz
        reset    : in  std_logic;              -- PIN_144, activo bajo
        motor_a1 : out std_logic;              -- PIN_24  (IZQ adelante)
        motor_a2 : out std_logic;              -- PIN_26  (IZQ reversa)
        motor_b1 : out std_logic;              -- PIN_8   (DER adelante)
        motor_b2 : out std_logic;              -- PIN_4   (DER reversa)
        led_1    : out std_logic;              -- PIN_3 (vida 1 Hz)
        led_2    : out std_logic;              -- PIN_7 (fase M1)
        led_3    : out std_logic               -- PIN_9 (fase M2)
    );
end TestMotores;

architecture rtl of TestMotores is

    signal rst_alto : std_logic;               -- reset normalizado a activo-alto

    signal fase     : integer range 0 to 7 := 0;
    signal fase_cnt : integer range 0 to PHASE_CYCLES-1 := 0;

    -- 1 Hz (LED de vida)
    signal clk_1s : std_logic := '0';
    signal cnt_1s : integer range 0 to 24_999_999 := 0;

begin

    rst_alto <= not reset;                     -- el reset de la placa es activo-bajo

    -- -------------------------------------------------------------------------
    -- Secuenciador de fases: avanza cada PHASE_CYCLES y reinicia en loop
    -- -------------------------------------------------------------------------
    p_seq : process(clk)
    begin
        if rising_edge(clk) then
            if rst_alto = '1' then
                fase     <= 0;
                fase_cnt <= 0;
            elsif fase_cnt = PHASE_CYCLES-1 then
                fase_cnt <= 0;
                if fase = 7 then fase <= 0; else fase <= fase + 1; end if;
            else
                fase_cnt <= fase_cnt + 1;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    -- Generador 1 Hz (LED de vida)
    -- -------------------------------------------------------------------------
    p_1hz : process(clk)
    begin
        if rising_edge(clk) then
            if rst_alto = '1' then
                cnt_1s <= 0; clk_1s <= '0';
            elsif cnt_1s = 24_999_999 then
                cnt_1s <= 0; clk_1s <= not clk_1s;
            else
                cnt_1s <= cnt_1s + 1;
            end if;
        end if;
    end process;

    -- -------------------------------------------------------------------------
    -- Salidas de motor según la fase (combinacional)
    --   fase 0..3 = prueba de M1 (IZQ); fase 4..7 = prueba de M2 (DER)
    -- -------------------------------------------------------------------------
    process(fase)
    begin
        -- por defecto: todo frenado/apagado
        motor_a1 <= '0'; motor_a2 <= '0';
        motor_b1 <= '0'; motor_b2 <= '0';

        case fase is
            when 0 => motor_a1 <= '1';   -- M1 full adelante
            when 1 => null;              -- M1 freno (ambas en 0)
            when 2 => motor_a2 <= '1';   -- M1 full atrás
            when 3 => null;              -- M1 freno
            when 4 => motor_b1 <= '1';   -- M2 full adelante
            when 5 => null;              -- M2 freno
            when 6 => motor_b2 <= '1';   -- M2 full atrás
            when 7 => null;              -- M2 freno
        end case;
    end process;

    -- -------------------------------------------------------------------------
    -- LEDs (activo-bajo): vida + qué motor debería moverse
    -- -------------------------------------------------------------------------
    led_1 <= clk_1s;
    led_2 <= '0' when fase <= 3 else '1';      -- fases de M1
    led_3 <= '0' when fase >= 4 else '1';      -- fases de M2

end rtl;
