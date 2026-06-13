-- ============================================================================
-- vl53l0x_pkg - Constantes y ROMs para el driver VHDL del sensor VL53L0X
-- FPGA: Cyclone II EP2C5T144C7 | Placa: RZ-EasyFPGA A2.2 | Reloj: 50 MHz
-- ----------------------------------------------------------------------------
-- Contiene:
--   * Dirección I2C y bytes de bus (escritura/lectura)
--   * Direcciones de registro del sensor (subconjunto mínimo funcional)
--   * Códigos de error (5 bits) reportados por el driver
--   * TUNING_ROM : las 80 escrituras de DefaultTuningSettings (vl53l0x_tuning.h)
--   * INIT_ROM / MEAS_ROM : secuencias de comandos para init y arranque de medida
--
-- Equivale funcionalmente a Adafruit begin() + readRange() del sensor.
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

package vl53l0x_pkg is

    -- ------------------------------------------------------------------------
    -- Dirección I2C (7-bit = 0x29) y bytes de bus
    -- ------------------------------------------------------------------------
    constant VL53_ADDR_W : std_logic_vector(7 downto 0) := x"52";  -- (0x29<<1)|0
    constant VL53_ADDR_R : std_logic_vector(7 downto 0) := x"53";  -- (0x29<<1)|1

    -- ------------------------------------------------------------------------
    -- Registros relevantes
    -- ------------------------------------------------------------------------
    constant REG_SYSRANGE_START : std_logic_vector(7 downto 0) := x"00";
    constant REG_SEQ_CONFIG     : std_logic_vector(7 downto 0) := x"01";
    constant REG_INT_CFG_GPIO   : std_logic_vector(7 downto 0) := x"0A";
    constant REG_INT_CLEAR      : std_logic_vector(7 downto 0) := x"0B";
    constant REG_INT_STATUS     : std_logic_vector(7 downto 0) := x"13";
    constant REG_RANGE_STATUS   : std_logic_vector(7 downto 0) := x"14";
    constant REG_RANGE_MM       : std_logic_vector(7 downto 0) := x"1E";  -- 0x14+10 (MSB)
    constant REG_GPIO_HV_MUX    : std_logic_vector(7 downto 0) := x"84";
    constant REG_MODEL_ID       : std_logic_vector(7 downto 0) := x"C0";
    constant REG_STOP_VAR       : std_logic_vector(7 downto 0) := x"91";
    constant EXPECTED_MODEL_ID  : std_logic_vector(7 downto 0) := x"EE";

    -- ------------------------------------------------------------------------
    -- Códigos de error (mostrados en los 5 bits de debug con led_2 encendido)
    -- ------------------------------------------------------------------------
    constant ERR_NONE      : std_logic_vector(4 downto 0) := "00000";
    constant ERR_NACK_ADDR : std_logic_vector(4 downto 0) := "00001"; -- sensor ausente
    constant ERR_MODEL_ID  : std_logic_vector(4 downto 0) := "00010"; -- ID != 0xEE
    constant ERR_NACK_INIT : std_logic_vector(4 downto 0) := "00011"; -- NACK en init/tuning
    constant ERR_TIMEOUT   : std_logic_vector(4 downto 0) := "00100"; -- sin data-ready

    -- ------------------------------------------------------------------------
    -- ROM de tuning: pares (registro, valor)
    -- Derivada de DefaultTuningSettings[] en vl53l0x_tuning.h: cada terna
    -- (0x01, reg, val) se convierte en una escritura (reg, val). Se descartan
    -- los flags 0x01 y el terminador 0x00,0x00,0x00. Total: 80 escrituras.
    -- ------------------------------------------------------------------------
    type reg_val_t is record
        reg : std_logic_vector(7 downto 0);
        val : std_logic_vector(7 downto 0);
    end record;
    type reg_val_array_t is array (natural range <>) of reg_val_t;

    constant TUNING_ROM : reg_val_array_t := (
        (x"FF", x"01"), (x"00", x"00"),
        (x"FF", x"00"), (x"09", x"00"), (x"10", x"00"), (x"11", x"00"),
        (x"24", x"01"), (x"25", x"FF"), (x"75", x"00"),
        (x"FF", x"01"), (x"4E", x"2C"), (x"48", x"00"), (x"30", x"20"),
        (x"FF", x"00"), (x"30", x"09"),
        (x"54", x"00"), (x"31", x"04"), (x"32", x"03"), (x"40", x"83"),
        (x"46", x"25"), (x"60", x"00"), (x"27", x"00"), (x"50", x"06"),
        (x"51", x"00"), (x"52", x"96"), (x"56", x"08"), (x"57", x"30"),
        (x"61", x"00"), (x"62", x"00"), (x"64", x"00"), (x"65", x"00"),
        (x"66", x"A0"),
        (x"FF", x"01"), (x"22", x"32"), (x"47", x"14"), (x"49", x"FF"),
        (x"4A", x"00"),
        (x"FF", x"00"), (x"7A", x"0A"), (x"7B", x"00"), (x"78", x"21"),
        (x"FF", x"01"), (x"23", x"34"), (x"42", x"00"), (x"44", x"FF"),
        (x"45", x"26"), (x"46", x"05"), (x"40", x"40"), (x"0E", x"06"),
        (x"20", x"1A"), (x"43", x"40"),
        (x"FF", x"00"), (x"34", x"03"), (x"35", x"44"),
        (x"FF", x"01"), (x"31", x"04"), (x"4B", x"09"), (x"4C", x"05"),
        (x"4D", x"04"),
        (x"FF", x"00"), (x"44", x"00"), (x"45", x"20"), (x"47", x"08"),
        (x"48", x"28"), (x"67", x"00"), (x"70", x"04"), (x"71", x"01"),
        (x"72", x"FE"), (x"76", x"00"), (x"77", x"00"),
        (x"FF", x"01"), (x"0D", x"01"),
        (x"FF", x"00"), (x"80", x"01"), (x"01", x"F8"),
        (x"FF", x"01"), (x"8E", x"01"), (x"00", x"01"), (x"FF", x"00"),
        (x"80", x"00")
    );

    -- ------------------------------------------------------------------------
    -- ROMs de comandos para la FSM de aplicación
    -- ------------------------------------------------------------------------
    type opcode_t is (OP_WR, OP_OR, OP_AND, OP_RD_SV, OP_WR_SV,
                      OP_CHK_ID, OP_TUNING, OP_END);
    type cmd_t is record
        op  : opcode_t;
        reg : std_logic_vector(7 downto 0);
        val : std_logic_vector(7 downto 0);
    end record;
    type cmd_array_t is array (natural range <>) of cmd_t;

    -- DataInit + StaticInit (mínimo, estilo Pololu/ST). OP_OR/OP_AND = RMW.
    constant INIT_ROM : cmd_array_t := (
        (OP_CHK_ID, x"C0", x"EE"),  -- verificar MODEL_ID
        (OP_OR,     x"89", x"01"),  -- habilitar 2.8 V (read-modify-write)
        (OP_WR,     x"88", x"00"),  -- I2C standard mode
        (OP_WR,     x"80", x"01"),
        (OP_WR,     x"FF", x"01"),
        (OP_WR,     x"00", x"00"),
        (OP_RD_SV,  x"91", x"00"),  -- leer y guardar stop-variable
        (OP_WR,     x"00", x"01"),
        (OP_WR,     x"FF", x"00"),
        (OP_WR,     x"80", x"00"),
        (OP_WR,     x"01", x"FF"),  -- SEQUENCE_CONFIG: habilitar todo
        (OP_TUNING, x"00", x"00"),  -- cargar las 80 escrituras de tuning
        (OP_WR,     x"0A", x"04"),  -- interrupt = new sample ready
        (OP_AND,    x"84", x"EF"),  -- GPIO activo bajo (read-modify-write)
        (OP_WR,     x"0B", x"01"),  -- limpiar interrupción
        (OP_WR,     x"01", x"E8"),  -- pasos de secuencia finales
        (OP_END,    x"00", x"00")
    );

    -- Arranque de una medición single-shot (magia con stop-variable + START)
    constant MEAS_ROM : cmd_array_t := (
        (OP_WR,     x"80", x"01"),
        (OP_WR,     x"FF", x"01"),
        (OP_WR,     x"00", x"00"),
        (OP_WR_SV,  x"91", x"00"),  -- escribir stop-variable guardada
        (OP_WR,     x"00", x"01"),
        (OP_WR,     x"FF", x"00"),
        (OP_WR,     x"80", x"00"),
        (OP_WR,     x"00", x"01"),  -- SYSRANGE_START
        (OP_END,    x"00", x"00")
    );

end package vl53l0x_pkg;
