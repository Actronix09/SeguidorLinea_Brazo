-- ============================================================================
-- tb_VL53L0X - Testbench autocomprobante del driver VL53L0X
-- ----------------------------------------------------------------------------
-- Incluye un modelo conductual de esclavo I2C que emula al VL53L0X:
--   * Responde 0xEE al leer 0xC0 (MODEL_ID).
--   * Acepta (ACK) todas las escrituras de init/tuning y las almacena.
--   * Devuelve bit0=1 al leer 0x14 (data-ready siempre listo).
--   * Devuelve 0x00 / 0x7B en 0x1E / 0x1F  -> rango = 123 mm.
--
-- Comprobaciones: tras la 1ª medición debe cumplirse
--   distance_mm = 0x0049 (73 = 123 crudo - 50 mm de offset), data_valid='1',
--   sensor_ok='1', err_code=0.
--
-- Simulación recomendada (ModelSim-Altera, incluido en Quartus 13.0 SP1):
--   vcom vl53l0x_pkg.vhd VL53L0X.vhd tb_VL53L0X.vhd
--   vsim tb_VL53L0X -do "run 5 ms"
-- ============================================================================

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_VL53L0X is
end tb_VL53L0X;

architecture sim of tb_VL53L0X is

    component VL53L0X
        generic (
            CLK_FREQ_HZ  : integer := 50_000_000;
            I2C_FREQ_HZ  : integer := 100_000;
            PWRUP_CYCLES : integer := 500_000
        );
        port (
            clk         : in    std_logic;
            rst         : in    std_logic;
            i2c_scl     : out   std_logic;
            i2c_sda     : inout std_logic;
            distance_mm : out   std_logic_vector(15 downto 0);
            data_valid  : out   std_logic;
            sensor_ok   : out   std_logic;
            err_code    : out   std_logic_vector(4 downto 0);
            meas_tick   : out   std_logic
        );
    end component;

    signal clk        : std_logic := '0';
    signal rst        : std_logic := '1';
    signal scl        : std_logic;
    signal sda        : std_logic;
    signal sda_clean  : std_logic;
    signal sda_drv    : std_logic := 'Z';

    signal distance_mm : std_logic_vector(15 downto 0);
    signal data_valid  : std_logic;
    signal sensor_ok   : std_logic;
    signal err_code    : std_logic_vector(4 downto 0);
    signal meas_tick   : std_logic;

    signal sim_done : boolean := false;

    constant SLAVE_ADDR7 : std_logic_vector(6 downto 0) := "0101001"; -- 0x29

begin

    -- ----------------------------------------------------------------
    -- Bus: pull-up en SDA; SCL lo conduce el master (DUT)
    -- ----------------------------------------------------------------
    sda       <= 'H';        -- pull-up débil
    sda       <= sda_drv;    -- driver del esclavo (open-drain: '0'/'Z')
    sda_clean <= '0' when sda = '0' else '1';

    -- ----------------------------------------------------------------
    -- DUT (generics reducidos para acelerar la simulación)
    -- ----------------------------------------------------------------
    dut : VL53L0X
        generic map (
            CLK_FREQ_HZ  => 2_000_000,
            I2C_FREQ_HZ  => 100_000,   -- QUARTER = 5 ciclos
            PWRUP_CYCLES => 50
        )
        port map (
            clk         => clk,
            rst         => rst,
            i2c_scl     => scl,
            i2c_sda     => sda,
            distance_mm => distance_mm,
            data_valid  => data_valid,
            sensor_ok   => sensor_ok,
            err_code    => err_code,
            meas_tick   => meas_tick
        );

    -- ----------------------------------------------------------------
    -- Reloj
    -- ----------------------------------------------------------------
    clk_proc : process
    begin
        while not sim_done loop
            clk <= '0'; wait for 10 ns;
            clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    -- ----------------------------------------------------------------
    -- Modelo conductual de esclavo I2C (emula al VL53L0X)
    -- ----------------------------------------------------------------
    slave : process(scl, sda_clean)
        constant PH_ADDR : integer := 0;
        constant PH_WR   : integer := 1;
        constant PH_RD   : integer := 2;
        type mem_t is array(0 to 255) of std_logic_vector(7 downto 0);
        variable mem     : mem_t := (16#C0# => x"EE",   -- MODEL_ID
                                     16#C2# => x"10",   -- REVISION_ID
                                     16#13# => x"04",   -- INTERRUPT_STATUS: listo (calibración)
                                     16#14# => x"01",   -- RANGE_STATUS: data ready
                                     16#1F# => x"7B",   -- rango LSB = 123
                                     others => x"00");
        variable phase   : integer := PH_ADDR;
        variable bcnt    : integer := 0;
        variable shin    : std_logic_vector(7 downto 0) := (others => '0');
        variable shout   : std_logic_vector(7 downto 0) := (others => '0');
        variable ptr     : integer range 0 to 255 := 0;
        variable first_wr: boolean := true;
        variable is_read : std_logic := '0';
        variable active  : boolean := false;
    begin
        -- START / STOP: transición de SDA con SCL alto
        if scl = '1' and sda_clean'event then
            if sda_clean = '0' then               -- START / repeated START
                active   := true;
                phase    := PH_ADDR;
                bcnt     := 0;
                first_wr := true;
                sda_drv  <= 'Z';
            else                                  -- STOP
                active  := false;
                phase   := PH_ADDR;
                bcnt    := 0;
                sda_drv <= 'Z';
            end if;

        elsif rising_edge(scl) then
            if active then
                if bcnt < 8 then
                    if phase = PH_ADDR or phase = PH_WR then
                        shin(7 - bcnt) := sda_clean;   -- recibe MSB primero
                    end if;
                    bcnt := bcnt + 1;
                else
                    -- bcnt = 8: pulso de ACK
                    if phase = PH_ADDR then
                        if is_read = '1' then
                            phase := PH_RD;
                            shout := mem(ptr);
                        else
                            phase    := PH_WR;
                            first_wr := true;
                        end if;
                        bcnt := 0;
                    elsif phase = PH_WR then
                        bcnt := 0;
                    else  -- PH_RD: muestrea ACK/NACK del master
                        if sda_clean = '0' then        -- ACK -> el master quiere más bytes
                            ptr   := (ptr + 1) mod 256;
                            shout := mem(ptr);
                        else                            -- NACK -> fin de lectura: soltar el bus
                            sda_drv <= 'Z';
                            phase   := PH_ADDR;         -- espera START/STOP (no conducir más)
                        end if;
                        bcnt := 0;
                    end if;
                end if;
            end if;

        elsif falling_edge(scl) then
            if active then
                if bcnt = 8 then
                    -- preparar ACK (recepción) o soltar (lectura)
                    if phase = PH_RD then
                        sda_drv <= 'Z';
                    elsif phase = PH_ADDR then
                        if shin(7 downto 1) = SLAVE_ADDR7 then
                            sda_drv <= '0';            -- ACK dirección
                            is_read := shin(0);
                        else
                            sda_drv <= 'Z';            -- NACK (no coincide)
                        end if;
                    else  -- PH_WR
                        sda_drv <= '0';                -- ACK dato
                        if first_wr then
                            ptr      := to_integer(unsigned(shin));
                            first_wr := false;
                        else
                            mem(ptr) := shin;
                            ptr      := (ptr + 1) mod 256;
                        end if;
                    end if;
                else
                    -- bits 0..7
                    if phase = PH_RD then
                        if shout(7 - bcnt) = '0' then sda_drv <= '0'; else sda_drv <= 'Z'; end if;
                    else
                        sda_drv <= 'Z';
                    end if;
                end if;
            end if;
        end if;
    end process;

    -- ----------------------------------------------------------------
    -- Estímulo y comprobaciones
    -- ----------------------------------------------------------------
    stim : process
        variable t0 : std_logic;
    begin
        rst <= '1';
        wait for 200 ns;
        rst <= '0';

        -- 1) Primera medición válida (con timeout de seguridad)
        wait until data_valid = '1' for 5 ms;
        assert data_valid = '1'
            report "FALLO: no se obtuvo medicion valida (timeout)" severity error;
        assert sensor_ok = '1'
            report "FALLO: sensor_ok no esta activo" severity error;
        assert err_code = "00000"
            report "FALLO: err_code distinto de 0" severity error;
        assert distance_mm = x"0049"
            report "FALLO: distancia incorrecta (esperado 73 mm = 123-50 offset / 0x0049)" severity error;
        report "Medicion 1 OK = " &
               integer'image(to_integer(unsigned(distance_mm))) & " mm" severity note;

        -- 2) Bucle continuo: confirmar 2 mediciones más (meas_tick conmuta)
        t0 := meas_tick;
        wait until meas_tick /= t0 for 1 ms;
        assert meas_tick /= t0
            report "FALLO: no hubo 2a medicion (meas_tick no conmuto)" severity error;
        assert distance_mm = x"0049" and err_code = "00000"
            report "FALLO: 2a medicion incorrecta" severity error;

        t0 := meas_tick;
        wait until meas_tick /= t0 for 1 ms;
        assert meas_tick /= t0
            report "FALLO: no hubo 3a medicion" severity error;
        assert distance_mm = x"0049" and err_code = "00000"
            report "FALLO: 3a medicion incorrecta" severity error;

        report "OK: bucle continuo (3 mediciones, 73 mm estable, err_code=0)" severity note;

        sim_done <= true;
        wait;
    end process;

end sim;
