-- ============================================================================
-- tb_VL53L0X_recover - Verifica la RECUPERACIÓN del driver ante un sensor que
--   se "calla" a mitad de operación (caso real: tras varias mediciones el sensor
--   deja de responder y antes el driver entraba en bucle de error).
--
--   Modelo de esclavo I2C (como tb_VL53L0X) + señal 'fault': cuando fault='1' el
--   esclavo NO da ACK (simula sensor mudo/atascado). Secuencia:
--     1) init + 1ª medición OK (meas_tick conmuta).
--     2) fault='1': el driver entra en error -> recuperación ligera x4 -> DURA
--        (limpia bus + re-init). meas_tick NO conmuta (sin mediciones).
--     3) fault='0': el sensor vuelve -> el driver debe REANUDAR (meas_tick conmuta).
--
--   vcom -2008 vl53l0x_pkg.vhd VL53L0X.vhd tb_VL53L0X_recover.vhd
--   vsim tb_VL53L0X_recover -do "run -all"
-- ============================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_VL53L0X_recover is
end tb_VL53L0X_recover;

architecture sim of tb_VL53L0X_recover is

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

    signal fault    : std_logic := '0';   -- '1' = el esclavo no responde (sensor mudo)
    signal sim_done : boolean := false;

    constant SLAVE_ADDR7 : std_logic_vector(6 downto 0) := "0101001"; -- 0x29

begin

    sda       <= 'H';
    sda       <= sda_drv;
    sda_clean <= '0' when sda = '0' else '1';

    dut : entity work.VL53L0X
        generic map (CLK_FREQ_HZ => 2_000_000, I2C_FREQ_HZ => 100_000, PWRUP_CYCLES => 50)
        port map (
            clk => clk, rst => rst, i2c_scl => scl, i2c_sda => sda,
            distance_mm => distance_mm, data_valid => data_valid,
            sensor_ok => sensor_ok, err_code => err_code, meas_tick => meas_tick
        );

    clk_proc : process
    begin
        while not sim_done loop
            clk <= '0'; wait for 10 ns; clk <= '1'; wait for 10 ns;
        end loop;
        wait;
    end process;

    -- Modelo de esclavo I2C; con fault='1' NACKea la dirección (sensor mudo).
    slave : process(scl, sda_clean)
        constant PH_ADDR : integer := 0;
        constant PH_WR   : integer := 1;
        constant PH_RD   : integer := 2;
        type mem_t is array(0 to 255) of std_logic_vector(7 downto 0);
        variable mem     : mem_t := (16#C0# => x"EE",
                                     16#C2# => x"10",
                                     16#13# => x"04",
                                     16#14# => x"01",
                                     16#1F# => x"7B",
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
        if scl = '1' and sda_clean'event then
            if sda_clean = '0' then
                active := true; phase := PH_ADDR; bcnt := 0; first_wr := true; sda_drv <= 'Z';
            else
                active := false; phase := PH_ADDR; bcnt := 0; sda_drv <= 'Z';
            end if;

        elsif rising_edge(scl) then
            if active then
                if bcnt < 8 then
                    if phase = PH_ADDR or phase = PH_WR then
                        shin(7 - bcnt) := sda_clean;
                    end if;
                    bcnt := bcnt + 1;
                else
                    if phase = PH_ADDR then
                        if is_read = '1' then
                            phase := PH_RD; shout := mem(ptr);
                        else
                            phase := PH_WR; first_wr := true;
                        end if;
                        bcnt := 0;
                    elsif phase = PH_WR then
                        bcnt := 0;
                    else
                        if sda_clean = '0' then
                            ptr := (ptr + 1) mod 256; shout := mem(ptr);
                        else
                            sda_drv <= 'Z'; phase := PH_ADDR;
                        end if;
                        bcnt := 0;
                    end if;
                end if;
            end if;

        elsif falling_edge(scl) then
            if active then
                if bcnt = 8 then
                    if phase = PH_RD then
                        sda_drv <= 'Z';
                    elsif phase = PH_ADDR then
                        -- ACK la dirección SOLO si no hay fallo inyectado
                        if shin(7 downto 1) = SLAVE_ADDR7 and fault = '0' then
                            sda_drv <= '0'; is_read := shin(0);
                        else
                            sda_drv <= 'Z';            -- NACK (mudo) -> el driver entra en error
                        end if;
                    else  -- PH_WR
                        sda_drv <= '0';
                        if first_wr then
                            ptr := to_integer(unsigned(shin)); first_wr := false;
                        else
                            mem(ptr) := shin; ptr := (ptr + 1) mod 256;
                        end if;
                    end if;
                else
                    if phase = PH_RD then
                        if shout(7 - bcnt) = '0' then sda_drv <= '0'; else sda_drv <= 'Z'; end if;
                    else
                        sda_drv <= 'Z';
                    end if;
                end if;
            end if;
        end if;
    end process;

    stim : process
        variable t0 : std_logic;
    begin
        rst <= '1'; wait for 300 ns; rst <= '0';

        -- 1) Init + 1ª medición OK.
        wait until data_valid = '1' for 5 ms;
        assert data_valid = '1' report "FALLO: no hubo 1a medicion (timeout)" severity error;
        assert distance_mm = x"0049" report "FALLO: distancia inicial incorrecta" severity error;
        report "1a medicion OK (73 mm)" severity note;

        -- confirma que mide en bucle (meas_tick conmuta)
        t0 := meas_tick;
        wait until meas_tick /= t0 for 2 ms;
        assert meas_tick /= t0 report "FALLO: no mide en bucle antes del fallo" severity error;
        report "mide en bucle OK antes del fallo" severity note;

        -- 2) FALLO: el sensor se calla. El driver debe escalar a recuperación dura.
        fault <= '1';
        t0 := meas_tick;
        wait for 3 ms;                      -- deja que entre en error y reintente (sin colgarse)
        assert meas_tick = t0
            report "NOTA: meas_tick conmuto durante el fallo (inesperado)" severity warning;
        report "durante el fallo: sin mediciones, el driver reintenta (no se cuelga)" severity note;

        -- 3) El sensor vuelve: el driver debe REANUDAR las mediciones.
        fault <= '0';
        t0 := meas_tick;
        wait until meas_tick /= t0 for 8 ms;
        assert meas_tick /= t0
            report "FALLO: el driver NO se recupero tras volver el sensor (sigue en bucle)" severity error;
        assert sensor_ok = '1'
            report "FALLO: sensor_ok no volvio a '1' tras recuperar" severity error;
        assert distance_mm = x"0049"
            report "FALLO: medicion incorrecta tras recuperar" severity error;
        report "RECUPERADO OK: tras volver el sensor, reanuda mediciones (73 mm)" severity note;

        report "OK: el driver se recupera de un sensor que se calla (sin bucle infinito)" severity note;
        sim_done <= true;
        wait;
    end process;

end sim;
