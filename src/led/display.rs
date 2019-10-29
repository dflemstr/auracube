use super::gamma;
use super::matrix;
use crate::config;

#[derive(Debug)]
pub struct LedDisplay<E, L, D>
where
    L: LayerSelector<Error = E>,
    D: DataBus<Error = E>,
{
    layer_selector: L,
    data_bus: D,
    z: usize,
    bit: u8,
}

pub trait LayerSelector {
    type Error;
    /// Configures the pin multiplexers to select a certain Z layer.
    fn select_layer(&mut self, layer: usize) -> Result<(), Self::Error>;
}

pub trait DataBus {
    type Error;

    /// Set whether the output of the shift registers should be shown on the actual LED matrix;
    /// this should be set to false while we update the LEDs.
    fn set_stk(&mut self, stk: bool) -> Result<(), Self::Error>;
    fn set_oe(&mut self, stk: bool) -> Result<(), Self::Error>;
    fn send_data(&mut self, data_a: bool, data_b: bool, data_c: bool) -> Result<(), Self::Error>;
}

pub struct GpioLayerSelector<E, A0, A1, A2, A3>
where
    A0: embedded_hal::digital::v2::OutputPin<Error = E>,
    A1: embedded_hal::digital::v2::OutputPin<Error = E>,
    A2: embedded_hal::digital::v2::OutputPin<Error = E>,
    A3: embedded_hal::digital::v2::OutputPin<Error = E>,
{
    layer_address0: A0,
    layer_address1: A1,
    layer_address2: A2,
    layer_address3: A3,
}

pub struct GpioDataBus<E, DA, DB, DC, CLK, STK, OE>
where
    DA: embedded_hal::digital::v2::OutputPin<Error = E>,
    DB: embedded_hal::digital::v2::OutputPin<Error = E>,
    DC: embedded_hal::digital::v2::OutputPin<Error = E>,
    CLK: embedded_hal::digital::v2::OutputPin<Error = E>,
    STK: embedded_hal::digital::v2::OutputPin<Error = E>,
    OE: embedded_hal::digital::v2::OutputPin<Error = E>,
{
    data_a: DA,
    data_b: DB,
    data_c: DC,
    clk: CLK,
    stk: STK,
    oe: OE,
}
// in order to reach UPDATE_RATE_HZ complete updates per second, we need to run this
// method CUBE_SIZE times per update (to go through all z layers) * the sum of the gamma values
// (which accounts for the delay due to gamma strobing).
const BIT_DEPTH: u8 = 8;
const ADJUSTED_UPDATE_RATE_HZ: f32 = gamma::GAMMA_SUM_WEIGHT
    * config::FRAME_RATE_HZ as f32
    * BIT_DEPTH as f32
    * matrix::CUBE_SIZE as f32;

impl<E, L, D> LedDisplay<E, L, D>
where
    L: LayerSelector<Error = E>,
    D: DataBus<Error = E>,
{
    pub fn new(layer_selector: L, data_bus: D) -> Self {
        let bit = 0;
        let z = 0;

        Self {
            layer_selector,
            data_bus,
            bit,
            z,
        }
    }

    pub fn is_at_frame_beginning(&self) -> bool {
        self.bit == 0 && self.z == 0
    }

    pub fn is_at_frame_end(&self) -> bool {
        self.bit == BIT_DEPTH - 1 && self.z == matrix::CUBE_SIZE - 1
    }

    #[inline]
    pub fn update(&mut self, matrix: &super::matrix::LedMatrix) -> Result<Option<u32>, E> {
        self.z += 1;
        self.z %= matrix::CUBE_SIZE;

        let next_freq = if self.z == 0 {
            self.bit += 1;
            self.bit %= BIT_DEPTH;

            // the frequency to set the clock to in order to wait for the next cycle
            let b = self.bit as usize;
            let new_freq_hz = ADJUSTED_UPDATE_RATE_HZ / gamma::GAMMA_TABLE[b] as f32;
            Some(new_freq_hz as u32)
        } else {
            None
        };

        // To ensure the cube updates top-down
        let flipped_z = matrix::CUBE_SIZE - self.z - 1;
        self.update_z_bit(flipped_z, self.bit, matrix)?;
        Ok(next_freq)
    }

    #[inline]
    fn update_z_bit(
        &mut self,
        z: usize,
        bit: u8,
        matrix: &super::matrix::LedMatrix,
    ) -> Result<(), E> {
        self.data_bus.set_oe(false)?;
        self.layer_selector.select_layer(z)?;

        self.data_bus.set_stk(false)?;

        // There are 3 shift registers each connected to data lines A, B and C.  Each shift
        // register owns 4 "slices" of the cube (hence 3 registers * 4 slices = 12 total)

        for y_slice in 0..matrix::CUBE_SIZE / 3 {
            // Not sure why but the shift registers expect 1 empty "row" of data
            for _ in 0..matrix::CUBE_SIZE {
                self.data_bus.send_data(false, false, false)?;
            }
            for x in 0..matrix::CUBE_SIZE {
                let color_a = matrix.xyz(x, 0 + y_slice, z);
                let color_b = matrix.xyz(x, 4 + y_slice, z);
                let color_c = matrix.xyz(x, 8 + y_slice, z);

                self.data_bus.send_data(
                    color_a.g & (1u8 << bit) != 0,
                    color_b.g & (1u8 << bit) != 0,
                    color_c.g & (1u8 << bit) != 0,
                )?;
                self.data_bus.send_data(
                    color_a.b & (1u8 << bit) != 0,
                    color_b.b & (1u8 << bit) != 0,
                    color_c.b & (1u8 << bit) != 0,
                )?;
                self.data_bus.send_data(
                    color_a.r & (1u8 << bit) != 0,
                    color_b.r & (1u8 << bit) != 0,
                    color_c.r & (1u8 << bit) != 0,
                )?;
            }
        }

        self.data_bus.set_stk(true)?;
        self.data_bus.set_oe(true)?;
        Ok(())
    }
}

impl<E, A0, A1, A2, A3> GpioLayerSelector<E, A0, A1, A2, A3>
where
    A0: embedded_hal::digital::v2::OutputPin<Error = E>,
    A1: embedded_hal::digital::v2::OutputPin<Error = E>,
    A2: embedded_hal::digital::v2::OutputPin<Error = E>,
    A3: embedded_hal::digital::v2::OutputPin<Error = E>,
{
    pub fn new(
        layer_address0: A0,
        layer_address1: A1,
        layer_address2: A2,
        layer_address3: A3,
    ) -> Self {
        Self {
            layer_address0,
            layer_address1,
            layer_address2,
            layer_address3,
        }
    }
}

impl<E, A0, A1, A2, A3> LayerSelector for GpioLayerSelector<E, A0, A1, A2, A3>
where
    A0: embedded_hal::digital::v2::OutputPin<Error = E>,
    A1: embedded_hal::digital::v2::OutputPin<Error = E>,
    A2: embedded_hal::digital::v2::OutputPin<Error = E>,
    A3: embedded_hal::digital::v2::OutputPin<Error = E>,
{
    type Error = E;

    #[inline]
    fn select_layer(&mut self, layer: usize) -> Result<(), Self::Error> {
        if layer & 0b0001 != 0 {
            self.layer_address0.set_high()?;
        } else {
            self.layer_address0.set_low()?;
        }

        if layer & 0b0010 != 0 {
            self.layer_address1.set_high()?;
        } else {
            self.layer_address1.set_low()?;
        }

        if layer & 0b0100 != 0 {
            self.layer_address2.set_high()?;
        } else {
            self.layer_address2.set_low()?;
        }

        if layer & 0b1000 != 0 {
            self.layer_address3.set_high()?;
        } else {
            self.layer_address3.set_low()?;
        }

        Ok(())
    }
}

impl<E, DA, DB, DC, CLK, STK, OE> GpioDataBus<E, DA, DB, DC, CLK, STK, OE>
where
    DA: embedded_hal::digital::v2::OutputPin<Error = E>,
    DB: embedded_hal::digital::v2::OutputPin<Error = E>,
    DC: embedded_hal::digital::v2::OutputPin<Error = E>,
    CLK: embedded_hal::digital::v2::OutputPin<Error = E>,
    STK: embedded_hal::digital::v2::OutputPin<Error = E>,
    OE: embedded_hal::digital::v2::OutputPin<Error = E>,
{
    pub fn new(data_a: DA, data_b: DB, data_c: DC, clk: CLK, stk: STK, oe: OE) -> Self {
        Self {
            data_a,
            data_b,
            data_c,
            clk,
            stk,
            oe,
        }
    }
}

impl<E, DA, DB, DC, CLK, STK, OE> DataBus for GpioDataBus<E, DA, DB, DC, CLK, STK, OE>
where
    DA: embedded_hal::digital::v2::OutputPin<Error = E>,
    DB: embedded_hal::digital::v2::OutputPin<Error = E>,
    DC: embedded_hal::digital::v2::OutputPin<Error = E>,
    CLK: embedded_hal::digital::v2::OutputPin<Error = E>,
    STK: embedded_hal::digital::v2::OutputPin<Error = E>,
    OE: embedded_hal::digital::v2::OutputPin<Error = E>,
{
    type Error = E;

    #[inline]
    fn set_stk(&mut self, stk: bool) -> Result<(), Self::Error> {
        if stk {
            self.stk.set_high()
        } else {
            self.stk.set_low()
        }
    }

    #[inline]
    fn set_oe(&mut self, oe: bool) -> Result<(), Self::Error> {
        if oe {
            self.oe.set_low()
        } else {
            self.oe.set_high()
        }
    }

    #[inline]
    fn send_data(&mut self, data_a: bool, data_b: bool, data_c: bool) -> Result<(), Self::Error> {
        self.clk.set_low()?;

        if data_a {
            self.data_a.set_high()?;
        } else {
            self.data_a.set_low()?;
        }
        if data_b {
            self.data_b.set_high()?;
        } else {
            self.data_b.set_low()?;
        }
        if data_c {
            self.data_c.set_high()?;
        } else {
            self.data_c.set_low()?;
        }

        // wait for gpio values to settle
        cortex_m::asm::nop();
        cortex_m::asm::nop();
        cortex_m::asm::nop();

        self.clk.set_high()?;
        Ok(())
    }
}
