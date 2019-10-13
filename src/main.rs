#![no_std]
#![no_main]
#![allow(deprecated)]

extern crate panic_itm; // logs messages over ITM; requires ITM support

mod led;
mod storage;

type LedOutputPinState = stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>;

#[rtfm::app(device = stm32f4xx_hal::stm32)]
const APP: () = {
    struct Resources {
        led_display: led::display::LedDisplay<
            core::convert::Infallible,
            led::display::GpioLayerSelector<
                core::convert::Infallible,
                stm32f4xx_hal::gpio::gpioc::PC11<LedOutputPinState>,
                stm32f4xx_hal::gpio::gpiod::PD0<LedOutputPinState>,
                stm32f4xx_hal::gpio::gpioc::PC10<LedOutputPinState>,
                stm32f4xx_hal::gpio::gpioc::PC12<LedOutputPinState>,
            >,
            led::display::GpioDataBus<
                core::convert::Infallible,
                stm32f4xx_hal::gpio::gpioc::PC8<LedOutputPinState>,
                stm32f4xx_hal::gpio::gpioe::PE2<LedOutputPinState>,
                stm32f4xx_hal::gpio::gpioe::PE3<LedOutputPinState>,
                stm32f4xx_hal::gpio::gpioc::PC7<LedOutputPinState>,
                stm32f4xx_hal::gpio::gpioc::PC6<LedOutputPinState>,
                stm32f4xx_hal::gpio::gpiod::PD15<LedOutputPinState>,
            >,
        >,
        led_matrix: led::matrix::LedMatrix,
        led_timer: stm32f4xx_hal::timer::Timer<stm32f4xx_hal::stm32::TIM4>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        init_impl(cx)
    }

    #[task(binds = TIM4, resources = [led_display, led_matrix, led_timer])]
    fn update_led_display(cx: update_led_display::Context) {
        update_led_display_impl(cx)
    }

    extern "C" {
        fn EXTI0();
    }
};

fn init_impl(_cx: init::Context) -> init::LateResources {
    use stm32f4xx_hal::time::U32Ext;

    // TODO: why are we not getting this through cx.device?
    let device = stm32f4xx_hal::stm32::Peripherals::take().unwrap();

    let clocks = stm32f4xx_hal::rcc::RccExt::constrain(device.RCC)
        .cfgr
        .sysclk(168.mhz())
        .hclk(168.mhz())
        .use_hse(25.mhz())
        .freeze();

    let gpioc = stm32f4xx_hal::gpio::GpioExt::split(device.GPIOC);
    let gpiod = stm32f4xx_hal::gpio::GpioExt::split(device.GPIOD);
    let gpioe = stm32f4xx_hal::gpio::GpioExt::split(device.GPIOE);

    // we need to use a macro because the pins don't implement a common trait
    macro_rules! output_pin {
        ($pin:expr) => {
            $pin.into_push_pull_output()
                .set_speed(stm32f4xx_hal::gpio::Speed::VeryHigh)
        };
    };

    let layer_address0 = output_pin!(gpioc.pc11);
    let layer_address1 = output_pin!(gpiod.pd0);
    let layer_address2 = output_pin!(gpioc.pc10);
    let layer_address3 = output_pin!(gpioc.pc12);

    let layer_selector = led::display::GpioLayerSelector::new(
        layer_address0,
        layer_address1,
        layer_address2,
        layer_address3,
    );

    let data_a = output_pin!(gpioc.pc8);
    let data_b = output_pin!(gpioe.pe2);
    let data_c = output_pin!(gpioe.pe3);
    let data_clk = output_pin!(gpioc.pc7);
    let data_stk = output_pin!(gpioc.pc6);
    let data_oe = output_pin!(gpiod.pd15);

    let data_bus =
        led::display::GpioDataBus::new(data_a, data_b, data_c, data_clk, data_stk, data_oe);

    let led_display = led::display::LedDisplay::new(layer_selector, data_bus);

    let mut led_matrix = led::matrix::LedMatrix::new();

    for z in 0..led::matrix::CUBE_SIZE {
        for y in 0..led::matrix::CUBE_SIZE {
            for x in 0..led::matrix::CUBE_SIZE {
                led_matrix[led::matrix::Coord::xyz(x, y, z)] = led::matrix::Color::rgb(
                    (x * 255 / led::matrix::CUBE_SIZE) as u8,
                    (y * 255 / led::matrix::CUBE_SIZE) as u8,
                    (z * 255 / led::matrix::CUBE_SIZE) as u8,
                );
            }
        }
    }

    let mut led_timer = stm32f4xx_hal::timer::Timer::tim4(device.TIM4, 100000.hz(), clocks);
    led_timer.listen(stm32f4xx_hal::timer::Event::TimeOut);

    init::LateResources {
        led_display,
        led_matrix,
        led_timer,
    }
}

fn update_led_display_impl(cx: update_led_display::Context) {
    use embedded_hal::timer::CountDown;
    use stm32f4xx_hal::time::U32Ext;

    cx.resources
        .led_timer
        .clear_interrupt(stm32f4xx_hal::timer::Event::TimeOut);

    let update_freq_hz = cx
        .resources
        .led_display
        .update(&cx.resources.led_matrix)
        .unwrap();

    if let Some(update_freq_hz) = update_freq_hz {
        cx.resources.led_timer.start(update_freq_hz.hz());
    }
}
