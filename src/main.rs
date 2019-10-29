#![no_std]
#![no_main]
#![feature(alloc_error_handler)]
#![feature(clamp)]
#![allow(deprecated)]

extern crate alloc;
extern crate panic_halt;

mod audio;
mod config;
mod led;

#[global_allocator]
static ALLOCATOR: alloc_cortex_m::CortexMHeap = alloc_cortex_m::CortexMHeap::empty();

type LedOutputPinState = stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>;

type LedDisplay = led::display::LedDisplay<
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
>;

type LedMatrix = led::matrix::LedMatrix;
type LedNextMatrix = Option<led::matrix::LedMatrix>;
type LedTimer = stm32f4xx_hal::timer::Timer<stm32f4xx_hal::stm32::TIM4>;

type AudioTimer = stm32f4xx_hal::timer::Timer<stm32f4xx_hal::stm32::TIM2>;
type AudioAdc = stm32f4xx_hal::adc::Adc<stm32f4xx_hal::stm32::ADC1>;
type AudioLineInPin = stm32f4xx_hal::gpio::gpioa::PA1<stm32f4xx_hal::gpio::Analog>;
type AudioMicPin = stm32f4xx_hal::gpio::gpioa::PA2<stm32f4xx_hal::gpio::Analog>;
type AudioSampler = audio::sampler::Sampler;
type AudioFft = audio::spectrum::Fft;
type AudioSpectrum = audio::spectrum::Spectrum;
type AudioSpectrumCube = audio::spectrum::SpectrumCube;

#[rtfm::app(device = stm32f4xx_hal::stm32)]
const APP: () = {
    struct Resources {
        // LED-related resources
        led_display: LedDisplay,
        led_matrix: LedMatrix,
        led_next_matrix: LedNextMatrix,
        led_timer: LedTimer,

        // Audio sampling related resources
        audio_timer: AudioTimer,
        audio_adc: AudioAdc,
        audio_line_in_pin: AudioLineInPin,
        audio_mic_pin: AudioMicPin,
        audio_sampler: AudioSampler,
        audio_fft: AudioFft,
        audio_spectrum: AudioSpectrum,
        audio_spectrum_cube: AudioSpectrumCube,
    }

    #[init]
    fn init(_cx: init::Context) -> init::LateResources {
        init_impl()
    }

    #[task(binds = ADC, priority = 3, resources = [audio_adc, audio_sampler], spawn = [compute_audio_spectrum])]
    fn sample_audio_done(cx: sample_audio_done::Context) {
        let spawn = cx.spawn;
        sample_audio_done_impl(cx.resources.audio_adc, cx.resources.audio_sampler, || {
            // Ignore if already scheduled
            let _ = spawn.compute_audio_spectrum();
        })
    }

    #[task(binds = TIM2, priority = 3, resources = [audio_adc, audio_timer, audio_line_in_pin, audio_mic_pin])]
    fn sample_audio_start(cx: sample_audio_start::Context) {
        sample_audio_start_impl(
            cx.resources.audio_timer,
            cx.resources.audio_adc,
            cx.resources.audio_mic_pin,
            cx.resources.audio_line_in_pin,
        )
    }

    #[task(binds = TIM4, priority = 2, resources = [led_display, led_matrix, led_next_matrix, led_timer], spawn = [generate_next_led_frame])]
    fn update_led_display(cx: update_led_display::Context) {
        let spawn = cx.spawn;
        update_led_display_impl(
            cx.resources.led_timer,
            cx.resources.led_display,
            cx.resources.led_matrix,
            cx.resources.led_next_matrix,
            || {
                let _ = spawn.generate_next_led_frame();
            },
        )
    }

    #[task(priority = 1, resources = [audio_sampler, audio_fft, audio_spectrum])]
    fn compute_audio_spectrum(cx: compute_audio_spectrum::Context) {
        compute_audio_spectrum_impl(
            cx.resources.audio_sampler,
            cx.resources.audio_fft,
            cx.resources.audio_spectrum,
        )
    }

    #[task(priority = 1, resources = [led_next_matrix, audio_spectrum, audio_spectrum_cube])]
    fn generate_next_led_frame(cx: generate_next_led_frame::Context) {
        generate_next_led_frame_impl(
            cx.resources.led_next_matrix,
            cx.resources.audio_spectrum,
            cx.resources.audio_spectrum_cube,
        )
    }

    extern "C" {
        fn USART6();
    }
};

fn init_impl() -> init::LateResources {
    use stm32f4xx_hal::time::U32Ext;

    let start = cortex_m_rt::heap_start() as usize;
    let size = 65536; // in bytes
    unsafe { ALLOCATOR.init(start, size) }

    // TODO: why are we not getting this through cx.device?
    let device = stm32f4xx_hal::stm32::Peripherals::take().unwrap();

    let clocks = stm32f4xx_hal::rcc::RccExt::constrain(device.RCC)
        .cfgr
        .sysclk(168.mhz())
        .hclk(168.mhz())
        .use_hse(25.mhz())
        .freeze();

    let gpioa = stm32f4xx_hal::gpio::GpioExt::split(device.GPIOA);
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

    let led_matrix = led::matrix::LedMatrix::new();
    let led_next_matrix = None;

    let mut audio_timer = stm32f4xx_hal::timer::Timer::tim2(
        device.TIM2,
        (config::AUDIO_SAMPLE_HZ as u32).hz(),
        clocks,
    );
    audio_timer.listen(stm32f4xx_hal::timer::Event::TimeOut);

    let mut led_timer = stm32f4xx_hal::timer::Timer::tim4(device.TIM4, 100_000.hz(), clocks);
    led_timer.listen(stm32f4xx_hal::timer::Event::TimeOut);

    let audio_line_in_pin = gpioa.pa1.into_analog();
    let audio_mic_pin = gpioa.pa2.into_analog();

    let mut audio_adc = stm32f4xx_hal::adc::Adc::adc1(
        device.ADC1,
        false,
        stm32f4xx_hal::adc::config::AdcConfig::default(),
    );

    audio_adc.calibrate();
    audio_adc.set_end_of_conversion_interrupt(stm32f4xx_hal::adc::config::Eoc::Sequence);

    let audio_sampler = audio::sampler::Sampler::new();
    let audio_fft = audio::spectrum::Fft::new();
    let audio_spectrum = audio::spectrum::Spectrum::new();
    let audio_spectrum_cube = audio::spectrum::SpectrumCube::new();

    init::LateResources {
        led_display,
        led_matrix,
        led_next_matrix,
        led_timer,
        audio_timer,
        audio_adc,
        audio_line_in_pin,
        audio_mic_pin,
        audio_sampler,
        audio_fft,
        audio_spectrum,
        audio_spectrum_cube,
    }
}

fn sample_audio_done_impl(
    audio_adc: &mut AudioAdc,
    audio_sampler: &mut AudioSampler,
    spawn_compute_audio_spectrum: impl FnOnce(),
) {
    audio_adc.clear_end_of_conversion_flag();
    let value = audio_adc.current_sample();
    match audio_sampler.add_sample(value) {
        audio::sampler::Status::Complete => {
            spawn_compute_audio_spectrum();
        }
        audio::sampler::Status::NeedsMore => {}
    }
}

fn sample_audio_start_impl(
    audio_timer: &mut AudioTimer,
    audio_adc: &mut AudioAdc,
    audio_mic_pin: &AudioMicPin,
    audio_line_in_pin: &AudioLineInPin,
) {
    audio_timer.clear_interrupt(stm32f4xx_hal::timer::Event::TimeOut);

    audio_adc.enable();
    audio_adc.configure_channel(
        audio_line_in_pin,
        stm32f4xx_hal::adc::config::Sequence::One,
        stm32f4xx_hal::adc::config::SampleTime::Cycles_28,
    );
    audio_adc.start_conversion();
}

fn update_led_display_impl(
    led_timer: &mut LedTimer,
    led_display: &mut LedDisplay,
    led_matrix: &mut LedMatrix,
    led_next_matrix: &mut LedNextMatrix,
    spawn_generate_next_led_frame: impl FnOnce(),
) {
    use embedded_hal::timer::CountDown;
    use stm32f4xx_hal::time::U32Ext;

    led_timer.clear_interrupt(stm32f4xx_hal::timer::Event::TimeOut);

    if led_display.is_at_frame_beginning() {
        spawn_generate_next_led_frame();
    }

    let update_freq_hz = led_display.update(&led_matrix).unwrap();

    if let Some(update_freq_hz) = update_freq_hz {
        led_timer.start(update_freq_hz.hz());
    }

    if led_display.is_at_frame_end() {
        if let Some(matrix) = led_next_matrix.take() {
            *led_matrix = matrix;
        }
    }
}

fn compute_audio_spectrum_impl(
    mut audio_sampler: impl rtfm::Mutex<T = AudioSampler>,
    audio_fft: &mut AudioFft,
    audio_spectrum: &mut AudioSpectrum,
) {
    // Create a local copy of samples
    let samples = audio_sampler.lock(|s| s.samples);
    *audio_spectrum = audio_fft.compute_spectrum(&samples);
}

fn generate_next_led_frame_impl(
    mut led_next_matrix: impl rtfm::Mutex<T = LedNextMatrix>,
    audio_spectrum: &AudioSpectrum,
    audio_spectrum_cube: &mut AudioSpectrumCube,
) {
    use num_traits::float::Float;

    let spectrum = &audio_spectrum.data;
    let spectrum_len = spectrum.len() / 2;

    if spectrum_len == 0 {
        return;
    }

    audio_spectrum_cube.cycle += 1;
    if audio_spectrum_cube.cycle == config::AUDIO_SPECTRUM_CYCLES {
        audio_spectrum_cube.cycle = 0;
        for y in (1..led::matrix::CUBE_SIZE).rev() {
            audio_spectrum_cube.cells[y] = audio_spectrum_cube.cells[y - 1];
        }
    }

    let mut new_y_plane = [[0.0f32; led::matrix::CUBE_SIZE]; led::matrix::CUBE_SIZE];
    for x in 0..led::matrix::CUBE_SIZE {
        let idx_lo =
            (x as f32 * spectrum_len as f32 / led::matrix::CUBE_SIZE as f32).floor() as usize;
        let idx_hi =
            ((x + 1) as f32 * spectrum_len as f32 / led::matrix::CUBE_SIZE as f32).floor() as usize;

        for idx in idx_lo..idx_hi {
            const CLAMP_POINT: f32 = 3.0;
            let amp: f32 = spectrum[idx].norm_sqr().sqrt();

            let amp = amp.log10();
            let amp = if amp > CLAMP_POINT {
                amp - CLAMP_POINT
            } else {
                0.0
            };
            let amp = amp * 4.0;

            let amp_min = (amp.floor() as usize).clamp(0, led::matrix::CUBE_SIZE - 1);

            new_y_plane[x][amp_min] += 1.0 / (idx_hi - idx_lo) as f32;
        }
    }
    audio_spectrum_cube.cells[0] = new_y_plane;

    let mut new_matrix = led::matrix::LedMatrix::new();
    for z in 0..led::matrix::CUBE_SIZE {
        for y in 0..led::matrix::CUBE_SIZE {
            for x in 0..led::matrix::CUBE_SIZE {
                let intensity = audio_spectrum_cube.cells[y][x][z];
                *new_matrix.xyz_mut(x, y, z) = led::matrix::Color::rgb(
                    (intensity * x as f32 * 255.0 / led::matrix::CUBE_SIZE as f32) as u8,
                    (intensity * y as f32 * 255.0 / led::matrix::CUBE_SIZE as f32) as u8,
                    (intensity * z as f32 * 255.0 / led::matrix::CUBE_SIZE as f32) as u8,
                );
            }
        }
    }

    led_next_matrix.lock(|m| *m = Some(new_matrix));
}

#[alloc_error_handler]
fn handle_alloc_error(_: core::alloc::Layout) -> ! {
    loop {}
}
