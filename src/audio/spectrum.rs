use crate::config;
use crate::led::matrix;

#[derive(Debug)]
pub struct Spectrum {
    pub data: alloc::vec::Vec<num_complex::Complex32>,
}

#[derive(Debug, Default)]
pub struct SpectrumCube {
    // indexed as [y][x][z] for performance reasons
    pub cells: [[[f32; matrix::CUBE_SIZE]; matrix::CUBE_SIZE]; matrix::CUBE_SIZE],
    pub cycle: usize,
}

#[derive(Debug)]
pub struct Fft {
    fft: chfft::RFft1D<f32>,
}

impl Spectrum {
    pub fn new() -> Self {
        let data = alloc::vec::Vec::new();

        Self { data }
    }
}

impl SpectrumCube {
    pub fn new() -> Self {
        Default::default()
    }
}

impl Fft {
    pub fn new() -> Self {
        let fft = chfft::RFft1D::new(config::AUDIO_SAMPLE_BUFFER_SIZE);
        Self { fft }
    }

    pub fn compute_spectrum(
        &mut self,
        samples: &[u16; config::AUDIO_SAMPLE_BUFFER_SIZE],
    ) -> Spectrum {
        let mut input = [0.0; config::AUDIO_SAMPLE_BUFFER_SIZE];

        for index in 0..config::AUDIO_SAMPLE_BUFFER_SIZE {
            input[index] = samples[index] as f32;
        }

        let data = self.fft.forward(&input);

        Spectrum { data }
    }
}
