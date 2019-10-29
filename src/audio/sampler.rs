use crate::config;

pub struct Sampler {
    pub samples: [u16; config::AUDIO_SAMPLE_BUFFER_SIZE],
    index: usize,
    count: usize,
}

pub enum Status {
    NeedsMore,
    Complete,
}

impl Sampler {
    pub fn new() -> Self {
        let samples = [0; config::AUDIO_SAMPLE_BUFFER_SIZE];
        let index = 0;
        let count = 0;

        Self {
            samples,
            index,
            count,
        }
    }

    pub fn add_sample(&mut self, sample: u16) -> Status {
        self.samples[self.index] = sample;
        self.index += 1;
        self.index %= config::AUDIO_SAMPLE_BUFFER_SIZE;
        self.count += 1;

        if self.count == config::AUDIO_SPECTRUM_INTERVAL_SAMPLES {
            self.count = 0;
            Status::Complete
        } else {
            Status::NeedsMore
        }
    }
}
