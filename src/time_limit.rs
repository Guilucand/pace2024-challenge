use std::time::{Duration, Instant};

pub struct TimeLimit {
    is_active: bool,
    start: Instant,
    max_duration: Duration,

    phase_start: Instant,
    phase_duration: Duration,
}

impl TimeLimit {
    pub fn new(duration: Option<Duration>) -> Self {
        Self {
            is_active: duration.is_some(),
            start: Instant::now(),
            max_duration: duration.unwrap_or_default(),
            phase_start: Instant::now(),
            phase_duration: duration.unwrap_or_default(),
        }
    }

    pub fn can_progress_global(&self) -> bool {
        self.start.elapsed() < self.max_duration
    }

    pub fn can_progress(&self) -> bool {
        if !self.is_active {
            return true;
        }
        self.start.elapsed() < self.max_duration && self.phase_start.elapsed() < self.phase_duration
    }

    pub fn start_proportion(&mut self, allocated_ratio: f64) {
        self.phase_start = Instant::now();
        self.phase_duration = self
            .max_duration
            .saturating_sub(self.phase_start.elapsed())
            .mul_f64(allocated_ratio);
    }

    pub fn start_phase(&mut self, duration: Duration) {
        self.phase_start = Instant::now();
        self.phase_duration = duration;
    }

    pub fn remaining_time_minus(&self, from_secs: Duration) -> Option<Duration> {
        if !self.is_active {
            return None;
        }

        let phase_remaining = self
            .phase_duration
            .saturating_sub(self.phase_start.elapsed());
        let total_remaining = self.max_duration.saturating_sub(self.start.elapsed());
        Some(
            total_remaining
                .min(phase_remaining)
                .saturating_sub(from_secs),
        )
    }
}
