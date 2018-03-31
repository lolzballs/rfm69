use core::time::Duration;

pub trait Timer {
    type Instant;

    fn now(&self) -> Self::Instant;
    fn since(&self, past: &Self::Instant) -> Duration;
}
