use core::time::Duration;

/// A generic timer
pub trait Timer {
    /// Represents a specific time
    type Instant;

    /// Returns the current `Instant`
    fn now(&self) -> Self::Instant;
    /// Returns the `Duration` since `past`
    fn since(&self, past: &Self::Instant) -> Duration;
}
