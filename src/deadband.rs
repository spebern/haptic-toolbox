/// Detector for values that are in the deadband of the previous value.
///
/// A value is in the deadband of another value if its magnitude is withing
/// a certain threshold.
///
/// Deadband compression is a technique that helps to filter out values
/// whose impact is significant. For example in haptic applications where
/// the feedback is some kind of force users are not able to differentiate
/// between forces that are very close to another.
/// Thresholds for just noticeable differences are available and often build
/// upon models like the [Weber Fechner Law](https://en.wikipedia.org/wiki/Weber%E2%80%93Fechner_law)
/// or (Steven's power law)[https://en.wikipedia.org/wiki/Stevens%27s_power_law].
/// Filtering out values that are not noticeable can help to significantly
/// reduce the network traffic in applications.
///
/// ```rust
/// use haptic_toolbox::DeadbandDetector;
///
/// // Create a new deadband detector with a threshold of 10%.
/// // All values that are within 10% of the previous value are considered to be
/// // in its deadband. The initial value for deadband detection is set to 0.0.
/// let mut deadband_detector = DeadbandDetector::new(0.1, 0.0);
///
/// // 0.1 is not within a 10% threshold of 0.0.
/// assert!(!deadband_detector.is_in_deadband(0.1));
///
/// // 0.11 is within a 10% threshold of 0.1.
/// assert!(deadband_detector.is_in_deadband(0.11));
///
/// // 0.12 is not within a 10% threshold of 0.1.
/// assert!(!deadband_detector.is_in_deadband(0.12));
///
/// ```
pub struct DeadbandDetector {
    prev_val: f64,
    threshold: f64,
}

impl DeadbandDetector {
    /// Creates a new `DeadbandDetector`.
    pub fn new(threshold: f64, initial_val: f64) -> Self {
        Self {
            prev_val: initial_val,
            threshold,
        }
    }

    /// Checks if `val` is in the deadband by comparing it to the previous
    /// value. If `val` is outside the threshold it saves it for future
    /// values to compare with.
    pub fn is_in_deadband(&mut self, val: f64) -> bool {
        let diff = val - self.prev_val;
        if diff / self.prev_val > self.threshold {
            self.prev_val = val;
            false
        } else {
            true
        }
    }
}
