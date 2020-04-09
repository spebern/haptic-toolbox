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
/// use nalgebra::Vector3;
/// use haptic_toolbox::DeadbandDetector;
///
/// // Create a new deadband detector with a threshold of 10%.
/// // All values that are within 10% of the previous value are considered to be
/// // in its deadband. The initial value for deadband detection is set to 0.0.
/// let mut deadband_detector = DeadbandDetector::new(0.1, Vector3::zeros());
///
/// assert!(!deadband_detector.is_in_deadband(&Vector3::new(0.1, 0.1, 0.1)));
/// assert!(deadband_detector.is_in_deadband(&Vector3::new(0.11, 0.11, 0.11)));
/// assert!(!deadband_detector.is_in_deadband(&Vector3::new(0.12, 0.12, 0.12)));
/// assert!(!deadband_detector.is_in_deadband(&Vector3::new(0.0, 0.0, 0.0)));
/// ```
use nalgebra::Vector3;

pub struct DeadbandDetector {
    prev_vals: Vector3<f64>,
    threshold: f64,
    deadband: f64,
}

impl Default for DeadbandDetector {
    fn default() -> Self {
        Self::new(0.1, Vector3::zeros())
    }
}

impl DeadbandDetector {
    /// Creates a new `DeadbandDetector`.
    pub fn new(threshold: f64, initial_vals: Vector3<f64>) -> Self {
        let mut deadband_detector = Self {
            prev_vals: initial_vals,
            deadband: 0.0,
            threshold,
        };
        deadband_detector.set_deadband();
        deadband_detector
    }

    /// Checks if `vals` are in the deadband of the previously saved vals.
    pub fn is_in_deadband(&mut self, vals: &Vector3<f64>) -> bool {
        let diff = (self.prev_vals - vals).norm();
        if diff > self.deadband {
            self.prev_vals = *vals;
            self.set_deadband();
            false
        } else {
            true
        }
    }

    /// Sets the new deadband threshold.
    pub fn set_threshold(&mut self, threshold: f64) {
        assert!(
            threshold >= 0.0 && threshold <= 1.0,
            "cannot assign threshold outside of range [0.0, 1.0]"
        );
        self.threshold = threshold;
    }

    /// Returns the current deadband threshold.
    pub fn threshold(&self) -> f64 {
        self.threshold
    }

    fn set_deadband(&mut self) {
        self.deadband = self.threshold * self.prev_vals.norm();
    }
}
