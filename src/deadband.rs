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
use nalgebra::{
    allocator::Allocator,
    dimension::{Dim, DimName},
    DefaultAllocator, RealField, VectorN,
};

#[derive(Debug)]
pub struct DeadbandDetector<N, D>
where
    N: RealField,
    D: Dim,
    DefaultAllocator: Allocator<N, D>,
{
    prev_vals: VectorN<N, D>,
    threshold: N,
    deadband: N,
}

impl<N, D> DeadbandDetector<N, D>
where
    N: RealField,
    D: Dim + DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Creates a new `DeadbandDetector`.
    pub fn new(threshold: N, initial_vals: VectorN<N, D>) -> Self {
        let mut deadband_detector = Self {
            prev_vals: initial_vals,
            deadband: N::zero(),
            threshold,
        };
        deadband_detector.set_deadband();
        deadband_detector
    }

    /// Checks if `vals` are in the deadband of the previously saved vals.
    pub fn is_in_deadband(&mut self, vals: &VectorN<N, D>) -> bool {
        let diff = (&self.prev_vals - vals).norm();
        if diff > self.deadband {
            self.prev_vals = vals.clone();
            self.set_deadband();
            false
        } else {
            true
        }
    }

    /// Sets the new deadband threshold.
    pub fn set_threshold(&mut self, threshold: N) {
        assert!(threshold >= N::zero(), "cannot assign a negative threshold");
        self.threshold = threshold;
    }

    /// Returns the current deadband threshold.
    pub fn threshold(&self) -> N {
        self.threshold
    }

    /// Sets the values the following ones should be compared to.
    pub fn set_prev_vals(&mut self, vals: &VectorN<N, D>) {
        self.prev_vals = vals.clone();
    }

    fn set_deadband(&mut self) {
        self.deadband = self.threshold * self.prev_vals.norm();
    }
}
