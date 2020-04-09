//! Input-to-State Stable (ISS)
//!
//! Similar to other passivity apporaches, but less conservative. While most
//! passivity approaches do not allow a system to generate energy the ISS
//! approach allows to generate energy that is bounded by a constant. [[1]](https://ieeexplore.ieee.org/abstract/document/7139013)
use nalgebra::{
    allocator::Allocator,
    dimension::{Dim, DimName},
    DefaultAllocator, RealField, VectorN,
};
use num_traits::Zero;

pub struct ISS<N, D>
where
    N: RealField,
    D: Dim,
    DefaultAllocator: Allocator<N, D>,
{
    tau: N,
    mu_max: N,
    prev_force: VectorN<N, D>,
}

impl<N, D> ISS<N, D>
where
    N: RealField,
    D: Dim + DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Creates a new ISS controller.
    ///
    /// `mu_max` should be set so thtat 0 <= f'(x) <= `mu_max` is fullfilled
    /// and `mu_max` != 0.
    pub fn new(tau: N, mu_max: N) -> Self {
        assert!(mu_max > N::zero());
        Self {
            tau,
            mu_max,
            prev_force: Zero::zero(),
        }
    }

    /// Calculate the ISS force.
    pub fn calculate_force(&mut self, force: &VectorN<N, D>, dt: N) -> VectorN<N, D> {
        let iss_force = force + (force - &self.prev_force) * self.tau / dt;
        self.prev_force = force.clone();
        iss_force
    }

    /// Calculates the ISS velocity.
    pub fn calculate_vel(
        &self,
        vel: &VectorN<N, D>,
        force: &VectorN<N, D>,
        dt: N,
    ) -> VectorN<N, D> {
        vel - (force - &self.prev_force) / dt / self.mu_max
    }

    /// Returns tau.
    pub fn tau(&self) -> N {
        self.tau
    }

    /// Returns mu max.
    pub fn mu_max(&self) -> N {
        self.mu_max
    }

    /// Sets tau.
    pub fn set_tau(&mut self, tau: N) {
        assert!(tau >= N::zero());
        self.tau = tau;
    }

    /// Sets mu max.
    pub fn set_mu_max(&mut self, mu_max: N) {
        assert!(mu_max > N::zero());
        self.mu_max = mu_max;
    }
}
