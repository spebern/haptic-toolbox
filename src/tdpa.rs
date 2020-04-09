//! Time domain passivity control
//!
//! An energy-based method is presented for controlling a haptic interface
//! system to ensure stable contact under a wide variety of operating
//! conditions. [[1]](https://ieeexplore.ieee.org/document/932880)
use nalgebra::{
    allocator::Allocator,
    dimension::{Dim, DimName},
    DefaultAllocator, RealField, VectorN,
};
use num_traits::Zero;

pub struct TDPA<N, D>
where
    N: RealField,
    D: Dim,
    DefaultAllocator: Allocator<N, D>,
{
    alpha: N,
    energy: N,
    prev_vel: VectorN<N, D>,
}

impl<N, D> Default for TDPA<N, D>
where
    N: RealField,
    D: Dim + DimName,
    DefaultAllocator: Allocator<N, D>,
{
    fn default() -> Self {
        Self {
            alpha: N::zero(),
            energy: N::zero(),
            prev_vel: Zero::zero(),
        }
    }
}

impl<N, D> TDPA<N, D>
where
    N: RealField,
    D: Dim,
    DefaultAllocator: Allocator<N, D>,
{
    /// Calculate the TDPA force while ensuring passivity.
    pub fn calculate_force(&mut self, vel: &VectorN<N, D>, force: &VectorN<N, D>) -> VectorN<N, D> {
        let energy = force.dot(vel) + self.alpha * self.prev_vel.dot(&self.prev_vel);
        self.energy += energy;
        self.prev_vel = vel.clone();
        self.alpha = if self.energy < N::zero() {
            -self.energy / (vel.dot(vel))
        } else {
            N::zero()
        };
        if self.alpha == N::zero() {
            force.clone()
        } else {
            force + vel * self.alpha
        }
    }
}
