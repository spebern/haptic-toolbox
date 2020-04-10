//! PID Controller
//!
//!  A proportional–integral–derivative controller (PID controller or three-term
//! controller) is a control loop mechanism employing feedback that is widely
//! used in industrial control systems and a variety of other applications
//! requiring continuously modulated control. A PID controller continuously
//! calculates an error value e(t) as the difference between a desired setpoint
//! (SP) and a measured process variable (PV) and applies a correction based on
//! proportional, integral, and derivative terms (denoted P, I, and D
//! respectively), hence the name. [[1]](https://en.wikipedia.org/wiki/PID_controller)
use nalgebra::{
    allocator::Allocator,
    dimension::{Dim, DimName},
    DefaultAllocator, RealField, VectorN,
};
use num_traits::Zero;

pub struct PID<N, D>
where
    N: RealField,
    D: Dim,
    DefaultAllocator: Allocator<N, D>,
{
    k_p: N,
    k_i: N,
    k_d: N,

    integral_error: VectorN<N, D>,
}

impl<N, D> PID<N, D>
where
    N: RealField,
    D: Dim + DimName,
    DefaultAllocator: Allocator<N, D>,
{
    // Creates a new PID controller.
    pub fn new(k_p: N, k_i: N, k_d: N) -> Self {
        Self {
            k_p,
            k_i,
            k_d,
            integral_error: Zero::zero(),
        }
    }

    /// Calculates the force for tracking reference position and velocity.
    pub fn calculate_force(
        &mut self,
        pos_ref: &VectorN<N, D>,
        pos: &VectorN<N, D>,
        vel_ref: &VectorN<N, D>,
        vel: &VectorN<N, D>,
        dt: N,
    ) -> VectorN<N, D> {
        let error = pos_ref - pos;
        self.integral_error += &error * dt;
        let comp_p = error * self.k_p;
        let comp_i = &self.integral_error * self.k_i;
        let comp_d = (vel_ref - vel) * self.k_d;

        comp_p + comp_i + comp_d
    }

    /// Returns k p.
    pub fn k_p(&self) -> N {
        self.k_p
    }

    /// Returns k i.
    pub fn k_i(&self) -> N {
        self.k_i
    }

    /// Returns k d.
    pub fn k_d(&self) -> N {
        self.k_d
    }

    /// Sets k p.
    pub fn set_k_p(&mut self, k_p: N) {
        self.k_p = k_p;
    }

    /// Sets k i.
    pub fn set_k_i(&mut self, k_i: N) {
        self.k_i = k_i;
    }

    /// Sets k d.
    pub fn set_k_d(&mut self, k_d: N) {
        self.k_d = k_d;
    }
}
