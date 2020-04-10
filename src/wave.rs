//! WAVE Variable Transformation
//!
//! Wave transformation is a control strategy for bilateral haptic data exchange
//! with time delay. [[1]](https://www.researchgate.net/publication/238498174_Some_Recent_Approaches_to_Teleoperation_in_the_Presence_of_Time_Delay)
use nalgebra::{
    allocator::Allocator,
    convert,
    dimension::{Dim, DimName},
    DefaultAllocator, RealField, VectorN,
};
use std::marker::PhantomData;

pub struct WAVE<N, D>
where
    N: RealField,
    D: Dim,
    DefaultAllocator: Allocator<N, D>,
{
    b: N,
    _phantom: PhantomData<D>,
}

impl<N, D> WAVE<N, D>
where
    N: RealField,
    D: Dim + DimName,
    DefaultAllocator: Allocator<N, D>,
{
    /// Creates a new WAVE controller with the wave impedance `b`.
    pub fn new(b: N) -> Self {
        Self {
            b,
            _phantom: PhantomData,
        }
    }

    /// Calculates the input wave by the master.
    pub fn calculate_u_m(&self, force_m: &VectorN<N, D>, vel_m: &VectorN<N, D>) -> VectorN<N, D> {
        (force_m + vel_m * self.b) / (self.b * convert(2.0)).sqrt()
    }

    /// Calculates the input wave by the slave.
    pub fn calculate_u_s(&self, force_s: &VectorN<N, D>, vel_s: &VectorN<N, D>) -> VectorN<N, D> {
        (force_s - vel_s * self.b) / (self.b * convert(2.0)).sqrt()
    }

    /// Calculates the output wave by the master.
    pub fn calculate_v_m(&self, force_m: &VectorN<N, D>, vel_m: &VectorN<N, D>) -> VectorN<N, D> {
        self.calculate_u_s(force_m, vel_m)
    }

    /// Calculates the output wave by the slave.
    pub fn calculate_v_s(&self, force_s: &VectorN<N, D>, vel_s: &VectorN<N, D>) -> VectorN<N, D> {
        self.calculate_u_m(force_s, vel_s)
    }

    /// Calculates the force for the master.
    pub fn calculate_force_m(&self, u_m: &VectorN<N, D>, v_m: &VectorN<N, D>) -> VectorN<N, D> {
        (u_m + v_m) * (self.b / convert(2.0)).sqrt()
    }

    /// Calculates the force for the slave.
    pub fn calculate_force_s(&self, u_s: &VectorN<N, D>, v_s: &VectorN<N, D>) -> VectorN<N, D> {
        (u_s + v_s) * (self.b / convert(2.0)).sqrt()
    }

    /// Calculates the velocity for the master.
    pub fn calculate_vel_m(&self, u_m: &VectorN<N, D>, vel_m: &VectorN<N, D>) -> VectorN<N, D> {
        (u_m - vel_m) / (self.b * convert(2.0))
    }

    /// Calculates the velocity for the slave.
    pub fn calculate_vel_s(&self, u_s: &VectorN<N, D>, vel_s: &VectorN<N, D>) -> VectorN<N, D> {
        (u_s + vel_s) / (self.b * convert(2.0))
    }
}
