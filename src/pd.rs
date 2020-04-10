//! PD Controller
//!
//! A proportional-derivative (PD) controller can be used to make a simple
//! system track some reference point. [[1]](https://www.matthewpeterkelly.com/tutorials/pdControl/index.html)
use nalgebra::{allocator::Allocator, dimension::Dim, DefaultAllocator, RealField, VectorN};
use std::marker::PhantomData;

pub struct PD<N, D>
where
    N: RealField,
    D: Dim,
    DefaultAllocator: Allocator<N, D>,
{
    k_p: N,
    k_d: N,
    _phantom: PhantomData<D>,
}

impl<N, D> PD<N, D>
where
    N: RealField,
    D: Dim,
    DefaultAllocator: Allocator<N, D>,
{
    // Creates a new PD controller.
    pub fn new(k_p: N, k_d: N) -> Self {
        Self {
            k_p,
            k_d,
            _phantom: PhantomData,
        }
    }

    /// Calculates the force for tracking reference position and velocity.
    pub fn calculate_force(
        &self,
        pos_ref: &VectorN<N, D>,
        pos: &VectorN<N, D>,
        vel_ref: &VectorN<N, D>,
        vel: &VectorN<N, D>,
    ) -> VectorN<N, D> {
        (pos_ref - pos) * self.k_p + (vel_ref - vel) * self.k_d
    }

    /// Returns k p.
    pub fn k_p(&self) -> N {
        self.k_p
    }

    /// Returns k d.
    pub fn k_d(&self) -> N {
        self.k_d
    }

    /// Sets k p.
    pub fn set_k_p(&mut self, k_p: N) {
        self.k_p = k_p
    }

    /// Sets k d.
    pub fn set_k_d(&mut self, k_d: N) {
        self.k_d = k_d
    }
}
