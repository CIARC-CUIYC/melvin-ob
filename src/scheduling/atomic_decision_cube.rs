use super::atomic_decision::AtomicDecision;

/// A flattened 3D data structure to manage atomic decisions for multiple dimensions with good cache performance.
pub struct AtomicDecisionCube {
    /// Length of the time dimension.
    dt_len: usize,
    /// Length of the energy dimension.
    e_len: usize,
    /// Length of the state dimension.
    s_len: usize,
    /// Array of atomic decisions.
    decisions: Box<[AtomicDecision]>,
}

impl AtomicDecisionCube {
    /// Creates a new [`AtomicDecisionCube`] with the specified dimensions and initializes all decisions to `StayInCharge`.
    ///
    /// # Arguments
    /// * `dt_len` - The length of the time dimension.
    /// * `e_len` - The length of the energy dimension.
    /// * `s_len` - The length of the state dimension.
    ///
    /// # Returns
    /// A new instance of [`AtomicDecisionCube`].
    pub fn new(dt_len: usize, e_len: usize, s_len: usize) -> Self {
        Self {
            dt_len,
            e_len,
            s_len,
            decisions: vec![AtomicDecision::StayInCharge; dt_len * e_len * s_len]
                .into_boxed_slice(),
        }
    }

    /// Retrieves the atomic decision at the specified indices.
    ///
    /// # Arguments
    /// * `dt` - The index along the time dimension.
    /// * `e` - The index along the energy dimension.
    /// * `s` - The index along the state dimension.
    ///
    /// # Returns
    /// The [`AtomicDecision`] at the specified indices.
    pub fn get(&self, dt: usize, e: usize, s: usize) -> AtomicDecision {
        self.decisions[dt * self.e_len * self.s_len + e * self.s_len + s]
    }

    /// Sets the atomic decision at the specified indices.
    ///
    /// # Arguments
    /// * `dt` - The index along the time dimension.
    /// * `e` - The index along the energy dimension.
    /// * `s` - The index along the state dimension.
    /// * `decision` - The [`AtomicDecision`] to set at the specified indices.
    pub fn set(&mut self, dt: usize, e: usize, s: usize, decision: AtomicDecision) {
        self.decisions[dt * self.e_len * self.s_len + e * self.s_len + s] = decision;
    }

    /// Returns the length of the time dimension.
    ///
    /// # Returns
    /// The length of the time dimension.
    pub fn dt_len(&self) -> usize { self.dt_len }

    /// Returns the length of the energy dimension.
    ///
    /// # Returns
    /// The length of the energy dimension.
    pub fn e_len(&self) -> usize { self.e_len }

    /// Returns the length of the state dimension.
    ///
    /// # Returns
    /// The length of the state dimension.
    pub fn s_len(&self) -> usize { self.s_len }
}
