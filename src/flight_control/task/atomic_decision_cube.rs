use crate::flight_control::task::atomic_decision::AtomicDecision;

pub struct AtomicDecisionCube {
    dt_len: usize,
    e_len: usize,
    s_len: usize,
    decisions: Box<[AtomicDecision]>,
}

impl AtomicDecisionCube {
    pub fn new(dt_len: usize, e_len: usize, s_len: usize) -> Self {
        Self {
            dt_len,
            e_len,
            s_len,
            decisions: vec![AtomicDecision::StayInCharge; dt_len * e_len * s_len]
                .into_boxed_slice(),
        }
    }

    pub fn get(&self, dt: usize, e: usize, s: usize) -> AtomicDecision {
        self.decisions[dt * self.e_len * self.s_len + e * self.s_len + s]
    }

    pub fn set(&mut self, dt: usize, e: usize, s: usize, decision: AtomicDecision) {
        self.decisions[dt * self.e_len * self.s_len + e * self.s_len + s] = decision;
    }

    pub fn dt_len(&self) -> usize { self.dt_len }
    pub fn e_len(&self) -> usize { self.e_len }
    pub fn s_len(&self) -> usize { self.s_len }
}
