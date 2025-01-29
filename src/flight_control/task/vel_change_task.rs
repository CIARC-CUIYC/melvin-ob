use crate::flight_control::orbit::burn_sequence::BurnSequence;

#[derive(Debug)]
pub struct VelocityChangeTask {
    burn: BurnSequence,
}

impl VelocityChangeTask {
    pub fn new(burn: BurnSequence) -> Self { Self { burn } }

    pub fn burn(&self) -> &BurnSequence { &self.burn }
}
