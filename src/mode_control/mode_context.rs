use crate::flight_control::objective::beacon_objective::BeaconObjective;
use crate::flight_control::objective::known_img_objective::KnownImgObjective;
use crate::flight_control::objective::objective_base::ObjectiveBase;
use crate::flight_control::objective::secret_img_objective::SecretImgObjective;
use crate::flight_control::orbit::OrbitCharacteristics;
use crate::keychain::KeychainWithOrbit;
use std::collections::{BinaryHeap, VecDeque};
use std::sync::Arc;
use tokio::sync::mpsc::Receiver;
use tokio::sync::{Notify, RwLock};

pub struct StateContext {
    k: Arc<KeychainWithOrbit>,
    o_ch: Arc<RwLock<OrbitCharacteristics>>,
    obj_mon: Receiver<ObjectiveBase>,
    safe_mon: Arc<Notify>,
    k_buffer: BinaryHeap<KnownImgObjective>,
    b_buffer: BinaryHeap<BeaconObjective>,
    s_buffer: VecDeque<SecretImgObjective>,
}

impl StateContext {
    pub fn new(
        key: KeychainWithOrbit,
        o_char: OrbitCharacteristics,
        obj_mon: Receiver<ObjectiveBase>,
        safe_mon: Arc<Notify>,
    ) -> Self {
        let k = Arc::new(key);
        let o_ch = Arc::new(RwLock::new(o_char));
        Self {
            k,
            o_ch,
            obj_mon,
            safe_mon,
            k_buffer: BinaryHeap::new(),
            b_buffer: BinaryHeap::new(),
            s_buffer: VecDeque::new(),
        }
    }

    pub fn k(&self) -> &Arc<KeychainWithOrbit> { &self.k }
    pub async fn o_ch_clone(&self) -> OrbitCharacteristics { self.o_ch.read().await.clone() }
    pub fn o_ch_lock(&self) -> &RwLock<OrbitCharacteristics> { &self.o_ch }
    pub fn obj_mon(&self) -> &Receiver<ObjectiveBase> { &self.obj_mon }
    pub fn safe_mon(&self) -> &Arc<Notify> { &self.safe_mon }
    pub fn k_buffer(&self) -> &BinaryHeap<KnownImgObjective> { &self.k_buffer }
    pub fn b_buffer(&self) -> &BinaryHeap<BeaconObjective> { &self.b_buffer }
    pub fn s_buffer(&self) -> &VecDeque<SecretImgObjective> { &self.s_buffer }
    pub fn k_buffer_mut(&mut self) -> &mut BinaryHeap<KnownImgObjective> { &mut self.k_buffer }
    pub fn b_buffer_mut(&mut self) -> &mut BinaryHeap<BeaconObjective> { &mut self.b_buffer }
    pub fn s_buffer_mut(&mut self) -> &mut VecDeque<SecretImgObjective> { &mut self.s_buffer }
}
