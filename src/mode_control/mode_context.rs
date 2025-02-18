use crate::flight_control::objective::{
    beacon_objective::BeaconObjective, known_img_objective::KnownImgObjective,
    objective_base::ObjectiveBase,
};
use crate::flight_control::orbit::OrbitCharacteristics;
use crate::keychain::KeychainWithOrbit;
use std::{
    collections::BinaryHeap,
    sync::Arc,
};
use tokio::sync::{mpsc::Receiver, Mutex, Notify, RwLock};

pub struct StateContext {
    k: Arc<KeychainWithOrbit>,
    o_ch: Arc<RwLock<OrbitCharacteristics>>,
    obj_mon: RwLock<Receiver<ObjectiveBase>>,
    safe_mon: Arc<Notify>,
    k_buffer: Mutex<BinaryHeap<KnownImgObjective>>,
    b_buffer: Mutex<BinaryHeap<BeaconObjective>>,
}

impl StateContext {
    pub fn new(
        key: KeychainWithOrbit,
        o_char: OrbitCharacteristics,
        objective_monitor: Receiver<ObjectiveBase>,
        safe_mon: Arc<Notify>,
    ) -> Self {
        let k = Arc::new(key);
        let o_ch = Arc::new(RwLock::new(o_char));
        let obj_mon = RwLock::new(objective_monitor);
        Self {
            k,
            o_ch,
            obj_mon,
            safe_mon,
            k_buffer: Mutex::new(BinaryHeap::new()),
            b_buffer: Mutex::new(BinaryHeap::new()),
        }
    }

    pub fn k(&self) -> &Arc<KeychainWithOrbit> { &self.k }
    pub async fn o_ch_clone(&self) -> OrbitCharacteristics { *self.o_ch.read().await }
    pub fn o_ch_lock(&self) -> &RwLock<OrbitCharacteristics> { &self.o_ch }
    pub fn obj_mon(&self) -> &RwLock<Receiver<ObjectiveBase>> { &self.obj_mon }
    pub fn safe_mon(&self) -> &Arc<Notify> { &self.safe_mon }
    pub fn k_buffer(&self) -> &Mutex<BinaryHeap<KnownImgObjective>> { &self.k_buffer }
    pub fn b_buffer(&self) -> &Mutex<BinaryHeap<BeaconObjective>> { &self.b_buffer }
}
