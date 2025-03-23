use crate::flight_control::beacon_controller::BeaconController;
use crate::flight_control::{
    objective::{
        beacon_objective::BeaconObjective, known_img_objective::KnownImgObjective,
        objective_base::ObjectiveBase,
    },
    orbit::OrbitCharacteristics,
    supervisor::Supervisor,
};
use crate::keychain::KeychainWithOrbit;
use std::{collections::BinaryHeap, sync::Arc};
use tokio::sync::{mpsc::Receiver, Mutex, RwLock};

pub struct ModeContext {
    k: Arc<KeychainWithOrbit>,
    o_ch: Arc<RwLock<OrbitCharacteristics>>,
    super_v: Arc<Supervisor>,
    obj_mon: RwLock<Receiver<ObjectiveBase>>,
    k_buffer: Mutex<BinaryHeap<KnownImgObjective>>,
    b_buffer: Mutex<BinaryHeap<BeaconObjective>>,
    beac_cont: Arc<BeaconController>,
}

impl ModeContext {
    pub fn new(
        key: KeychainWithOrbit,
        o_char: OrbitCharacteristics,
        objective_monitor: Receiver<ObjectiveBase>,
        super_v: Arc<Supervisor>,
        beac_cont: Arc<BeaconController>,
    ) -> Self {
        let k = Arc::new(key);
        let o_ch = Arc::new(RwLock::new(o_char));
        let obj_mon = RwLock::new(objective_monitor);
        Self {
            k,
            o_ch,
            super_v,
            obj_mon,
            k_buffer: Mutex::new(BinaryHeap::new()),
            b_buffer: Mutex::new(BinaryHeap::new()),
            beac_cont,
        }
    }

    pub fn k(&self) -> &Arc<KeychainWithOrbit> { &self.k }
    pub async fn o_ch_clone(&self) -> OrbitCharacteristics { *self.o_ch.read().await }
    pub fn o_ch_lock(&self) -> &RwLock<OrbitCharacteristics> { &self.o_ch }
    pub fn obj_mon(&self) -> &RwLock<Receiver<ObjectiveBase>> { &self.obj_mon }
    pub fn super_v(&self) -> &Arc<Supervisor> { &self.super_v }
    pub fn k_buffer(&self) -> &Mutex<BinaryHeap<KnownImgObjective>> { &self.k_buffer }
    pub fn b_buffer(&self) -> &Mutex<BinaryHeap<BeaconObjective>> { &self.b_buffer }
    pub fn beac_cont(&self) -> &Arc<BeaconController> { &self.beac_cont }
}
