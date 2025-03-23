use crate::flight_control::beacon_controller::{BeaconController, BeaconControllerState};
use crate::flight_control::{
    objective::known_img_objective::KnownImgObjective, orbit::OrbitCharacteristics,
    supervisor::Supervisor,
};
use crate::keychain::KeychainWithOrbit;
use std::{collections::BinaryHeap, sync::Arc};
use tokio::sync::{Mutex, RwLock, mpsc::Receiver, watch};

pub struct ModeContext {
    k: Arc<KeychainWithOrbit>,
    o_ch: Arc<RwLock<OrbitCharacteristics>>,
    super_v: Arc<Supervisor>,
    zo_mon: RwLock<Receiver<KnownImgObjective>>,
    bo_mon: RwLock<watch::Receiver<BeaconControllerState>>,
    k_buffer: Mutex<BinaryHeap<KnownImgObjective>>,
    beac_cont: Arc<BeaconController>,
}

impl ModeContext {
    pub fn new(
        key: KeychainWithOrbit,
        o_char: OrbitCharacteristics,
        zo_mon_un: Receiver<KnownImgObjective>,
        bo_mon_un: watch::Receiver<BeaconControllerState>,
        super_v: Arc<Supervisor>,
        beac_cont: Arc<BeaconController>,
    ) -> Self {
        let k = Arc::new(key);
        let o_ch = Arc::new(RwLock::new(o_char));
        let zo_mon = RwLock::new(zo_mon_un);
        let bo_mon = RwLock::new(bo_mon_un);
        Self {
            k,
            o_ch,
            super_v,
            zo_mon,
            bo_mon,
            k_buffer: Mutex::new(BinaryHeap::new()),
            beac_cont,
        }
    }

    pub fn k(&self) -> &Arc<KeychainWithOrbit> { &self.k }
    pub async fn o_ch_clone(&self) -> OrbitCharacteristics { *self.o_ch.read().await }
    pub fn o_ch_lock(&self) -> &RwLock<OrbitCharacteristics> { &self.o_ch }
    pub fn zo_mon(&self) -> &RwLock<Receiver<KnownImgObjective>> { &self.zo_mon }
    pub fn bo_mon(&self) -> &RwLock<watch::Receiver<BeaconControllerState>> { &self.bo_mon }
    pub fn super_v(&self) -> &Arc<Supervisor> { &self.super_v }
    pub fn k_buffer(&self) -> &Mutex<BinaryHeap<KnownImgObjective>> { &self.k_buffer }
    pub fn beac_cont(&self) -> &Arc<BeaconController> { &self.beac_cont }
}
