use crate::flight_control::{
    orbit::OrbitCharacteristics,
    Supervisor,
};
use crate::objective::{BeaconController, BeaconControllerState, KnownImgObjective};
use crate::util::KeychainWithOrbit;
use std::{collections::BinaryHeap, sync::Arc};
use tokio::sync::{Mutex, RwLock, mpsc::Receiver, watch};

/// [`ModeContext`] is a central context container used by [`GlobalMode`] in the onboard software.
/// It provides shared access to key mission-critical resources such as orbit state,
/// supervisory control, objective channels, and internal buffers.
///
/// This structure enables modular FSM behavior by providing thread-safe, concurrent access
/// to both live and buffered data, allowing [`GlobalMode`] to remain more or less stateless and focused
/// solely on decision logic.
/// 
/// # Fields
/// - `k`: A shared [`KeychainWithOrbit`] object providing access to the controllers and the orbit.
/// - `o_ch`: A shared and locked [`OrbitCharacteristics`] object providing orbit related dynamic parameters.
/// - `super_v`: Shared access to the [`Supervisor`] handling observation updates, ... 
/// - `zo_mon`: Receiver for events regarding Zoned Objectives.
/// - `bo_mon`: Watch Receiver broadcasting the current state of the [`BeaconController`].
/// - `k_buffer`: A locked buffer containing additional [`KnownImgObjective`].
/// - `beac_cont`: The beacon controller handling beacon related functionality.
pub(crate) struct ModeContext {
    /// Shared keychain containing the various controllers and the orbit configuration.
    k: Arc<KeychainWithOrbit>,
    /// Orbit characteristics, updated during operation (e.g., after burns).
    o_ch: Arc<RwLock<OrbitCharacteristics>>,
    /// Supervisor instance providing new observation data, objective data, etc.
    super_v: Arc<Supervisor>,
    /// Receiver for new Known Image Objectives (Zoned Objectives).
    zo_mon: RwLock<Receiver<KnownImgObjective>>,
    /// Watch receiver for the current state of the Beacon Controller.
    bo_mon: RwLock<watch::Receiver<BeaconControllerState>>,
    /// Priority buffer for scheduled image objectives, used by internal planners.
    k_buffer: Mutex<BinaryHeap<KnownImgObjective>>,
    /// Shared access to the Beacon Controller for retrieval logic and updates.
    beac_cont: Arc<BeaconController>,
}

impl ModeContext {

    /// Constructs a new [`ModeContext`], initializing all internal references.
    ///
    /// # Arguments
    /// - `key`: The mission keychain, including the basic controllers and the orbit.
    /// - `o_char`: Initial orbital characteristics.
    /// - `zo_mon_un`: Receiver for incoming Known Image Objectives.
    /// - `bo_mon_un`: Watch receiver for beacon controller state updates.
    /// - `super_v`: Shared [`Supervisor`] handle.
    /// - `beac_cont`: Shared [`BeaconController`] for beacon objective management.
    pub(crate) fn new(
        key: KeychainWithOrbit,
        o_char: OrbitCharacteristics,
        zo_mon_un: Receiver<KnownImgObjective>,
        bo_mon_un: watch::Receiver<BeaconControllerState>,
        super_v: Arc<Supervisor>,
        beac_cont: Arc<BeaconController>,
    ) -> Arc<Self> {
        let k = Arc::new(key);
        let o_ch = Arc::new(RwLock::new(o_char));
        let zo_mon = RwLock::new(zo_mon_un);
        let bo_mon = RwLock::new(bo_mon_un);
        Arc::new(Self {
            k,
            o_ch,
            super_v,
            zo_mon,
            bo_mon,
            k_buffer: Mutex::new(BinaryHeap::new()),
            beac_cont,
        })
    }

    /// Provides a reference to the [`KeychainWithOrbit`].
    pub(super) fn k(&self) -> &Arc<KeychainWithOrbit> { &self.k }
    /// Provides a copy of the current [`OrbitCharacteristics`]. 
    pub(crate) async fn o_ch_clone(&self) -> OrbitCharacteristics { *self.o_ch.read().await }
    /// Provides a reference to the shared and locked [`OrbitCharacteristics`].
    pub(super) fn o_ch_lock(&self) -> &RwLock<OrbitCharacteristics> { &self.o_ch }
    /// Provides a reference to the locked Zoned Objective Event Receiver.
    pub(super) fn zo_mon(&self) -> &RwLock<Receiver<KnownImgObjective>> { &self.zo_mon }
    /// Provides a reference to the watch resembling the current state of the Beacon controller.
    pub(super) fn bo_mon(&self) -> &RwLock<watch::Receiver<BeaconControllerState>> { &self.bo_mon }
    /// Provides a shared reference to the [`Supervisor`].
    pub(super) fn super_v(&self) -> &Arc<Supervisor> { &self.super_v }
    /// Provides a reference to the locked Zoned Objective Buffer implemented as a [`BinaryHeap`].
    pub(super) fn k_buffer(&self) -> &Mutex<BinaryHeap<KnownImgObjective>> { &self.k_buffer }
    /// Provides a shared reference to the [`BeaconController`].
    pub(super) fn beac_cont(&self) -> &Arc<BeaconController> { &self.beac_cont }
}
