#[derive(serde::Deserialize, serde::Serialize, Debug)]
pub struct ZonedObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    decrease_rate: i64,
    enabled: bool,
    zone: [i32; 4],
    // TODO: make an enum out of optic_required
    optic_required: String,
    coverage_required: usize,
    description: String,
    sprite: String,
    secret: bool,
}

impl Timed for ZonedObjective {
    fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
}

#[derive(serde::Deserialize, serde::Serialize, Debug)]
pub struct BeaconObjective {
    id: usize,
    name: String,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
}


impl Timed for BeaconObjective {
    fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
}

#[derive(serde::Deserialize, Debug)]
pub struct CommunicationSlot {
    id: usize,
    start: chrono::DateTime<chrono::Utc>,
    end: chrono::DateTime<chrono::Utc>,
    enabled: bool,
}

impl CommunicationSlot {
    fn is_enabled(&self) -> bool { self.enabled }
}

impl Timed for CommunicationSlot {
    fn start(&self) -> chrono::DateTime<chrono::Utc> { self.start }
    fn end(&self) -> chrono::DateTime<chrono::Utc> { self.end }
}

#[derive(serde::Deserialize, Debug)]
pub struct Achievement {
    name: String,
    done: bool,
    points: f32,
    description: String,
    goal_parameter_threshold: bool,
    goal_parameter: bool,
}

trait Timed {
    fn start(&self) -> chrono::DateTime<chrono::Utc>;
    fn end(&self) -> chrono::DateTime<chrono::Utc>;

    fn time_to_start(&self) -> Option<chrono::Duration> {
        let now = chrono::Utc::now();
        if now < self.start() { Some(self.start() - now) } else { None }
    }

    fn time_to_end(&self) -> Option<chrono::Duration> {
        let now = chrono::Utc::now();
        if now < self.end() { Some(self.end() - now) } else { None }
    }
}