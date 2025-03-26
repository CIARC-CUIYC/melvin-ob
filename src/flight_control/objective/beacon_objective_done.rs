use super::beacon_objective::BeaconObjective;
use crate::flight_control::{common::vec2d::Vec2D, flight_computer::FlightComputer};
use crate::http_handler::{
    http_client::HTTPClient,
    http_request::{
        beacon_position_put::BeaconPositionRequest, request_common::NoBodyHTTPRequestType,
    },
};
use crate::{error, obj};
use chrono::{DateTime, Utc};
use fixed::types::I32F32;
use rand::Rng;
use std::sync::Arc;

#[derive(Clone)]
pub struct BeaconObjectiveDone {
    id: usize,
    name: String,
    start: DateTime<Utc>,
    end: DateTime<Utc>,
    guesses: Vec<Vec2D<I32F32>>,
    submitted: bool,
}

impl BeaconObjectiveDone {
    const MAP_WIDTH_RANGE: std::ops::Range<u32> = 0..21600;
    const MAP_HEIGHT_RANGE: std::ops::Range<u32> = 0..10800;
    const MIN_DISTANCE_RAND_GUESSES: f32 = 75.0;
    pub fn id(&self) -> usize { self.id }
    pub fn name(&self) -> &str { &self.name }
    pub fn start(&self) -> DateTime<Utc> { self.start }
    pub fn end(&self) -> DateTime<Utc> { self.end }
    pub fn guesses(&self) -> &Vec<Vec2D<I32F32>> { &self.guesses }
    pub fn submitted(&self) -> bool { self.submitted }

    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    pub async fn guess_max(&mut self, client: Arc<HTTPClient>) {
        self.submitted = true;
        obj!(
            "Guessing max for {}: {} guesses...",
            self.id,
            self.guesses.len()
        );
        let id_u16 = self.id() as u16;
        let guess_cloned = self.guesses().clone();
        for (i, guess) in guess_cloned.iter().enumerate() {
            let width = guess.x().abs().to_num::<u32>();
            let height = guess.y().abs().to_num::<u32>();
            let req = BeaconPositionRequest { beacon_id: id_u16, width, height };
            obj!("Sending request for beacon {id_u16} with width {width} and height {height}...");
            if self.submit_guess(req, client.clone(), guess, i).await.is_err() {
                return
            };
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub async fn randomize_no_meas_guesses(&mut self, client: Arc<HTTPClient>) {
        if !self.guesses.is_empty() {
            obj!("Guesses are provided already, skipping randomization.");
            return self.guess_max(client).await;
        }
        obj!("No guesses for {}, randomizing 10 guesses.", self.id);

        let random_guesses = Self::generate_random_guesses();
        self.submitted = true;
        for (i, guess) in random_guesses.iter().enumerate() {
            let guess_req = BeaconPositionRequest {
                beacon_id: self.id as u16,
                width: guess.x().abs().to_num::<u32>(),
                height: guess.y().abs().to_num::<u32>(),
            };
            if self.submit_guess(guess_req, Arc::clone(&client), guess, i).await.is_err() {
                return;
            }
        }
    }

    async fn submit_guess(
        &mut self,
        req: BeaconPositionRequest,
        client: Arc<HTTPClient>,
        guess: &Vec2D<I32F32>,
        guess_num: usize,
    ) -> Result<(), std::io::Error>{
        loop {
            if let Ok(msg) = req.send_request(&client).await {
                if msg.is_success() {
                    obj!("And Rohan will answer! Mustered Rohirrim {} at {}!", req.beacon_id, guess);
                    return Ok(());
                } else if msg.is_last() {
                    obj!(
                        "Where was Gondor when the Westfold fell! Could not find beacon {} after {} tries!",
                        req.beacon_id,
                        guess_num
                    );
                    return Ok(());
                } else if msg.is_unknown() {
                    obj!("Beacon {} is unknown!", req.beacon_id);
                    return Err(std::io::Error::new(std::io::ErrorKind::Other, "Beacon unknown!"));
                } else if msg.is_fail() {
                    continue;
                }
                obj!("Unknown Message: {}! Returning!", msg.msg());
                return Err(std::io::Error::new(std::io::ErrorKind::Other, "Unknown Message!"));
            }
            error!("Unnoticed HTTP Error in updateObservation()");
            tokio::time::sleep(FlightComputer::STD_REQUEST_DELAY).await;
        }
    }

    fn generate_random_guesses() -> Vec<Vec2D<I32F32>> {
        let mut rng = rand::rng();
        let mut random_guesses = Vec::new();
        while random_guesses.len() <= 10 {
            let random_width = rng.random_range(Self::MAP_WIDTH_RANGE);
            let random_height = rng.random_range(Self::MAP_HEIGHT_RANGE);
            let rand_guess = Vec2D::new(
                I32F32::from_num(random_width),
                I32F32::from_num(random_height),
            );

            let too_close = random_guesses.iter().any(|prev_guesses: &Vec2D<I32F32>| {
                prev_guesses.euclid_distance(&rand_guess)
                    <= I32F32::from_num(Self::MIN_DISTANCE_RAND_GUESSES)
            });

            if too_close {
                continue;
            }

            random_guesses.push(rand_guess);
        }
        random_guesses
    }
}

impl From<BeaconObjective> for BeaconObjectiveDone {
    fn from(obj: BeaconObjective) -> Self {
        let guesses =
            if let Some(meas) = obj.measurements() { meas.pack_perfect_circles() } else { vec![] };
        Self {
            id: obj.id(),
            name: String::from(obj.name()),
            start: obj.start(),
            end: obj.end(),
            guesses,
            submitted: false,
        }
    }
}
