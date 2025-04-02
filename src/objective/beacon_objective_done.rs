use super::BeaconObjective;
use crate::util::Vec2D;
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
use std::io::{Error, ErrorKind};
use std::sync::Arc;

/// Represents a completed beacon objective.
///
/// Stores relevant details such as the objective's ID, name, start and end time,
/// and additional metadata like guesses and submission status.
#[derive(Clone)]
pub struct BeaconObjectiveDone {
    /// The unique identifier of the objective.
    id: usize,
    /// The name of the objective.
    name: String,
    /// The start time of the objective.
    start: DateTime<Utc>,
    /// The end time of the objective.
    end: DateTime<Utc>,
    /// A collection of guesses made for the beacon position.
    guesses: Vec<Vec2D<I32F32>>,
    /// Status indicating whether the guesses have been submitted.
    submitted: bool,
}

impl BeaconObjectiveDone {
    /// The allowed range for map width values.
    const MAP_WIDTH_RANGE: std::ops::Range<u32> = 0..21600;
    /// The allowed range for map height values.
    const MAP_HEIGHT_RANGE: std::ops::Range<u32> = 0..10800;
    /// The minimum allowable distance between random guesses.
    const MIN_DISTANCE_RAND_GUESSES: f32 = 75.0;

    /// Returns the ID of the beacon objective.
    pub fn id(&self) -> usize { self.id }
    /// Returns the name of the beacon objective.
    pub fn name(&self) -> &str { &self.name }
    /// Returns the start time of the beacon objective.
    pub fn start(&self) -> DateTime<Utc> { self.start }
    /// Returns the end time of the beacon objective.
    pub fn end(&self) -> DateTime<Utc> { self.end }
    /// Returns a reference to the guesses for the beacon objective.
    pub fn guesses(&self) -> &Vec<Vec2D<I32F32>> { &self.guesses }
    /// Returns whether the guesses have been submitted.
    pub fn submitted(&self) -> bool { self.submitted }
    /// Sets the submission status of the guesses to true.
    pub fn set_submitted(&mut self) { self.submitted = true }

    /// Sends all guesses for the beacon to the DRS.
    ///
    /// # Arguments
    ///
    /// * `client` - HTTP client used to send requests.
    #[allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]
    pub async fn guess_max(&self, client: Arc<HTTPClient>) {
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
                return;
            };
        }
    }

    /// Randomizes guesses for the beacon if none are provided and submits them.
    ///
    /// # Arguments
    ///
    /// * `client` - HTTP client used to send requests.
    #[allow(clippy::cast_possible_truncation)]
    pub async fn randomize_no_meas_guesses(&self, client: Arc<HTTPClient>) {
        if !self.guesses.is_empty() {
            obj!("Guesses are provided already, skipping randomization.");
            return self.guess_max(client).await;
        }
        obj!("No guesses for {}, randomizing 10 guesses.", self.id);

        let random_guesses = Self::generate_random_guesses();
        for (i, guess) in random_guesses.iter().enumerate() {
            let guess_req = BeaconPositionRequest {
                beacon_id: self.id as u16,
                width: guess.x().abs().to_num::<u32>(),
                height: guess.y().abs().to_num::<u32>(),
            };
            let res = self.submit_guess(guess_req, Arc::clone(&client), guess, i).await;
            match res {
                Ok(done) => {
                    if done.is_some() { return; };
                }
                Err(_) => return,
            }
        }
    }

    /// Submits a single guess to the server.
    ///
    /// # Arguments
    ///
    /// * `req` - The request containing the guess information.
    /// * `client` - HTTP client used to send the request.
    /// * `guess` - The guessed position.
    /// * `guess_num` - The number of the guess to provide contextual information.
    ///
    /// # Returns
    ///
    /// * `Ok(Some(()))` if the guess is successful.
    /// * `Ok(None)` if the guess fails but the process is not over.
    /// * `Err` if an error occurs or all attempts are exhausted.
    async fn submit_guess(
        &self,
        req: BeaconPositionRequest,
        client: Arc<HTTPClient>,
        guess: &Vec2D<I32F32>,
        guess_num: usize,
    ) -> Result<Option<()>, Error> {
        if let Ok(msg) = req.send_request(&client).await {
            if msg.is_success() {
                obj!(
                    "And Rohan will answer! Mustered Rohirrim {} at {}!",
                    req.beacon_id,
                    guess
                );
                return Ok(Some(()));
            } else if msg.is_fail() {
                obj!(
                    "What can men do against such reckless hate? Still searching beacon {} after {} tries!",
                    req.beacon_id,
                    guess_num
                );
                return Ok(None);
            } else if msg.is_last() {
                obj!(
                    "Where was Gondor when the Westfold fell! Could not find beacon {} after {} tries!",
                    req.beacon_id,
                    guess_num
                );
                return Err(Error::new(ErrorKind::Other, "Beacon over!"));
            } else if msg.is_unknown() {
                obj!("Beacon {} is unknown!", req.beacon_id);
                return Err(Error::new(ErrorKind::Other, "Beacon unknown!"));
            }
            obj!("Unknown Message: {}! Returning!", msg.msg());
            return Err(Error::new(ErrorKind::Other, "Unknown Message!"));
        }
        error!("Unnoticed HTTP Error in submit_guess()");
        Err(Error::new(ErrorKind::Other, "HTTP Error!"))
    }

    /// Generates a vector of random guesses, ensuring each guess
    /// is sufficiently spaced apart from the others.
    ///
    /// # Returns
    ///
    /// A vector of random beacon position guesses.
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
    /// Converts a `BeaconObjective` into a `BeaconObjectiveDone`.
    ///
    /// # Arguments
    ///
    /// * `obj` - The original objective to be converted.
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
