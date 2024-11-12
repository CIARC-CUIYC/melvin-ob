// TODO: 422 Response Code: Validation Error -> not implemented

#[derive(serde::Deserialize)]
pub struct BeaconPositionResponse{
    status: String,
    attempts_made: i8
}



