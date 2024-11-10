// TODO: 422 Response Code: Validation Error -> not implemented

#[cfg(debug_assertions)]
#[derive(serde::Deserialize)]
pub struct ConfigureSimulationResponse{
    return_message: String,
}

