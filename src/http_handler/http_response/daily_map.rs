// TODO: 422 Response Code: Validation Error -> not implemented

#[derive(serde::Deserialize, Debug)]
pub struct DailyMapResponse{
    return_message: String,
}

