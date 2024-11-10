// TODO: 422 Response Code: Validation Error -> not implemented

#[cfg(debug_assertions)]
#[derive(serde::Deserialize)]
pub struct ModifyObjectiveResponse{
    added: Vec<usize>,
    modified: Vec<usize>,
}

