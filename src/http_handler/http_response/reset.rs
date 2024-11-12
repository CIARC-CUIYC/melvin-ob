// TODO: is deserialize possible here? Just a string gets returned

#[cfg(debug_assertions)]
#[derive(serde::Deserialize, Debug)]
pub struct ResetResponse{
    return_message: String,
}