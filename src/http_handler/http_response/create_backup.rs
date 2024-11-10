// TODO: is deserialize possible here? Just a string gets returned

#[derive(serde::Deserialize)]
pub struct CreateBackupResponse{
    return_message: String
}