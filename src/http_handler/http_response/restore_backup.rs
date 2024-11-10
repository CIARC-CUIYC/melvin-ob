// TODO: is deserialize possible here? Just a string gets returned

#[derive(serde::Deserialize)]
pub struct RestoreBackupResponse{
    return_message: String
}