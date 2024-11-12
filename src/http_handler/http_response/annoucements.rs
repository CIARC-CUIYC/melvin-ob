// TODO: is deserialize possible here? Just a string gets returned

#[derive(serde::Deserialize, Debug)]
pub struct AnnouncementsResponse{
    return_message: String,
}