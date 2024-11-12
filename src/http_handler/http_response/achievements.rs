use crate::http_handler::http_handler_common::Achievement;

#[derive(serde::Deserialize, Debug)]
pub struct AchievementsResponse{
    achievements: Vec<Achievement>,
}