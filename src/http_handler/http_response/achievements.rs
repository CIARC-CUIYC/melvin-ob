use crate::http_handler::http_handler_common::Achievement;

#[derive(serde::Deserialize)]
pub struct AchievementsResponse{
    achievements: Vec<Achievement>,
}