use crate::flight_control::common::orbit::Orbit;
use crate::flight_control::common::vec2d::Vec2D;
use crate::http_handler::http_client::HTTPClient;
use crate::http_handler::http_handler_common::{Timed, ZonedObjective};
use crate::http_handler::http_request::objective_list_get::ObjectiveListRequest;
use crate::http_handler::http_request::request_common::NoBodyHTTPRequestType;

pub struct ObjectiveSchedule {
    scheduled_objectives: Vec<ZonedObjective>,
}

impl ObjectiveSchedule {
    pub fn new() -> Self {
        Self {
            scheduled_objectives: Vec::new(),
        }
    }

    pub async fn update(&mut self, httpclient: &HTTPClient) {
        loop {
            match (ObjectiveListRequest {}.send_request(httpclient).await) {
                Ok(response) => {
                    for zoned_obj in response.zoned_objectives() {
                        if !self.scheduled_objectives.iter().any(|obj| obj.id() == zoned_obj.id()) {
                            self.scheduled_objectives.push(zoned_obj.clone());
                        }
                    }
                    self.scheduled_objectives.retain(|objective| objective.is_in_time_window());
                }
                Err(_) => {
                    /* TODO: Error logging? */
                    println!("[ERROR] Error while fetching objectives in update_current_objective");
                }
            }
        }
    }

    fn fetch_next_deadline_objective(&self) -> &ZonedObjective {
        let mut next_deadline_objective = &self.scheduled_objectives[0];

        for objective in &self.scheduled_objectives {
            if objective.end() < next_deadline_objective.end() {
                next_deadline_objective = objective;
            }
        }

        next_deadline_objective
    }

    fn min_images(objective: &ZonedObjective) -> i32 {
        let zone = objective.zone();
        let coverage_needed = objective.coverage_required() as i32;
        let lens_square_side_length = objective.optic_required().get_square_side_length();

        let zone_width = zone[2] - zone[0];
        let zone_height = zone[3] - zone[1];

        let total_zone_area_size = zone_width * zone_height;
        let lens_area_size = lens_square_side_length.pow(2) as i32;

        let min_area_required = total_zone_area_size * coverage_needed;

        let min_number_of_images_required = (min_area_required / lens_area_size) + 1;

        min_number_of_images_required
    }

    fn is_objective_part_of_current_orbit(
        objective: &ZonedObjective,
        satellite_pos: &Orbit,
    ) -> Option<Vec<Vec2D<f32>>> {
        let x_min = objective.zone()[0];
        let y_min = objective.zone()[1];
        let x_max = objective.zone()[2];
        let y_max = objective.zone()[3];

        let mut visited_points = Vec::new();

        for x in x_min..=x_max {
            for y in y_min..=y_max {
                /* NOTE: How do I do this safely?*/
                let point = Vec2D::new(x as f32, y as f32);
                if let Some(time_to_point) = satellite_pos.will_visit(point) {
                    if objective.is_in_future_time_window(time_to_point as i64) {
                        visited_points.push(point);
                    }
                }
            }
        }

        if visited_points.is_empty() {
            None
        } else {
            Some(visited_points)
        }
    }
}
