use crate::flight_control::common::vec2d::Vec2D;
use fixed::types::I32F32;
use std::collections::{HashMap, HashSet};
use num::traits::FloatConst;

pub struct SquareSlice {
    offset: Vec2D<I32F32>,
    side_length: Vec2D<I32F32>,
}

impl SquareSlice {

    pub fn new(pos: Vec2D<I32F32>, max_dist: Vec2D<I32F32>) -> Self {
        let offset = (pos - max_dist).wrap_around_map();
        Self {
            offset,
            side_length: max_dist * I32F32::from_num(2),
        }
    }

    pub fn intersect(&self, other: &SquareSlice) -> Self {
        let start_x = self.offset.x().max(other.offset.x());
        let start_y = self.offset.y().max(other.offset.y());

        let end_x =
            (self.offset.x() + self.side_length.x()).min(other.offset.x() + other.side_length.x());
        let end_y =
            (self.offset.y() + self.side_length.y()).min(other.offset.y() + other.side_length.y());

        println!("start_x: {start_x}, start_y: {start_y}, end_x: {end_x}, end_y: {end_y}");
        
        // If there's no overlap, return a zero-sized square
        if start_x >= end_x || start_y >= end_y {
            return Self {
                offset: Vec2D::new(start_x, start_y),
                side_length: Vec2D::new(I32F32::from_num(0), I32F32::from_num(0)),
            };
        }

        // Create a new square slice representing the intersection
        Self {
            offset: Vec2D::new(start_x, start_y),
            side_length: Vec2D::new(end_x - start_x, end_y - start_y),
        }
    }

    pub fn get_coord_set(
        &self,
        pos: Vec2D<I32F32>,
        min_dist: I32F32,
        max_dist: I32F32,
    ) -> HashSet<Vec2D<i32>> {
        let mut coord_set = HashSet::new();
        let x_start = self.offset.x().ceil().to_num::<i32>();
        let y_start = self.offset.y().ceil().to_num::<i32>();
        let x_end = x_start + self.side_length.x().ceil().to_num::<i32>();
        let y_end = y_start + self.side_length.y().ceil().to_num::<i32>();
        for x in x_start..x_end {
            for y in y_start..y_end {
                let coord = Vec2D::new(I32F32::from_num(x), I32F32::from_num(y)).wrap_around_map();
                let dist = coord.unwrapped_to(&pos).abs();
                if dist >= min_dist && dist <= max_dist {
                    coord_set.insert(Vec2D::new(x, y));
                }
            }
        }
        coord_set
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn generate_hex_grid(&self) -> Vec<Vec2D<i32>> {
        let r = BayesianSet::MAX_RES_UNCERTAINTY_RAD;
        let mut centers = Vec::new();
        let dx = (3.0 * r / 2.0) as i32;
        let dy = ((r * 3.0f32.sqrt()) / 2.0) as i32;

        for y in (0..self.side_length.y().to_num::<i32>()).step_by(dy as usize) {
            for x in (0..self.side_length.x().to_num::<i32>()).step_by(dx as usize) {
                // Offset every other row
                let x_offset = if (y / dy) % 2 == 1 { dx / 2 } else { 0 };
                centers.push(Vec2D::new(x + x_offset, y ));
            }
        }

        centers
    }
}

pub struct BayesianSet {
    set: HashSet<Vec2D<i32>>,
    curr_slice: SquareSlice,
}

impl BayesianSet {
    const K_FAC_MAX: I32F32 = I32F32::lit("0.9");
    const K_FAC_MIN: I32F32 = I32F32::lit("1.1");
    pub const MAX_DIST: I32F32 = I32F32::lit("2000");
    const MIN_DIST: I32F32 = I32F32::lit("0");
    pub const K_ADD: I32F32 = I32F32::lit("225.1");
    const MAX_RES_UNCERTAINTY_RAD: f32 = 75.0;
    const SIDE_LENGTH_SAFETY: I32F32 = I32F32::lit("10.0");

    fn get_dists(d_noisy: I32F32) -> (I32F32, I32F32) {
        let min_dist = ((d_noisy - Self::K_ADD) / Self::K_FAC_MIN).max(Self::MIN_DIST);
        let max_dist = ((d_noisy + Self::K_ADD) / Self::K_FAC_MAX).min(Self::MAX_DIST);
        println!("min_dist: {min_dist}, max_dist: {max_dist}");
        (min_dist, max_dist)
    }

    pub fn new(pos: Vec2D<I32F32>, d_noisy: I32F32) -> Self {
        let (min_dist, max_dist) = Self::get_dists(d_noisy);
        let side_len = I32F32::from_num(max_dist + Self::SIDE_LENGTH_SAFETY);
        let slice = SquareSlice::new(pos, Vec2D::new(side_len, side_len));
        let set = slice.get_coord_set(pos, min_dist, max_dist);
        println!("sq-slice-dims: {}", slice.side_length);
        Self {
            set,
            curr_slice: slice,
        }
    }

    pub fn update(&mut self, pos: Vec2D<I32F32>, d_noisy: I32F32) {
        let (min_dist, max_dist) = Self::get_dists(d_noisy);
        let slice = self.curr_slice.intersect(&SquareSlice::new(pos, Vec2D::new(max_dist, max_dist)));
        let new_set = slice.get_coord_set(pos, min_dist, max_dist);
        self.set.retain(|coord| new_set.contains(coord));
        self.curr_slice = slice;
        println!("sq-slice-dims: {}", self.curr_slice.side_length);
    }
    
    pub fn is_in_set(&self, pos: Vec2D<i32>) -> bool {
        self.set.contains(&pos)
    }

    #[allow(clippy::cast_sign_loss, clippy::cast_precision_loss, clippy::cast_possible_truncation)]
    pub fn guess_estimate(&self) -> usize {
        let len = self.set.len();
        let max_one_guess_area = Self::MAX_RES_UNCERTAINTY_RAD.powi(2) * f32::PI();
        (len as f32 / max_one_guess_area).ceil() as usize
    }

    pub fn pack_perfect_circles(&self) -> Vec<Vec2D<i32>> {
        let hex_centers = self.curr_slice.generate_hex_grid();
        let assignments = self.assign_points_to_hexes(&hex_centers);
        Self::select_minimal_circles(&assignments)
    }

    /// Assign points to the nearest hexagonal center
    fn assign_points_to_hexes(&self, hex_centers: &[Vec2D<i32>]) -> HashMap<Vec2D<i32>, Vec<Vec2D<i32>>> {
        let mut assignments = HashMap::new();

        for &p in &self.set {
            let nearest = hex_centers.iter()
                .min_by_key(|&&h| (h.x() - p.x()).pow(2) + (h.y() - p.y()).pow(2))
                .unwrap();

            assignments.entry(*nearest).or_insert_with(Vec::new).push(p);
        }

        assignments
    }

    /// Select only the hex centers needed to cover the points
    fn select_minimal_circles(assignments: &HashMap<Vec2D<i32>, Vec<Vec2D<i32>>>) -> Vec<Vec2D<i32>> {
        let mut selected_centers = Vec::new();
        let mut uncovered: HashSet<Vec2D<i32>> = assignments.values().flatten().copied().collect();

        while !uncovered.is_empty() {
            let best_center = assignments.iter()
                .max_by_key(|(_, covered)| covered.len())
                .map(|(center, _)| center)
                .unwrap();

            selected_centers.push(*best_center);
            uncovered.retain(|p| !assignments[best_center].contains(p));
        }

        selected_centers
    }
}
