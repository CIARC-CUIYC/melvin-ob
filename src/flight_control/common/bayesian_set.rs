use crate::flight_control::common::vec2d::{MapSize, Vec2D};
use fixed::types::I32F32;
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use num::traits::FloatConst;
use std::collections::{HashMap, HashSet};
use std::time::Instant;

pub struct SquareSlice {
    offset: Vec2D<I32F32>,
    side_length: Vec2D<I32F32>,
}

impl SquareSlice {
    pub const STD_X_ADD_MAP_FACTOR: f64 = 5.0;
    pub const MIN_X_ADD_MAP_FACTOR: f64 = 3.0;

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
        let min_dist_sq = min_dist * min_dist;
        let max_dist_sq = max_dist * max_dist;
        
        let x_start = self.offset.x().ceil().to_num::<i32>();
        let y_start = self.offset.y().ceil().to_num::<i32>();
        let x_end = x_start + self.side_length.x().ceil().to_num::<i32>();
        let y_end = y_start + self.side_length.y().ceil().to_num::<i32>();
        for x in x_start..x_end {
            for y in y_start..y_end {
                let coord = Vec2D::new(I32F32::from_num(x), I32F32::from_num(y)).wrap_around_map();
                let dist_sq = coord.unwrapped_to(&pos).abs_sq();
                if dist_sq >= min_dist_sq && dist_sq <= max_dist_sq {
                    coord_set.insert(Vec2D::new(x, y).wrap_around_map());
                }
            }
        }
        coord_set
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn generate_hex_grid(&self) -> (ImmutableKdTree<f64, 2>, Vec<Vec2D<f64>>) {
        let x_add = Vec2D::<f64>::map_size().x() * Self::STD_X_ADD_MAP_FACTOR;

        let r = BayesianSet::MAX_RES_UNCERTAINTY_RAD;
        let mut centers = Vec::new();
        let dx = (3.0 * r / 2.0) as i32;
        let dy = ((r * 3.0f32.sqrt()) / 2.0) as i32;

        let side_x = self.side_length.x().to_num::<i32>();
        let side_y = self.side_length.y().to_num::<i32>();

        for y in (0..side_y).step_by(dy as usize) {
            for x in (0..side_x).step_by(dx as usize) {
                // Offset every other row
                let x_offset = if (y / dy) % 2 == 1 { dx / 2 } else { 0 };
                centers.push([f64::from(x + x_offset), f64::from(y)]);
            }
        }
        (
            ImmutableKdTree::<f64, 2>::new_from_slice(&centers),
            centers.iter().map(Vec2D::from).collect(),
        )
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
    pub const MAX_RES_UNCERTAINTY_RAD: f32 = 75.0;
    const SIDE_LENGTH_SAFETY: I32F32 = I32F32::lit("10.0");

    fn get_dists(d_noisy: I32F32) -> (I32F32, I32F32) {
        let min_dist = ((d_noisy - Self::K_ADD) / Self::K_FAC_MIN).max(Self::MIN_DIST);
        let max_dist = ((d_noisy + Self::K_ADD) / Self::K_FAC_MAX).min(Self::MAX_DIST);
        (min_dist, max_dist)
    }

    pub fn new(pos: Vec2D<I32F32>, d_noisy: I32F32) -> Self {
        let (min_dist, max_dist) = Self::get_dists(d_noisy);
        let side_len = I32F32::from_num(max_dist + Self::SIDE_LENGTH_SAFETY);
        let slice = SquareSlice::new(pos, Vec2D::new(side_len, side_len));
        let set = slice.get_coord_set(pos, min_dist, max_dist);
        Self {
            set,
            curr_slice: slice,
        }
    }

    pub fn update(&mut self, pos: Vec2D<I32F32>, d_noisy: I32F32) {
        let (min_dist, max_dist) = Self::get_dists(d_noisy);
        let slice =
            self.curr_slice.intersect(&SquareSlice::new(pos, Vec2D::new(max_dist, max_dist)));
        let new_set = slice.get_coord_set(pos, min_dist, max_dist);
        self.set = self.set.intersection(&new_set).copied().collect();
        self.curr_slice = slice;
    }

    pub fn is_in_set(&self, pos: Vec2D<i32>) -> bool { self.set.contains(&pos) }

    #[allow(clippy::cast_sign_loss, clippy::cast_precision_loss, clippy::cast_possible_truncation)]
    pub fn guess_estimate(&self) -> usize {
        let len = self.set.len();
        let max_one_guess_area = Self::MAX_RES_UNCERTAINTY_RAD.powi(2) * f32::PI();
        (len as f32 / max_one_guess_area).ceil() as usize
    }

    pub fn pack_perfect_circles(&self) -> Vec<Vec2D<I32F32>> {
        let start = Instant::now();
        let (h_c_tree, h_c) = self.curr_slice.generate_hex_grid();
        println!(
            "Finished generating Hex Grid after : {}",
            start.elapsed().as_secs()
        );
        let assignments = self.assign_points_to_hexes(&h_c_tree, &h_c);
        println!(
            "Finished assigning points after : {}",
            start.elapsed().as_secs()
        );
        let circles = Self::select_minimal_circles(assignments);
        println!("Finished ranking circles : {}", start.elapsed().as_secs());
        circles
            .iter()
            .map(|circ| (self.curr_slice.offset + Vec2D::from_real(circ)).wrap_around_map())
            .collect()
    }

    /// Assign points to the nearest hexagonal center
    #[allow(clippy::cast_possible_truncation)]
    fn assign_points_to_hexes(
        &self,
        h_c_tree: &ImmutableKdTree<f64, 2>,
        h_c: &[Vec2D<f64>],
    ) -> HashMap<Vec2D<i32>, HashSet<Vec2D<i32>>> {
        let mut assignments = HashMap::new();

        for &p in &self.set {
            let p_fix = Vec2D::from_real(&p);
            let p_scaled_fix = self.curr_slice.offset.unwrapped_to(&p_fix);
            let p_search = [
                p_scaled_fix.x().to_num::<f64>(),
                p_scaled_fix.y().to_num::<f64>(),
            ];
            let n_res = h_c_tree.nearest_one::<SquaredEuclidean>(&p_search);
            let center_p = h_c[usize::try_from(n_res.item).unwrap()];
            if n_res.distance > f64::from(BayesianSet::MAX_RES_UNCERTAINTY_RAD).powi(2) {
                println!(
                    "WARNING: Point {} is {} away: TOO FAR, nearest_point: {}",
                    Vec2D::from(&p_search),
                    n_res.distance,
                    h_c[usize::try_from(n_res.item).unwrap()],
                );
            }
            let nearest = Vec2D::new(center_p.x().round() as i32, center_p.y().round() as i32);

            assignments.entry(nearest).or_insert_with(HashSet::new).insert(p);
        }

        assignments
    }

    /// Select only the hex centers needed to cover the points
    fn select_minimal_circles(
        mut assignments: HashMap<Vec2D<i32>, HashSet<Vec2D<i32>>>,
    ) -> Vec<Vec2D<i32>> {
        let mut selected_centers = Vec::new();
        let mut uncovered: HashSet<Vec2D<i32>> = assignments.values().flatten().copied().collect();

        while !uncovered.is_empty() {
            let best_center = *assignments
                .iter()
                .max_by_key(|(_, covered)| covered.len())
                .map(|(center, _)| center)
                .unwrap();

            selected_centers.push(best_center);
            uncovered = uncovered.difference(&assignments[&best_center]).copied().collect();
            assignments.remove(&best_center);
        }

        selected_centers
    }
}
