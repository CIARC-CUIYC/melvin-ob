use crate::flight_control::common::vec2d::Vec2D;
use fixed::types::I32F32;
use kiddo::{ImmutableKdTree, SquaredEuclidean};
use num::traits::FloatConst;
use std::collections::{HashMap, HashSet};
use std::num::NonZero;
use crate::flight_control::objective::beacon_objective::BeaconMeas;

#[derive(Debug, Clone)]
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

    pub fn map_right_top(&self, p: Vec2D<I32F32>) -> Vec2D<I32F32> {
        self.offset + self.offset.unwrapped_to_top_right(&p)
    }
    
    fn map_right_bottom(&self, p: Vec2D<I32F32>) -> Vec2D<I32F32> {
        self.offset + self.offset.unwrapped_to_bottom_right(&p)
    }

    pub fn intersect(&self, other: &SquareSlice) -> Self {
        let corr_offs = self.map_right_top(other.offset);
        let start_x = self.offset.x().max(corr_offs.x());
        let start_y = self.offset.y().max(corr_offs.y());

        let end_x =
            (self.offset.x() + self.side_length.x()).min(corr_offs.x() + other.side_length.x());
        let end_y =
            (self.offset.y() + self.side_length.y()).min(corr_offs.y() + other.side_length.y());

        if end_x - start_x > self.side_length.x() || end_y - start_y > self.side_length.y() {
            println!("I think wrapping should have occured here");
        }

        // If there's no overlap, return a zero-sized square
        if start_x >= end_x || start_y >= end_y {
            println!("corr_offs: {corr_offs}, self_offs: {}, other_offs: {}", self.offset, other.offset);
            println!("self_side_length: {}, other_side_length: {}", self.side_length, other.side_length);
            println!("start_x: {start_x}, start_y: {start_y}, end_x: {end_x}, end_y: {end_y}");
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

        let u_pos = self.map_right_top(pos);
        let x_start = self.offset.x().to_num::<i32>();
        let y_start = self.offset.y().to_num::<i32>();
        let x_end = x_start + self.side_length.x().to_num::<i32>();
        let y_end = y_start + self.side_length.y().to_num::<i32>();

        for x in x_start..x_end {
            let x_coord = I32F32::from_num(x);
            let delt_x_sq = (x_coord - u_pos.x()) * (x_coord - u_pos.x());

            if delt_x_sq > max_dist_sq {
                //println!("This is weird!");
                continue;
            }

            let max_y_dist = (max_dist_sq - delt_x_sq).sqrt().ceil().to_num::<i32>();
            let side_len = self.side_length.y().to_num::<i32>() / 2 - max_y_dist;
            let y_min = y_start.max(y_start + side_len);
            let y_max = y_end.min(y_end - side_len);

            for y in y_min..=y_max {
                let y_coord = I32F32::from_num(y);
                let delt_y_sq = (y_coord - u_pos.y()) * (y_coord - u_pos.y());
                let dist_sq = delt_x_sq + delt_y_sq;
                if dist_sq >= min_dist_sq && dist_sq <= max_dist_sq {
                    coord_set.insert(Vec2D::new(x, y).wrap_around_map());
                }
            }
        }
        coord_set
    }

    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn generate_hex_grid(&self) -> (ImmutableKdTree<f64, 2>, Vec<Vec2D<f64>>) {
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

#[derive(Debug, Clone)]
pub struct BayesianSet {
    set: HashSet<Vec2D<i32>>,
    curr_slice: SquareSlice,
    measurements: Vec<BeaconMeas>
}

impl BayesianSet {
    const K_FAC_MAX: I32F32 = I32F32::lit("0.9");
    const K_FAC_MIN: I32F32 = I32F32::lit("1.1");
    pub const STD_DIST_SAFETY: I32F32 = I32F32::lit("5");
    pub const MAX_DIST: I32F32 = I32F32::lit("2000");
    const MIN_DIST: I32F32 = I32F32::lit("0");
    pub const K_ADD: I32F32 = I32F32::lit("225.1");
    pub const MAX_RES_UNCERTAINTY_RAD: f32 = 75.0;
    const MAX_ITEMS: NonZero<usize> = unsafe { NonZero::new_unchecked(6) };

    fn get_dists(d_noisy: I32F32) -> (I32F32, I32F32) {
        let min_dist = ((d_noisy - Self::K_ADD) / Self::K_FAC_MIN) - Self::STD_DIST_SAFETY;
        let max_dist = ((d_noisy + Self::K_ADD) / Self::K_FAC_MAX) + Self::STD_DIST_SAFETY;
        (min_dist.max(Self::MIN_DIST).floor(), max_dist.min(Self::MAX_DIST).ceil())
    }

    pub fn new(meas: BeaconMeas) -> Self {
        let (min_dist, max_dist) = Self::get_dists(I32F32::from_num(meas.rssi()));
        let side_len = I32F32::from_num(max_dist);
        let pos = meas.corr_pos();
        let slice = SquareSlice::new(pos, Vec2D::new(side_len, side_len));
        let set = slice.get_coord_set(pos, min_dist, max_dist);
        Self {
            set,
            curr_slice: slice,
            measurements: vec![meas]
        }
    }

    pub fn update(&mut self, meas: &BeaconMeas) {
        let (min_dist, max_dist) = Self::get_dists(I32F32::from_num(meas.rssi()));
        println!("min_dist: {min_dist}, max_dist: {max_dist}");
        let pos = meas.corr_pos();
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
        let (h_c_tree, h_c) = self.curr_slice.generate_hex_grid();
        let assignments = self.assign_points_to_hexes(&h_c_tree, &h_c);
        let circles = Self::select_minimal_circles(assignments);
        circles
            .iter()
            .map(|circ| (self.curr_slice.offset + Vec2D::from_real(circ)).wrap_around_map().round())
            .collect()
    }

    /// Assign points to the nearest hexagonal center
    #[allow(clippy::cast_possible_truncation)]
    fn assign_points_to_hexes(
        &self,
        h_c_tree: &ImmutableKdTree<f64, 2>,
        h_c: &[Vec2D<f64>],
    ) -> HashMap<Vec2D<i32>, HashSet<Vec2D<i32>>> {
        let mut assignments: HashMap<Vec2D<i32>, HashSet<Vec2D<i32>>> = HashMap::new();

        for &p in &self.set {
            let p_fix = Vec2D::from_real(&p);
            let p_scaled_fix = self.curr_slice.offset.unwrapped_to(&p_fix);
            let p_search = [
                p_scaled_fix.x().to_num::<f64>(),
                p_scaled_fix.y().to_num::<f64>(),
            ];
            let n_res = h_c_tree.nearest_n_within::<SquaredEuclidean>(
                &p_search,
                75.0,
                Self::MAX_ITEMS,
                true,
            );
            for n in &n_res {
                let center_p = h_c[usize::try_from(n.item).unwrap()];
                if n.distance > f64::from(BayesianSet::MAX_RES_UNCERTAINTY_RAD).powi(2) {
                    println!(
                        "WARNING: Point {} is {} away: TOO FAR, nearest_point: {}",
                        Vec2D::from(&p_search),
                        n.distance,
                        h_c[usize::try_from(n.item).unwrap()],
                    );
                }
                let nearest = Vec2D::new(center_p.x().round() as i32, center_p.y().round() as i32);
                assignments.entry(nearest).or_default().insert(p);
            }
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
