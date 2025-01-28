use std::fmt::{Debug, Display};

#[derive(Debug, Clone)]
pub struct ScoreGrid {
    e_size: usize,
    s_size: usize,
    score: Box<[i32]>,
}

impl ScoreGrid {
    pub fn new(e_size: usize, s_size: usize) -> Self {
        Self {
            e_size,
            s_size,
            score: vec![0i32; e_size * s_size].into_boxed_slice(),
        }
    }

    pub fn new_from_condition(
        e_size: usize,
        s_size: usize,
        (end_s, end_min_e): (usize, usize),
    ) -> Self {
        let mut min_score = vec![i32::MIN; e_size * s_size].into_boxed_slice();
        if end_s < s_size && end_min_e < e_size {
            let start_ind = end_min_e * s_size + end_s;
            let end_ind = s_size * e_size;

            for i in (start_ind..end_ind).step_by(s_size) {
                min_score[i] = 0;
            }
        }

        Self {
            e_size,
            s_size,
            score: min_score,
        }
    }
    pub fn get(&self, e: usize, s: usize) -> i32 { self.score[e * self.s_size + s] }
    pub fn set(&mut self, e: usize, s: usize, score: i32) {
        self.score[e * self.s_size + s] = score;
    }
    pub fn e_size(&self) -> usize { self.e_size }
    pub fn s_size(&self) -> usize { self.s_size }
}
