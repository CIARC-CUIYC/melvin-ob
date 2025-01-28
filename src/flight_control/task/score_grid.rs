use std::fmt::{Debug, Display};

#[derive(Debug, Clone)]
pub struct ScoreGrid {
    e_len: usize,
    s_len: usize,
    score: Box<[i32]>,
}

impl ScoreGrid {
    pub fn new(e_len: usize, s_len: usize) -> Self {
        Self {
            e_len,
            s_len,
            score: vec![0i32; e_len * s_len].into_boxed_slice(),
        }
    }

    pub fn new_from_condition(
        e_len: usize,
        s_len: usize,
        (end_s, end_min_e): (usize, usize),
    ) -> Self {
        let mut min_score = vec![i32::MIN; e_len * s_len].into_boxed_slice();
        if end_s < s_len && end_min_e < e_len {
            let start_ind = end_min_e * s_len + end_s;
            let end_ind = s_len * e_len;

            for i in (start_ind..end_ind).step_by(s_len) {
                min_score[i] = 0;
            }
        }

        Self {
            e_len,
            s_len,
            score: min_score,
        }
    }
    pub fn get(&self, e: usize, s: usize) -> i32 { self.score[e * self.s_len + s] }
    pub fn set(&mut self, e: usize, s: usize, score: i32) {
        self.score[e * self.s_len + s] = score;
    }
    pub fn e_len(&self) -> usize { self.e_len }
    pub fn s_lenomm(&self) -> usize { self.s_len }
}
