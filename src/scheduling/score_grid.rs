use std::fmt::Debug;

/// A 2D grid structure to store integer scores, implemented as a flat array.
#[derive(Debug, Clone)]
pub struct ScoreGrid {
    /// The length of the energy dimension (number of rows).
    e_len: usize,
    /// The length of the state dimension (number of columns).
    s_len: usize,
    /// A flattened array representing the grid's scores.
    score: Box<[i32]>,
}

impl ScoreGrid {
    /// The minimum score used to initialize unwanted final states
    pub const MIN_SCORE: i32 = i32::MIN + 2;
    /// Creates a new [`ScoreGrid`] with specified dimensions, initializing all values to `0`.
    ///
    /// # Arguments
    /// * `e_len` - The length of the energy dimension (number of rows).
    /// * `s_len` - The length of the state dimension (number of columns).
    ///
    /// # Returns
    /// A [`ScoreGrid`] instance with all scores initialized to `0`.
    pub fn new(e_len: usize, s_len: usize) -> Self {
        Self { e_len, s_len, score: vec![0i32; e_len * s_len].into_boxed_slice() }
    }

    /// Creates a [`ScoreGrid`] and initializes scores based on the specified condition.
    ///
    /// # Arguments
    /// * `e_len` - The length of the energy dimension (number of rows).
    /// * `s_len` - The length of the state dimension (number of columns).
    /// * `(end_s, end_min_e)` - A tuple indicating the start column (`end_s`)
    ///   and the minimum row (`end_min_e`) from which to initialize the scores.
    ///
    /// # Returns
    /// A [`ScoreGrid`] with scores set to `i32::MIN` by default, and rows
    /// starting at `(end_min_e, end_s)` initialized to `0`.
    pub fn new_from_condition(
        e_len: usize,
        s_len: usize,
        (end_s, end_min_e): (Option<usize>, usize),
    ) -> Self {
        let (end_st, step) = if let Some(s) = end_s { (s, s_len) } else { (0, 1) };
        let mut min_score = vec![Self::MIN_SCORE; e_len * s_len].into_boxed_slice();
        if end_st < s_len && end_min_e < e_len {
            let start_ind = end_min_e * s_len + end_st;
            let end_ind = s_len * e_len;
            for i in (start_ind..end_ind).step_by(step) {
                min_score[i] = 0;
            }
        }

        Self { e_len, s_len, score: min_score }
    }

    /// Retrieves the score at a specific position in the grid.
    ///
    /// # Arguments
    /// * `e` - The index along the energy dimension (row).
    /// * `s` - The index along the state dimension (column).
    ///
    /// # Returns
    /// The score at the specified position.
    pub fn get(&self, e: usize, s: usize) -> i32 { self.score[e * self.s_len + s] }

    /// Retrieves the state with the maximum score at a specific energy level.
    ///
    /// # Arguments
    /// * `e` - The index along the energy dimension (row).
    ///
    /// # Returns
    /// The score at the specified position.
    pub fn get_max_s(&self, e: usize) -> usize {
        (0..self.s_len).max_by_key(|&i| self.score[e * self.s_len + i]).unwrap()
    }

    /// Sets the score at a specific position in the grid.
    ///
    /// # Arguments
    /// * `e` - The index along the energy dimension (row).
    /// * `s` - The index along the state dimension (column).
    /// * `score` - The value to set at the specified position.
    pub fn set(&mut self, e: usize, s: usize, score: i32) {
        self.score[e * self.s_len + s] = score;
    }

    /// Returns the length of the energy dimension (number of rows).
    ///
    /// # Returns
    /// The length of the energy dimension (`e_len`).
    pub fn e_len(&self) -> usize { self.e_len }

    /// Returns the length of the state dimension (number of columns).
    ///
    /// # Returns
    /// The length of the state dimension (`s_len`).
    pub fn s_len(&self) -> usize { self.s_len }
}
