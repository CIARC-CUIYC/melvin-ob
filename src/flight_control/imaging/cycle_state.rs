use chrono::{DateTime, TimeDelta, Utc};
use fixed::types::I32F32;

pub struct CycleState {
    last_mark: (isize, DateTime<Utc>),
    last_pic: Option<DateTime<Utc>>,
    last_image_flag: bool,
    done_ranges: Vec<(isize, isize)>,
    overlap: TimeDelta,
}

impl CycleState {
    #[allow(clippy::cast_possible_truncation)]
    pub fn init_cycle(img_max_dt: I32F32, start_i: isize) -> Self {
        let overlap = {
            let overlap_dt = (img_max_dt.floor() / I32F32::lit("2.0")).to_num::<isize>();
            TimeDelta::seconds(overlap_dt as i64)
        };
        Self {
            last_mark: (start_i - overlap.num_seconds() as isize, Utc::now() - overlap),
            last_pic: None,
            last_image_flag: false,
            done_ranges: Vec::new(),
            overlap,
        }
    }

    fn get_p_secs(&self) -> i64 {
        if let Some(last_pic_val) = self.last_pic {
            (last_pic_val - self.last_mark.1 + self.overlap).num_seconds()
        } else {
            0
        }
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn update_failed(&mut self, img_t: DateTime<Utc>) {
        let p_secs = self.get_p_secs();
        self.done_ranges.push((self.last_mark.0, self.last_mark.0 + p_secs as isize));
        let tot_passed_secs = (img_t - self.last_mark.1 - self.overlap).num_seconds();
        self.last_mark = (tot_passed_secs as isize, Utc::now());
        self.last_pic = None;
    }

    pub fn update_success(&mut self, img_t: DateTime<Utc>) {
        self.last_pic = Some(img_t);
    }

    #[allow(clippy::cast_possible_truncation)]
    pub fn finish(mut self) -> Vec<(isize, isize)> {
        let p_secs = self.get_p_secs();
        self.done_ranges.push((self.last_mark.0, self.last_mark.0 + p_secs as isize));
        self.done_ranges
    }
}