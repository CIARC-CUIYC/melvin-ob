/// Helper function to calculate the greatest common divisor (GCD) for floating-point numbers.
pub fn gcd_f32(a: f32, b: f32) -> f32 {
    let mut x = a.abs();
    let mut y = b.abs();
    while y > f32::EPSILON {
        let temp = y;
        y = x % y;
        x = temp;
    }
    x
}

pub fn gcd_f64(a: f64, b: f64) -> f64 {
    let mut x = a.abs();
    let mut y = b.abs();
    while y > f64::EPSILON {
        let temp = y;
        y = x % y;
        x = temp;
    }
    x
}

/// Helper function to calculate the greatest common divisor (GCD) for signed integers.
pub fn gcd_i32(a: i32, b: i32) -> i32 {
    let mut x = a.abs();
    let mut y = b.abs();
    while y != 0 {
        let temp = y;
        y = x % y;
        x = temp;
    }
    x
}

/// Helper function to calculate the modulo for floating-point numbers
pub fn fmod_f32(a: f32, b: f32) -> f32 { ((a % b) + b) % b }

/// Calculate the least common multiple (LCM) for floating-point numbers.
pub fn lcm_f32(a: f32, b: f32) -> f32 { (a * b / gcd_f32(a, b)).abs() }
pub fn lcm_f64(a: f64, b: f64) -> f64 { (a * b / gcd_f64(a, b)).abs() }

/// Calculate the least common multiple (LCM) for signed integers
pub fn lcm_i32(a: i32, b: i32) -> i32 { (a / gcd_i32(a, b)) * b }

/// Generalized method to normalize a value within a given range.
///
/// # Arguments
/// - `value`: The value to normalize.
/// - `min`: The minimum value of the range.
/// - `max`: The maximum value of the range.
///
/// # Returns
/// - A `f32` representing the normalized value in the range `[0.0, 1.0]`.
/// - Returns `None` if `min` and `max` are the same (to prevent division by zero).
pub fn normalize_f32(value: f32, min: f32, max: f32) -> Option<f32> {
    if (max - min).abs() < f32::EPSILON {
        // Avoid division by zero when min and max are effectively the same
        None
    } else {
        Some((value - min) / (max - min))
    }
}


pub fn find_min_y_for_x_range(
    a_x: f32,
    a_y: (f32, f32),
    b_x: f32,
    b_y: (f32, f32),
) -> (f32, (f32, f32)) {
    // Determine the smaller and larger time bounds
    let (min_t, min_pos, max_t, max_pos) = if a_x < b_x {
        (a_x, a_y, b_x, b_y)
    } else {
        (b_x, b_y, a_x, a_y)
    };

    let (min_x, min_y) = min_pos;
    let (max_x, max_y) = max_pos;

    // Calculate deltas
    let dx = max_x - min_x;
    let dy = max_y - min_y;
    let dt = max_t - min_t;

    // Coefficients for the quadratic function f(t) = c2 * t^2 + c1 * t + c0
    let c2 = (dx / dt).powi(2) + (dy / dt).powi(2);
    let c1 = 2.0 * ((min_x * dx / dt) + (min_y * dy / dt));
    let c0 = min_x.powi(2) + min_y.powi(2);

    // Minimize f(t) -> t_min = -c1 / (2 * c2)
    let t_min = -c1 / (2.0 * c2);

    // Clamp t_min to the [min_t, max_t] range
    let t_min_clamped = t_min.clamp(min_t, max_t);

    // Compute the position at t_min
    let alpha = (t_min_clamped - min_t) / dt;
    let pos_min = (
        min_x + alpha * dx, // Interpolated x
        min_y + alpha * dy, // Interpolated y
    );

    // Return the clamped t_min and the corresponding position
    (t_min_clamped, pos_min)
}

