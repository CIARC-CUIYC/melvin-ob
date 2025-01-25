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
pub fn fmod_f32(a: f32, b: f32) -> f32 {
    ((a % b) + b) % b
}

/// Calculate the least common multiple (LCM) for floating-point numbers.
pub fn lcm_f32(a: f32, b: f32) -> f32 {
    (a * b / gcd_f32(a, b)).abs()
}
pub fn lcm_f64(a: f64, b: f64) -> f64 {
    (a * b / gcd_f64(a, b)).abs()
}

/// Calculate the least common multiple (LCM) for signed integers
pub fn lcm_i32(a: i32, b: i32) -> i32 {
    (a / gcd_i32(a, b)) * b
}
