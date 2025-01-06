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

pub fn fmod_f32(a: f32, b: f32) -> f32 {
    ((a % b) + b) % b
}

/// Calculate the least common multiple (LCM) for floating-point numbers.
pub fn lcm_f32(a: f32, b: f32) -> f32 {
    (a * b / gcd_f32(a, b)).abs()
}
