use fixed::traits::{Fixed, FixedSigned, ToFixed};
use num::traits::{real::Real, Num, NumAssignOps, NumCast};
use std::{
    cmp::Ordering,
    fmt::Display,
    ops::{Add, Deref, Div, Mul, Sub},
};
use std::ops::AddAssign;
use fixed::types::I32F32;

/// A 2D vector generic over any numeric type.
///
/// This struct represents a 2D point or vector in space and provides common
/// mathematical operations such as addition, normalization, rotation, and distance calculations.
///
/// # Type Parameters
/// * `T` - The functionality for the vector depends on traits implemented by `T`.
#[derive(Debug, PartialEq, Eq, Clone, Copy, Hash)]
pub struct Vec2D<T> {
    /// The x-component of the vector.
    x: T,
    /// The y-component of the vector.
    y: T,
}

pub struct Wrapped2D<T, const X: u32, const Y: u32>(Vec2D<T>);

impl<T, const X: u32, const Y: u32> Wrapped2D<T, X, Y>
where T: Fixed
{
    pub fn wrap_around_map(&self) -> Self {
        Wrapped2D(Vec2D::new(
            Self::wrap_coordinate(self.0.x, T::from_num(X)),
            Self::wrap_coordinate(self.0.y, T::from_num(Y)),
        ))
    }

    pub fn wrap_coordinate(value: T, max_value: T) -> T { (value + max_value) % max_value }
}

impl<T, const X: u32, const Y: u32> Deref for Wrapped2D<T, X, Y> {
    type Target = Vec2D<T>;

    fn deref(&self) -> &Self::Target { &self.0 }
}

impl<T> Display for Vec2D<T>
where T: Display
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}, {}]", self.x, self.y)
    }
}

impl<T> Vec2D<T>
where T: FixedSigned + NumAssignOps
{
    /// Computes the magnitude (absolute value) of the vector.
    ///
    /// # Returns
    /// The magnitude of the vector as a scalar of type `T`.
    pub fn abs(&self) -> T { (self.x * self.x + self.y * self.y).sqrt() }

    pub fn round(&self) -> Vec2D<T> { Vec2D::new(self.x.round(), self.y.round()) }

    /// Creates a vector pointing from the current vector (`self`) to another vector (`other`).
    ///
    /// # Arguments
    /// * `other` - The target vector.
    ///
    /// # Returns
    /// A new vector representing the direction from `self` to `other`.
    pub fn to(&self, other: &Vec2D<T>) -> Vec2D<T> {
        Vec2D::new(self.x - other.x, self.y - other.y)
    }

    pub fn unwrapped_to(&self, other: &Vec2D<T>) -> Vec2D<T> {
        let mut options = Vec::new();
        for x_sign in [1, -1] {
            for y_sign in [1, -1] {
                let target: Vec2D<T> = Vec2D::new(
                    other.x + Self::map_size().x() * T::from_num(x_sign),
                    other.y + Self::map_size().y() * T::from_num(y_sign),
                );
                let to_target = self.to(&target);
                let to_target_abs = to_target.abs();
                options.push((to_target, to_target_abs));
            }
        }
        options.iter().min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Less)).unwrap().0
    }

    pub fn perp_unit_to(&self, other: &Vec2D<T>) -> Vec2D<T> {
        match self.is_clockwise_to(other) {
            Some(dir) => self.perp_unit(dir),
            None => Vec2D::zero(),
        }
    }

    pub fn perp_unit(&self, clockwise: bool) -> Vec2D<T> {
        let perp =
            if clockwise { Vec2D::new(self.y, -self.x) } else { Vec2D::new(-self.y, self.x) };
        perp.normalize()
    }

    pub fn flip_unit(&self) -> Vec2D<T> {
        let flip = Vec2D::new(-self.y, -self.x);
        flip.normalize()
    }

    pub fn is_clockwise_to(&self, other: &Vec2D<T>) -> Option<bool> {
        let cross = self.cross(other);
        if cross > 0.0 {
            // Counterclockwise
            Some(false)
        } else if cross < 0.0 {
            // Clockwise
            Some(true)
        } else {
            // Aligned or opposite
            None
        }
    }

    pub fn unit(&self) -> Vec2D<T> {
        let magnitude = self.abs();
        if magnitude.is_zero() {
            *self
        } else {
            *self / magnitude
        }
    }

    pub fn angle_to(&self, other: &Vec2D<T>) -> T {
        let dot = self.dot(other);

        let a_abs = self.abs();
        let b_abs = other.abs();

        if a_abs == 0.0 || b_abs == 0.0 {
            return T::zero();
        }
        let cos_theta = dot / (a_abs * b_abs);
        let clamped_cos_theta = cos_theta.clamp(T::from_num(-1.0), T::from_num(1.0));
        let angle_radians = T::from_num(clamped_cos_theta.to_num::<f64>().acos());
        angle_radians * T::from_num(180.0) / T::PI()
    }

    /// Normalizes the vector to have a magnitude of 1.
    /// If the magnitude is zero, the original vector is returned unmodified.
    ///
    /// # Returns
    /// A normalized vector.
    pub fn normalize(self) -> Self {
        let magnitude = self.abs();
        if magnitude.is_zero() {
            self
        } else {
            Self::new(self.x / magnitude, self.y / magnitude)
        }
    }

    /// Rotates the vector by a given angle in degrees.
    ///
    /// # Arguments
    /// * `angle_degrees` - The angle to rotate by, in degrees.
    pub fn rotate_by(&mut self, angle_degrees: T) {
        let angle_radians = angle_degrees.to_num::<f64>().to_radians();
        let sin = T::from_num(angle_radians.sin());
        let cos = T::from_num(angle_radians.cos());
        let new_x = self.x * cos - self.y * sin;
        self.y = self.x * sin + self.y * cos;
        self.x = new_x;
    }

    /// Computes the Euclidean distance between the current vector and another vector.
    ///
    /// # Arguments
    /// * `other` - The other vector to compute the distance to.
    ///
    /// # Returns
    /// The Euclidean distance as a scalar of type `T`.
    pub fn euclid_distance(&self, other: &Self) -> T {
        ((self.x - other.x) * (self.x - other.x) + (self.y - other.y) * (self.y - other.y)).sqrt()
    }
}

impl<T: Copy> Vec2D<T> {
    /// Creates a new vector with the given x and y components.
    ///
    /// # Arguments
    /// * `x` - The x-component of the vector.
    /// * `y` - The y-component of the vector.
    ///
    /// # Returns
    /// A new `Vec2D` object.
    pub const fn new(x: T, y: T) -> Self { Self { x, y } }

    /// Returns the x-component of the vector.
    ///
    /// # Returns
    /// The `x` value of type `T`.
    pub const fn x(&self) -> T { self.x }

    /// Returns the y-component of the vector.
    ///
    /// # Returns
    /// The `y` value of type `T`.
    pub const fn y(&self) -> T { self.y }
}

impl<T: Fixed + Copy> Vec2D<T> {
    /// Computes the dot product of the current vector with another vector.
    /// The dot product is defined as:
    ///
    /// ```text
    /// dot_product = (x1 * x2) + (y1 * y2)
    /// ```
    ///
    /// # Arguments
    /// * `other` - Another `Vec2D` vector to compute the dot product with.
    ///
    /// # Returns
    /// A scalar value of type `T` that represents the dot product of the two vectors.
    pub fn dot(self, other: &Vec2D<T>) -> T { self.x * other.x + self.y * other.y }

    pub fn cross(self, other: &Vec2D<T>) -> T { self.x * other.y - self.y * other.x }

    /// Computes the magnitude (absolute value) of the vector as an `f64`.
    /// This enables magnitude calculation for integer types `T`.
    ///
    /// # Returns
    /// The magnitude of the vector as an `f64`.
    pub fn abs_f64(self) -> f64 { (self.x * self.x + self.y * self.y).to_f64().unwrap().sqrt() }

    /// Creates a zero vector (x = 0, y = 0).
    ///
    /// # Returns
    /// A zero-initialized `Vec2D` with member type `T`.
    pub fn zero() -> Self { Self::new(T::zero(), T::zero()) }

    /// Returns the dimensions of a predefined map as a 2D vector.
    ///
    /// # Returns
    /// A `Vec2D` object representing the map dimensions. Values are hardcoded
    /// based on an assumed map size (21600.0 x 10800.0).
    pub fn map_size() -> Vec2D<T> {
        Vec2D {
            x: T::from_num(21600.0),
            y: T::from_num(10800.0),
        }
    }

    /// Wraps the vector around a predefined 2D map.
    ///
    /// This method ensures the vector’s coordinates do not exceed the boundaries
    /// of the map defined by `map_size()`. If coordinates go beyond these boundaries,
    /// they are wrapped to remain within valid values.
    pub fn wrap_around_map(&self) -> Self {
        let map_size_x = Self::map_size().x();
        let map_size_y = Self::map_size().y();

        Vec2D::new(
            Self::wrap_coordinate(self.x, map_size_x),
            Self::wrap_coordinate(self.y, map_size_y),
        )
    }

    /// Wraps a single coordinate around a specific maximum value.
    ///
    /// # Arguments
    /// * `value` - The value to wrap.
    /// * `max_value` - The maximum value to wrap around.
    ///
    /// # Returns
    /// The wrapped coordinate as type `T`.
    pub fn wrap_coordinate(value: T, max_value: T) -> T {
        ((value % max_value) + max_value) % max_value
    }

    pub fn cast<D: NumCast>(self) -> Vec2D<D> {
        Vec2D {
            x: D::from(self.x).unwrap(),
            y: D::from(self.y).unwrap(),
        }
    }
}

impl<T> Vec2D<T> 
where T: Num + NumCast + Copy
{
    pub fn map_size_num() -> Vec2D<T> {
        Vec2D {
            x: T::from(21600.0).unwrap(),
            y: T::from(10800.0).unwrap(),
        }
    }
}

impl<T, TAdd> Add<Vec2D<TAdd>> for Vec2D<T>
where
    T: Fixed,
    TAdd: Fixed,
{
    type Output = Vec2D<T>;

    /// Implements the `+` operator for two `Vec2D` objects.
    ///
    /// # Arguments
    /// * `rhs` - The vector to add.
    ///
    /// # Returns
    /// A new `Vec2D` representing the sum of the vectors.
    fn add(self, rhs: Vec2D<TAdd>) -> Self::Output {
        Self::Output {
            x: self.x + T::from_num(rhs.x),
            y: self.y + T::from_num(rhs.y),
        }
    }
}

impl<T, TAdd> AddAssign<Vec2D<TAdd>> for Vec2D<T>
where
    T: Fixed,
    TAdd: Fixed,
{
    /// Implements the `+` operator for two `Vec2D` objects.
    ///
    /// # Arguments
    /// * `rhs` - The vector to add.
    ///
    /// # Returns
    /// A new `Vec2D` representing the sum of the vectors.
    fn add_assign(&mut self, rhs: Vec2D<TAdd>) {
        self.x = self.x + T::from_num(rhs.x);
        self.y = self.y + T::from_num(rhs.y);
    }
}

impl<T, TMul> Mul<TMul> for Vec2D<T>
where
    T: Fixed,
    TMul: Fixed + Copy,
{
    type Output = Vec2D<T>;

    /// Implements the `*` operator for a `Vec2D` and a scalar.
    ///
    /// # Arguments
    /// * `rhs` - The scalar value to multiply by.
    ///
    /// # Returns
    /// A new scaled vector.
    fn mul(self, rhs: TMul) -> Self::Output {
        Self::Output {
            x: self.x * T::from_num(rhs),
            y: self.y * T::from_num(rhs),
        }
    }
}

impl<T, TMul> Div<TMul> for Vec2D<T>
where
    T: Fixed,
    TMul: Fixed + Copy,
{
    type Output = Vec2D<T>;

    /// Implements the `/` operator for a `Vec2D` and a scalar.
    ///
    /// # Arguments
    /// * `rhs` - The scalar value to divide by.
    ///
    /// # Returns
    /// A new scaled vector.
    fn div(self, rhs: TMul) -> Self::Output {
        Self::Output {
            x: self.x / T::from_num(rhs),
            y: self.y / T::from_num(rhs)
        }
    }
}

impl<T, TSub> Sub<Vec2D<TSub>> for Vec2D<T>
where
    T: FixedSigned,
    TSub: Fixed,
{
    type Output = Vec2D<T>;

    /// Implements the `-` operator for two `Vec2D`.
    ///
    /// # Arguments
    /// * `rhs` - The `Vec2D` to subtract.
    ///
    /// # Returns
    /// A new vector.
    fn sub(self, rhs: Vec2D<TSub>) -> Self::Output {
        Self::Output {
            x: self.x - T::from_num(rhs.x),
            y: self.y - T::from_num(rhs.y),
        }
    }
}

impl<T: Num> From<(T, T)> for Vec2D<T> {
    /// Creates a `Vec2D` from a tuple of (x, y) values.
    ///
    /// # Arguments
    /// * `tuple` - A tuple representing the x and y values.
    ///
    /// # Returns
    /// A new `Vec2D` created from the tuple.
    fn from(tuple: (T, T)) -> Self {
        Vec2D {
            x: tuple.0,
            y: tuple.1,
        }
    }
}

impl<T: Num> Into<(T, T)> for Vec2D<T> {
    /// Creates a tuple from a `Vec2D` of (x, y) values.
    ///
    /// # Arguments
    /// * `tuple` - A tuple representing the x and y values.
    ///
    /// # Returns
    /// A new `Vec2D` created from the tuple.
    fn into(self) -> (T, T) {
        (self.x, self.y)
    }
}
