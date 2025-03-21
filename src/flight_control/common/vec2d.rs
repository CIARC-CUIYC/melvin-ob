use fixed::prelude::{FromFixed, ToFixed};
use fixed::types::{I32F32, I64F64};
use fixed::{
    traits::{Fixed, FixedSigned},
    types::I32F0,
};
use num::traits::{Num, NumAssignOps};
use std::{
    cmp::Ordering,
    fmt::Display,
    ops::{Add, Deref, Div, Mul, Rem, Sub},
};

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

/// A 2D vector wrapper with fixed-size wrapping capabilities.
///
/// This struct is generic over a numeric type `T` and two constants `X` and `Y`,
/// which represent the fixed size of the 2D wrapping area.
///
/// # Type Parameters
/// * `T` - Any numeric type that implements the `Fixed` trait.
/// * `X` - The maximum bound for the x-axis.
/// * `Y` - The maximum bound for the y-axis.
pub struct Wrapped2D<T, const X: u32, const Y: u32>(Vec2D<T>);

impl<T, const X: u32, const Y: u32> Wrapped2D<T, X, Y>
where T: Fixed
{
    /// Wraps the coordinates of the vector around the bounds defined by `X` and `Y`.
    ///
    /// # Returns
    /// A new `Wrapped2D` instance with its coordinates wrapped within the bounds.
    pub fn wrap_around_map(&self) -> Self {
        Wrapped2D(Vec2D::new(
            Self::wrap_coordinate(self.0.x, T::from_num(X)),
            Self::wrap_coordinate(self.0.y, T::from_num(Y)),
        ))
    }

    /// Wraps a single coordinate value around a maximum value.
    ///
    /// # Arguments
    /// * `value` - The coordinate value to be wrapped.
    /// * `max_value` - The maximum bound for the coordinate.
    ///
    /// # Returns
    /// The wrapped coordinate value.
    pub fn wrap_coordinate(value: T, max_value: T) -> T {
        ((value % max_value) + max_value) % max_value
    }
}

impl<T, const X: u32, const Y: u32> Deref for Wrapped2D<T, X, Y> {
    type Target = Vec2D<T>;

    /// Dereferences the `Wrapped2D` wrapper to access its inner `Vec2D` value.
    ///
    /// # Returns
    /// A reference to the inner `Vec2D`.
    fn deref(&self) -> &Self::Target { &self.0 }
}

impl<T> Display for Vec2D<T>
where T: Display
{
    /// Formats the `Vec2D` as a string in the format `[x, y]`.
    ///
    /// # Arguments
    /// * `f` - A mutable reference to the formatter.
    ///
    /// # Returns
    /// A `Result` indicating the success of the formatting operation.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}, {}]", self.x, self.y)
    }
}

/// A trait providing a method to define the size of a 2D map.
///
/// This is used to determine the dimensions of the map for wrapping operations.
pub trait MapSize {
    /// The output type of the map dimensions.
    type Output;

    /// Returns the size of the map as a `Vec2D` object.
    ///
    /// # Returns
    /// A `Vec2D` representing the width (`x`) and height (`y`) of the map.
    fn map_size() -> Vec2D<Self::Output>;
}

/// Implementation of the `MapSize` trait for the `I32F32` fixed-point number type.
impl MapSize for I32F32 {
    type Output = I32F32;

    /// Defines the size of the map as a `Vec2D` with dimensions 21600.0 x 10800.0.
    ///
    /// # Returns
    /// A `Vec2D` with fixed-point components representing the map dimensions.
    fn map_size() -> Vec2D<I32F32> {
        Vec2D {
            x: I32F32::from_num(21600.0),
            y: I32F32::from_num(10800.0),
        }
    }
}

impl MapSize for f64 {
    type Output = f64;
    /// Defines the size of the map as a `Vec2D` with dimensions 21600.0 x 10800.0.
    ///
    /// # Returns
    /// A `Vec2D` with floating-point components representing the map dimensions.
    fn map_size() -> Vec2D<f64> {
        Vec2D {
            x: 21600.0,
            y: 10800.0,
        }
    }
}

/// Implementation of the `MapSize` trait for the `I32F0` fixed-point number type.
impl MapSize for I32F0 {
    type Output = I32F0;

    /// Defines the size of the map as a `Vec2D` with dimensions 21600 x 10800.
    ///
    /// # Returns
    /// A `Vec2D` with fixed-point integer components representing the map dimensions.
    fn map_size() -> Vec2D<I32F0> {
        Vec2D {
            x: I32F0::from_num(21600),
            y: I32F0::from_num(10800),
        }
    }
}

/// Implementation of the `MapSize` trait for the `u32` type.
impl MapSize for u32 {
    type Output = u32;

    /// Defines the size of the map as a `Vec2D` with dimensions 21600 x 10800.
    ///
    /// # Returns
    /// A `Vec2D` with unsigned 32-bit integer components representing the map dimensions.
    fn map_size() -> Vec2D<u32> { Vec2D { x: 21600, y: 10800 } }
}

/// Implementation of the `MapSize` trait for the `i32` type.
impl MapSize for i32 {
    type Output = i32;

    /// Defines the size of the map as a `Vec2D` with dimensions 21600 x 10800.
    ///
    /// # Returns
    /// A `Vec2D` with signed 32-bit integer components representing the map dimensions.
    fn map_size() -> Vec2D<i32> { Vec2D { x: 21600, y: 10800 } }
}

/// Implementation of the `MapSize` trait for a `Vec2D` type with components
/// that also implement the `MapSize` trait.
impl<T> MapSize for Vec2D<T>
where T: MapSize<Output = T>
{
    type Output = T;

    /// Defines the size of the map by delegating to the `map_size` implementation of `T`.
    ///
    /// # Returns
    /// A `Vec2D` with components of type `T` representing the map dimensions.
    fn map_size() -> Vec2D<Self::Output> { T::map_size() }
}

impl<T> Vec2D<T>
where T: FixedSigned + NumAssignOps
{
    /// Computes the magnitude (absolute value) of the vector.
    ///
    /// # Returns
    /// The magnitude of the vector as a scalar of type `T`.
    pub fn abs(&self) -> T { (self.x * self.x + self.y * self.y).sqrt() }

    pub fn abs_sq(&self) -> T { self.x * self.x + self.y * self.y }

    pub fn round(&self) -> Self {
        Self {
            x: self.x.round(),
            y: self.y.round(),
        }
    }

    pub fn round_to_2(&self) -> Self {
        let factor = T::from_num(100);
        let new_x = (self.x * factor).round() / factor;
        let new_y = (self.y * factor).round() / factor;
        Self { x: new_x, y: new_y }
    }

    pub fn floor(&self) -> Self { Vec2D::new(self.x.floor(), self.y.floor()) }

    pub fn from_real<R>(&other: &Vec2D<R>) -> Self
    where R: Copy + ToFixed {
        Self {
            x: T::from_num(other.x()),
            y: T::from_num(other.y()),
        }
    }

    /// Creates a vector pointing from the current vector (`self`) to another vector (`other`).
    ///
    /// # Arguments
    /// * `other` - The target vector.
    ///
    /// # Returns
    /// A new vector representing the direction from `self` to `other`.
    pub fn to(&self, other: &Self) -> Self { Vec2D::new(other.x - self.x, other.y - self.y) }
    
    pub fn to_num<R: FromFixed + Copy>(&self) -> Vec2D<R> {
        Vec2D::new(self.x.to_num::<R>(), self.y.to_num::<R>())
    }

    /// Computes an "unwrapped" vector pointing from the current vector (`self`) to another vector (`other`).
    ///
    /// This method considers potential wrapping around a 2D map (based on the map size) and calculates
    /// the smallest vector that connects `self` to `other`. The wrapping allows for efficient navigation
    /// across boundaries.
    ///
    /// # Arguments
    /// * `other` - The target vector to which the direction is computed.
    ///
    /// # Returns
    /// A `Vec2D` representing the shortest unwrapped direction from `self` to `other`.
    pub fn unwrapped_to(&self, other: &Self) -> Self {
        let options = self.get_projected_in_range(other, (&[1, 0, -1], &[1, 0, -1]));
        options.iter().min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Less)).unwrap().0
    }

    pub fn unwrapped_to_top_right(&self, other: &Self) -> Self {
        let options = self.get_projected_in_range(other, (&[1, 0], &[1, 0]));
        options.iter().min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Less)).unwrap().0
    }

    pub fn unwrapped_to_bottom_right(&self, other: &Self) -> Self {
        let options = self.get_projected_in_range(other, (&[1, 0], &[-1, 0]));
        options.iter().min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Less)).unwrap().0
    }

    fn get_projected_in_range(&self, to: &Self, range: (&[i8], &[i8])) -> Vec<(Self, I64F64)> {
        let mut options = Vec::new();
        for x_sign in range.0 {
            for y_sign in range.1 {
                let target: Vec2D<T> = Vec2D::new(
                    to.x + T::from_num(u32::map_size().x()) * T::from_num(*x_sign),
                    to.y + T::from_num(u32::map_size().y()) * T::from_num(*y_sign),
                );
                let to_target = self.to(&target);
                let tt_scale =
                    Vec2D::new(I64F64::from_num(to_target.x), I64F64::from_num(to_target.y));
                let to_target_abs_sq = tt_scale.abs_sq();
                options.push((to_target, to_target_abs_sq));
            }
        }
        options
    }

    /// Computes a perpendicular unit vector pointing to another vector (`other`).
    ///
    /// The direction of the perpendicular vector depends on whether `self` is clockwise
    /// or counterclockwise to `other`.
    ///
    /// # Arguments
    /// * `other` - The target vector to compare directions with.
    ///
    /// # Returns
    /// A unit `Vec2D` perpendicular to `self` pointing towards `other`.
    /// Returns a zero vector if `self` and `other` are collinear.
    pub fn perp_unit_to(&self, other: &Self) -> Self {
        match self.is_clockwise_to(other) {
            Some(dir) => self.perp_unit(dir),
            None => Self::zero(),
        }
    }

    /// Computes a perpendicular unit vector to the current vector.
    ///
    /// The direction of the perpendicular vector depends on the `clockwise` parameter.
    ///
    /// # Arguments
    /// * `clockwise` - A boolean indicating the direction. `true` for clockwise, `false` for counterclockwise.
    ///
    /// # Returns
    /// A normalized perpendicular `Vec2D`.
    pub fn perp_unit(&self, clockwise: bool) -> Self {
        let perp = if clockwise { Self::new(self.y, -self.x) } else { Self::new(-self.y, self.x) };
        perp.normalize()
    }

    /// Computes a flipped collinear unit vector to the current vector.
    ///
    /// This is equivalent to rotating the vector 180 degrees.
    ///
    /// # Returns
    /// A normalized flipped collinear `Vec2D`.
    pub fn flip_unit(&self) -> Self {
        let flip = Self::new(-self.y, -self.x);
        flip.normalize()
    }

    /// Determines whether the current vector is clockwise relative to another vector (`other`).
    ///
    /// This is determined using the cross product:
    /// * `Some(true)` if `self` is clockwise to `other`.
    /// * `Some(false)` if `self` is counterclockwise to `other`.
    /// * `None` if `self` and `other` are collinear.
    ///
    /// # Arguments
    /// * `other` - The vector to compare relative direction with.
    ///
    /// # Returns
    /// An `Option<bool>` indicating the relative direction.
    pub fn is_clockwise_to(&self, other: &Self) -> Option<bool> {
        let cross = self.cross(other);
        match cross.partial_cmp(&T::zero()) {
            Some(Ordering::Less) => Some(true),
            Some(Ordering::Greater) => Some(false),
            _ => None,
        }
    }

    /// Computes the angle (in degrees) between the current vector and another vector (`other`).
    ///
    /// The method calculates the cosine of the angle using the dot product, clamps it to
    /// the valid range of `[-1, 1]`, and then computes the angle in degrees.
    ///
    /// # Arguments
    /// * `other` - The target vector to compute the angle to.
    ///
    /// # Returns
    /// The angle in degrees as a scalar of type `T`.
    pub fn angle_to(&self, other: &Self) -> T {
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
        if magnitude.is_zero() { self } else { Self::new(self.x / magnitude, self.y / magnitude) }
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
    pub fn dot(self, other: &Self) -> T { self.x * other.x + self.y * other.y }

    /// Computes the cross product of the current vector with another vector.
    ///
    /// The cross product is defined as:
    ///
    /// ```text
    /// cross_product = (x1 * y2) - (y1 * x2)
    /// ```
    ///
    /// # Arguments
    /// * `other` - Another `Vec2D` vector to compute the cross product with.
    ///
    /// # Returns
    /// A scalar value of type `T` that represents the cross product of the two vectors.
    pub fn cross(self, other: &Self) -> T { self.x * other.y - self.y * other.x }

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
}

impl Vec2D<i32> {
    /// Converts the vector to an unsigned equivalent.
    ///
    /// This method casts both the x and y components of the vector from `i32` to `u32`.
    ///
    /// # Returns
    /// A `Vec2D<u32>` with the unsigned components of the original vector.
    ///
    /// # Note
    /// The conversion may cause loss of sign. Negative values will wrap around.
    #[allow(clippy::cast_sign_loss)]
    pub fn to_unsigned(self) -> Vec2D<u32> {
        Vec2D {
            x: self.x as u32,
            y: self.y as u32,
        }
    }
}

impl<T> Vec2D<T>
where T: Add<Output = T> + Rem<Output = T> + Copy + MapSize<Output = T>
{
    /// Wraps the vector around a predefined 2D map.
    ///
    /// This method ensures the vector’s coordinates do not exceed the boundaries
    /// of the map defined by `map_size()`. If coordinates go beyond these boundaries,
    /// they are wrapped to remain within valid values.
    pub fn wrap_around_map(&self) -> Self {
        let map_size_x = T::map_size().x;
        let map_size_y = T::map_size().y;

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
}

impl<T> Add for Vec2D<T>
where T: Add<Output = T>
{
    type Output = Vec2D<T>;

    /// Implements the `+` operator for two `Vec2D` objects.
    ///
    /// # Arguments
    /// * `rhs` - The vector to add.
    ///
    /// # Returns
    /// A new `Vec2D` representing the sum of the vectors.
    fn add(self, rhs: Vec2D<T>) -> Self::Output {
        Self::Output {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
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

impl<T> Div<T> for Vec2D<T>
where T: Div<T, Output = T> + Copy
{
    type Output = Vec2D<T>;

    /// Implements the `/` operator for a `Vec2D` and a scalar.
    ///
    /// # Arguments
    /// * `rhs` - The scalar value to divide by.
    ///
    /// # Returns
    /// A new scaled vector.
    fn div(self, rhs: T) -> Self::Output {
        Self::Output {
            x: self.x / rhs,
            y: self.y / rhs,
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

impl<T: Num> From<Vec2D<T>> for (T, T) {
    /// Creates a tuple from a `Vec2D` of (x, y) values.
    ///
    /// # Arguments
    /// * `tuple` - A tuple representing the x and y values.
    ///
    /// # Returns
    /// A new `Vec2D` created from the tuple.
    fn from(value: Vec2D<T>) -> Self { (value.x, value.y) }
}

impl<T> From<&[T; 2]> for Vec2D<T>
where T: Copy
{
    /// Creates a `Vec2D` from a slice of (x, y) values.
    ///
    /// # Arguments
    /// * `slice` - A tuple representing the x and y values.
    ///
    /// # Returns
    /// A new `Vec2D` created from the slice.
    fn from(slice: &[T; 2]) -> Self {
        Self {
            x: slice[0],
            y: slice[1],
        }
    }
}
