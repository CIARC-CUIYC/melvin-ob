use std::ops::{Add, Deref, Div, Mul};
use num_traits::{Num, NumAssignOps, NumCast, real::Real};

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
where
    T: Num + NumCast + Copy,
{
    pub fn wrap_around_map(&self) -> Self {
        Wrapped2D(Vec2D::new(Self::wrap_coordinate(self.0.x, T::from(X).unwrap()), Self::wrap_coordinate(self.0.y, T::from(Y).unwrap())))
    }

    pub fn wrap_coordinate(value: T, max_value: T) -> T {
        (value + max_value) % max_value
    }
}


impl<T, const X: u32, const Y: u32> Deref for Wrapped2D<T, X, Y> {
    type Target = Vec2D<T>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> Vec2D<T>
where
    T: Real + NumCast + NumAssignOps,
{
    /// Computes the magnitude (absolute value) of the vector.
    ///
    /// # Returns
    /// The magnitude of the vector as a scalar of type `T`.
    pub fn abs(&self) -> T { (self.x.powi(2) + self.y.powi(2)).sqrt() }

    /// Creates a vector pointing from the current vector (`self`) to another vector (`other`).
    ///
    /// # Arguments
    /// * `other` - The target vector.
    ///
    /// # Returns
    /// A new vector representing the direction from `self` to `other`.
    pub fn to(&self, other: &Vec2D<T>) -> Vec2D<T> {
        Vec2D::new(other.x - self.x, other.y - self.y)
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
    pub fn rotate_by(&mut self, angle_degrees: f32) {
        let angle_radians = T::from(angle_degrees.to_radians()).unwrap();
        let new_x = self.x * angle_radians.cos() - self.y * angle_radians.sin();
        self.y = self.x * angle_radians.sin() + self.y * angle_radians.cos();
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
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
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

impl<T: Num + NumCast + Copy> Vec2D<T> {
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
    pub fn dot(self, other: Vec2D<T>) -> T { self.x * other.x + self.y * other.y }

    /// Computes the Euclidean distance between the current vector and another vector as an `f64`.
    /// This enables Euclidean distance calculation for integer type `T`. 
    ///
    /// # Arguments
    /// * `other` - Another `Vec2D` vector to compute the distance to.
    ///
    /// # Returns
    /// The Euclidean distance between the two vectors as an `f64`.
    pub fn euclid_distance_f64(&self, other: &Self) -> f64 {
        let self_x = self.x.to_f64().unwrap();
        let self_y = self.y.to_f64().unwrap();
        let other_x = other.x.to_f64().unwrap();
        let other_y = other.y.to_f64().unwrap();
        ((self_x - self_y).powi(2) + (other_x - other_y).powi(2)).sqrt()
    }

    /// Checks if the current vector is within a radius of another vector.
    /// The radius (`rad`) is given as a scalar of type `T`, which is converted to `f64`
    /// for comparison with the Euclidean distance calculated between the vectors.
    ///
    /// # Arguments
    /// * `other` - The other `Vec2D` vector to compare against.
    /// * `rad` - The radius as a value of type `T`.
    ///
    /// # Returns
    /// A boolean value:
    /// - `true` if the current vector is within the radius of `other`. `false` otherwise.
    pub fn in_radius_of(&self, other: &Self, rad: T) -> bool {
        self.euclid_distance_f64(other) <= rad.to_f64().unwrap()
    }

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
            x: T::from(21600.0).unwrap(),
            y: T::from(10800.0).unwrap(),
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

        Vec2D::new(Self::wrap_coordinate(self.x, map_size_x), Self::wrap_coordinate(self.y, map_size_y))
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
        (value + max_value) % max_value
    }

    pub fn cast<D: NumCast>(self) -> Vec2D<D> {
        Vec2D {
            x: D::from(self.x).unwrap(),
            y: D::from(self.y).unwrap(),
        }
    }
}

impl<T, TAdd> Add<Vec2D<TAdd>> for Vec2D<T>
where
    T: Num + NumCast,
    TAdd: Num + NumCast,
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
            x: self.x + T::from(rhs.x).unwrap(),
            y: self.y + T::from(rhs.y).unwrap(),
        }
    }
}

impl<T, TMul> Mul<TMul> for Vec2D<T>
where
    T: Num + NumCast,
    TMul: Num + NumCast + Copy,
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
            x: self.x * T::from(rhs).unwrap(),
            y: self.y * T::from(rhs).unwrap(),
        }
    }
}

impl<T, TMul> Div<TMul> for Vec2D<T>
where
    T: Num + NumCast,
    TMul: Num + NumCast + Copy,
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
            x: self.x / T::from(rhs).unwrap(),
            y: self.y / T::from(rhs).unwrap(),
        }
    }
}

impl<T: Num + NumCast> From<(T, T)> for Vec2D<T> {
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