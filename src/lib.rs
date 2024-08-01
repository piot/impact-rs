/*----------------------------------------------------------------------------------------------------------
 *  Copyright (c) Peter Bjorklund. All rights reserved. https://github.com/piot/impact-rs
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *--------------------------------------------------------------------------------------------------------*/

/*!
This crate provides utilities for performing collision queries between rectangles and rays,
including swept checks for moving rectangles. It leverages fixed-point arithmetic provided by the [`fixed32`] crate to
handle the computations.
*/

pub mod prelude;
mod test;

use std::cmp::{max, min, Ordering};

use fixed32::Fp;
use fixed32_math::{Rect, Vector};

#[derive(Debug, Clone)]
pub struct RayIntersectionResult {
    pub contact_point: Vector,
    pub contact_normal: Vector,
    pub closest_time: Fp,
}

/// Checks for intersection between a swept rectangle and a target rectangle.
///
/// This function determines if a rectangle, which is moving along a vector
/// from its initial position, intersects with a target rectangle at any point
/// during its motion. The swept rectangle is calculated based on the initial
/// lower-left position and size of the origin rectangle, expanded to account for its
/// movement. The function performs a ray-rectangle intersection test to
/// check if there is an intersection within the valid time range.
///
/// # Parameters
///
/// - `origin`: A [`Rect`] representing the starting rectangle
///   that is swept along a direction and length defined by the `delta` vector.
/// - `target`: A [`Rect`] representing the target rectangle
///   to check for intersection with the swept rectangle.
/// - `delta`: The vector representing the movement direction and magnitude
///   of the `origin` rectangle. This vector determines how far and in which
///   direction the origin rectangle is moved.
///
/// # Returns
///
/// Returns `Some(RayIntersectionResult)` if there is an intersection between
/// the swept rectangle and the target rectangle within the valid time range.
///
/// The [`RayIntersectionResult`] contains information about the intersection
/// point and other related details. If there is no intersection or the
/// intersection does not occur within the valid time range, `None` is returned.
///
/// # Example
///
/// ```rust
/// use fixed32_math::{Rect, Vector};
/// use impact_rs::prelude::*;
///
/// let origin = Rect::from((0.0, 0.0, 10.0, 10.0));
/// let target = Rect::from((5.0, 5.0, 10.0, 10.0));
/// let delta = Vector::from((15.0, 15.0));
///
/// match swept_rect_vs_rect(origin, target, delta) {
///     Some(result) => {
///         println!("Intersection found: {:?}", result);
///     }
///     None => {
///         println!("No intersection.");
///     }
/// }
/// ```
pub fn swept_rect_vs_rect(
    origin: Rect,
    target: Rect,
    delta: Vector,
) -> Option<RayIntersectionResult> {
    let expanded_target = Rect {
        pos: target.pos - origin.size / 2,
        size: target.size + origin.size,
    };

    let origin_point = origin.pos + origin.size;

    let maybe_intersected = ray_vs_rect(origin_point, delta, expanded_target);
    if let Some(result) = maybe_intersected {
        let time = result.closest_time;
        if time >= Fp::zero() && time < Fp::one() {
            return Some(result);
        }
    }

    None
}

/// Performs a ray-rectangle intersection test.
///
/// This function determines if a ray intersects with a given rectangle. The ray
/// is defined by its origin and direction, and the rectangle is defined by its
/// lower-left position and size. The function computes the intersection time and contact
/// point if an intersection occurs.
///
/// # Parameters
///
/// - `ray_origin`: The origin point of the ray as a [`Vector`]. This is where
///   the ray starts.
/// - `ray_direction`: The ray as a [`Vector`]. This indicates
///   the direction and length in which the ray is cast. The direction vector must not be zero.
/// - `target`: The [`Rect`] representing the target rectangle to test for intersection.
///   It is defined by its lower-left position and size.
///
/// # Returns
///
/// Returns `Some(RayIntersectionResult)` if there is an intersection between
/// the ray and the rectangle. The [`RayIntersectionResult`] includes:
/// - `contact_point`: The point of intersection between the ray and the rectangle.
/// - `contact_normal`: The normal vector of the rectangle at the point of intersection.
/// - `closest_time`: The normalized time along the ray at which the intersection occurs.
///
/// Returns `None` if there is no intersection or if the ray direction is zero.
///
/// # Example
///
/// ```rust
/// use fixed32_math::{Rect, Vector};
/// use impact_rs::prelude::*;
///
/// let ray_origin = Vector::from((0.0, 0.0));
/// let ray_direction = Vector::from((1.0, 1.0));
/// let target = Rect::from((5.0, 5.0, 10.0, 10.0));
///
/// match ray_vs_rect(ray_origin, ray_direction, target) {
///     Some(result) => {
///         println!("Intersection found at point: {:?}", result.contact_point);
///         println!("Contact normal: {:?}", result.contact_normal);
///         println!("Intersection time: {:?}", result.closest_time);
///     },
///     None => {
///         println!("No intersection.");
///     },
/// }
/// ```
pub fn ray_vs_rect(
    ray_origin: Vector,
    ray_direction: Vector,
    target: Rect,
) -> Option<RayIntersectionResult> {
    if ray_direction.x.is_zero() && ray_direction.y.is_zero() {
        return None;
    }

    let mut time_near = Vector::default();
    let mut time_far = Vector::default();

    let inverted_direction = Vector::new(
        if ray_direction.x != 0 {
            Fp::one() / ray_direction.x
        } else {
            Fp::zero()
        },
        if ray_direction.y != 0 {
            Fp::one() / ray_direction.y
        } else {
            Fp::zero()
        },
    );

    match ray_direction.x.cmp(&Fp::zero()) {
        Ordering::Greater => {
            time_near.x = (target.pos.x - ray_origin.x) * inverted_direction.x;
            time_far.x = (target.pos.x + target.size.x - ray_origin.x) * inverted_direction.x;
        }
        Ordering::Less => {
            time_near.x = (target.pos.x + target.size.x - ray_origin.x) * inverted_direction.x;
            time_far.x = (target.pos.x - ray_origin.x) * inverted_direction.x;
        }
        Ordering::Equal => {
            // Ray direction is purely vertical
            if ray_origin.x < target.pos.x || ray_origin.x > target.pos.x + target.size.x {
                return None;
            }
            time_near.x = Fp::MIN;
            time_far.x = Fp::MAX;
        }
    }

    match ray_direction.y.cmp(&Fp::zero()) {
        Ordering::Greater => {
            time_near.y = (target.pos.y - ray_origin.y) * inverted_direction.y;
            time_far.y = (target.pos.y + target.size.y - ray_origin.y) * inverted_direction.y;
        }
        Ordering::Less => {
            time_near.y = (target.pos.y + target.size.y - ray_origin.y) * inverted_direction.y;
            time_far.y = (target.pos.y - ray_origin.y) * inverted_direction.y;
        }
        Ordering::Equal => {
            // Ray direction is purely horizontal
            if ray_origin.y < target.pos.y || ray_origin.y > target.pos.y + target.size.y {
                return None;
            }
            time_near.y = Fp::MIN;
            time_far.y = Fp::MAX;
        }
    }

    // Sort distances
    if time_near.x > time_far.x {
        std::mem::swap(&mut time_near.x, &mut time_far.x);
    }

    if time_near.y > time_far.y {
        std::mem::swap(&mut time_near.y, &mut time_far.y);
    }

    if time_near.x >= time_far.y || time_near.y >= time_far.x {
        return None;
    }

    let time_far_magnitude = min(time_far.x, time_far.y);

    if time_far_magnitude < 0 {
        return None;
    }

    let closest_time = max(time_near.x, time_near.y);

    let contact_point = ray_origin + closest_time * ray_direction;

    let mut contact_normal: Vector = Vector::default();

    match time_near.x.cmp(&time_near.y) {
        Ordering::Greater => {
            contact_normal = if ray_direction.x > 0 {
                Vector::right()
            } else {
                Vector::left()
            };
        }
        Ordering::Less => {
            contact_normal = if ray_direction.y > 0 {
                Vector::up()
            } else {
                Vector::down()
            };
        }
        Ordering::Equal => {
            // Handle the case where time_near.x == time_near.y, if needed
        }
    }

    Some(RayIntersectionResult {
        contact_point,
        contact_normal,
        closest_time,
    })
}

/// Checks for intersection between a vertically swept rectangle and a target rectangle.
///
/// This function determines if a rectangle, swept vertically from its initial
/// position, intersects with a target rectangle. The swept rectangle is calculated
/// based on the initial position and size of the origin rectangle, expanded to account
/// for its vertical movement. The function performs a vertical intersection test
/// to determine if there is an intersection within the valid time range.
///
/// # Parameters
///
/// - `origin`: A [`Rect`] representing the starting rectangle
///   that is swept vertically.
/// - `target`: A [`Rect`] representing the target rectangle
///   to check for intersection with the swept rectangle.
/// - `y_delta`: The vertical movement distance of the `origin` rectangle.
///   This value indicates how far the origin rectangle moves along the y-axis.
///
/// # Returns
///
/// Returns `Some(Fp)` if there is an intersection between the swept rectangle
/// and the target rectangle within the valid time range `[0, 1)`. The [`Fp`] value
/// represents the normalized time at which the intersection occurs. If there is no intersection
/// or if the intersection does not occur within the valid time range, `None` is returned.
///
/// # Example
///
/// ```rust
/// use fixed32_math::{Rect, Vector};
/// use impact_rs::prelude::*;
/// use fixed32::Fp;
///
/// let origin = Rect::from((0.0, 0.0, 10.0, 10.0));
/// let target = Rect::from((5.0, 15.0, 10.0, 10.0));
/// let y_delta = Fp::from(20.0);
///
/// match swept_rect_vs_rect_vertical_time(origin, target, y_delta) {
///     Some(time) => {
///         println!("Intersection found at time: {:?}", time);
///     },
///     None => {
///         println!("No intersection.");
///     },
/// }
/// ```
///
pub fn swept_rect_vs_rect_vertical_time(origin: Rect, target: Rect, y_delta: Fp) -> Option<Fp> {
    let combined_target_rect = Rect {
        pos: target.pos,
        size: target.size + origin.size,
    };

    let ray_origin = origin.pos + origin.size;

    let maybe_intersected = ray_vs_rect_vertical_time(ray_origin, y_delta, combined_target_rect);
    if let Some(time) = maybe_intersected {
        if time >= Fp::zero() && time < Fp::one() {
            return maybe_intersected;
        }
    }

    None
}

/// Computes the intersection time of a vertical ray with a target rectangle.
///
/// This function calculates the time at which a vertical ray intersects a given
/// rectangle. The ray is defined by its origin and its length of movement along
/// the y-axis. The function determines if and when this ray intersects the vertical
/// sides of the rectangle based on the ray's direction and position.
///
/// # Parameters
///
/// - `ray_origin`: The starting point of the ray in 2D space. This represents the
///   position from which the ray begins.
/// - `ray_length_in_y`: The length or direction of the ray's movement along the y-axis.
///   A negative value indicates movement downward, while a positive value indicates
///   movement upward.
/// - `target_rect`: The rectangle with which the ray is tested for intersection.
///   This rectangle is defined by its lower-left position and size.
///
/// # Returns
///
/// Returns `Some(Fp)` containing the intersection time if the ray intersects the
/// target rectangle along the vertical axis. The returned [`Fp`] value represents the
/// time at which the intersection occurs. If the ray does not intersect the rectangle
/// or if it does not move vertically, `None` is returned.
///
/// # Example
///
/// ```rust
/// use fixed32_math::{Rect, Vector};
/// use impact_rs::prelude::*;
/// use fixed32::Fp;
///
/// let ray_origin = Vector::from((5.0, 0.0));
/// let ray_length_in_y = Fp::from(10.0);
/// let target_rect = Rect::from((0.0, 5.0, 10.0, 10.0));
///
/// match ray_vs_rect_vertical_time(ray_origin, ray_length_in_y, target_rect) {
///     Some(time) => {
///         println!("Intersection occurs at time: {:?}", time);
///     },
///     None => {
///         println!("No intersection or ray does not move vertically.");
///     },
/// }
/// ```
pub fn ray_vs_rect_vertical_time(
    ray_origin: Vector,
    ray_length_in_y: Fp,
    target_rect: Rect,
) -> Option<Fp> {
    if ray_length_in_y.is_zero() {
        return None;
    }

    if ray_origin.x < target_rect.pos.x || ray_origin.x > target_rect.pos.x + target_rect.size.x {
        return None;
    }

    let closest_time = if ray_length_in_y > 0 {
        (target_rect.pos.y - ray_origin.y) / ray_length_in_y
    } else {
        (target_rect.pos.y + target_rect.size.y - ray_origin.y) / ray_length_in_y
    };

    Some(closest_time)
}

/// Checks for intersection between a swept rectangle and a target rectangle
/// along the horizontal axis.
///
/// This function determines if a rectangle, swept horizontally from its initial
/// position, intersects with a target rectangle at any point during its horizontal
/// movement. The swept rectangle is calculated based on the initial lower-left position and
/// size of the origin rectangle, expanded to account for its movement along the
/// x-axis. The function performs a horizontal ray-rectangle intersection test
/// to check if there is an intersection within the valid time range.
///
/// # Parameters
///
/// - `origin`: A [`Rect`] representing the starting rectangle
///   that is swept horizontally.
/// - `target`: A [`Rect`] representing the target rectangle
///   to check for intersection with the swept rectangle.
/// - `x_delta`: The horizontal movement distance of the `origin` rectangle.
///   This value represents how far the origin rectangle moves along the x-axis.
///
/// # Returns
///
/// Returns `Some(Fp)` if there is an intersection between the swept rectangle
/// and the target rectangle along the horizontal axis within the valid time range.
/// The `Fp` value represents the time at which the intersection occurs. If there
/// is no intersection or if the intersection does not occur within the valid normalized time
/// range `[0, 1)`, `None` is returned.
///
/// # Example
///
/// ```rust
/// use fixed32_math::{Rect, Vector};
/// use impact_rs::prelude::*;
/// use fixed32::Fp;
///
/// let origin = Rect::from((0.0, 0.0, 10.0, 10.0));
/// let target = Rect::from((15.0, 5.0, 10.0, 10.0));
/// let x_delta = Fp::from(20.0);
///
/// match swept_rect_vs_rect_horizontal_time(origin, target, x_delta) {
///     Some(time) => {
///         println!("Intersection found at time: {:?}", time);
///     },
///     None => {
///         println!("No intersection.");
///     },
/// }
/// ```
pub fn swept_rect_vs_rect_horizontal_time(origin: Rect, target: Rect, x_delta: Fp) -> Option<Fp> {
    let expanded_target = Rect {
        pos: target.pos,
        size: target.size + origin.size,
    };

    let origin_point = origin.pos + origin.size;

    let maybe_intersected = ray_vs_rect_horizontal_time(origin_point, x_delta, expanded_target);
    if let Some(time) = maybe_intersected {
        if time >= Fp::zero() && time < Fp::one() {
            return maybe_intersected;
        }
    }

    None
}

/// Computes the intersection time of a horizontal ray with a target rectangle.
///
/// This function calculates the time at which a horizontal ray intersects a given
/// rectangle. The ray is defined by its origin and its length of movement along
/// the x-axis. The function determines if and when this ray intersects the horizontal
/// sides of the rectangle based on the ray's direction and position.
///
/// # Parameters
///
/// - `ray_origin`: The starting point of the ray in 2D space. This represents the
///   position from which the ray begins.
/// - `ray_length_in_x`: The length or direction of the ray's movement along the x-axis.
///   A positive value indicates movement to the right, while a negative value indicates
///   movement to the left.
/// - `target_rect`: The rectangle with which the ray is tested for intersection.
///   This rectangle is defined by its lower-left position and size.
///
/// # Returns
///
/// Returns `Some(Fp)` containing the intersection time if the ray intersects the
/// target rectangle along the horizontal axis. The returned [`Fp`] value represents the
/// time at which the intersection occurs. If the ray does not intersect the rectangle
/// or if it does not move horizontally, `None` is returned.
///
/// # Example
///
/// ```rust
/// use fixed32_math::{Rect, Vector};
/// use impact_rs::prelude::*;
/// use fixed32::Fp;
///
/// let ray_origin = Vector::from((0.0, 5.0));
/// let ray_length_in_x = Fp::from(10.0);
/// let target_rect = Rect::from((5.0, 0.0, 10.0, 10.0));
///
/// match ray_vs_rect_horizontal_time(ray_origin, ray_length_in_x, target_rect) {
///     Some(time) => {
///         println!("Intersection occurs at time: {:?}", time);
///     },
///     None => {
///         println!("No intersection or ray does not move horizontally.");
///     },
/// }
/// ```
pub fn ray_vs_rect_horizontal_time(
    ray_origin: Vector,
    ray_length_in_x: Fp,
    target_rect: Rect,
) -> Option<Fp> {
    if ray_length_in_x == 0 {
        return None;
    }

    if ray_origin.y < target_rect.pos.y || ray_origin.y >= target_rect.pos.y + target_rect.size.y {
        return None;
    }

    let closest_time = if ray_length_in_x > 0 {
        (target_rect.pos.x - ray_origin.x) / ray_length_in_x
    } else {
        (target_rect.pos.x + target_rect.size.x - ray_origin.x) / ray_length_in_x
    };

    Some(closest_time)
}
