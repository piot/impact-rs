/*----------------------------------------------------------------------------------------------------------
 *  Copyright (c) Peter Bjorklund. All rights reserved. https://github.com/piot/impact-rs
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *--------------------------------------------------------------------------------------------------------*/
use fixed32::Fp;
 use fixed32_math::{Rect, Vector};
 use std::cmp::{max, min};

 #[derive(Debug, Clone)]
 pub struct RayIntersectionResult {
     pub contact_point: Vector,
     pub contact_normal: Vector,
     pub closest_time: Fp,
 }

 pub fn swept_rect_vs_rect(
     origin: &Rect,
     target: &Rect,
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

 pub fn ray_vs_rect(
     ray_origin: Vector,
     ray_direction: Vector,
     target: Rect,
 ) -> Option<RayIntersectionResult> {
     if ray_direction.x.0 == 0 && ray_direction.y.0 == 0 {
         return None;
     }

     let mut time_near = Vector::default();
     let mut time_far = Vector::default();

     let inverted_direction = Vector::new(
         if ray_direction.x.0 != 0 {
             Fp::one() / ray_direction.x
         } else {
             Fp::zero()
         },
         if ray_direction.y.0 != 0 {
             Fp::one() / ray_direction.y
         } else {
             Fp::zero()
         },
     );

     if ray_direction.x.0 > 0 {
         time_near.x = (target.pos.x - ray_origin.x) * inverted_direction.x;
         time_far.x = (target.pos.x + target.size.x - ray_origin.x) * inverted_direction.x;
     } else if ray_direction.x.0 < 0 {
         time_near.x = (target.pos.x + target.size.x - ray_origin.x) * inverted_direction.x;
         time_far.x = (target.pos.x - ray_origin.x) * inverted_direction.x;
     } else {
         // Ray direction is purely vertical
         if ray_origin.x < target.pos.x || ray_origin.x > target.pos.x + target.size.x {
             return None;
         }
         time_near.x = Fp::MIN;
         time_far.x = Fp::MAX;
     }

     if ray_direction.y.0 > 0 {
         time_near.y = (target.pos.y - ray_origin.y) * inverted_direction.y;
         time_far.y = (target.pos.y + target.size.y - ray_origin.y) * inverted_direction.y;
     } else if ray_direction.y.0 < 0 {
         time_near.y = (target.pos.y + target.size.y - ray_origin.y) * inverted_direction.y;
         time_far.y = (target.pos.y - ray_origin.y) * inverted_direction.y;
     } else {
         // Ray direction is purely horizontal
         if ray_origin.y < target.pos.y || ray_origin.y > target.pos.y + target.size.y {
             return None;
         }
         time_near.y = Fp::MIN;
         time_far.y = Fp::MAX;
     }

     // Sort distances
     if time_near.x > time_far.x {
         let temp = time_near.x;
         time_near.x = time_far.x;
         time_far.x = temp;
     }

     if time_near.y > time_far.y {
         let temp = time_near.y;
         time_near.y = time_far.y;
         time_far.y = temp;
     }

     if time_near.x >= time_far.y || time_near.y >= time_far.x {
         return None;
     }

     let time_far_magnitude = min(time_far.x, time_far.y);

     if time_far_magnitude.0 < 0 {
         return None;
     }

     let closest_time = max(time_near.x, time_near.y);

     let contact_point = ray_origin + closest_time * ray_direction;

     let mut contact_normal: Vector = Vector::default();

     if time_near.x > time_near.y {
         if ray_direction.x.0 > 0 {
             contact_normal = Vector::right();
         } else {
             contact_normal = Vector::left();
         }
     } else if time_near.x < time_near.y {
         if ray_direction.y.0 > 0 {
             contact_normal = Vector::up();
         } else {
             contact_normal = Vector::down();
         }
     }

     Some(RayIntersectionResult {
         contact_point,
         contact_normal,
         closest_time,
     })
 }

 pub fn swept_rect_vs_rect_vertical_time(origin: &Rect, target: &Rect, y_delta: Fp) -> Option<Fp> {
    let combined_target_rect = Rect {
        pos: target.pos,
        size: target.size + origin.size,
    };

    let ray_origin = origin.pos + origin.size;

     let maybe_intersected = ray_vs_rect_horizontal_time(ray_origin, y_delta, combined_target_rect);
     if let Some(time) = maybe_intersected {
         if time >= Fp::zero() && time < Fp::one() {
             return maybe_intersected;
         }
     }

     return None
}

pub fn ray_vs_rect_vertical_time(
    ray_origin: Vector,
    ray_length_in_y: Fp,
    target_rect: Rect,
) -> Option<Fp> {
    if ray_length_in_y.0 == 0 {
        return None;
    }

    if ray_origin.x < target_rect.pos.x || ray_origin.x > target_rect.pos.x + target_rect.size.x {
        return None;
    }

    let closest_time = if ray_length_in_y.0 > 0 {
        (target_rect.pos.y - ray_origin.y) / ray_length_in_y
    } else {
        (target_rect.pos.y + target_rect.size.y - ray_origin.y) / ray_length_in_y
    };

    Some(closest_time)
}

pub fn swept_rect_vs_rect_horizontal_time(
    origin: &Rect,
    target: &Rect,
    x_delta: Fp,
) -> Option<Fp> {
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

pub fn ray_vs_rect_horizontal_time(
    ray_origin: Vector,
    ray_length_in_x: Fp,
    target_rect: Rect,
) -> Option<Fp> {
    if ray_length_in_x.0 == 0 {
        return None;
    }

    if ray_origin.y < target_rect.pos.y || ray_origin.y > target_rect.pos.y + target_rect.size.y {
        return None;
    }

    let closest_time = if ray_length_in_x.0 > 0 {
        (target_rect.pos.x - ray_origin.x) / ray_length_in_x
    } else {
        (target_rect.pos.x + target_rect.size.x - ray_origin.x) / ray_length_in_x
    };

    Some(closest_time)
}