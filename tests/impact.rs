/*
 * Copyright (c) Peter Bjorklund. All rights reserved. https://github.com/piot/impact-rs
 * Licensed under the MIT License. See LICENSE in the project root for license information.
 */

use fixed32::Fp;
use fixed32_math::{Rect, Vector};
use impact_rs::prelude::*;

#[test]
fn test_ray_vs_rect() {
    let ray_origin = Vector::from((1, 2));
    let ray_direction = Vector::from((3, 4));
    let target_rect = Rect::from((5, 6, 7, 8));

    let collision_result = ray_vs_rect(ray_origin, ray_direction, target_rect);
    let ray_intersect = collision_result.expect("should have intersected");
    assert_eq!(ray_intersect.closest_time, Fp::from(1.33332));
}
