use crate::ray_vs_rect;
#[cfg(test)]
use fixed32::Fp;
use fixed32_math::{Rect, Vector};

#[test]
fn test_ray_vs_rect() {
    let ray_origin = Vector::from((1, 2));
    let ray_direction = Vector::from((3, 4));
    let target_rect = Rect::from((5, 6, 7, 8));

    let collision_result = ray_vs_rect(ray_origin, ray_direction, target_rect);
    let ray_intersect = collision_result.expect("should have intersected");
    assert_eq!(ray_intersect.closest_time, Fp::from(1.33332));
}
