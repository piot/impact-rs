# Impact-rs

This crate provides utilities for performing collision queries between rectangles and rays, 
including swept checks for moving rectangles. It leverages fixed-point arithmetic provided by the `fixed32` crate to 
handle the computations.

## Features

- **Ray vs. Rect Collision Detection**: Detect collisions between a ray and a rectangle, returning contact point, contact normal,  
  and the closest time of collision.
- **Swept Rectangle Collision**: Check for potential collisions as one rectangle moves towards another, 
  using both horizontal and vertical sweeps.
- **Fixed-Point Precision**: Uses fixed-point numbers (`Fp`) from the `fixed32` crate for precise calculations 
  without floating-point errors.

## Usage

To use this crate in your project, add it to your `Cargo.toml`:

```toml
[dependencies]
impact_rs = "0.0.11"
```

## Example

```rust
use fixed32::Fp;
use fixed32_math::{Rect, Vector};
use impact_rs::ray_vs_rect;

fn main() {
  let ray_origin = Vector::from((1, 2));
  let ray_direction = Vector::from((3, 4));
  let target_rect = Rect::from((5, 6, 7, 8));

  let collision_result = ray_vs_rect(ray_origin, ray_direction, target_rect);

  if let Some(result) = collision_result {
    println!("Collision at: {:?}", result.contact_point);
    println!("Normal at collision: {:?}", result.contact_normal);
    println!("Closest collision time: {:?}", result.closest_time);
  } else {
    println!("No collision detected.");
  }
}
```
