use bevy::{prelude::*};

pub fn inertia_from_cuboid(M: f32, dim: Vec3) -> Mat3
{
  let mut ret = Mat3::ZERO;
  let Vec3 {x: x0, y: y0, z: z0} = dim;
  ret.x_axis[0] = (y0*y0+z0*z0) * (M/12.0);
  ret.y_axis[1] = (x0*x0+z0*z0) * (M/12.0);
  ret.z_axis[2] = (x0*x0+y0*y0) * (M/12.0);
  return ret;
}

pub fn inertiainv_from_cuboid(M: f32, dim: Vec3) -> Mat3
{
  let mut ret = Mat3::ZERO;
  let Vec3 {x: x0, y: y0, z: z0} = dim;
  ret.x_axis[0] = 12.0 / ((y0*y0+z0*z0)*M);
  ret.y_axis[1] = 12.0 / ((x0*x0+z0*z0)*M);
  ret.z_axis[2] = 12.0 / ((x0*x0+y0*y0)*M);
  return ret;
}


#[cfg(test)]
mod tests
{
  use super::*;

  fn assert_f32_equal(a: f32, b: f32, epslion: f32)
  {
    assert!((a-b).abs() < epslion);
  }

  #[test]
  fn test_inertia()
  {
    let mass: f32 = 1.0;
    let dim: Vec3 = Vec3::new(1.0, 1.0, 1.0);

    let inertia = inertia_from_cuboid(mass, dim);

    assert_f32_equal(inertia.row(0)[0], 0.1667, 0.001);
    assert_f32_equal(inertia.row(1)[1], 0.1667, 0.001);
    assert_f32_equal(inertia.row(2)[2], 0.1667, 0.001);

    let inertia_inv = inertiainv_from_cuboid(mass, dim);
    assert_f32_equal(inertia_inv.row(0)[0], 6.0, 0.001);
    assert_f32_equal(inertia_inv.row(1)[1], 6.0, 0.001);
    assert_f32_equal(inertia_inv.row(2)[2], 6.0, 0.001);
  }
}
