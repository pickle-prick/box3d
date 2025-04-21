pub fn ode_euler<F: FnMut(&[f32], f32) -> Vec<f32>>(X: &[f32], delta_secs: f32, mut dxdt: F) -> Vec<f32>
{
  let mut ret: Vec<f32> = vec![0.0; X.len()];

  let mut Xdot = dxdt(X, delta_secs);

  // TODO(XXX): SIMD?
  for (i, xdot) in Xdot.iter().enumerate()
  {
    ret[i] = xdot*delta_secs + X[i];
  }

  return ret;
}

// TODO(XXX): rk2 or rk4
