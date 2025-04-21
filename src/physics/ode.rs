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

pub fn ode_rk2<F: FnMut(&[f32], f32) -> Vec<f32>>(X: &[f32], delta_secs: f32, mut dxdt: F) -> Vec<f32> {
    let n = X.len();
    let mut ret: Vec<f32> = vec![0.0; n];

    // k1 = Δt * dX/dt(X, t)
    let k1 = dxdt(X, delta_secs);
    let k1_scaled: Vec<f32> = k1.iter().map(|&x| x * delta_secs).collect();

    // Compute X + k1/2 for midpoint
    let mut X_mid: Vec<f32> = vec![0.0; n];
    for i in 0..n
    {
        X_mid[i] = X[i] + 0.5 * k1_scaled[i];
    }

    // k2 = Δt * dX/dt(X + k1/2, t + Δt/2)
    let k2 = dxdt(&X_mid, delta_secs);
    let k2_scaled: Vec<f32> = k2.iter().map(|&x| x * delta_secs).collect();

    // Update: X(t + Δt) = X(t) + k2
    for i in 0..n
    {
        ret[i] = X[i] + k2_scaled[i];
    }
    return ret;
}

pub fn ode_rk4<F: FnMut(&[f32], f32) -> Vec<f32>>(X: &[f32], delta_secs: f32, mut dxdt: F) -> Vec<f32> {
    let n = X.len();
    let mut ret: Vec<f32> = vec![0.0; n];

    // k1 = Δt * dX/dt(X, t)
    let k1 = dxdt(X, delta_secs);
    let k1_scaled: Vec<f32> = k1.iter().map(|&x| x * delta_secs).collect();

    // Compute X + k1/2 for k2
    let mut X_temp: Vec<f32> = vec![0.0; n];
    for i in 0..n
    {
      X_temp[i] = X[i] + 0.5 * k1_scaled[i];
    }

    // k2 = Δt * dX/dt(X + k1/2, t + Δt/2)
    let k2 = dxdt(&X_temp, delta_secs);
    let k2_scaled: Vec<f32> = k2.iter().map(|&x| x * delta_secs).collect();

    // Compute X + k2/2 for k3
    for i in 0..n
    {
      X_temp[i] = X[i] + 0.5 * k2_scaled[i];
    }

    // k3 = Δt * dX/dt(X + k2/2, t + Δt/2)
    let k3 = dxdt(&X_temp, delta_secs);
    let k3_scaled: Vec<f32> = k3.iter().map(|&x| x * delta_secs).collect();

    // Compute X + k3 for k4
    for i in 0..n
    {
      X_temp[i] = X[i] + k3_scaled[i];
    }

    // k4 = Δt * dX/dt(X + k3, t + Δt)
    let k4 = dxdt(&X_temp, delta_secs);
    let k4_scaled: Vec<f32> = k4.iter().map(|&x| x * delta_secs).collect();

    // Update: X(t + Δt) = X(t) + (k1 + 2*k2 + 2*k3 + k4)/6
    for i in 0..n
    {
      ret[i] = X[i] + (k1_scaled[i] + 2.0 * k2_scaled[i] + 2.0 * k3_scaled[i] + k4_scaled[i]) / 6.0;
    }
    return ret;
}
