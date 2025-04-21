use bevy::prelude::*;
use std::cmp;
use std::ops;
use std::collections::HashMap;
use std::mem::swap;

pub struct Mat
{
  i_dim: usize,
  j_dim: usize,
  rows: Vec<Vec<f32>>,
}

impl ops::Index<usize> for Mat
{
  type Output = Vec<f32>;
  fn index(&self, index: usize) -> &Self::Output
  {
    &self.rows[index]
  }
}

impl cmp::PartialEq for Mat
{
  fn eq(&self, other: &Self) -> bool
  {
    let mut ret = true;
    debug_assert!(self.i_dim == other.i_dim);
    debug_assert!(self.j_dim == other.j_dim);

    'outer: for i in 0..self.i_dim
    {
      for j in 0..self.j_dim
      {
        if (self[i][j]-other[i][j]).abs() > 0.001
        {
          ret = false;
          break 'outer;
        }
      }
    }
    
    return ret;
  }
}

impl ops::IndexMut<usize> for Mat
{
  fn index_mut(&mut self, index: usize) -> &mut Self::Output
  {
    &mut self.rows[index]
  }
}

impl Mat
{
  fn tranpose(&self) -> Mat
  {
    let i_dim = self.j_dim;
    let j_dim = self.i_dim;
    let mut ret: Mat = Mat::from_dim(i_dim, j_dim);

    for i in 0..i_dim
    {
      for j in 0..j_dim
      {
        ret[i][j] = self[j][i];
      }
    }
    return ret;
  }

  fn swap(&mut self, a: (usize, usize), b: (usize, usize))
  {
    let temp = self[b.0][b.1];
    self[b.0][b.1] = self[a.0][a.1];
    self[a.0][a.1] = temp;
  }

  fn from_dim(i_dim: usize, j_dim: usize) -> Self
  {
    let mut rows = vec![vec![0.0; j_dim]; i_dim];
    Self
    {
      i_dim,
      j_dim,
      rows,
    }
  }
}

impl ops::Mul<&Vec<f32>> for &Mat
{
  type Output = Vec<f32>;
  fn mul(self, rhs: &Vec<f32>) -> Vec<f32>
  {
    // row major
    debug_assert_eq!(self.j_dim, rhs.len());
    let mut ret: Vec<f32> = vec![0.0; self.i_dim];

    for i in 0..self.i_dim
    {
      let mut acc: f32 = 0.0;
      for j in 0..self.j_dim
      {
        acc += self[i][j] * rhs[j];
      }
      ret[i] = acc;
    }
    return ret;
  }
}

impl ops::Mul<&Mat> for &Mat
{
  type Output = Mat;
  fn mul(self, rhs: &Mat) -> Mat
  {
    debug_assert_eq!(self.j_dim, rhs.i_dim);
    let mut ret: Mat = Mat::from_dim(self.i_dim, rhs.j_dim);
    for i in 0..self.i_dim
    {
      for j in 0..rhs.j_dim
      {
        let mut acc: f32 = 0.0;
        for k in 0..self.j_dim
        {
          acc += self[i][k] * rhs[k][j];
        }
        ret[i][j] = acc;
      }
    }
    return ret;
  }
}

pub struct Gravity3D
{
  g: f32,
  dir: Vec3,
}

pub struct VisousDrag3D
{
  pub kd: f32,
  pub target: Option<Entity>,
}

pub struct ConstantForce3D
{
  pub dir: Vec3,
  pub strength: f32,
  // position relative to the body's center of mass (in world space)
  pub contact: Vec3,
  pub target: Entity,
}

pub enum Force3D
{
  Gravity(Gravity3D),
  VisousDrag(VisousDrag3D),
  Constant(ConstantForce3D),
}

// TODO(XXX): remove Clone trait
#[derive(Copy, Clone)]
pub struct DistanceConstraint3D
{
  pub body_a: Entity,
  pub body_b: Entity,
  // position relative to the body's center of mass (in body space)
  pub point_a: Vec3,
  pub point_b: Vec3,
  pub d: f32,
  // min_distance: f32,
  // max_distance: f32,
  pub stiffness: f32, /* ks */
  pub damping: f32, /* kd */
}

#[derive(Copy, Clone)]
pub struct HingeConstraint3D
{
  body_a: Entity,
  body_b: Entity,
  pivot: Vec3,
  axis: Vec3,
  angle_limits: (f32, f32),
}

#[derive(Component, Copy, Clone)]
pub enum Constraint3D
{
  Distance(DistanceConstraint3D),
  Hinge(HingeConstraint3D),
}

#[derive(PartialEq)]
pub enum Rigidbody3DKind
{
  Static,
  Sleep,
  Dynamic,
}

pub enum Rigidbody3DShape
{
  Sphere { r: f32 },
  Cuboid { x: f32, y: f32, z: f32 }, /* dimension */
}

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

#[derive(Component)]
pub struct Rigidbody3D
{
  pub kind: Rigidbody3DKind,
  pub shape: Rigidbody3DShape,

  // constant quantities 
  pub mass: f32,
  pub Ibody: Mat3,
  pub Ibodyinv: Mat3,

  // state variables
  pub x: Vec3, /* position */
  pub q: Quat, /* rotation */
  pub P: Vec3, /* linear momentum */
  pub L: Vec3, /* angular momentum */

  // derived quantities (auxiliary variables)
  pub R: Mat3, /* rotation matrix, derived from q */
  pub Iinv: Mat3, /* inverse of inertia(world) */
  pub v: Vec3, /* linear velocity */
  pub omega: Vec3, /* angular velocity ω(t) */

  // computed quantities(artifacts)
  pub force: Vec3,
  pub torque: Vec3,
}

impl Rigidbody3D
{
  pub fn cuboid(position: Vec3, mass: f32, dim: Vec3) -> Self
  {
      Self
      {
        kind: Rigidbody3DKind::Dynamic,
        shape: Rigidbody3DShape::Cuboid{x: dim.x, y: dim.y, z: dim.z},
        mass,
        Ibody: inertia_from_cuboid(mass, dim),
        Ibodyinv: inertiainv_from_cuboid(mass, dim),
        
        x: position,
        q: Quat::IDENTITY,
        P: Vec3::ZERO,
        L: Vec3::ZERO,

        R: Mat3::ZERO,
        Iinv: Mat3::ZERO,
        v: Vec3::ZERO,
        omega: Vec3::ZERO,

        force: Vec3::ZERO,
        torque: Vec3::ZERO,
      }
  }
}

impl Rigidbody3D
{
  pub fn reset_energy(&mut self)
  {
    self.v = Vec3::ZERO;
    self.omega = Vec3::ZERO;
    self.P = Vec3::ZERO;
    self.L = Vec3::ZERO;
  }
}

#[derive(Resource)]
pub struct RigidbodySystem3D
{
  // bodies: Vec<Rigidbody3D>,

  // global forces
  pub gravity: Gravity3D,
  pub visous_drag: VisousDrag3D,

  // reset every frame
  pub forces: Vec<Force3D>,
  pub constraints: Vec<Constraint3D>,
}

impl RigidbodySystem3D
{
  pub fn reset(mut system: ResMut<RigidbodySystem3D>)
  {
    system.forces.clear();
    system.constraints.clear();
  }

  pub fn step(fixed_time: Res<Time<Fixed>>, system: Res<RigidbodySystem3D>, mut bodies: Query<(Entity, &mut Rigidbody3D, &mut Transform)>)
  {
    let delta_secs = fixed_time.delta_secs();

    // let mut entities: Vec<Entity> = vec![];
    // collect state
    // x, q, P, L [1, 13]
    let mut state: Vec<f32> = Vec::new();
    for (idx, (entity, rb, mut transform)) in bodies.iter_mut().enumerate()
    {
      // entities.push(entity);
      let local: [f32; 13] = [
        // x
        rb.x.x,
        rb.x.y,
        rb.x.z,

        // q
        rb.q.x,
        rb.q.y,
        rb.q.z,
        rb.q.w,

        // P
        rb.P.x,
        rb.P.y,
        rb.P.z,

        // L
        rb.L.x,
        rb.L.y,
        rb.L.z,

      ];
      state.extend_from_slice(&local[..]);
    }

    let state_next = ode_euler(&state, delta_secs, |X: &[f32], delta_secs: f32| {RigidbodySystem3D::dxdt(X, delta_secs, &mut bodies, &system)} );;
    RigidbodySystem3D::state_set(&state_next, &mut bodies);
  }

  fn state_set(X: &[f32], bodies: &mut Query<(Entity, &mut Rigidbody3D, &mut Transform)>) -> ()
  {
    const stride: usize = 13;
    for (i, (entity, mut rb, mut transform)) in bodies.iter_mut().enumerate()
    {
      if rb.kind == Rigidbody3DKind::Dynamic
      {
        let src = &X[i*stride..];
        // x
        rb.x.x = src[0];
        rb.x.y = src[1];
        rb.x.z = src[2];
        // q
        rb.q.x = src[3];
        rb.q.y = src[4];
        rb.q.z = src[5];
        rb.q.w = src[6];
        rb.q = rb.q.normalize();
        // P
        rb.P.x = src[7];
        rb.P.y = src[8];
        rb.P.z = src[9];
        // L
        rb.L.x = src[10];
        rb.L.y = src[11];
        rb.L.z = src[12];

        // compute auxiliary variables ...

        // R (rotation matrix)
        rb.R = Mat3::from_quat(rb.q);

        // Iinv
        rb.Iinv = rb.R * (rb.Ibodyinv * rb.R.transpose());

        // v(t)
        rb.v = rb.P * (1.0/rb.mass);

        // ω(t) = I−1(t)L(t)
        rb.omega = rb.Iinv * rb.L;

        // copy translation and rotation to transform
        transform.translation = rb.x;
        transform.rotation = rb.q;
      }
    }
  }

  fn dxdt(X: &[f32], delta_secs: f32, query: &mut Query<(Entity, &mut Rigidbody3D, &mut Transform)>, system: &RigidbodySystem3D) -> Vec<f32>
  {
    // TODO(XXX): some ode will generate X multiple times, some won't, we should pass that information

    let mut body_count: usize = 0;
    // update state
    RigidbodySystem3D::state_set(X, query);

    // zero out the force accumulators
    for (idx, (entity, mut rb, mut transform)) in query.iter_mut().enumerate()
    {
      rb.force.x = 0.0;
      rb.force.y = 0.0;
      rb.force.z = 0.0;

      rb.torque.x = 0.0;
      rb.torque.y = 0.0;
      rb.torque.z = 0.0;

      body_count += 1;
    }

    /////////////////////////////////////////////////////////////////////////////////////
    //~ compute forces (applied force/torque + constrained force)

    /////////////////////////////////////////////////////////////////////////////////////
    //- global forces (uniform force field)

    for (idx, (entity, mut rb, mut transform)) in query.iter_mut().enumerate()
    {
      if rb.kind == Rigidbody3DKind::Dynamic
      {
        // gravity
        let mut f = system.gravity.dir * (system.gravity.g*rb.mass);
        rb.force += f;
        
        // global visous_drag
        f = rb.v * (system.visous_drag.kd*-1.0);
        rb.force += f;
        f = rb.omega * (system.visous_drag.kd*-1.0);
        rb.torque += f;
      }
    }

    /////////////////////////////////////////////////////////////////////////////////////
    //- unconstraint force/torque (contact position dependent)

    for i in system.forces.iter() 
    {
      match(i)
      {
        Force3D::Constant(f) => {
          if let Ok((entity, mut rb, _)) = query.get_mut(f.target)
          {
            if rb.kind == Rigidbody3DKind::Dynamic
            {
              let F: Vec3 = f.dir * f.strength;

              // linear
              rb.force += F;

              // torque
              let torque: Vec3 = f.contact.cross(F);
              rb.torque += torque;
            }
          }
        }
        Force3D::VisousDrag(f) => {
          if let Some(target) = f.target
          {
            if let Ok((entity, mut rb, _)) = query.get_mut(target)
            {
              if rb.kind == Rigidbody3DKind::Dynamic
              {
                let mut F: Vec3 = rb.v * (f.kd*-1.0);

                // linear
                rb.force += F;

                // apply torque
                F = rb.omega * (f.kd*-1.0);
                rb.torque += F;
              }
            }
          }
        }
        _ => {}
      }
    }

    /////////////////////////////////////////////////////////////////////////////////////
    //- constrant force/torque (contact position dependent)

    if system.constraints.len() > 0
    {
      /////////////////////////////////////////////////////////////////////////////////
      // Collect components

      let m: usize = system.constraints.len();
      let n: usize = body_count;
      let N: usize = n*3; /* only position */
      let mut idx_map: HashMap<Entity, usize> = HashMap::new(); /* entity index to state X */

      // force vector [3n, 1]
      let mut Q: Vec<f32> = vec![0.0; N];
      // force position (relatvie to the body's mass center) vector [n, 1]
      let mut QP: Vec<Vec3> = vec![Vec3::ZERO; n];

      // velocity vector [3n, 1]
      // Unless phase space, q only contains positions, qdot only contains velocities
      let mut qdot: Vec<f32> = vec![0.0; N];

      // mass matrix [3n, 3n]
      // TODO(XXX): if we were going to use cg, mass matrix could be stored as vector for easier computation
      // W is just the reciprocal of M
      let mut W: Mat = Mat::from_dim(N, N);

      // loop through all bodies to collect above values
      for (i, (entity, mut rb, mut transform)) in query.iter_mut().enumerate()
      {
        let mut src = &mut Q[i*3..];
        // Q
        src[0] = rb.force.x;
        src[1] = rb.force.y;
        src[2] = rb.force.z;

        src = &mut qdot[i*3..];
        // qdot
        src[0] = rb.v.x;
        src[1] = rb.v.y;
        src[2] = rb.v.z;

        // W
        let ii = i*3;
        W[ii+0][ii+0] = 1.0 / rb.mass;
        W[ii+1][ii+1] = 1.0 / rb.mass;
        W[ii+2][ii+2] = 1.0 / rb.mass;

        idx_map.insert(entity, i);
      }

      // jacobian of C(q) [m, 3n]
      let mut J: Mat = Mat::from_dim(m, N);
      let mut Jdot: Mat = Mat::from_dim(m, N);
      let mut C_q: Vec<f32> = vec![0.0; m]; /* [1,m] */

      // collect J & Jt & QP
      for (c_idx, c) in system.constraints.iter().enumerate()
      {
        match(c)
        {
          Constraint3D::Distance(c) => {
            if let Ok([(entity_a, rb_a, _), (entity_b, rb_b, _)]) = query.get_many([c.body_a, c.body_b])
            {
              let mut a_idx = idx_map.get(&entity_a).unwrap();
              let mut b_idx = idx_map.get(&entity_b).unwrap();

              // relative position after rotation
              let pos_rel_a = rb_a.q.mul_vec3(c.point_a);
              let pos_rel_b = rb_b.q.mul_vec3(c.point_b);
              // world space
              let pos_a = pos_rel_a + rb_a.x;
              let pos_b = pos_rel_b + rb_b.x;

              // J & Jdot for a
              let mut i = c_idx;
              let mut j = a_idx*3;
              for k in 0..3
              {
                let jj = j+k;
                J[i][jj] = pos_a[k] - pos_b[k];
                Jdot[i][jj] = rb_a.v[k] - rb_b.v[k];
              }

              // J & Jdot for b
              j = b_idx*3;
              for k in 0..3
              {
                let jj = j+k;
                J[i][jj] = -(pos_a[k] - pos_b[k]);
                Jdot[i][jj] = -(rb_a.v[k] - rb_b.v[k]);
              }

              // C(q)
              let x0 = pos_a.x - pos_b.x;
              let y0 = pos_a.y - pos_b.y;
              let z0 = pos_a.z - pos_b.z;
              C_q[c_idx] = 0.5 * (x0*x0 + y0*y0 + z0*z0 - c.d*c.d);

              // QP
              QP[*a_idx] = pos_rel_a;
              QP[*b_idx] = pos_rel_b;
            }
            else
            {
              assert!(false);
            }
          },
          _ => todo!(),
        }
      }

      // tranpose of J
      let Jt: Mat = J.tranpose();

      // Cdot(q) [1, m]
      let Cdot_q = &J * &qdot;
      debug_assert_eq!(m, Cdot_q.len());

      ///////////////////////////////////////////////////////////////////////////////////
      //~ Compute

      //- solve larange multipliers
      // [m, 3n] [3n, 1] => [m, 1]
      let Jdot_qdot: Vec<f32> = &Jdot * &qdot;
      debug_assert!(Jdot_qdot.len() == m);
      // [3n, 3n] * [3n, 1] => [3n, 1]
      let WQ: Vec<f32> = &W * &Q;
      debug_assert!(WQ.len() == N);
      // [m, 3n] * [3n, 1] => [m, 1]
      let JWQ: Vec<f32> = &J * &WQ;
      debug_assert!(JWQ.len() == m);
      // Jdot_qdot + JWQ => [m, 1]
      let mut b: Vec<f32> = Jdot_qdot.iter().zip(JWQ.iter()).map(|(x,y)| x+y).collect();
      // ks * C
      b = b.into_iter().zip(C_q.iter()).map(|(x, y)| x+y*1000.0).collect();
      b = b.into_iter().zip(Cdot_q.iter()).map(|(x, y)| x+y*64.0).collect();
      b = b.into_iter().map(|x| -x).collect();


      //- solve the linear system
      // [m, 1]
      let mut A: Mat = &J * &W;
      A = &A * &Jt;
      let mut lambda = b.clone();
      debug_assert_eq!(lambda.len(), m);
      debug_assert_eq!(A.i_dim, A.j_dim);
      gaussj(&mut A, &mut lambda);

      //- compute the constraint force
      let Q_c = &Jt * &lambda;
      debug_assert_eq!(Q_c.len(), N);

      //- add constraint force
      for (i, (entity, mut rb, mut transform)) in query.iter_mut().enumerate()
      {
        if rb.kind == Rigidbody3DKind::Dynamic
        {
          let F = Vec3::new(Q_c[i*3+0], Q_c[i*3+1], Q_c[i*3+2]);
          // linear
          rb.force += F;

          // torque
          let contact = QP[i];
          rb.torque += contact.cross(F);
        }
      }
    }

    /////////////////////////////////////////////////////////////////////////////////////
    //~ gather result

    const stride: usize = 13;
    let mut ret: Vec<f32> = vec![0.0; X.len()];
    for (i, (entity, mut rb, mut transform)) in query.iter_mut().enumerate()
    {
      let src = &mut ret[i*stride..];

      // v(t) linear velocity
      src[0] = rb.v.x;
      src[1] = rb.v.y;
      src[2] = rb.v.z;

      // qdot
      let qdot = Quat::from_xyzw(rb.omega.x, rb.omega.y, rb.omega.z, 0.0) * rb.q;
      src[3] = qdot.x;
      src[4] = qdot.y;
      src[5] = qdot.z;
      src[6] = qdot.w;

      // Pdot(t) = F(t)
      src[7] = rb.force.x;
      src[8] = rb.force.y;
      src[9] = rb.force.z;

      // Ldot(t) = τ(t)
      src[10] = rb.torque.x;
      src[11] = rb.torque.y;
      src[12] = rb.torque.z;
    }

    return ret;
  }
}

impl Default for RigidbodySystem3D
{
  fn default() -> Self
  {
    Self
    {
      gravity: Gravity3D { g: 9.81, dir: Vec3::new(0.0, -1.0, 0.0) },
      // gravity: Gravity3D { g: 0.0, dir: Vec3::new(0.0, -1.0, 0.0) },
      visous_drag: VisousDrag3D { kd: 0.6, target: None},
      forces: Vec::new(),
      constraints: Vec::new(),
    }
  }
}

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin
{
  fn build(&self, app: &mut App)
  {
    app
      .init_resource::<RigidbodySystem3D>()
      .add_systems(FixedUpdate, (RigidbodySystem3D::reset, RigidbodySystem3D::step).chain());
  }
}

fn ode_euler<F: FnMut(&[f32], f32) -> Vec<f32>>(X: &[f32], delta_secs: f32, mut dxdt: F) -> Vec<f32>
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

fn gaussj(a: &mut Mat, b: &mut Vec<f32>) -> ()
{
  let n = a.i_dim;
  assert_eq!(a.i_dim, a.j_dim); /* expecting identity matrix */

  let mut icol: usize = 0;
  let mut irow: usize = 0;

  let mut big: f32 = 0.0;
  let mut dum: f32 = 0.0;
  let mut pivinv: f32 = 0.0;
  let mut temp: f32 = 0.0;

  let mut indxc: Vec<i32> = vec![0; n];
  let mut indxr: Vec<i32> = vec![0; n];
  let mut ipiv: Vec<i32> = vec![0; n];

  for i in 0..n
  {
    big = 0.0;
    for j in 0..n
    {
      if ipiv[j] != 1
      {
        for k in 0..n
        {
          if(ipiv[k] == 0)
          {
            if a[j][k].abs() >= big
            {
              big = a[j][k].abs();
              irow = j;
              icol = k;
            }
          }
        }
      }
    }
    ipiv[icol] += 1;

    if(irow != icol)
    {
      for l in 0..n
      {
        a.swap((irow, l), (icol, l));
      }
      b.swap(irow, icol);
    }
    indxr[i] = irow as i32;
    indxc[i] = icol as i32;
    if(a[icol][icol] == 0.0)
    {
      // singular matrix ?
      assert!(false);
    }
    pivinv = 1.0/a[icol][icol];
    a[icol][icol] = 1.0;
    for l in 0..n
    {
      a[icol][l] *= pivinv;
    }
    b[icol] *= pivinv;
    for ll in 0..n
    {
      if(ll != icol)
      {
        dum = a[ll][icol];
        a[ll][icol] = 0.0;
        for l in 0..n
        {
          a[ll][l] -= a[icol][l]*dum;
        }
        b[ll] -= b[icol]*dum;
      }
    }
  }

  // for(l = n-1; l >= 0; l--)
  for l in (0..n).rev()
  {
    if(indxr[l] != indxc[l])
    {
      for k in 0..n
      {
        a.swap((k, indxr[l] as usize), (k, indxc[l] as usize));
      }
    }
  }
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
  fn test_gaussj()
  {
    let mut a = Mat::from_dim(3,3);
    a[0][0] = 2.0;
    a[1][1] = 2.0;
    a[2][2] = 2.0;
    let mut b = vec![1.0; 3];
    gaussj(&mut a, &mut b);
    for (i, v) in vec![0.5, 0.5, 0.5].into_iter().enumerate()
    {
      assert_f32_equal(b[i], v, 0.001);
    }

    let mut a = Mat::from_dim(3,3);
    a[0][0] = 1.0;
    a[0][1] = 3.0;
    a[0][2] = 2.0;
    a[1][1] = 4.0;
    a[2][0] = 5.0;
    a[2][2] = 6.0;
    let mut b = vec![11.0, 4.0, 28.0];
    gaussj(&mut a, &mut b);
    for (i, v) in vec![2.0, 1.0, 3.0].into_iter().enumerate()
    {
      assert_f32_equal(b[i], v, 0.001);
    }

    let mut a = Mat::from_dim(3,3);
    a[0][0] = 1.0;
    a[0][2] = 5.0;
    a[1][0] = 3.0;
    a[1][1] = 4.0;
    a[2][0] = 2.0;
    a[2][2] = 6.0;
    let mut b = vec![17.0, 10.0, 22.0];
    gaussj(&mut a, &mut b);
    for (i, v) in vec![2.0, 1.0, 3.0].into_iter().enumerate()
    {
      assert_f32_equal(b[i], v, 0.001);
    }
  }

  #[test]
  fn test_Mat()
  {
    // test swap
    let mut a = Mat::from_dim(3,3);
    a[0][0] = 1.0;
    a[0][1] = 3.0;
    a[0][2] = 2.0;
    a[1][1] = 4.0;
    a[2][0] = 5.0;
    a[2][2] = 6.0;
    a.swap((0,0), (1,1));
    assert_eq!(a[0][0], 4.0);
    assert_eq!(a[1][1], 1.0);

    // test Mat*Mat
    let mut a = Mat::from_dim(2,3);
    a[0] = vec![1.0, 2.0, 3.0];
    a[1] = vec![4.0, 5.0, 6.0];

    let mut b = Mat::from_dim(3,2);
    b[0] = vec![7.0, 8.0];
    b[1] = vec![9.0, 10.0];
    b[2] = vec![11.0, 12.0];

    let c = &a*&b;
    assert_eq!(c[0][0], 58.0);
    assert_eq!(c[0][1], 64.0);
    assert_eq!(c[1][0], 139.0);
    assert_eq!(c[1][1], 154.0);

    // test eq
    let mut a = Mat::from_dim(2,3);
    a[0] = vec![1.0, 2.0, 3.0];
    a[1] = vec![4.0, 5.0, 6.0];
    let mut b = Mat::from_dim(2,3);
    b[0] = vec![1.0, 2.0, 3.0];
    b[1] = vec![4.0, 5.0, 6.0];
    assert!(a == b);
    b[0][0] = 0.0;
    assert!(a != b);

    // test transpose
    let mut a = Mat::from_dim(2,3);
    a[0] = vec![1.0, 2.0, 3.0];
    a[1] = vec![4.0, 5.0, 6.0];
    let mut b = Mat::from_dim(3,2);
    b[0] = vec![1.0, 4.0];
    b[1] = vec![2.0, 5.0];
    b[2] = vec![3.0, 6.0];
    assert!(a.tranpose() == b);

    // test Mat * Vec
    let mut a = Mat::from_dim(3,3);
    a[0][0] = 1.0;
    a[0][1] = 3.0;
    a[0][2] = 2.0;
    a[1][1] = 4.0;
    a[2][0] = 5.0;
    a[2][2] = 6.0;
    let mut b = vec![2.0, 1.0, 3.0];
    let mut c = &a * &b;
    gaussj(&mut a, &mut b);
    for (i, v) in vec![11.0, 4.0, 28.0].into_iter().enumerate()
    {
      assert_f32_equal(v, c[i], 0.001);
    }
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
