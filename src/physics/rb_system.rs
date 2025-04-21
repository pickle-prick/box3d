use bevy::{prelude::*};
use std::collections::HashMap;
use std::mem::swap;
use super::constraint::*;
use super::ode::*;
use super::force::*;
use super::math::*;
use super::helper::*;

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

    // let state_next = ode_euler(&state, delta_secs, |X: &[f32], delta_secs: f32| {RigidbodySystem3D::dxdt(X, delta_secs, &mut bodies, &system)} );;
    // let state_next = ode_rk2(&state, delta_secs, |X: &[f32], delta_secs: f32| {RigidbodySystem3D::dxdt(X, delta_secs, &mut bodies, &system)} );;
    let state_next = ode_rk4(&state, delta_secs, |X: &[f32], delta_secs: f32| {RigidbodySystem3D::dxdt(X, delta_secs, &mut bodies, &system)} );;
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
      // ks and kd
      let mut Ks: Vec<f32> = vec![0.0; m]; /* [1, m]] */
      let mut Kd: Vec<f32> = vec![0.0; m]; /* [1, m]] */

      // collect J & Jt & QP
      for (c_idx, c) in system.constraints.iter().enumerate()
      {
        match(c)
        {
          Constraint3D::Distance(c) => {
            Ks[c_idx] = c.stiffness;
            Kd[c_idx] = c.damping;

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
      // + ks * C
      b = b.into_iter().zip(C_q.iter()).zip(Ks.iter()).map(|((b, cq), ks)| b + cq*ks).collect();
      // + kd * Cdot
      b = b.into_iter().zip(Cdot_q.iter()).zip(Kd.iter()).map(|((b, cdq), ks)| b + cdq*ks).collect();
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
