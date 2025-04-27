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
        // TODO(XXX): not sure if this is right
        f = rb.omega * (system.visous_drag.kd*-1.0);
        rb.torque += f;
      }
    }

    /////////////////////////////////////////////////////////////////////////////////////
    //- unconstraint(F_ext) force/torque (contact position dependent)

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
    //- constrant(F_c) force/torque (contact position dependent)

    if system.constraints.len() > 0
    {
      /////////////////////////////////////////////////////////////////////////////////
      // Collect components

      let m: usize = system.constraints.len(); /* num of constraint */
      let n: usize = body_count; /* number of body */
      let N: usize = n*6; /* position and rotation(euler) */
      let mut idx_map: HashMap<Entity, usize> = HashMap::new(); /* entity index to state X */

      // F_ext force&torque vector [N, 1]
      let mut F_ext: Vec<f32> = vec![0.0; N];
      // force position (relatvie to the body's mass center) vector [n, 1]
      let mut FP: Vec<Vec3> = vec![Vec3::ZERO; n];

      // velocity(linear+angular) vector [N, 1]
      let mut v: Vec<f32> = vec![0.0; N];

      // mass matrix [N, N]
      // W is just the reciprocal of M (mass+Iinv)
      let mut W: Mat = Mat::from_dim(N, N);

      // loop through all bodies to collect above values
      for (i, (entity, mut rb, mut transform)) in query.iter_mut().enumerate()
      {
        let mut src = &mut F_ext[i*6..];
        // F_ext
        src[0] = rb.force.x;
        src[1] = rb.force.y;
        src[2] = rb.force.z;
        src[3] = rb.torque.x;
        src[4] = rb.torque.y;
        src[5] = rb.torque.z;

        src = &mut v[i*6..];
        // v
        src[0] = rb.v.x;
        src[1] = rb.v.y;
        src[2] = rb.v.z;
        src[3] = rb.omega.x;
        src[4] = rb.omega.y;
        src[5] = rb.omega.z;

        // W
        let mut ii = i*6 + 0;
        // mass
        W[ii+0][ii+0] = 1.0 / rb.mass;
        W[ii+1][ii+1] = 1.0 / rb.mass;
        W[ii+2][ii+2] = 1.0 / rb.mass;
        // Iinv
        // TODO(XXX): is all Iinv matrix symmetric
        ii += 3;
        for iii in 0..3
        {
          for jjj in 0..3
          {
            W[iii+ii][jjj+ii] = rb.Iinv.row(iii)[jjj];
          }
        }

        // cache body index
        idx_map.insert(entity, i);
      }

      // jacobian of C(q) [m, N]
      let mut J_trans: Mat = Mat::from_dim(m, N);
      let mut J_rot: Mat = Mat::from_dim(m, N);
      let mut C_trans: Vec<f32>  = vec![0.; m];
      let mut C_rot: Vec<f32>  = vec![0.; m];

      // collect J & Jt & FP
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
              let ra = rb_a.q.mul_vec3(c.point_a);
              let rb = rb_b.q.mul_vec3(c.point_b);
              // world space
              let pa = ra + rb_a.x;
              let pb = rb + rb_b.x;
              let d = (pa-pb).length();
              let mut d_target = d;
              if d < c.min_distance
              {
                d_target = c.min_distance;
              }
              if d > c.max_distance
              {
                d_target = c.max_distance;
              }

              let u = pa-pb;
              let n = u.normalize();
              let n: Vec<f32> = vec![n.x, n.y, n.z];
              let Rsa = Mat::skew_symmetric_from_vec3(&ra);
              let Rsb = Mat::skew_symmetric_from_vec3(&rb);

              let mut J0: Vec<f32> = vec![0.; 3];
              let mut J1: Vec<f32> = vec![0.; 3];
              let mut J2: Vec<f32> = vec![0.; 3];
              let mut J3: Vec<f32> = vec![0.; 3];

              if d != d_target
              {
                J0 = n.clone();
                J1 = (&Rsa * &n);
                J2 = negate_vf32(&J0);
                J3 = negate_vf32(&(&Rsb * &n));
              }

              // J for a
              let mut i = c_idx;
              let mut j = a_idx*6;
              J_trans[i][j+0] = J0[0];
              J_trans[i][j+1] = J0[1];
              J_trans[i][j+2] = J0[2];
              J_trans[i][j+3] = J1[0];
              J_trans[i][j+4] = J1[1];
              J_trans[i][j+5] = J1[2];

              // J for b
              j = b_idx*6;
              J_trans[i][j+0] = J2[0];
              J_trans[i][j+1] = J2[1];
              J_trans[i][j+2] = J2[2];
              J_trans[i][j+3] = J3[0];
              J_trans[i][j+4] = J3[1];
              J_trans[i][j+5] = J3[2];

              // C(q)
              C_trans[c_idx] = d - d_target;
              C_rot[c_idx] = 0.;

              // FP
              FP[*a_idx] = ra;
              FP[*b_idx] = rb;
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
      let Jt_trans: Mat = J_trans.tranpose();
      let Jt_rot: Mat = J_rot.tranpose();

      ///////////////////////////////////////////////////////////////////////////////////
      //~ Compute

      // K
      let mut K_trans = &(&J_trans * &W) * &Jt_trans;
      let mut K_rot = &(&J_rot * &W) * &Jt_rot;
      debug_assert_eq!(K_trans.i_dim, K_trans.j_dim);
      debug_assert_eq!(K_rot.i_dim, K_rot.j_dim);

      // bias velocity vector
      let b_trans = scale_vf32(&C_trans, 0.2/delta_secs);
      let b_rot = scale_vf32(&C_rot, 0.2/delta_secs);

      // vdot
      let vdot = add_vf32(&v, &(&W * &scale_vf32(&F_ext,delta_secs)));

      // B
      let B_trans = negate_vf32(&add_vf32(&(&J_trans*&vdot), &b_trans));
      let B_rot = negate_vf32(&add_vf32(&(&J_rot*&vdot), &b_rot));

      //- solve the linear system

      //- solve larange multipliers
      let mut lambda_trans = B_trans.clone();
      let mut lambda_rot = B_rot.clone();
      gaussj(&mut K_trans, &mut lambda_trans);
      gaussj(&mut K_rot, &mut lambda_rot);

      //- compute the constraint force
      let Fc_trans = scale_vf32(&(&Jt_trans*&lambda_trans), 1./delta_secs);
      let Fc_rot = scale_vf32(&(&Jt_rot*&lambda_rot), 1./delta_secs);
      debug_assert_eq!(Fc_trans.len(), N);

      //- add constraint force
      for (i, (entity, mut rb, mut transform)) in query.iter_mut().enumerate()
      {
        let ii = i*6;
        if rb.kind == Rigidbody3DKind::Dynamic
        {
          // linear
          rb.force.x += Fc_trans[ii+0];
          rb.force.y += Fc_trans[ii+1];
          rb.force.z += Fc_trans[ii+2];

          rb.force.x += Fc_rot[ii+0];
          rb.force.y += Fc_rot[ii+1];
          rb.force.z += Fc_rot[ii+2];

          // torque
          rb.torque.x += Fc_trans[ii+3];
          rb.torque.y += Fc_trans[ii+4];
          rb.torque.z += Fc_trans[ii+5];

          rb.torque.x += Fc_rot[ii+3];
          rb.torque.y += Fc_rot[ii+4];
          rb.torque.z += Fc_rot[ii+5];
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
      // gravity: Gravity3D { g: 0., dir: Vec3::new(0.0, -1.0, 0.0) },
      visous_drag: VisousDrag3D { kd: 0.6, target: None},
      forces: Vec::new(),
      constraints: Vec::new(),
    }
  }
}
