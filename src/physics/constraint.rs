use bevy::{prelude::*};

#[derive(Component, Copy, Clone)]
pub enum Constraint3D
{
  Distance(DistanceConstraint3D),
  Hinge(HingeConstraint3D),
}

#[derive(Copy, Clone)]
pub struct DistanceConstraint3D
{
  pub body_a: Entity,
  pub body_b: Entity,
  // position relative to the body's center of mass (in body space)
  pub point_a: Vec3,
  pub point_b: Vec3,
  pub d: f32,
  pub min_distance: f32,
  pub max_distance: f32,
  // pub stiffness: f32, /* ks */
  // pub damping: f32, /* kd */
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
