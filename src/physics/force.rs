use bevy::{prelude::*};

pub struct Gravity3D
{
  pub g: f32,
  pub dir: Vec3,
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
