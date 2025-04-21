use bevy::prelude::*;

pub mod force;
pub mod math;
pub mod ode;
pub mod constraint;
pub mod helper;
pub mod rb_system;
pub mod prelude;

use rb_system::RigidbodySystem3D;

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
