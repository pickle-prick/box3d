use bevy::{input::mouse::{AccumulatedMouseMotion, MouseScrollUnit, MouseWheel}, prelude::*};
use std::{f32::consts::FRAC_PI_2, ops::Range};

#[derive(Debug, Resource)]
pub struct CameraSettings
{
  pub orbit_distance: f32,
  pub pitch_speed: f32,
  // Clamp pitch to this range
  pub pitch_range: Range<f32>,
  pub roll_speed: f32,
  pub yaw_speed: f32,
}

impl Default for CameraSettings
{
  fn default() -> Self
  {
    // limiting pitch stops some unexpected rotation past 90 degree up or down
    let pitch_limit = FRAC_PI_2 - 0.01;
    Self
    {
      orbit_distance: 20.0,
      // pitch_speed: 0.003,
      pitch_speed: 0.001,
      pitch_range: -pitch_limit..pitch_limit,
      roll_speed: 1.0,
      // yaw_speed: 0.004,
      yaw_speed: 0.002,
    }
  }
}

fn camera_update(
  mut camera: Single<&mut Transform, With<Camera>>,
  camera_settings: Res<CameraSettings>,
  mouse_buttons: Res<ButtonInput<MouseButton>>,
  mouse_motion: Res<AccumulatedMouseMotion>,
  mut evr_scroll: EventReader<MouseWheel>,
  time: Res<Time>,
)
{
  let delta = mouse_motion.delta;
  let mut delta_yaw = 0.0;
  let mut delta_pitch = 0.0;
  if mouse_buttons.pressed(MouseButton::Right) && (!mouse_buttons.just_pressed(MouseButton::Right))
  {
    delta_yaw = delta.x * camera_settings.yaw_speed;
    delta_pitch = delta.y * camera_settings.pitch_speed;
  }

  // TODO: euler won't work well, use quaternion instead
  let (yaw, pitch, roll) = camera.rotation.to_euler(EulerRot::YXZ);
  let pitch = (pitch-delta_pitch).clamp(
    camera_settings.pitch_range.start,
    camera_settings.pitch_range.end,
  );
  let yaw = yaw-delta_yaw;
  camera.rotation = Quat::from_euler(EulerRot::YXZ, yaw, pitch, roll);

  // translation
  // TODO(XXX): vertical and horizontal
  let mut translation_delta: Vec3 = Vec3::new(0.0,0.0,0.0);
  for ev in evr_scroll.read()
  {
    match ev.unit
    {
      MouseScrollUnit::Line =>
      {
        // TODO(XXX): move this scale into camera settings
        let scale = ev.y;
        translation_delta += scale*camera.forward();
      }
      MouseScrollUnit::Pixel => {todo!()}
    }
  }
  if mouse_buttons.pressed(MouseButton::Middle)
  {
    // TODO(XXX): move this scale into camera settings
    let scale = 0.003;
    translation_delta += (scale*delta.x) * camera.left();
    translation_delta += (scale*delta.y) * camera.up();
  }
  camera.translation += translation_delta;
}

fn instructions(mut commands: Commands)
{
  commands.spawn((
      Name::new("Instructions"),
      Text::new("Mouse up"),
      Node
      {
        position_type: PositionType::Absolute,
        top: Val::Px(32.),
        left: Val::Px(12.),
        ..default()
      }
  ));
}

pub struct CameraPlugin;
impl Plugin for CameraPlugin
{
  fn build(&self, app: &mut App)
  {
    app.init_resource::<CameraSettings>().add_systems(Update, camera_update).add_systems(Update, instructions);
  }
}
