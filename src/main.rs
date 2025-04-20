#![allow(non_snake_case)]
#![allow(warnings, unused)]
#![allow(dead_code)]

use bevy::{color::palettes::css::*, color::palettes::tailwind::*, picking::pointer::PointerInteraction, prelude::*};
use std::f32::consts::PI;
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiContextPass};

mod camera;
use camera::CameraPlugin;
mod physics;
use physics::{PhysicsPlugin, Rigidbody3D, RigidbodySystem3D, Constraint3D, DistanceConstraint3D, Rigidbody3DKind, ConstantForce3D, Force3D, VisousDrag3D};

fn main()
{
  App::new()
    // .init_gizmo_group::<MyRoundGizmos>()
    .add_plugins((DefaultPlugins, MeshPickingPlugin))
    .add_plugins(PhysicsPlugin)
    .add_plugins(CameraPlugin)
    .add_plugins(EguiPlugin { enable_multipass_for_primary_context: true })
    .init_resource::<World>()
    .add_systems(Startup, setup)
    .add_systems(FixedUpdate, (collect_constraints, hit_with_lazer_eye).chain().before(RigidbodySystem3D::step).after(RigidbodySystem3D::reset))
    .add_systems(Update, draw_gizmos)
    .add_systems(EguiContextPass, inspector)
    .add_systems(Update, draw_lazer_eye)
    // systems run in parallel by default
    .run();
}

#[derive(Resource)]
struct World
{
  cube: Option<Entity>,
  anchor: Option<Entity>,
  // hot_key: Option<Entity>,
  // active_key: Option<Entity>,
}

impl Default for World
{
  fn default() -> Self
  {
    Self
    {
      cube: None,
      anchor: None,
      // hot_key: None,
      // active_key: None,
    }
  }
}

#[derive(Component)]
struct Hittable;

fn setup(mut commands: Commands,
         mut meshes: ResMut<Assets<Mesh>>,
         mut materials: ResMut<Assets<StandardMaterial>>,
         mut world: ResMut<World>)
{
  // circular base
  commands.spawn((
    Mesh3d(meshes.add(Circle::new(4.0))),
    MeshMaterial3d(materials.add(Color::WHITE)),
    Transform::from_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)),
  ));

  // anchor
  let anchor_pos = Vec3::new(0.0, 5.0, 0.0);
  let anchor_radius = 0.05;
  // TODO(XXX): we should use sphere inertia here
  let mut anchor_body = Rigidbody3D::cuboid(anchor_pos, 1000000.0, Vec3::splat(anchor_radius*2.0));
  anchor_body.kind = Rigidbody3DKind::Static;
  let anchor = commands.spawn((
    Mesh3d(meshes.add(Sphere::new(0.05))),
    MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
    Transform::from_xyz(anchor_pos.x, anchor_pos.y, anchor_pos.z),
    anchor_body,
  )).id();

  // cube
  let cube_pos = Vec3::new(0.0, 2.0, 0.0);
  let cube = commands.spawn((
    Mesh3d(meshes.add(Cuboid::new(1.0, 1.0, 1.0))),
    MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
    Transform::from_xyz(cube_pos.x, cube_pos.y, cube_pos.z),
    Rigidbody3D::cuboid(cube_pos, 1.0, Vec3::new(1.0,1.0,1.0)),
    Hittable,
  )).id();

  let d_constraint = Constraint3D::Distance(DistanceConstraint3D {
    body_a: cube,
    body_b: anchor,
    point_a: Vec3::new(0.0, 0.5, 0.0),
    point_b: Vec3::ZERO,
    d: 2.5,
    stiffness: 100.0,
    damping: 0.1,
  });

  commands.entity(cube).insert(d_constraint);

  // save to world
  world.cube = Some(cube);
  world.anchor = Some(anchor);

  // light
  commands.spawn((
    PointLight {
      shadows_enabled: true,
      ..default()
    },
    Transform::from_xyz(4.0, 8.0, 4.0),
  ));

  // camera
  commands.spawn((
    Camera3d::default(),
    Transform::from_xyz(-2.5, 4.5, 9.0).looking_at(Vec3::ZERO, Vec3::Y),
  ));

  commands.spawn((
    Text::new(
      "Press 'T' to toggle drawing gizmos on top of everything else in the scene\n\
          Press 'P' to toggle perspective for line gizmos\n\
          Hold 'Left' or 'Right' to change the line width of straight gizmos\n\
          Press 'J' or 'K' to cycle through line joins for straight or round gizmos",
    ),
    Node {
      position_type: PositionType::Absolute,
      top: Val::Px(12.0),
      left: Val::Px(12.0),
      ..default()
    },
  ));
}

// We can create our own gizmo config group!
#[derive(Default, Reflect, GizmoConfigGroup)]
struct MyRoundGizmos {}

fn draw_gizmos(mut gizmos: Gizmos,
               mut query: Query<(Entity, &mut Rigidbody3D, &mut Transform, Option<&mut Constraint3D>)>,
               time: Res<Time>,
               mut world: ResMut<World>)
{
  gizmos.grid(
    Quat::from_rotation_x(PI / 2.),
    UVec2::splat(20),
    Vec2::new(2., 2.),
    // Light gray
    LinearRgba::gray(0.65),
  );

  match (world.cube, world.anchor)
  {
    (Some(cube), Some(anchor)) => {
      if let Ok([(_, rb_a, t_a, c), (_, rb_b, t_b, _)]) = query.get_many([cube, anchor])
      {
        if let Some(Constraint3D::Distance(c)) = c
        {
          // draw the distance constraint line
          let ray_start = rb_a.q.mul_vec3(c.point_a) + t_a.translation;
          gizmos
            .arrow(ray_start, t_b.translation, ORANGE_RED)
            .with_double_end()
            .with_tip_length(0.1); 

          // draw the contact point
          // gizmos.sphere(ray_start, 0.05, GREEN);
          gizmos.circle(
            Isometry3d::new(ray_start, t_a.rotation.mul_quat(Quat::from_rotation_x(PI / 2.))),
            0.05, GREEN);
        }

        // draw velocity
        gizmos
          .arrow(t_a.translation, t_a.translation+rb_a.force, LIGHT_SALMON)
          .with_tip_length(0.1); 

        // gizmos.ray(
        //   Vec3::new(1.0, 0.0, 0.0),
        //   Vec3::new(-3., ops::sin(time.elapsed_secs() * 3.), 0.),
        //   BLUE,
        // );
      }
    },
    _ => todo!(),
  }

}

fn inspector(mut contexts: EguiContexts)
{
  egui::Window::new("Hello").show(contexts.ctx_mut(), |ui| {
    ui.label("world");
  });
}

fn collect_constraints(mut query: Query<(Entity, &mut Rigidbody3D, &mut Transform, &mut Constraint3D)>,
                       mut system: ResMut<RigidbodySystem3D>)
{
    for (idx, (entity, rb, mut transform, mut c)) in query.iter_mut().enumerate()
    {
      system.constraints.push(*c);
    }
}

fn draw_lazer_eye(pointers: Query<&PointerInteraction>,
                  mut camera: Single<&mut Transform, With<Camera>>,
                  mut hittables: Query<(&Transform, &mut Rigidbody3D), (With<Hittable>, Without<Camera>)>,
                  mut gizmos: Gizmos)
{
  let camera_transform = &mut *camera;
  for (point, normal) in pointers.iter()
                                 .filter_map(|interaction| interaction.get_nearest_hit())
                                 .filter_map(|(entity, hit)| {if hittables.contains(*entity) { hit.position.zip(hit.normal) } else { None }})
  {
    let ray_eye_to_p = (point-camera_transform.translation).normalize();
    gizmos.sphere(point, 0.05, YELLOW_300);
    // gizmos.arrow(camera_transform.translation, point, PINK_100).with_tip_length(0.1); 
    // TODO(XXX): only a big old arrow is shown, no line is drawn
    gizmos.arrow(point-(ray_eye_to_p*0.3), point, PINK_300).with_tip_length(0.3); 
  }
}

fn hit_with_lazer_eye(pointers: Query<&PointerInteraction>,
                  mut camera: Single<&mut Transform, With<Camera>>,
                  mut hittables: Query<(&Transform, &mut Rigidbody3D, &mut Constraint3D), (With<Hittable>, Without<Camera>)>,
                  // mut anchors: Query<(&Transform, &mut Rigidbody3D), (Without<Hittable>, Without<Camera>)>,
                  mut system: ResMut<RigidbodySystem3D>,
                  mouse_buttons: Res<ButtonInput<MouseButton>>,
                  mut gizmos: Gizmos)
{

  let camera_transform = &mut *camera;
  for (point, normal, entity) in pointers.iter()
                                         .filter_map(|interaction| interaction.get_nearest_hit())
                                         .filter_map(|(entity, hit)| hit.position.zip(hit.normal).map(|(point, normal)| (point, normal, *entity)))
  {
    if !hittables.contains(entity)
    {
      continue;
    }

    if let Ok((t_a, rb_a, mut c)) = hittables.get_mut(entity)
    {
      if mouse_buttons.just_pressed(MouseButton::Left)
      {
        let F = ConstantForce3D {
          dir: (point-camera_transform.translation).normalize(),
          strength: 100.0,
          // relative to the body position (in world space)
          contact: point-t_a.translation,
          target: entity,
        };
        system.forces.push(Force3D::Constant(F));
      }

      // change contact point
      // if mouse_buttons.just_pressed(MouseButton::Right)
      // {
      //   match &mut *c
      //   {
      //     Constraint3D::Distance(cc) => {
      //       let point = point - t_a.translation;
      //       let point_local = t_a.rotation.inverse().mul_vec3(point);
      //       cc.point_a = point_local;

      //       // TODO: reset distance
      //       // NOTE(k): we are expecting the anchor is fixed and static, so just use pint_b
      //       cc.d = cc

      //       // TODO: reset P and L? (not sure)
      //     },
      //     _ => todo!(),
      //   }
      // }
    }
  }
}
