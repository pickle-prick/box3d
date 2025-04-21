#![allow(non_snake_case)]
#![allow(warnings, unused)]
#![allow(dead_code)]

use std::f32::consts::{PI, TAU};
use bevy_egui::{egui, EguiContexts, EguiPlugin, EguiContextPass};
use bevy::{color::palettes::css::*,
           color::palettes::tailwind::*,
           picking::pointer::PointerInteraction,
           prelude::*};

mod camera;
use camera::CameraPlugin;
mod physics;
use physics::{PhysicsPlugin, Rigidbody3D, RigidbodySystem3D, Constraint3D, DistanceConstraint3D, Rigidbody3DKind, ConstantForce3D, Force3D, VisousDrag3D, inertia_from_cuboid, inertiainv_from_cuboid};

fn main()
{
  App::new()
    // .init_gizmo_group::<MyRoundGizmos>()
    .add_plugins((DefaultPlugins, MeshPickingPlugin))
    .add_plugins(PhysicsPlugin)
    .add_plugins(CameraPlugin)
    .add_plugins(EguiPlugin { enable_multipass_for_primary_context: true })
    .init_resource::<World>()
    // systems run in parallel by default
    .add_systems(Startup, setup)
    .add_systems(FixedUpdate, (collect_constraints, hit_with_lazer_eye).chain().before(RigidbodySystem3D::step).after(RigidbodySystem3D::reset))
    .add_systems(Update, draw_gizmos)
    .add_systems(EguiContextPass, inspector)
    .add_systems(Update, draw_lazer_eye)
    .add_systems(Update, update_constraint_indicator)
    .add_systems(Update, update_world)
    .add_systems(Update, update_bodies)
    .run();
}

#[derive(Resource)]
struct World
{
  cube: Option<Entity>,
  anchor: Option<Entity>,
  // hot_key: Option<Entity>,
  active_key: Option<Entity>,
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
      active_key: None,
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
  let white_matl = materials.add(Color::WHITE);
  let ground_matl = materials.add(Color::from(GRAY_300));
  let hover_matl = materials.add(Color::from(CYAN_300));
  let pressed_matl = materials.add(Color::from(YELLOW_300));

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

  // constraint
  let d_constraint = commands.spawn((
    Constraint3D::Distance(
      DistanceConstraint3D {
        body_a: cube,
        body_b: anchor,
        point_a: Vec3::new(0.0, 0.5, 0.0),
        point_b: Vec3::ZERO,
        d: 2.5,
        stiffness: 100.0,
        damping: 0.1,
      },
    ),
    // TODO(XXX): make two sphere to indicate joint
    Mesh3d(meshes.add(Sphere::new(0.09))),
    MeshMaterial3d(white_matl.clone()),
    Transform::from_xyz(cube_pos.x, cube_pos.y, cube_pos.z),
  ))
  .observe(change_contact_on_drag)
  .observe(update_material_on::<Pointer<Over>>(hover_matl.clone()))
  .observe(update_material_on::<Pointer<Released>>(white_matl.clone()))
  .observe(update_material_on::<Pointer<Out>>(white_matl.clone()));


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

fn draw_gizmos(mut gizmos: Gizmos,
               mut bodies: Query<(Entity, &mut Rigidbody3D, &mut Transform)>,
               mut constraints: Query<(&mut Constraint3D)>,
               time: Res<Time>,
               mut world: ResMut<World>)
{
  gizmos.grid(
    Quat::from_rotation_x(TAU / 4.),
    UVec2::splat(20),
    Vec2::new(2., 2.),
    LinearRgba::gray(0.65),
  );

  for c in constraints.iter()
  {
    match c
    {
      Constraint3D::Distance(c) => 
      {
        let body_a = bodies.get(c.body_a);
        let body_b = bodies.get(c.body_b);

        if body_a.is_ok() && body_b.is_ok()
        {
          let (_, rb_a, t_a) = body_a.unwrap();
          let (_, rb_b, t_b) = body_b.unwrap();

          let ray_start = rb_a.q.mul_vec3(c.point_a) + t_a.translation;
          gizmos
            .arrow(ray_start, t_b.translation, ORANGE_RED)
            .with_double_end()
            .with_tip_length(0.1); 

          // draw the contact point
          gizmos.circle(
            Isometry3d::new(ray_start, t_a.rotation.mul_quat(Quat::from_rotation_x(PI / 2.))),
            0.05, GREEN);
        }
      },
      _ => todo!(),
    }
  }

  // draw body velocity
  for (_, rb, t) in bodies.iter()
  {
    if rb.kind == Rigidbody3DKind::Dynamic
    {
      gizmos
        .arrow(t.translation, t.translation+rb.force, LIGHT_SALMON)
        .with_tip_length(0.1); 
    }

    // gizmos.ray(
    //   Vec3::new(1.0, 0.0, 0.0),
    //   Vec3::new(-3., ops::sin(time.elapsed_secs() * 3.), 0.),
    //   BLUE,
    // );
  }
}

fn inspector(mut contexts: EguiContexts,
             mut bodies: Query<(Entity, &mut Rigidbody3D, &mut Transform)>,
             mut constraints: Query<(Entity, &mut Transform, &mut Constraint3D), Without<Rigidbody3D>>,
             mut world: ResMut<World>)
{
  egui::Window::new("Inspector")
    .default_size([300., 0.])
    .resizable([true, false])
    .show(contexts.ctx_mut(), |ui| {
      if let Some(cube) = world.cube
      {
        if let Ok((e, mut rb, mut t)) = bodies.get_mut(cube)
        {
          ui.heading(format!("Body {}-{}", e.index(), e.generation()));
          ui.separator();
          egui::Grid::new("main")
            .num_columns(2)
            .spacing([40., 6.0])
            .striped(true)
            .show(ui, |ui| {
              ui.label("translation");
              ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut t.translation.x).speed(0.1).prefix("x: "));
                ui.add(egui::DragValue::new(&mut t.translation.y).speed(0.1).prefix("y: "));
                ui.add(egui::DragValue::new(&mut t.translation.z).speed(0.1).prefix("z: "));
              });
              ui.end_row();

              ui.label("rotation");
              ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut t.rotation.x).speed(0.1).prefix("x: "));
                ui.add(egui::DragValue::new(&mut t.rotation.y).speed(0.1).prefix("y: "));
                ui.add(egui::DragValue::new(&mut t.rotation.z).speed(0.1).prefix("z: "));
                ui.add(egui::DragValue::new(&mut t.rotation.w).speed(0.1).prefix("w: "));
              });
              ui.end_row();

              ui.label("scale");
              let Vec3 {mut x, mut y, mut z} = t.scale;
              ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut x).speed(0.1).prefix("x: "));
                ui.add(egui::DragValue::new(&mut y).speed(0.1).prefix("y: "));
                ui.add(egui::DragValue::new(&mut z).speed(0.1).prefix("z: "));
              });
              if x != t.scale.x || y != t.scale.y || z != t.scale.z
              {
                let dim = Vec3::new(x, y, z);
                // TODO(XXX): what about negative scale, should we use abs values?
                rb.Ibody = inertia_from_cuboid(rb.mass, dim);
                rb.Ibodyinv = inertiainv_from_cuboid(rb.mass, dim);
                t.scale = dim;
                rb.reset_energy();
                // TODO(XXX): maybe we need to reset P & L?
              }
              ui.end_row();

              ui.label("mass");
              let mut mass = rb.mass;
              ui.add(egui::DragValue::new(&mut mass).speed(0.1).prefix("M: "));
              if mass != rb.mass
              {
                rb.Ibody = inertia_from_cuboid(mass, t.scale);
                rb.Ibodyinv = inertiainv_from_cuboid(mass, t.scale);
                rb.mass = mass;
                // TODO(XXX): maybe we need to reset P & L
              }
              ui.end_row();

              ui.label("velocity");
              ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut rb.v.x).speed(0.1).prefix("x: "));
                ui.add(egui::DragValue::new(&mut rb.v.y).speed(0.1).prefix("y: "));
                ui.add(egui::DragValue::new(&mut rb.v.z).speed(0.1).prefix("z: "));
              });
              ui.end_row();

              ui.label("omega");
              ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut rb.omega.x).speed(0.1).prefix("x: "));
                ui.add(egui::DragValue::new(&mut rb.omega.y).speed(0.1).prefix("y: "));
                ui.add(egui::DragValue::new(&mut rb.omega.z).speed(0.1).prefix("z: "));
              });
              ui.end_row();

              ui.label("P");
              ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut rb.P.x).speed(0.1).prefix("x: "));
                ui.add(egui::DragValue::new(&mut rb.P.y).speed(0.1).prefix("y: "));
                ui.add(egui::DragValue::new(&mut rb.P.z).speed(0.1).prefix("z: "));
              });
              ui.end_row();

              ui.label("L");
              ui.horizontal(|ui| {
                ui.add(egui::DragValue::new(&mut rb.L.x).speed(0.1).prefix("x: "));
                ui.add(egui::DragValue::new(&mut rb.L.y).speed(0.1).prefix("y: "));
                ui.add(egui::DragValue::new(&mut rb.L.z).speed(0.1).prefix("z: "));
              });
              ui.end_row();
            });
        }
      }

      ui.separator();

      for (e, mut c_t, mut c) in constraints.iter_mut()
      {
        match *c
        {
          Constraint3D::Distance(ref mut c) => {
            ui.heading(format!("Constraint::Distance {}-{}", e.generation(), e.index()));
            egui::Grid::new("e")
              .num_columns(2)
              .spacing([40., 6.0])
              .striped(true)
              .show(ui, |ui| {
                ui.label("Translation");
                ui.horizontal(|ui| {
                  ui.add(egui::DragValue::new(&mut c_t.translation.x).speed(0.1).prefix("x: "));
                  ui.add(egui::DragValue::new(&mut c_t.translation.y).speed(0.1).prefix("y: "));
                  ui.add(egui::DragValue::new(&mut c_t.translation.z).speed(0.1).prefix("z: "));
                });
                ui.end_row();

                let mut d = c.d;
                ui.label("Distance");
                ui.add(egui::DragValue::new(&mut d).speed(0.1).prefix("D: "));
                if d != c.d
                {
                  if let Ok([(entity_a, mut rb_a, t_a), (entity_b, mut rb_b, t_b)]) = bodies.get_many_mut([c.body_a, c.body_b])
                  {
                    rb_a.reset_energy();
                    rb_b.reset_energy();
                    c.d = d;
                  }
                }
                ui.end_row();
              });
          },
          _ => todo!(),
        }
      }

      use egui::special_emojis::GITHUB;
      ui.hyperlink_to(format!("{GITHUB} egui on GitHub"), "https://github.com/emilk/egui");
  });
}

fn collect_constraints(mut constraints: Query<(&mut Constraint3D)>,
                       mut system: ResMut<RigidbodySystem3D>)
{
    for c in constraints.iter()
    {
      system.constraints.push(*c);
    }
}

fn draw_lazer_eye(pointers: Query<&PointerInteraction>,
                  mut camera: Single<&mut Transform, With<Camera>>,
                  mut hittables: Query<(&Transform, &mut Rigidbody3D), (With<Hittable>, Without<Camera>)>,
                  world: ResMut<World>,
                  mut gizmos: Gizmos)
{
  if let Some(active_key) = world.active_key {return;}
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
                      world: ResMut<World>,
                      mut camera: Single<&mut Transform, With<Camera>>,
                      mut hittables: Query<(&Transform, &mut Rigidbody3D), (With<Hittable>, Without<Camera>)>,
                      mut constraints: Query<(&mut Constraint3D)>,
                      mut system: ResMut<RigidbodySystem3D>,
                      mouse_buttons: Res<ButtonInput<MouseButton>>,
                      mut gizmos: Gizmos)
{

  if let Some(active_key) = world.active_key {return;}
  let camera_transform = &mut *camera;
  for (point, normal, entity) in pointers.iter()
                                         .filter_map(|interaction| interaction.get_nearest_hit())
                                         .filter_map(|(entity, hit)| hit.position.zip(hit.normal).map(|(point, normal)| (point, normal, *entity)))
  {
    if !hittables.contains(entity)
    {
      continue;
    }

    if let Ok((t_a, rb_a)) = hittables.get_mut(entity)
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
    }
  }
}

fn update_world(mut world: ResMut<World>,
                mouse_buttons: Res<ButtonInput<MouseButton>>)
{
  if mouse_buttons.just_released(MouseButton::Left)
  {
    world.active_key = None;
  }
}

fn update_bodies(mut world: ResMut<World>,
                 mut bodies: Query<(Entity, &mut Rigidbody3D, &mut Transform)>,
                 mouse_buttons: Res<ButtonInput<MouseButton>>)
{
  for (e, mut rb, t) in bodies.iter_mut()
  {
    if let Some(active_key) = world.active_key
    {
      if e == active_key
      {
        rb.kind = Rigidbody3DKind::Sleep;
        rb.reset_energy();
      }
    }
    else
    {
      if rb.kind == Rigidbody3DKind::Sleep
      {
        rb.kind = Rigidbody3DKind::Dynamic;
      }
    }
  }
}

fn update_constraint_indicator(mut bodies: Query<(Entity, &mut Rigidbody3D, &mut Transform)>,
                               mut constraints: Query<(&mut Transform, &mut Constraint3D), Without<Rigidbody3D>>)
{

  for (mut c_transform, c) in constraints.iter_mut()
  {
    if let Constraint3D::Distance(ref c) = *c
    {
      let Ok((entity_a, rb_a, t_a)) = bodies.get(c.body_a) else { continue; };

      let point = t_a.rotation.mul_vec3(c.point_a) + t_a.translation;
      c_transform.translation = point;
    }
  }
}

fn change_contact_on_drag(drag: Trigger<Pointer<Drag>>,
                          mut world: ResMut<World>,
                          pointers: Query<&PointerInteraction>,
                          mut bodies: Query<(Entity, &mut Rigidbody3D, &mut Transform)>,
                          mut constraints: Query<(&mut Transform, &mut Constraint3D), Without<Rigidbody3D>>)
{
  let (mut transform, mut constraint) = constraints.get_mut(drag.target()).unwrap();
  let Constraint3D::Distance(ref mut constraint) = *constraint else { return; };
  let Ok([(entity_a, mut rb_a, t_a), (entity_b, rb_b, t_b)]) = bodies.get_many_mut([constraint.body_a, constraint.body_b]) else { return; };

  world.active_key = Some(entity_a);
  for (point, normal, entity) in pointers.iter()
                                         .filter_map(|interaction| interaction.get_nearest_hit())
                                         .filter_map(|(entity, hit)| hit.position.zip(hit.normal).map(|(point, normal)| (point, normal, *entity)))
  {
    if entity == entity_a
    {
      transform.translation = point;
      let rel = point - t_a.translation;
      let contact = t_a.rotation.inverse().mul_vec3(rel);
      constraint.point_a = contact;
      constraint.d = (point-t_b.translation).length();
    }
  }
}

fn update_material_on<E>(
    new_material: Handle<StandardMaterial>,
) -> impl Fn(Trigger<E>, Query<&mut MeshMaterial3d<StandardMaterial>>) {
    // An observer closure that captures `new_material`. We do this to avoid needing to write four
    // versions of this observer, each triggered by a different event and with a different hardcoded
    // material. Instead, the event type is a generic, and the material is passed in.
    move |trigger, mut query| {
        if let Ok(mut material) = query.get_mut(trigger.target()) {
            material.0 = new_material.clone();
        }
    }
}
