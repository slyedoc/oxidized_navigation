//! Nav-mesh set up example for multiple floors.
//!
//! Press A to run async path finding.
//!
//! Press B to run blocking path finding.
//!

use avian3d::prelude::*;
use bevy::tasks::futures_lite::future;
use bevy::{
    color::palettes,
    prelude::*,
    tasks::{AsyncComputeTaskPool, Task},
};
use oxidized_navigation::DetailMeshSettings;
use oxidized_navigation::{
    colliders::avian::AvianCollider,
    debug_draw::{DrawNavMesh, DrawPath, OxidizedNavigationDebugDrawPlugin},
    query::{find_path, find_polygon_path, perform_string_pulling_on_path},
    tiles::NavMeshTiles,
    NavMesh, NavMeshAffector, NavMeshSettings, OxidizedNavigationPlugin,
};
use std::num::{NonZeroU16, NonZeroU8};
use std::sync::{Arc, RwLock};

fn main() {
    App::new()
        // Default Plugins
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Oxidized Navigation: Avian 3d Heightfield".to_owned(),
                    ..default()
                }),
                ..default()
            }),
            OxidizedNavigationPlugin::<AvianCollider>::new(
                NavMeshSettings::from_agent_and_bounds(0.5, 1.9, 250.0, -20.0)
                    .with_max_tile_generation_tasks(Some(NonZeroU16::MIN))
                    .with_experimental_detail_mesh_generation(DetailMeshSettings {
                        max_height_error: NonZeroU16::new(4).unwrap(),
                        sample_step: NonZeroU8::new(16).unwrap(),
                    }),
            ),
            OxidizedNavigationDebugDrawPlugin,
            // The rapier plugin needs to be added for the scales of colliders to be correct if the scale of the entity is not uniformly 1.
            // An example of this is the "Thin Wall" in [setup_world_system]. If you remove this plugin, it will not appear correctly.
            PhysicsPlugins::default(),
            PhysicsDebugPlugin::default(),
        ))
        .insert_resource(AsyncPathfindingTasks::default())
        .add_systems(Startup, setup_world_system)
        .add_systems(
            Update,
            (
                run_blocking_pathfinding,
                run_async_pathfinding,
                poll_pathfinding_tasks_system,
                toggle_nav_mesh_debug_draw,
                spawn_or_despawn_affector_system,
            ),
        )
        .run();
}

fn run_blocking_pathfinding(
    mut commands: Commands,
    keys: Res<ButtonInput<KeyCode>>,
    nav_mesh_settings: Res<NavMeshSettings>,
    nav_mesh: Res<NavMesh>,
) {
    if !keys.just_pressed(KeyCode::KeyB) {
        return;
    }

    // Get the underlying nav_mesh.
    if let Ok(nav_mesh) = nav_mesh.get().read() {
        let start_pos = Vec3::new(5.0, 1.0, 5.0);
        let end_pos = Vec3::new(-15.0, 1.0, -15.0);

        // Run pathfinding to get a polygon path.
        match find_polygon_path(
            &nav_mesh,
            &nav_mesh_settings,
            start_pos,
            end_pos,
            None,
            Some(&[1.0, 0.5]),
        ) {
            Ok(path) => {
                info!("Path found (BLOCKING): {:?}", path);

                // Convert polygon path to a path of Vec3s.
                match perform_string_pulling_on_path(&nav_mesh, start_pos, end_pos, &path) {
                    Ok(string_path) => {
                        info!("String path (BLOCKING): {:?}", string_path);
                        commands.spawn(DrawPath {
                            timer: Some(Timer::from_seconds(4.0, TimerMode::Once)),
                            pulled_path: string_path,
                            color: palettes::css::RED.into(),
                        });
                    }
                    Err(error) => error!("Error with string path: {:?}", error),
                };
            }
            Err(error) => error!("Error with pathfinding: {:?}", error),
        }
    }
}

//  Running pathfinding in a task without blocking the frame.
//  Also check out Bevy's async compute example.
//  https://github.com/bevyengine/bevy/blob/main/examples/async_tasks/async_compute.rs

// Holder resource for tasks.
#[derive(Default, Resource)]
struct AsyncPathfindingTasks {
    tasks: Vec<Task<Option<Vec<Vec3>>>>,
}

// Queue up pathfinding tasks.
fn run_async_pathfinding(
    keys: Res<ButtonInput<KeyCode>>,
    nav_mesh_settings: Res<NavMeshSettings>,
    nav_mesh: Res<NavMesh>,
    mut pathfinding_task: ResMut<AsyncPathfindingTasks>,
) {
    if !keys.just_pressed(KeyCode::KeyA) {
        return;
    }

    let thread_pool = AsyncComputeTaskPool::get();

    let nav_mesh_lock = nav_mesh.get();
    let start_pos = Vec3::new(5.0, 1.0, 5.0);
    let end_pos = Vec3::new(-15.0, 1.0, -15.0);

    let task = thread_pool.spawn(async_path_find(
        nav_mesh_lock,
        nav_mesh_settings.clone(),
        start_pos,
        end_pos,
        None,
    ));

    pathfinding_task.tasks.push(task);
}

// Poll existing tasks.
fn poll_pathfinding_tasks_system(
    mut commands: Commands,
    mut pathfinding_task: ResMut<AsyncPathfindingTasks>,
) {
    // Go through and remove completed tasks.
    pathfinding_task.tasks.retain_mut(|task| {
        if let Some(string_path) = future::block_on(future::poll_once(task)).unwrap_or(None) {
            info!("Async path task finished with result: {:?}", string_path);
            commands.spawn(DrawPath {
                timer: Some(Timer::from_seconds(4.0, TimerMode::Once)),
                pulled_path: string_path,
                color: palettes::css::BLUE.into(),
            });

            false
        } else {
            true
        }
    });
}

/// Async wrapper function for path finding.
async fn async_path_find(
    nav_mesh_lock: Arc<RwLock<NavMeshTiles>>,
    nav_mesh_settings: NavMeshSettings,
    start_pos: Vec3,
    end_pos: Vec3,
    position_search_radius: Option<f32>,
) -> Option<Vec<Vec3>> {
    // Get the underlying nav_mesh.
    let Ok(nav_mesh) = nav_mesh_lock.read() else {
        return None;
    };

    // Run pathfinding to get a path.
    match find_path(
        &nav_mesh,
        &nav_mesh_settings,
        start_pos,
        end_pos,
        position_search_radius,
        Some(&[1.0, 0.5]),
    ) {
        Ok(path) => {
            info!("Found path (ASYNC): {:?}", path);
            return Some(path);
        }
        Err(error) => error!("Error with pathfinding: {:?}", error),
    }

    None
}

fn toggle_nav_mesh_debug_draw(
    keys: Res<ButtonInput<KeyCode>>,
    mut show_navmesh: ResMut<DrawNavMesh>,
) {
    if keys.just_pressed(KeyCode::KeyM) {
        show_navmesh.0 = !show_navmesh.0;
    }
}

fn setup_world_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    print_controls();

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(30.0, 20.0, 40.0).looking_at(Vec3::new(0.0, 2.0, 0.0), Vec3::Y),
    ));

    commands.spawn((
        DirectionalLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -1.0, -0.5, 0.0)),
    ));

    let resolution = 70;    
    let mut heightfield = vec![vec![0.0; resolution]; resolution];
    for x in 0..resolution {
        for y in 0..resolution {
            heightfield[x][y] = ((x as f32 / 10.0).sin() + (y as f32 / 10.0).cos());
        }
    }

    // Heightfield.
    commands.spawn((        
        Transform::IDENTITY,
        Collider::heightfield(heightfield, Vec3::new(resolution as f32, 9.0, resolution as f32)),
        NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
    ));
}

fn spawn_or_despawn_affector_system(
    keys: Res<ButtonInput<KeyCode>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut spawned_entity: Local<Option<Entity>>,
) {
    if !keys.just_pressed(KeyCode::KeyX) {
        return;
    }

    if let Some(entity) = *spawned_entity {
        commands.entity(entity).despawn();
        *spawned_entity = None;
    } else {
        let entity = commands
            .spawn((
                Mesh3d(meshes.add(Cuboid::new(2.5, 2.5, 2.5))),
                MeshMaterial3d(materials.add(Color::srgb(1.0, 0.1, 0.5))),
                Transform::from_xyz(5.0, 0.8, -5.0),
                Collider::cuboid(2.5, 2.5, 2.5),
                NavMeshAffector, // Only entities with a NavMeshAffector component will contribute to the nav-mesh.
            ))
            .id();

        *spawned_entity = Some(entity);
    }
}

fn print_controls() {
    info!("=========================================");
    info!("| Press A to run ASYNC path finding.    |");
    info!("| Press B to run BLOCKING path finding. |");
    info!("| Press M to toggle drawing nav-mesh.   |");
    info!("| Press X to spawn or despawn red cube. |");
    info!("=========================================");
}
