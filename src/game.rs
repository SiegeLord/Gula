use crate::astar::Node;
use crate::error::Result;
use crate::utils::DT;
use crate::{astar, components as comps, controls, game_state, scene, sprite, ui, utils};
use allegro::*;
use allegro_font::*;
use na::{
	Isometry3, Matrix4, Perspective3, Point2, Point3, Quaternion, RealField, Rotation2, Rotation3,
	Similarity3, Unit, UnitQuaternion, Vector2, Vector3, Vector4,
};
use nalgebra as na;
use rand::prelude::*;
use rapier3d::dynamics::{
	CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
	RigidBodyBuilder, RigidBodyHandle, RigidBodySet,
};
use rapier3d::geometry::{
	Ball, ColliderBuilder, ColliderSet, CollisionEvent, ContactPair, DefaultBroadPhase,
	NarrowPhase, SharedShape,
};
use rapier3d::pipeline::{ActiveEvents, EventHandler, PhysicsPipeline, QueryPipeline};

use std::collections::HashMap;
use std::f32::consts::PI;
use std::sync::RwLock;

const GROW_TIME: f64 = 1.;
const MESSAGE_TIME: f64 = 8.;
const PHI: f32 = 1.618033988749;

pub struct Game
{
	map: Map,
	subscreens: ui::SubScreens,
}

impl Game
{
	pub fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		Ok(Self {
			map: Map::new(state)?,
			subscreens: ui::SubScreens::new(state),
		})
	}

	pub fn logic(
		&mut self, state: &mut game_state::GameState,
	) -> Result<Option<game_state::NextScreen>>
	{
		if self.subscreens.is_empty()
		{
			self.map.logic(state)
		}
		else
		{
			Ok(None)
		}
	}

	pub fn input(
		&mut self, event: &Event, state: &mut game_state::GameState,
	) -> Result<Option<game_state::NextScreen>>
	{
		match *event
		{
			Event::MouseAxes { x, y, .. } =>
			{
				if state.track_mouse
				{
					let (x, y) = state.transform_mouse(x as f32, y as f32);
					state.mouse_pos = Point2::new(x as i32, y as i32);
				}
			}
			_ => (),
		}
		if self.subscreens.is_empty()
		{
			let mut in_game_menu = false;
			let handled = false; // In case there's other in-game UI to handle this.
			state.hide_mouse = true;
			if state
				.game_ui_controls
				.get_action_state(controls::Action::UICancel)
				> 0.5
			{
				in_game_menu = true;
				state.hide_mouse = false;
			}
			else if !handled
			{
				let res = self.map.input(event, state);
				if let Ok(Some(game_state::NextScreen::InGameMenu)) = res
				{
					in_game_menu = true;
				}
				else
				{
					return res;
				}
			}
			if in_game_menu
			{
				self.subscreens
					.push(ui::SubScreen::InGameMenu(ui::InGameMenu::new(state)));
				self.subscreens.reset_transition(state);
			}
		}
		else
		{
			if let Some(action) = self.subscreens.input(state, event)?
			{
				match action
				{
					ui::Action::MainMenu =>
					{
						return Ok(Some(game_state::NextScreen::Menu));
					}
					ui::Action::Start =>
					{
						return Ok(Some(game_state::NextScreen::Game));
					}
					_ => (),
				}
			}
			if self.subscreens.is_empty()
			{
				state.controls.clear_action_states();
				state.hide_mouse = true;
			}
		}
		Ok(None)
	}

	pub fn draw(&mut self, state: &mut game_state::GameState) -> Result<()>
	{
		self.map.draw(state)?;
		if !self.subscreens.is_empty()
		{
			state.prim.draw_filled_rectangle(
				0.,
				0.,
				state.buffer_width(),
				state.buffer_height(),
				Color::from_rgba_f(0., 0., 0., 0.75),
			);
			self.subscreens.draw(state);
		}
		Ok(())
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		self.subscreens.resize(state);
	}
}

pub fn spawn_animal(
	pos: Point3<f32>, kind: comps::AnimalKind, state: &mut game_state::GameState,
	physics: &mut Physics, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let scene_name = match kind
	{
		comps::AnimalKind::Cat => "data/cat.glb",
		comps::AnimalKind::Rhino => "data/rhino.glb",
		comps::AnimalKind::Zebra => "data/zebra.glb",
	};
	game_state::cache_scene(state, scene_name)?;

	let animal = comps::Animal::new(kind);
	let init_size = animal.size;
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Scene {
			scene: scene_name.to_string(),
		},
		comps::GroundTracker::new(),
		comps::Controller::new(),
		animal,
		comps::Light {
			intensity: 105.0,
			color: Color::from_rgb_f(1., 1., 1.),
			static_: false,
		},
	));

	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(init_size / 2.)
		.restitution(0.8)
		.user_data(entity.to_bits().get() as u128)
		.active_events(ActiveEvents::COLLISION_EVENTS | ActiveEvents::CONTACT_FORCE_EVENTS)
		.build();
	let ball_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		ball_body_handle,
		&mut physics.rigid_body_set,
	);
	world.insert_one(
		entity,
		comps::Physics {
			handle: ball_body_handle,
		},
	)?;
	Ok(entity)
}

pub fn spawn_level(
	scene_name: &str, state: &mut game_state::GameState, physics: &mut Physics,
	world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	game_state::cache_scene(state, scene_name)?;

	let entity = world.spawn((
		comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
		comps::Scene {
			scene: scene_name.to_string(),
		},
	));

	let level_scene = state.get_scene(scene_name).unwrap();
	let mut vertices = vec![];
	let mut indices = vec![];
	for object in &level_scene.objects
	{
		match &object.kind
		{
			scene::ObjectKind::MultiMesh { meshes } =>
			{
				let mut index_offset = 0;
				for mesh in meshes
				{
					for vtx in &mesh.vtxs
					{
						vertices.push(Point3::new(vtx.x, vtx.y, vtx.z));
					}
					for idxs in mesh.idxs.chunks(3)
					{
						indices.push([
							idxs[0] as u32 + index_offset,
							idxs[1] as u32 + index_offset,
							idxs[2] as u32 + index_offset,
						]);
					}
					index_offset += mesh.vtxs.len() as u32;
				}
			}
			_ => (),
		}
	}
	let rigid_body = RigidBodyBuilder::fixed()
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::trimesh(vertices, indices)?
		.user_data(entity.to_bits().get() as u128)
		.build();
	let rigid_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		rigid_body_handle,
		&mut physics.rigid_body_set,
	);
	world.insert_one(
		entity,
		comps::Physics {
			handle: rigid_body_handle,
		},
	)?;
	Ok(entity)
}

pub fn spawn_shard(
	scene_name: &str, state: &mut game_state::GameState, physics: &mut Physics,
	world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	game_state::cache_scene(state, scene_name)?;

	let entity = world.spawn((
		comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
		comps::Scene {
			scene: scene_name.to_string(),
		},
	));

	let level_scene = state.get_scene(scene_name).unwrap();
	let mut vertices = vec![];
	let mut indices = vec![];
	for object in &level_scene.objects
	{
		match &object.kind
		{
			scene::ObjectKind::MultiMesh { meshes } =>
			{
				let mut index_offset = 0;
				for mesh in meshes
				{
					for vtx in &mesh.vtxs
					{
						vertices.push(Point3::new(vtx.x, vtx.y, vtx.z));
					}
					for idxs in mesh.idxs.chunks(3)
					{
						indices.push([
							idxs[0] as u32 + index_offset,
							idxs[1] as u32 + index_offset,
							idxs[2] as u32 + index_offset,
						]);
					}
					index_offset += mesh.vtxs.len() as u32;
				}
			}
			_ => (),
		}
	}
	let rigid_body = RigidBodyBuilder::dynamic()
		.user_data(entity.to_bits().get() as u128)
		.gravity_scale(0.25)
		.build();
	let collider = ColliderBuilder::trimesh(vertices, indices)?
		.user_data(entity.to_bits().get() as u128)
		.mass(0.05)
		.build();
	let rigid_body_handle = physics.rigid_body_set.insert(rigid_body);
	physics.collider_set.insert_with_parent(
		collider,
		rigid_body_handle,
		&mut physics.rigid_body_set,
	);
	world.insert_one(
		entity,
		comps::Physics {
			handle: rigid_body_handle,
		},
	)?;
	Ok(entity)
}

pub fn spawn_light(
	pos: Point3<f32>, light: comps::Light, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let entity = world.spawn((comps::Position::new(pos, UnitQuaternion::identity()), light));
	Ok(entity)
}

pub fn spawn_food(
	pos: Point3<f32>, kind: comps::FoodKind, state: &mut game_state::GameState,
	physics: &mut Physics, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let scene_name;
	match kind
	{
		comps::FoodKind::Fat =>
		{
			scene_name = "data/cupcake.glb";
		}
		comps::FoodKind::Muscle =>
		{
			scene_name = "data/banana.glb";
		}
	}
	game_state::cache_scene(state, scene_name)?;

	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Scene {
			scene: scene_name.to_string(),
		},
		comps::Light {
			intensity: 105.0,
			color: kind.color(),
			static_: false,
		},
		comps::Food::new(pos, kind),
	));
	let collider = ColliderBuilder::ball(0.5)
		.sensor(true)
		.translation(pos.coords)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let sensor_handle = physics.collider_set.insert(collider);
	world.insert_one(
		entity,
		comps::Sensor {
			handle: sensor_handle,
		},
	)?;
	Ok(entity)
}

pub fn spawn_food_spawner(pos: Point3<f32>, world: &mut hecs::World) -> Result<hecs::Entity>
{
	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::FoodSpawner::new(),
	));
	Ok(entity)
}

pub fn spawn_message(text: &str, time_to_show: f64, world: &mut hecs::World)
	-> Result<hecs::Entity>
{
	let entity = world.spawn((comps::Message::new(text, time_to_show),));
	Ok(entity)
}

pub struct PhysicsEventHandler
{
	collision_events: RwLock<Vec<(CollisionEvent, Option<ContactPair>)>>,
	contact_force_events: RwLock<Vec<(f32, ContactPair)>>,
}

impl PhysicsEventHandler
{
	pub fn new() -> Self
	{
		Self {
			collision_events: RwLock::new(vec![]),
			contact_force_events: RwLock::new(vec![]),
		}
	}
}

impl EventHandler for PhysicsEventHandler
{
	fn handle_collision_event(
		&self, _bodies: &RigidBodySet, _colliders: &ColliderSet, _event: CollisionEvent,
		_contact_pair: Option<&ContactPair>,
	)
	{
		//let mut events = self.collision_events.write().unwrap();
		//events.push((event, contact_pair.cloned()));
	}

	fn handle_contact_force_event(
		&self, _dt: f32, _bodies: &RigidBodySet, _colliders: &ColliderSet,
		contact_pair: &ContactPair, total_force_magnitude: f32,
	)
	{
		self.contact_force_events
			.write()
			.unwrap()
			.push((total_force_magnitude, contact_pair.clone()));
	}
}

pub struct Physics
{
	rigid_body_set: RigidBodySet,
	collider_set: ColliderSet,
	integration_parameters: IntegrationParameters,
	physics_pipeline: PhysicsPipeline,
	island_manager: IslandManager,
	broad_phase: DefaultBroadPhase,
	narrow_phase: NarrowPhase,
	impulse_joint_set: ImpulseJointSet,
	multibody_joint_set: MultibodyJointSet,
	ccd_solver: CCDSolver,
	query_pipeline: QueryPipeline,
}

impl Physics
{
	fn new() -> Self
	{
		Self {
			rigid_body_set: RigidBodySet::new(),
			collider_set: ColliderSet::new(),
			integration_parameters: IntegrationParameters {
				dt: DT,
				..IntegrationParameters::default()
			},
			physics_pipeline: PhysicsPipeline::new(),
			island_manager: IslandManager::new(),
			broad_phase: DefaultBroadPhase::new(),
			narrow_phase: NarrowPhase::new(),
			impulse_joint_set: ImpulseJointSet::new(),
			multibody_joint_set: MultibodyJointSet::new(),
			ccd_solver: CCDSolver::new(),
			query_pipeline: QueryPipeline::new(),
		}
	}

	fn step(&mut self, event_handler: &PhysicsEventHandler)
	{
		let gravity = Vector3::y() * -5.;
		self.physics_pipeline.step(
			&gravity,
			&self.integration_parameters,
			&mut self.island_manager,
			&mut self.broad_phase,
			&mut self.narrow_phase,
			&mut self.rigid_body_set,
			&mut self.collider_set,
			&mut self.impulse_joint_set,
			&mut self.multibody_joint_set,
			&mut self.ccd_solver,
			Some(&mut self.query_pipeline),
			&(),
			event_handler,
		);
	}
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum State
{
	Playing,
	Dead,
	Won,
}

struct Map
{
	world: hecs::World,
	physics: Physics,
	camera_target: comps::Position,
	player: hecs::Entity,
	player_kind: comps::AnimalKind,
	level: hecs::Entity,
	time_to_snap_camera: f64,
	navmesh: Vec<scene::NavNode>,
	show_path: bool,
	state: State,
}

impl Map
{
	fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		let mut world = hecs::World::new();
		let mut physics = Physics::new();
		state
			.sfx
			.play_music("data/BANANAS - Gameplay 1.2.ogg", 0.25, &state.core);

		game_state::cache_scene(state, "data/sphere.glb")?;
		game_state::cache_scene(state, "data/sky.glb")?;
		state.cache_bitmap("data/cat_icon.png")?;
		state.cache_bitmap("data/zebra_icon.png")?;
		state.cache_bitmap("data/rhino_icon.png")?;
		state.cache_bitmap("data/fat_icon.png")?;
		state.cache_bitmap("data/muscle_icon.png")?;

		let level_scene_name = "data/level2.glb";
		let level = spawn_level(level_scene_name, state, &mut physics, &mut world)?;

		let level_scene = state.get_scene(level_scene_name).unwrap();
		let mut animal_spawns = vec![];
		let mut food_spawns = vec![];
		let mut navmesh = vec![];
		for object in &level_scene.objects
		{
			match &object.kind
			{
				scene::ObjectKind::Light { color, intensity } =>
				{
					//println!("Spawning light at {:?}", object.position);
					spawn_light(
						object.position,
						comps::Light {
							color: *color,
							intensity: intensity / 50.,
							static_: true,
						},
						&mut world,
					)?;
				}
				scene::ObjectKind::Empty =>
				{
					if object.name.starts_with("AnimalSpawn")
					{
						animal_spawns.push(object.position);
					}
					else if object.name.starts_with("FoodSpawn")
					{
						food_spawns.push(object.position);
					}
				}
				scene::ObjectKind::NavMesh { nodes } =>
				{
					navmesh = nodes.clone();
				}
				_ => (),
			}
		}

		let mut rng = thread_rng();

		let mut animals = [
			comps::AnimalKind::Rhino,
			comps::AnimalKind::Zebra,
			comps::AnimalKind::Cat,
		];

		animals.shuffle(&mut rng);

		let mut player = None;
		let mut player_kind = animals[0];
		for i in 0..8
		{
			let kind = if i < animals.len()
			{
				animals[i]
			}
			else
			{
				*animals[1..].choose(&mut rng).unwrap()
			};
			let entity = spawn_animal(
				animal_spawns.choose(&mut rng).unwrap()
					+ Vector3::new(rng.gen_range(-1.0..1.0), 0., rng.gen_range(-1.0..1.0)) * 0.1,
				kind,
				state,
				&mut physics,
				&mut world,
			)?;
			if i == 0
			{
				player = Some(entity);
				player_kind = kind;
			}
			else
			{
				world.insert_one(entity, comps::AI::new())?;
			}
		}

		for food_spawn in food_spawns
		{
			spawn_food_spawner(food_spawn, &mut world)?;
		}

		spawn_message(
			&format!(
				"Press {} to jump!",
				state
					.controls
					.get_controls()
					.get_action_string(controls::Action::Jump)
			),
			5.,
			&mut world,
		)?;
		spawn_message(
			&format!(
				"Eat fat and press {} to reproduce!",
				state
					.controls
					.get_controls()
					.get_action_string(controls::Action::Reproduce)
			),
			15.,
			&mut world,
		)?;
		spawn_message("Push others off the edge!", 25., &mut world)?;

		Ok(Self {
			world: world,
			physics: physics,
			camera_target: comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
			level: level,
			player: player.unwrap(),
			player_kind: player_kind,
			time_to_snap_camera: 0.,
			navmesh: navmesh,
			show_path: false,
			state: State::Playing,
		})
	}

	fn logic(&mut self, state: &mut game_state::GameState)
		-> Result<Option<game_state::NextScreen>>
	{
		let mut to_die = vec![];

		// Position snapshotting.
		for (_, position) in self.world.query::<&mut comps::Position>().iter()
		{
			position.snapshot();
		}
		self.camera_target.snapshot();

		let look_left = state.controls.get_action_state(controls::Action::LookLeft);
		let look_right = state.controls.get_action_state(controls::Action::LookRight);
		let look_up = state.controls.get_action_state(controls::Action::LookUp);
		let look_down = state.controls.get_action_state(controls::Action::LookDown);

		if look_left > 0.1 || look_right > 0.1 || look_up > 0.1 || look_down > 0.1
		{
			self.time_to_snap_camera = state.time() + 3.0;
		}

		let forward = self.camera_target.rot * (-Vector3::z());
		let right = Unit::new_normalize(Vector3::new(-forward.z, 0., forward.x));
		let up = Unit::new_normalize(Vector3::y());

		let rot_horiz = UnitQuaternion::from_axis_angle(
			&up,
			state.options.camera_speed * DT * (look_left - look_right),
		);
		let mut rot_vert = UnitQuaternion::from_axis_angle(
			&right,
			state.options.camera_speed * DT * (look_up - look_down),
		);
		if forward.dot(&up) > 0.99 && (look_up - look_down) > 0.
		{
			rot_vert = UnitQuaternion::identity();
		}
		if forward.dot(&up) < -0.99 && (look_up - look_down) < 0.
		{
			rot_vert = UnitQuaternion::identity();
		}
		self.camera_target.rot = rot_horiz * rot_vert * self.camera_target.rot;

		// Player.
		if self.world.contains(self.player)
		{
			let move_left = state.controls.get_action_state(controls::Action::MoveLeft);
			let move_right = state.controls.get_action_state(controls::Action::MoveRight);
			let move_fwd = state
				.controls
				.get_action_state(controls::Action::MoveForward);
			let move_bwd = state
				.controls
				.get_action_state(controls::Action::MoveBackward);

			let jump = state.controls.get_action_state(controls::Action::Jump);

			let mut controller = self
				.world
				.get::<&mut comps::Controller>(self.player)
				.unwrap();
			controller.want_jump = jump > 0.5;
			controller.want_reproduce =
				state.controls.get_action_state(controls::Action::Reproduce) > 0.5;

			let position = self.world.get::<&comps::Position>(self.player).unwrap();

			let forward2 = self.camera_target.rot * (Vector3::z());
			let forward2 = Vector3::new(forward2.x, 0., forward2.z).normalize();
			let right2 = Vector3::new(-forward2.z, 0., forward2.x).normalize();
			controller.want_move =
				(move_fwd - move_bwd) * forward2 + (move_right - move_left) * right2;

			let physics = self.world.get::<&comps::Physics>(self.player).unwrap();
			let velocity =
				self.physics.rigid_body_set[physics.handle].velocity_at_point(&position.pos);
			if velocity.xz().norm() > 1. && state.time() > self.time_to_snap_camera
			{
				let lagging_point =
					Vector3::new(velocity.x, 0., velocity.z).normalize() - 0.5_f32 * Vector3::y();
				let lagging_rot = UnitQuaternion::look_at_rh(&(-lagging_point), &Vector3::y());

				self.camera_target.rot = self.camera_target.rot.slerp(&lagging_rot.inverse(), DT);
			}

			self.camera_target.pos = position.pos;
			self.camera_target.scale = position.scale;
		}
		else
		{
			let mut found_id = None;
			for (id, animal) in self.world.query::<&comps::Animal>().iter()
			{
				if animal.kind == self.player_kind
				{
					found_id = Some(id);
				}
			}
			if let Some(id) = found_id
			{
				self.world.remove_one::<comps::AI>(id)?;
				self.player = id;
			}
			else
			{
				if self.state == State::Playing
				{
					for (id, _) in self.world.query::<&comps::Message>().iter()
					{
						to_die.push(id);
					}
					spawn_message(
						self.player_kind.defeat_message(),
						state.time(),
						&mut self.world,
					)?;
					self.state = State::Dead;
				}
				self.camera_target.pos = Point3::origin();
				self.camera_target.scale = 1.;
			}
		}

		// AI.
		for (_, (position, controller, animal, ai)) in self
			.world
			.query::<(
				&comps::Position,
				&mut comps::Controller,
				&comps::Animal,
				&mut comps::AI,
			)>()
			.iter()
		{
			controller.want_move = Vector3::zeros();
			controller.want_jump = false;
			controller.want_reproduce = false;

			let mut new_state = None;
			match &mut ai.state
			{
				comps::AIState::Idle =>
				{
					if animal.size > ai.reproduce_threshold
					{
						controller.want_reproduce = true;
					}
					let mut best_distance = f32::INFINITY;
					let mut best_food = None;
					for (food_id, (food_position, _)) in self
						.world
						.query::<(&comps::Position, &comps::Food)>()
						.iter()
					{
						let dist = (position.pos - food_position.pos).norm();
						if dist < best_distance
						{
							best_food = Some((food_id, food_position.pos));
							best_distance = dist;
						}
					}
					if let Some((food_id, food_pos)) = best_food
					{
						let dist_fn = |from: &scene::NavNode, to: &scene::NavNode| {
							let dist = (from.get_pos() - to.get_pos()).norm();
							let v1 = to.triangle[1] - to.triangle[0];
							let v2 = to.triangle[2] - to.triangle[0];
							let normal = v1.cross(&v2).normalize();
							if normal.dot(&Vector3::y()).abs() < 0.4
							{
								if (to.get_pos().y - position.pos.y > 0.5)
									|| (to.get_pos().y - position.pos.y > 1. && animal.size > 1.3)
								{
									return std::f32::INFINITY;
								}
							}
							dist
						};
						let mut path = astar::AStarContext::new(&self.navmesh).solve(
							position.pos,
							food_pos,
							dist_fn,
						);
						path[0] = food_pos;
						path.pop();
						if !path.is_empty()
						{
							new_state = Some(comps::AIState::Hunt {
								target: food_id,
								path: path,
							});
						}
					}
				}
				comps::AIState::Hunt { target, path } =>
				{
					if !self.world.contains(*target)
						|| path.is_empty() || state.time() > ai.time_to_replan
					{
						new_state = Some(comps::AIState::Idle);
						ai.time_to_replan = state.time() + 1.;
					}
					else
					{
						let next_pos = path.last().unwrap();
						let diff = next_pos - position.pos;
						if diff.norm() < 1.
						{
							path.pop();
						}
						else
						{
							let diff_horiz = diff.xz().normalize();
							controller.want_move = Vector3::new(diff_horiz.x, 0., diff_horiz.y);
							let mut rng = thread_rng();
							if ((next_pos.xz() - position.pos.xz()).norm() < 2. && diff.y > 0.5)
								|| rng.gen_bool((0.1 * DT) as f64)
							{
								controller.want_jump = true;
							}
						}
					}
				}
			}
			if let Some(new_state) = new_state
			{
				ai.state = new_state;
			}
		}

		// Friction.
		for (id, (position, physics)) in self
			.world
			.query::<(&comps::Position, &mut comps::Physics)>()
			.iter()
		{
			let f = if id == self.player { 0.2 } else { 0.5 };
			let body = &mut self.physics.rigid_body_set[physics.handle];
			body.reset_forces(true);
			body.add_force(-f * body.velocity_at_point(&position.pos), true);
		}

		// Controller.
		for (id, (position, controller, physics, ground_tracker)) in self
			.world
			.query::<(
				&comps::Position,
				&mut comps::Controller,
				&comps::Physics,
				&comps::GroundTracker,
			)>()
			.iter()
		{
			let body = self.physics.rigid_body_set.get_mut(physics.handle).unwrap();
			let force = 1. * controller.power;
			let jump_impulse = 2. * controller.power;

			body.add_force(controller.want_move * force, true);
			if ground_tracker.on_ground
			{
				if controller.want_jump
				{
					body.apply_impulse(Vector3::y() * jump_impulse, true);
					if let Ok(animal) = self.world.get::<&comps::Animal>(id)
					{
						state.sfx.play_positional_sound_3d(
							animal.kind.jump_sound(),
							position.pos,
							self.camera_pos(0.),
							self.camera_target.rot,
							1. / animal.size.sqrt(),
						)?;
					}
				}
			}
		}

		// Controller-animal interaction
		let mut animals_to_add = vec![];
		for (_, (position, controller, animal)) in self
			.world
			.query::<(&comps::Position, &mut comps::Controller, &mut comps::Animal)>()
			.iter()
		{
			controller.power = 1. + (0.25 * animal.muscle as f32 + 0.75 * animal.size).powf(3.);
			if animal.fat > 1 && controller.want_reproduce
			{
				let fat = animal.fat;
				animal.fat = 0;
				animal.muscle = 0;
				animal.new_size = 1.;

				animals_to_add.push((position.pos, fat - 1, animal.kind));
				state.sfx.play_positional_sound_3d(
					"data/reproduce.ogg",
					position.pos,
					self.camera_pos(0.),
					self.camera_target.rot,
					1. / animal.size.sqrt(),
				)?;
			}
		}
		for (pos, fat, kind) in animals_to_add
		{
			let mut rng = thread_rng();
			for _ in 0..fat
			{
				let entity = spawn_animal(
					pos + Vector3::new(rng.gen_range(-1.0..1.), 0., rng.gen_range(-1.0..1.)) * 0.1,
					kind,
					state,
					&mut self.physics,
					&mut self.world,
				)?;
				self.world.insert_one(entity, comps::AI::new())?;
			}
		}

		// Physics.
		let handler = PhysicsEventHandler::new();
		self.physics.step(&handler);
		if self.world.contains(self.level)
		{
			let mut shards = vec![];
			{
				let level_physics = self.world.get::<&comps::Physics>(self.level).unwrap();
				for (total_force_magnitude, contact_pair) in
					handler.contact_force_events.read().unwrap().iter()
				{
					let level_collider_handle =
						self.physics.rigid_body_set[level_physics.handle].colliders()[0];
					if contact_pair.has_any_active_contact
						&& (contact_pair.collider1 == level_collider_handle
							|| contact_pair.collider2 == level_collider_handle)
					{
						let mut other_handle = contact_pair.collider1;
						if contact_pair.collider1 == level_collider_handle
						{
							other_handle = contact_pair.collider2;
						}
						let other_body = &self.physics.rigid_body_set
							[self.physics.collider_set[other_handle].parent().unwrap()];
						let center = other_body.translation();
						let mut crash = false;
						if let Some(other_id) = hecs::Entity::from_bits(other_body.user_data as u64)
						{
							if let Ok(animal) = self.world.get::<&comps::Animal>(other_id)
							{
								if *total_force_magnitude > 100. * animal.size.powf(3.)
								{
									state.sfx.play_positional_sound_3d(
										"data/land.ogg",
										Point3::origin() + center,
										self.camera_pos(0.),
										self.camera_target.rot,
										1.,
									)?;
								}

								crash = animal.size >= 6.;
							}
						}
						if !crash
						{
							continue;
						}

						to_die.push(self.level);

						let num_shards = 6;
						state.sfx.play_positional_sound_3d(
							"data/crash.ogg",
							Point3::origin() + center,
							self.camera_pos(0.),
							self.camera_target.rot,
							1.,
						)?;
						let rot = UnitQuaternion::from_axis_angle(
							&Unit::new_normalize(Vector3::y()),
							2. * PI / (num_shards as f32),
						);

						let mut v1 = Vector3::x();
						let mut v2 = rot * -v1;

						let scene = state
							.get_scene(&self.world.get::<&comps::Scene>(self.level)?.scene)
							.unwrap();
						for i in 0..6
						{
							let mut new_scene = scene.clone();
							new_scene.clip_meshes(|tri_center| {
								let diff = tri_center - center;
								diff.dot(&v1) > 0. && diff.dot(&v2) > 0.
							});
							for object in &mut new_scene.objects
							{
								if let scene::ObjectKind::MultiMesh { meshes } = &mut object.kind
								{
									for mesh in meshes
									{
										mesh.material.as_mut().map(|m| m.desc.two_sided = true);
									}
								}
							}
							shards.push((format!("shard_{i}"), new_scene));
							v1 = rot * v1;
							v2 = rot * v2
						}
					}
				}
			}
			if !shards.is_empty()
			{
				for (id, _) in self.world.query::<&comps::Message>().iter()
				{
					to_die.push(id);
				}
				spawn_message(
					"Got too fat and broke the world!",
					state.time(),
					&mut self.world,
				)?;
				self.state = State::Dead;
			}
			for (name, scene) in shards
			{
				state.insert_scene(&name, scene);
				spawn_shard(&name, state, &mut self.physics, &mut self.world)?;
			}
		}

		for (_, (position, physics)) in self
			.world
			.query::<(&mut comps::Position, &comps::Physics)>()
			.iter()
		{
			let body = &self.physics.rigid_body_set[physics.handle];
			position.pos = Point3::from(*body.translation());
			position.rot = *body.rotation();
		}

		if self.world.contains(self.level)
		{
			let level_physics = self.world.get::<&comps::Physics>(self.level).unwrap();
			for (_, (position, physics, ground_tracker)) in self
				.world
				.query::<(&comps::Position, &comps::Physics, &mut comps::GroundTracker)>()
				.iter()
			{
				let obj_body = &self.physics.rigid_body_set[physics.handle];
				let level_body = &self.physics.rigid_body_set[level_physics.handle];
				let obj_collider_handle = obj_body.colliders()[0];
				let level_collider_handle = level_body.colliders()[0];

				let mut found_ground = false;
				if let Some(contact_pair) = self
					.physics
					.narrow_phase
					.contact_pair(obj_collider_handle, level_collider_handle)
				{
					if contact_pair.has_any_active_contact
					{
						for manifold in &contact_pair.manifolds
						{
							for point in &manifold.points
							{
								if point.local_p1.y < position.pos.y - 0.2
								{
									found_ground = true;
								}
							}
						}
					}
				}
				ground_tracker.on_ground = found_ground;
			}
		}
		else
		{
			for (_, ground_tracker) in self.world.query::<&mut comps::GroundTracker>().iter()
			{
				ground_tracker.on_ground = false;
			}
		}

		// Food.
		let mut spawn_foods = vec![];
		for (id, (position, food_spawner)) in self
			.world
			.query::<(&comps::Position, &mut comps::FoodSpawner)>()
			.iter()
		{
			if !self.world.contains(food_spawner.food) && !food_spawner.waiting
			{
				food_spawner.time_to_spawn = state.time() + 10.0;
				food_spawner.waiting = true;
			}
			if state.time() > food_spawner.time_to_spawn
			{
				food_spawner.waiting = false;
				food_spawner.time_to_spawn = std::f64::INFINITY;
				spawn_foods.push((id, position.pos));
			}
		}
		for (parent, pos) in spawn_foods
		{
			let mut rng = thread_rng();
			let kind = *([comps::FoodKind::Fat, comps::FoodKind::Muscle])
				.choose(&mut rng)
				.unwrap();
			let food = spawn_food(pos, kind, state, &mut self.physics, &mut self.world)?;
			let mut food_spawner = self.world.get::<&mut comps::FoodSpawner>(parent)?;
			food_spawner.food = food;
		}

		for (id, (position, food, sensor)) in self
			.world
			.query::<(&mut comps::Position, &comps::Food, &comps::Sensor)>()
			.iter()
		{
			position.pos =
				food.spawn_pos + Vector3::new(0., 0.25 * (5. * state.time()).cos() as f32, 0.);
			position.rot = UnitQuaternion::from_axis_angle(
				&Unit::new_normalize(Vector3::y()),
				((5. * state.time()) % (2. * PI as f64)) as f32,
			);

			for (collider_handle1, collider_handle2, intersecting) in self
				.physics
				.narrow_phase
				.intersection_pairs_with(sensor.handle)
			{
				let mut other_handle = collider_handle1;
				if other_handle == sensor.handle
				{
					other_handle = collider_handle2;
				}
				if intersecting
				{
					let other_collider = self.physics.collider_set.get(other_handle).unwrap();
					if let Some(id2) = hecs::Entity::from_bits(other_collider.user_data as u64)
					{
						if let Ok(mut animal) = self.world.get::<&mut comps::Animal>(id2)
						{
							to_die.push(id);
							match food.kind
							{
								comps::FoodKind::Fat =>
								{
									animal.fat += 1;
									animal.new_size *= PHI;
								}
								comps::FoodKind::Muscle =>
								{
									animal.muscle += 1;
									animal.new_size *= PHI.sqrt();
								}
							}
							state.sfx.play_positional_sound_3d(
								"data/eat.ogg",
								position.pos,
								self.camera_pos(0.),
								self.camera_target.rot,
								1.,
							)?;
							break;
						}
					}
				}
			}
		}

		let mut see_non_player_kind = false;
		// Animal.
		for (_, (position, animal, physics)) in self
			.world
			.query::<(&mut comps::Position, &mut comps::Animal, &comps::Physics)>()
			.iter()
		{
			if animal.kind != self.player_kind
			{
				see_non_player_kind = true;
			}

			let change = DT / GROW_TIME as f32;
			if (animal.size - animal.new_size).abs() < change
			{
				animal.size = animal.new_size;
				continue;
			}

			animal.size += (animal.new_size - animal.size).signum() * change;
			position.scale = animal.size;

			let body = &self.physics.rigid_body_set[physics.handle];
			let collider = self
				.physics
				.collider_set
				.get_mut(body.colliders()[0])
				.unwrap();

			collider.set_shape(SharedShape::new(Ball {
				radius: 0.5 * animal.size,
			}));
		}
		if !see_non_player_kind && self.state == State::Playing
		{
			self.state = State::Won;
			for (id, _) in self.world.query::<&comps::Message>().iter()
			{
				to_die.push(id);
			}
			spawn_message(
				self.player_kind.victory_message(),
				state.time(),
				&mut self.world,
			)?;
		}

		// Abyss
		for (id, position) in self.world.query::<&mut comps::Position>().iter()
		{
			if position.pos.y < -10.
			{
				if self.world.get::<&comps::Animal>(id).is_ok()
				{
					state.sfx.play_sound("data/death.ogg")?;
				}
				to_die.push(id);
			}
		}

		for (id, message) in self.world.query::<&comps::Message>().iter()
		{
			if state.time() > message.time_to_show + MESSAGE_TIME
			{
				to_die.push(id);
			}
		}

		// Remove dead entities
		to_die.sort();
		to_die.dedup();
		for id in to_die
		{
			if let Ok(physics) = self.world.get::<&comps::Physics>(id)
			{
				self.physics.rigid_body_set.remove(
					physics.handle,
					&mut self.physics.island_manager,
					&mut self.physics.collider_set,
					&mut self.physics.impulse_joint_set,
					&mut self.physics.multibody_joint_set,
					true,
				);
			}
			if let Ok(sensor) = self.world.get::<&comps::Sensor>(id)
			{
				self.physics.collider_set.remove(
					sensor.handle,
					&mut self.physics.island_manager,
					&mut self.physics.rigid_body_set,
					true,
				);
			}
			//println!("died {id:?}");
			self.world.despawn(id)?;
		}

		Ok(None)
	}

	fn input(
		&mut self, _event: &Event, _state: &mut game_state::GameState,
	) -> Result<Option<game_state::NextScreen>>
	{
		Ok(None)
	}

	fn make_project(&self, state: &game_state::GameState) -> Perspective3<f32>
	{
		utils::projection_transform(state.buffer_width(), state.buffer_height(), PI / 2.)
	}

	fn camera_pos(&self, alpha: f32) -> Point3<f32>
	{
		self.camera_target.draw_pos(alpha)
			+ self.camera_target.draw_rot(alpha)
				* (2. * self.camera_target.draw_scale(alpha) * -Vector3::z())
	}

	fn make_camera(&self, alpha: f32) -> Isometry3<f32>
	{
		utils::make_camera(self.camera_pos(alpha), self.camera_target.draw_pos(alpha))
	}

	fn draw(&mut self, state: &mut game_state::GameState) -> Result<()>
	{
		let project = self.make_project(state);
		let camera = self.make_camera(state.alpha);

		// Forward pass.
		state
			.core
			.use_projection_transform(&utils::mat4_to_transform(project.to_homogeneous()));
		state
			.core
			.use_transform(&utils::mat4_to_transform(camera.to_homogeneous()));
		state
			.deferred_renderer
			.as_mut()
			.unwrap()
			.begin_forward_pass(&state.core)?;
		state
			.core
			.use_shader(Some(&*state.forward_shader.upgrade().unwrap()))
			.unwrap();

		let material_mapper = |material: &scene::Material, texture_name: &str| -> Result<&Bitmap> {
			if material.desc.two_sided
			{
				unsafe {
					gl::Disable(gl::CULL_FACE);
				}
			}
			else
			{
				unsafe {
					gl::Enable(gl::CULL_FACE);
				}
			}
			if !material.desc.lightmap.is_empty()
			{
				state
					.core
					.set_shader_sampler("lightmap", state.get_bitmap(&material.desc.lightmap)?, 1)
					.ok();
			}
			state.get_bitmap(texture_name)
		};

		let shift = Isometry3 {
			translation: self.camera_pos(state.alpha).coords.into(),
			rotation: UnitQuaternion::identity(),
		}
		.to_homogeneous();

		state
			.core
			.use_transform(&utils::mat4_to_transform(camera.to_homogeneous() * shift));
		state
			.core
			.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift))
			.ok();

		state
			.get_scene("data/sky.glb")
			.unwrap()
			.draw(&state.core, &state.prim, material_mapper);

		for (_, (position, scene)) in self
			.world
			.query::<(&comps::Position, &comps::Scene)>()
			.iter()
		{
			let shift = Isometry3 {
				translation: position.draw_pos(state.alpha).coords.into(),
				rotation: position.draw_rot(state.alpha),
			}
			.to_homogeneous();
			let scale =
				Similarity3::from_scaling(position.draw_scale(state.alpha)).to_homogeneous();

			state.core.use_transform(&utils::mat4_to_transform(
				camera.to_homogeneous() * shift * scale,
			));
			state
				.core
				.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift * scale))
				.ok();

			state
				.get_scene(&scene.scene)
				.unwrap()
				.draw(&state.core, &state.prim, material_mapper);
		}

		if self.show_path
		{
			for (_, ai) in self.world.query::<&comps::AI>().iter()
			{
				if let comps::AIState::Hunt { path, .. } = &ai.state
				{
					for pos in path
					{
						let shift = Isometry3 {
							translation: pos.coords.into(),
							rotation: UnitQuaternion::identity(),
						}
						.to_homogeneous();

						state.core.use_transform(&utils::mat4_to_transform(
							camera.to_homogeneous() * shift,
						));
						state
							.core
							.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift))
							.ok();

						state.get_scene("data/sphere.glb").unwrap().draw(
							&state.core,
							&state.prim,
							material_mapper,
						);
					}
				}
			}
		}

		// Light pass.
		state.deferred_renderer.as_mut().unwrap().begin_light_pass(
			&state.core,
			state.light_shader.clone(),
			&utils::mat4_to_transform(project.to_homogeneous()),
			self.camera_pos(state.alpha),
		)?;

		for (_, (position, light)) in self
			.world
			.query::<(&comps::Position, &comps::Light)>()
			.iter()
		{
			let shift = Isometry3::new(position.draw_pos(state.alpha).coords, Vector3::zeros());
			let transform = Similarity3::from_isometry(shift, 0.5 * light.intensity.sqrt());
			let light_pos = transform.transform_point(&Point3::origin());

			let (r, g, b) = light.color.to_rgb_f();

			state
				.core
				.set_shader_uniform("light_color", &[[r, g, b, 1.0]][..])
				.ok(); //.unwrap();
			state
				.core
				.set_shader_uniform("light_pos", &[[light_pos.x, light_pos.y, light_pos.z]][..])
				.ok(); //.unwrap();
			state
				.core
				.set_shader_uniform("light_intensity", &[light.intensity][..])
				.ok(); //.unwrap();
			state
				.core
				.set_shader_uniform("is_static", &[light.static_ as i32][..])
				.ok(); //.unwrap();

			state.core.use_transform(&utils::mat4_to_transform(
				camera.to_homogeneous() * transform.to_homogeneous(),
			));

			if let Ok(scene) = state.get_scene("data/sphere.glb")
			{
				scene.draw(&state.core, &state.prim, |_, s| state.get_bitmap(s));
			}
		}

		// Final pass.
		state.deferred_renderer.as_mut().unwrap().final_pass(
			&state.core,
			&state.prim,
			state.final_shader.clone(),
			state.buffer1.as_ref().unwrap(),
		)?;

		state
			.core
			.use_shader(Some(&*state.basic_shader.upgrade().unwrap()))
			.unwrap();
		unsafe {
			gl::Disable(gl::CULL_FACE);
		}
		state.core.set_depth_test(None);
		state
			.core
			.set_blender(BlendOperation::Add, BlendMode::One, BlendMode::InverseAlpha);

		let ortho_mat =
			Matrix4::new_orthographic(0., state.buffer_width(), state.buffer_height(), 0., -1., 1.);
		state
			.core
			.use_projection_transform(&utils::mat4_to_transform(ortho_mat));

		let h = 200.;

		let mut x = state.buffer_width() / 2. - h;
		let y = 32.;

		for kind in [
			comps::AnimalKind::Rhino,
			comps::AnimalKind::Zebra,
			comps::AnimalKind::Cat,
		]
		{
			let icon = state.get_bitmap(kind.icon())?;
			state.core.draw_bitmap(
				icon,
				x,
				y + (4. * (2. * state.time() + x as f64).cos() as f32).floor(),
				Flag::zero(),
			);

			let mut count = 0;
			for (_, animal) in self.world.query::<&comps::Animal>().iter()
			{
				if animal.kind == kind
				{
					count += 1;
				}
			}

			state.core.draw_text(
				state.ui_font.as_ref().unwrap(),
				Color::from_rgb_f(1., 0.8, 1.),
				x + 48.,
				y,
				FontAlign::Centre,
				&format!("{}", count),
			);

			x += h;
		}

		if self.world.contains(self.player)
		{
			let animal = self.world.get::<&comps::Animal>(self.player)?;

			let x = 64.;
			let y = state.buffer_height() - 48.;
			let icon = state.get_bitmap("data/fat_icon.png")?;
			state.core.draw_bitmap(
				icon,
				x,
				y + (4. * (2. * state.time()).cos() as f32).floor(),
				Flag::zero(),
			);

			state.core.draw_text(
				state.ui_font.as_ref().unwrap(),
				Color::from_rgb_f(1., 0.8, 1.),
				x + 48.,
				y,
				FontAlign::Centre,
				&format!("{}", animal.fat),
			);
			let x = state.buffer_width() - 64. - 48.;
			let icon = state.get_bitmap("data/muscle_icon.png")?;
			state.core.draw_bitmap(
				icon,
				x,
				y + (4. * (2. * state.time()).cos() as f32 + 3.).floor(),
				Flag::zero(),
			);

			state.core.draw_text(
				state.ui_font.as_ref().unwrap(),
				Color::from_rgb_f(1., 0.8, 1.),
				x + 48.,
				y,
				FontAlign::Centre,
				&format!("{}", animal.muscle),
			);
		}

		for (_, message) in self.world.query::<&comps::Message>().iter()
		{
			if state.time() > message.time_to_show
			{
				let x = state.buffer_width() / 2. + 16. * state.time().cos() as f32;
				let y = state.buffer_height() / 2. + 16. * state.time().sin() as f32;

				state.core.draw_text(
					state.ui_font.as_ref().unwrap(),
					Color::from_rgb_f(1., 1., 0.8),
					x,
					y,
					FontAlign::Centre,
					&message.text,
				);
			}
		}

		Ok(())
	}
}
