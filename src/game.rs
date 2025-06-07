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
		if !self.subscreens.is_empty()
		{
			state.core.clear_to_color(Color::from_rgb_f(0.0, 0.0, 0.0));
			self.subscreens.draw(state);
		}
		else
		{
			self.map.draw(state)?;
		}
		Ok(())
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		self.subscreens.resize(state);
	}
}

pub fn spawn_animal(
	pos: Point3<f32>, state: &mut game_state::GameState, physics: &mut Physics,
	world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let scene_name = "data/cat.glb";
	game_state::cache_scene(state, scene_name)?;

	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Scene {
			scene: scene_name.to_string(),
		},
		comps::GroundTracker::new(),
		comps::Controller::new(),
		comps::Animal::new(),
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
	let collider = ColliderBuilder::ball(0.5)
		.restitution(0.7)
		.user_data(entity.to_bits().get() as u128)
		.active_events(ActiveEvents::COLLISION_EVENTS)
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

pub fn spawn_light(
	pos: Point3<f32>, light: comps::Light, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let entity = world.spawn((comps::Position::new(pos, UnitQuaternion::identity()), light));
	Ok(entity)
}

pub fn spawn_food(
	pos: Point3<f32>, state: &mut game_state::GameState, physics: &mut Physics,
	world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let scene_name = "data/banana.glb";
	game_state::cache_scene(state, scene_name)?;

	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Scene {
			scene: scene_name.to_string(),
		},
		comps::Light {
			intensity: 105.0,
			color: Color::from_rgb_f(0.9, 0.9, 0.0),
			static_: false,
		},
		comps::Food::new(pos),
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

pub struct PhysicsEventHandler
{
	collision_events: RwLock<Vec<(CollisionEvent, Option<ContactPair>)>>,
}

impl PhysicsEventHandler
{
	pub fn new() -> Self
	{
		Self {
			collision_events: RwLock::new(vec![]),
		}
	}
}

impl EventHandler for PhysicsEventHandler
{
	fn handle_collision_event(
		&self, _bodies: &RigidBodySet, _colliders: &ColliderSet, event: CollisionEvent,
		contact_pair: Option<&ContactPair>,
	)
	{
		let mut events = self.collision_events.write().unwrap();
		events.push((event, contact_pair.cloned()));
	}
	fn handle_contact_force_event(
		&self, _dt: f32, _bodies: &RigidBodySet, _colliders: &ColliderSet,
		_contact_pair: &ContactPair, _total_force_magnitude: f32,
	)
	{
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

struct Map
{
	world: hecs::World,
	physics: Physics,
	camera_target: comps::Position,
	player: hecs::Entity,
	level_body_handle: RigidBodyHandle,
	food_spawns: Vec<Point3<f32>>,
	animal_spawns: Vec<Point3<f32>>,
	time_to_spawn_food: Option<f64>,
	time_to_snap_camera: f64,
}

impl Map
{
	fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		let mut world = hecs::World::new();
		let mut physics = Physics::new();

		game_state::cache_scene(state, "data/level1.glb")?;
		state.cache_bitmap("data/level1_lightmap.png")?;
		game_state::cache_scene(state, "data/sphere.glb")?;

		let level_scene = state.get_scene("data/level1.glb").unwrap();

		let mut animal_spawns = vec![];
		let mut food_spawns = vec![];
		let mut vertices = vec![];
		let mut indices = vec![];
		for object in &level_scene.objects
		{
			match &object.kind
			{
				scene::ObjectKind::Light { color, intensity } =>
				{
					println!("Spawning light at {:?}", object.position);
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
			}
		}
		let rigid_body = RigidBodyBuilder::fixed().build();
		let collider = ColliderBuilder::trimesh(vertices, indices)?.build();
		let rigid_body_handle = physics.rigid_body_set.insert(rigid_body);
		physics.collider_set.insert_with_parent(
			collider,
			rigid_body_handle,
			&mut physics.rigid_body_set,
		);

		let mut player = None;
		for (i, animal_spawn) in animal_spawns.iter().enumerate()
		{
			let entity = spawn_animal(*animal_spawn, state, &mut physics, &mut world)?;
			if i == 0
			{
				player = Some(entity);
			}
			else
			{
				world.insert_one(entity, comps::AI::new())?;
			}
		}

		Ok(Self {
			world: world,
			physics: physics,
			camera_target: comps::Position::new(Point3::origin(), UnitQuaternion::identity()),
			level_body_handle: rigid_body_handle,
			player: player.unwrap(),
			animal_spawns: animal_spawns,
			food_spawns: food_spawns,
			time_to_spawn_food: Some(0.),
			time_to_snap_camera: 0.,
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

			let look_left = state.controls.get_action_state(controls::Action::LookLeft);
			let look_right = state.controls.get_action_state(controls::Action::LookRight);
			let look_up = state.controls.get_action_state(controls::Action::LookUp);
			let look_down = state.controls.get_action_state(controls::Action::LookDown);

			if look_left > 0.1 || look_right > 0.1 || look_up > 0.1 || look_down > 0.1
			{
				self.time_to_snap_camera = state.time() + 1.0;
			}

			let jump = state.controls.get_action_state(controls::Action::Jump);

			let animal = self.world.get::<&comps::Animal>(self.player).unwrap();
			let mut controller = self
				.world
				.get::<&mut comps::Controller>(self.player)
				.unwrap();
			controller.want_jump = jump > 0.5;
			controller.power = animal.size.powf(2.);

			let position = self.world.get::<&comps::Position>(self.player).unwrap();

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

			let forward2 = self.camera_target.rot * (Vector3::z());
			let forward2 = Vector3::new(forward2.x, 0., forward2.z).normalize();
			let right2 = Vector3::new(-forward2.z, 0., forward2.x).normalize();
			controller.want_move =
				(move_fwd - move_bwd) * forward2 + (move_right - move_left) * right2;

			let physics = self.world.get::<&comps::Physics>(self.player).unwrap();
			let velocity =
				self.physics.rigid_body_set[physics.handle].velocity_at_point(&position.pos);
			if velocity.xz().norm() > 0.3 && state.time() > self.time_to_snap_camera
			{
				let lagging_point =
					Vector3::new(velocity.x, 0., velocity.z).normalize() - 0.5_f32 * Vector3::y();
				let lagging_rot = UnitQuaternion::look_at_rh(&(-lagging_point), &Vector3::y());

				self.camera_target.rot = self.camera_target.rot.slerp(&lagging_rot.inverse(), DT);
			}

			self.camera_target.pos = position.pos;
			self.camera_target.scale = position.scale;
		}

		// AI.
		for (_, (position, controller, animal, _ai)) in self
			.world
			.query::<(
				&comps::Position,
				&mut comps::Controller,
				&comps::Animal,
				&comps::AI,
			)>()
			.iter()
		{
			let mut best_distance = f32::INFINITY;
			let mut best_food = None;
			for (_, (food_position, _)) in self
				.world
				.query::<(&comps::Position, &comps::Food)>()
				.iter()
			{
				let dist = (position.pos - food_position.pos).norm();
				if dist < best_distance
				{
					best_food = Some(food_position.pos);
					best_distance = dist;
				}
			}
			if let Some(best_food) = best_food
			{
				if best_distance > 0.
				{
					let diff = best_food - position.pos;
					let diff_horiz = diff.xz().normalize();
					controller.want_move = Vector3::new(diff_horiz.x, 0., diff_horiz.y);
					if best_distance < 2. && diff.y > 0.5
					{
						controller.want_jump = true;
					}
				}
			}
			else
			{
				controller.want_move = Vector3::zeros();
				controller.want_jump = false;
			}
			controller.power = animal.size.sqrt();
		}

		// Controller.
		for (_, (controller, physics, ground_tracker)) in self
			.world
			.query::<(
				&mut comps::Controller,
				&comps::Physics,
				&comps::GroundTracker,
			)>()
			.iter()
		{
			let body = self.physics.rigid_body_set.get_mut(physics.handle).unwrap();
			let force = 1. * controller.power;
			let jump_impulse = 2. * controller.power;

			body.reset_forces(true);
			body.add_force(controller.want_move * force, true);
			if ground_tracker.on_ground
			{
				body.apply_impulse(
					Vector3::new(0., controller.want_jump as i32 as f32, 0.) * jump_impulse,
					true,
				);
			}
		}

		// Physics.
		let handler = PhysicsEventHandler::new();
		self.physics.step(&handler);

		for (_, (pos, physics)) in self
			.world
			.query::<(&mut comps::Position, &comps::Physics)>()
			.iter()
		{
			let body = &self.physics.rigid_body_set[physics.handle];
			pos.pos = Point3::from(*body.translation());
			pos.rot = *body.rotation();
		}

		for (_, (physics, ground_tracker)) in self
			.world
			.query::<(&comps::Physics, &mut comps::GroundTracker)>()
			.iter()
		{
			let obj_body = &self.physics.rigid_body_set[physics.handle];
			let level_body = &self.physics.rigid_body_set[self.level_body_handle];
			let obj_collider_handle = obj_body.colliders()[0];
			let level_collider_handle = level_body.colliders()[0];

			ground_tracker.on_ground = false;
			if let Some(contact_pair) = self
				.physics
				.narrow_phase
				.contact_pair(obj_collider_handle, level_collider_handle)
			{
				if contact_pair.has_any_active_contact
				{
					ground_tracker.on_ground = true;
				}
			}
		}

		// Food.
		if let Some(time_to_spawn_food) = self.time_to_spawn_food
		{
			if state.time() > time_to_spawn_food
			{
				self.time_to_spawn_food = None;
				let mut rng = thread_rng();
				let spawn = self.food_spawns.choose(&mut rng).unwrap();
				spawn_food(*spawn, state, &mut self.physics, &mut self.world)?;
			}
		}

		for (id, (pos, food, sensor)) in self
			.world
			.query::<(&mut comps::Position, &comps::Food, &comps::Sensor)>()
			.iter()
		{
			pos.pos =
				food.spawn_pos + Vector3::new(0., 0.25 * (5. * state.time()).cos() as f32, 0.);
			pos.rot = UnitQuaternion::from_axis_angle(
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
							animal.food += 1;
							animal.new_size += 1.;
							self.time_to_spawn_food = Some(state.time() + 1.);
							break;
						}
					}
				}
			}
		}

		// Animal.
		for (_, (pos, animal, physics)) in self
			.world
			.query::<(&mut comps::Position, &mut comps::Animal, &comps::Physics)>()
			.iter()
		{
			if animal.size >= animal.new_size
			{
				animal.size = animal.new_size;
				continue;
			}

			animal.size += DT / GROW_TIME as f32;
			pos.scale = animal.size;

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

		let shift = Isometry3::new(Vector3::zeros(), Vector3::zeros()).to_homogeneous();

		state
			.core
			.use_transform(&utils::mat4_to_transform(camera.to_homogeneous() * shift));
		state
			.core
			.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift))
			.ok();

		let material_mapper = |_material: &scene::Material,
		                       texture_name: &str|
		 -> Result<&Bitmap> { state.get_bitmap(texture_name) };

		state
			.core
			.set_shader_sampler("lightmap", state.get_bitmap("data/level1_lightmap.png")?, 1)
			.ok();
		state
			.get_scene("data/level1.glb")
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
		Ok(())
	}
}
