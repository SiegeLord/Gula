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
	ColliderBuilder, ColliderSet, CollisionEvent, ContactPair, DefaultBroadPhase, NarrowPhase,
};
use rapier3d::pipeline::{ActiveEvents, EventHandler, PhysicsPipeline, QueryPipeline};

use std::collections::HashMap;
use std::f32::consts::PI;
use std::sync::RwLock;

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
	));

	let rigid_body = RigidBodyBuilder::dynamic()
		.translation(pos.coords)
		.user_data(entity.to_bits().get() as u128)
		.build();
	let collider = ColliderBuilder::ball(0.5)
		.restitution(0.7)
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
	pos: Point3<f32>, state: &mut game_state::GameState, world: &mut hecs::World,
) -> Result<hecs::Entity>
{
	let scene_name = "data/banana.glb";
	game_state::cache_scene(state, scene_name)?;

	let entity = world.spawn((
		comps::Position::new(pos, UnitQuaternion::identity()),
		comps::Food::new(pos),
		comps::Scene {
			scene: scene_name.to_string(),
		},
		comps::Light {
			intensity: 105.0,
			color: Color::from_rgb_f(0.9, 0.9, 0.0),
			static_: false,
		},
	));
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
				dt: DT as f32,
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
	camera_target: Point3<f32>,
	player: hecs::Entity,
	level_body_handle: RigidBodyHandle,
}

impl Map
{
	fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		let mut world = hecs::World::new();
		let mut physics = Physics::new();
		let player = spawn_animal(Point3::new(0., 1., -5.), state, &mut physics, &mut world)?;

		let animal = spawn_animal(Point3::new(2., 1., -5.), state, &mut physics, &mut world)?;
		world.insert_one(animal, comps::AI::new())?;
		let animal = spawn_animal(Point3::new(3., 1., -5.), state, &mut physics, &mut world)?;
		world.insert_one(animal, comps::AI::new())?;

		spawn_food(Point3::new(3., 1., 1.), state, &mut world)?;
		spawn_food(Point3::new(6., 1., 1.), state, &mut world)?;

		game_state::cache_scene(state, "data/level1.glb")?;
		state.cache_bitmap("data/level1_lightmap.png")?;
		game_state::cache_scene(state, "data/sphere.glb")?;

		let level_scene = state.get_scene("data/level1.glb").unwrap();
		let mut vertices = vec![];
		let mut indices = vec![];
		for object in &level_scene.objects
		{
			if let scene::ObjectKind::Light { color, intensity } = object.kind
			{
				println!("Spawning light at {:?}", object.position);
				spawn_light(
					object.position,
					comps::Light {
						color: color,
						intensity: intensity / 50.,
						static_: true,
					},
					&mut world,
				)?;
			}
			else if let scene::ObjectKind::MultiMesh { meshes } = &object.kind
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
		}
		let rigid_body = RigidBodyBuilder::fixed().build();
		let collider = ColliderBuilder::trimesh(vertices, indices)?.build();
		let rigid_body_handle = physics.rigid_body_set.insert(rigid_body);
		physics.collider_set.insert_with_parent(
			collider,
			rigid_body_handle,
			&mut physics.rigid_body_set,
		);

		Ok(Self {
			world: world,
			physics: physics,
			camera_target: Point3::origin(),
			level_body_handle: rigid_body_handle,
			player: player,
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

		// Player.
		if self.world.contains(self.player)
		{
			let left = state.controls.get_action_state(controls::Action::Left);
			let right = state.controls.get_action_state(controls::Action::Right);
			let up = state.controls.get_action_state(controls::Action::Up);
			let down = state.controls.get_action_state(controls::Action::Down);
			let jump = state.controls.get_action_state(controls::Action::Jump);

			let mut controller = self
				.world
				.get::<&mut comps::Controller>(self.player)
				.unwrap();
			controller.want_move = Vector3::new(left - right, 0., up - down);
			controller.want_jump = jump > 0.5;
		}

		// AI.
		for (_, (position, controller, _ai)) in self
			.world
			.query::<(&comps::Position, &mut comps::Controller, &comps::AI)>()
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
					let diff = (best_food - position.pos).xz().normalize();
					controller.want_move = Vector3::new(diff.x, 0., diff.y)
				}
			}
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
			let force = 1.;
			let jump_impulse = 2.;

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
		for (_, (pos, food)) in self
			.world
			.query::<(&mut comps::Position, &comps::Food)>()
			.iter()
		{
			pos.pos =
				food.spawn_pos + Vector3::new(0., 0.25 * (5. * state.time()).cos() as f32, 0.);
			pos.rot = UnitQuaternion::from_axis_angle(
				&Unit::new_normalize(Vector3::y()),
				((5. * state.time()) % (2. * PI as f64)) as f32,
			);
		}

		// Remove dead entities
		to_die.sort();
		to_die.dedup();
		for id in to_die
		{
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
		utils::projection_transform(state.buffer_width(), state.buffer_height(), PI / 3.)
	}

	fn camera_pos(&self) -> Point3<f32>
	{
		self.camera_target + Vector3::new(0., 2., -2.)
	}

	fn make_camera(&self) -> Isometry3<f32>
	{
		utils::make_camera(self.camera_pos(), self.camera_target)
	}

	fn draw(&mut self, state: &mut game_state::GameState) -> Result<()>
	{
		if self.world.contains(self.player)
		{
			{
				let position = self.world.get::<&comps::Position>(self.player).unwrap();
				self.camera_target = position.draw_pos(state.alpha);
			}
		}

		let project = self.make_project(state);
		let camera = self.make_camera();

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

			state
				.core
				.use_transform(&utils::mat4_to_transform(camera.to_homogeneous() * shift));
			state
				.core
				.set_shader_transform("model_matrix", &utils::mat4_to_transform(shift))
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
			self.camera_pos(),
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
