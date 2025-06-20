use crate::sprite;
use allegro::*;
use nalgebra::{Point3, Quaternion, Unit, Vector3};
use rand::prelude::*;
use rapier3d::dynamics::RigidBodyHandle;
use rapier3d::geometry::ColliderHandle;

#[derive(Debug, Copy, Clone)]
pub struct Light
{
	pub color: Color,
	pub intensity: f32,
	pub static_: bool,
}

#[derive(Debug, Copy, Clone)]
pub struct Position
{
	pub pos: Point3<f32>,
	old_pos: Point3<f32>,

	pub rot: Unit<Quaternion<f32>>,
	old_rot: Unit<Quaternion<f32>>,

	pub scale: f32,
	old_scale: f32,
}

impl Position
{
	pub fn new(pos: Point3<f32>, rot: Unit<Quaternion<f32>>) -> Self
	{
		Self {
			pos,
			old_pos: pos,
			rot,
			old_rot: rot,
			scale: 1.,
			old_scale: 1.,
		}
	}

	pub fn snapshot(&mut self)
	{
		self.old_pos = self.pos;
		self.old_rot = self.rot;
		self.old_scale = self.scale;
	}

	pub fn draw_pos(&self, alpha: f32) -> Point3<f32>
	{
		self.pos + alpha * (self.pos - self.old_pos)
	}

	pub fn draw_rot(&self, alpha: f32) -> Unit<Quaternion<f32>>
	{
		self.old_rot.slerp(&self.rot, alpha)
	}

	pub fn draw_scale(&self, alpha: f32) -> f32
	{
		self.scale + alpha * (self.scale - self.old_scale)
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Velocity
{
	pub pos: Vector3<f32>,
}

impl Velocity
{
	pub fn new() -> Self
	{
		Self {
			pos: Vector3::zeros(),
		}
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Physics
{
	pub handle: RigidBodyHandle,
}

#[derive(Debug, Copy, Clone)]
pub struct Sensor
{
	pub handle: ColliderHandle,
}

#[derive(Debug, Clone)]
pub struct Scene
{
	pub scene: String,
}

#[derive(Debug, Copy, Clone)]
pub struct GroundTracker
{
	pub on_ground: bool,
}

impl GroundTracker
{
	pub fn new() -> Self
	{
		Self { on_ground: false }
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Controller
{
	pub want_move: Vector3<f32>,
	pub want_jump: bool,
	pub want_reproduce: bool,
	pub power: f32,
}

impl Controller
{
	pub fn new() -> Self
	{
		Self {
			want_move: Vector3::zeros(),
			want_jump: false,
			want_reproduce: false,
			power: 1.0,
		}
	}
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum AnimalKind
{
	Cat,
	Zebra,
	Rhino,
}

impl AnimalKind
{
	pub fn icon(&self) -> &str
	{
		match self
		{
			AnimalKind::Cat => "data/cat_icon.png",
			AnimalKind::Zebra => "data/zebra_icon.png",
			AnimalKind::Rhino => "data/rhino_icon.png",
		}
	}

	pub fn jump_sound(&self) -> &str
	{
		match self
		{
			AnimalKind::Cat => "data/cat_jump.ogg",
			AnimalKind::Zebra => "data/zebra_jump.ogg",
			AnimalKind::Rhino => "data/rhino_jump.ogg",
		}
	}

	pub fn victory_message(&self) -> &str
	{
		match self
		{
			AnimalKind::Cat => "Victory! The lions are all the prey!",
			AnimalKind::Zebra => "Victory! The zebras dazzled everyone!",
			AnimalKind::Rhino => "Victory! The rhinos flipped the opposition!",
		}
	}

	pub fn defeat_message(&self) -> &str
	{
		match self
		{
			AnimalKind::Cat => "Defeat! The lions roared their last roar!",
			AnimalKind::Zebra => "Defeat! No more running for the zebras!",
			AnimalKind::Rhino => "Defeat! No more horns for the rhinos!",
		}
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Animal
{
	pub fat: i32,
	pub muscle: i32,
	pub size: f32,
	pub new_size: f32,
	pub kind: AnimalKind,
}

impl Animal
{
	pub fn new(kind: AnimalKind) -> Self
	{
		Self {
			fat: 0,
			muscle: 0,
			size: 1.,
			new_size: 1.,
			kind: kind,
		}
	}
}

#[derive(Debug, Clone)]
pub enum AIState
{
	Idle,
	Hunt
	{
		path: Vec<Point3<f32>>,
		target: hecs::Entity,
	},
}

#[derive(Debug, Clone)]
pub struct AI
{
	pub state: AIState,
	pub time_to_replan: f64,
	pub reproduce_threshold: f32,
}

impl AI
{
	pub fn new() -> Self
	{
		let mut rng = thread_rng();
		Self {
			state: AIState::Idle,
			time_to_replan: 0.,
			reproduce_threshold: rng.gen_range(1.0..3.05),
		}
	}
}

#[derive(Debug, Copy, Clone)]
pub enum FoodKind
{
	Muscle,
	Fat,
}

impl FoodKind
{
	pub fn color(&self) -> Color
	{
		match self
		{
			FoodKind::Muscle => Color::from_rgb_f(0.9, 0.9, 0.0),
			FoodKind::Fat => Color::from_rgb_f(0.9, 0.0, 0.0),
		}
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Food
{
	pub spawn_pos: Point3<f32>,
	pub kind: FoodKind,
}

impl Food
{
	pub fn new(spawn_pos: Point3<f32>, kind: FoodKind) -> Self
	{
		Self {
			spawn_pos: spawn_pos,
			kind: kind,
		}
	}
}

pub struct FoodSpawner
{
	pub food: hecs::Entity,
	pub time_to_spawn: f64,
	pub waiting: bool,
}

impl FoodSpawner
{
	pub fn new() -> Self
	{
		Self {
			food: hecs::Entity::DANGLING,
			time_to_spawn: 0.0,
			waiting: true,
		}
	}
}

#[derive(Debug, Clone)]
pub struct Message
{
	pub text: String,
	pub time_to_show: f64,
}

impl Message
{
	pub fn new(text: &str, time_to_show: f64) -> Self
	{
		Self {
			text: text.into(),
			time_to_show: time_to_show,
		}
	}
}
