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
	pub power: f32,
}

impl Controller
{
	pub fn new() -> Self
	{
		Self {
			want_move: Vector3::zeros(),
			want_jump: false,
			power: 1.0,
		}
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Animal
{
	pub food: i32,
	pub size: f32,
	pub new_size: f32,
}

impl Animal
{
	pub fn new() -> Self
	{
		Self {
			food: 0,
			size: 1.,
			new_size: 1.,
		}
	}
}

#[derive(Debug, Copy, Clone)]
pub struct AI {}

impl AI
{
	pub fn new() -> Self
	{
		Self {}
	}
}

#[derive(Debug, Copy, Clone)]
pub struct Food
{
	pub spawn_pos: Point3<f32>,
}

impl Food
{
	pub fn new(spawn_pos: Point3<f32>) -> Self
	{
		Self {
			spawn_pos: spawn_pos,
		}
	}
}
