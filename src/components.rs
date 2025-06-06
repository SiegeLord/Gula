use crate::sprite;
use allegro::*;
use nalgebra::{Point3, Vector3};
use rand::prelude::*;

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

#[derive(Debug, Clone)]
pub struct Scene
{
	pub scene: String,
}

impl Position
{
	pub fn new(pos: Point3<f32>) -> Self
	{
		Self { pos, old_pos: pos }
	}

	pub fn snapshot(&mut self)
	{
		self.old_pos = self.pos;
	}

	pub fn draw_pos(&self, alpha: f32) -> Point3<f32>
	{
		self.pos + alpha * (self.pos - self.old_pos)
	}
}
