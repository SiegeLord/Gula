use crate::error::Result;
use crate::{atlas, controls, sfx, sprite, utils};
use allegro::*;
use allegro_font::*;
use allegro_image::*;
use allegro_primitives::*;
use allegro_ttf::*;
use nalgebra::Point2;
use serde_derive::{Deserialize, Serialize};
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::{fmt, path, sync};

pub const VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Options
{
	pub fullscreen: bool,
	pub width: i32,
	pub height: i32,
	pub play_music: bool,
	pub vsync_method: i32,
	pub sfx_volume: f32,
	pub music_volume: f32,
	pub camera_speed: i32,
	pub grab_mouse: bool,
	pub ui_scale: f32,
	pub frac_scale: bool,

	pub controls: controls::Controls,
}

impl Default for Options
{
	fn default() -> Self
	{
		Self {
			fullscreen: false,
			width: 960,
			height: 864,
			play_music: true,
			vsync_method: if cfg!(target_os = "windows") { 1 } else { 2 },
			sfx_volume: 1.,
			music_volume: 1.,
			camera_speed: 4,
			grab_mouse: false,
			ui_scale: 1.,
			frac_scale: true,
			controls: controls::Controls::new_game(),
		}
	}
}

#[derive(Debug)]
pub enum NextScreen
{
	Game,
	Menu,
	InGameMenu,
	Quit,
}

pub struct GameState
{
	pub core: Core,
	pub prim: PrimitivesAddon,
	pub image: ImageAddon,
	pub font: FontAddon,
	pub ttf: TtfAddon,
	pub tick: i64,
	pub paused: bool,

	pub sfx: sfx::Sfx,
	pub atlas: atlas::Atlas,
	pub ui_font: Option<Font>,
	pub options: Options,
	bitmaps: HashMap<String, Bitmap>,
	sprites: HashMap<String, sprite::Sprite>,
	pub controls: controls::ControlsHandler,
	pub game_ui_controls: controls::ControlsHandler,
	pub menu_controls: controls::ControlsHandler,
	pub track_mouse: bool,
	pub mouse_pos: Point2<i32>,

	pub draw_scale: f32,
	pub display_width: f32,
	pub display_height: f32,
	pub buffer1: Option<Bitmap>,
	pub buffer2: Option<Bitmap>,

	pub basic_shader: sync::Weak<Shader>,

	pub alpha: f32,
}

pub fn load_options(core: &Core) -> Result<Options>
{
	Ok(utils::load_user_data(core, "options.cfg")?.unwrap_or_default())
}

pub fn save_options(core: &Core, options: &Options) -> Result<()>
{
	utils::save_user_data(core, "options.cfg", options)
}

impl GameState
{
	pub fn new() -> Result<Self>
	{
		let core = Core::init()?;
		core.set_app_name("HackInit");
		core.set_org_name("SiegeLord");

		let options = load_options(&core)?;
		let prim = PrimitivesAddon::init(&core)?;
		let image = ImageAddon::init(&core)?;
		let font = FontAddon::init(&core)?;
		let ttf = TtfAddon::init(&font)?;
		core.install_keyboard()
			.map_err(|_| "Couldn't install keyboard".to_string())?;
		core.install_mouse()
			.map_err(|_| "Couldn't install mouse".to_string())?;
		core.set_joystick_mappings("data/gamecontrollerdb.txt")
			.map_err(|_| "Couldn't set joystick mappings".to_string())?;
		core.install_joystick()
			.map_err(|_| "Couldn't install joysticks".to_string())?;

		let sfx = sfx::Sfx::new(options.sfx_volume, options.music_volume, &core)?;
		//sfx.set_music_file("data/lemonade-sinus.xm");
		//sfx.play_music()?;

		let controls = controls::ControlsHandler::new(options.controls.clone());
		Ok(Self {
			options: options,
			core: core,
			prim: prim,
			image: image,
			tick: 0,
			bitmaps: HashMap::new(),
			sprites: HashMap::new(),
			font: font,
			ttf: ttf,
			sfx: sfx,
			paused: false,
			atlas: atlas::Atlas::new(1024),
			ui_font: None,
			draw_scale: 1.,
			display_width: 0.,
			display_height: 0.,
			buffer1: None,
			buffer2: None,
			controls: controls,
			game_ui_controls: controls::ControlsHandler::new(controls::Controls::new_game_ui()),
			menu_controls: controls::ControlsHandler::new(controls::Controls::new_menu()),
			track_mouse: true,
			mouse_pos: Point2::new(0, 0),
			basic_shader: Default::default(),
			alpha: 0.,
		})
	}

	pub fn buffer1(&self) -> &Bitmap
	{
		self.buffer1.as_ref().unwrap()
	}

	pub fn buffer2(&self) -> &Bitmap
	{
		self.buffer2.as_ref().unwrap()
	}

	pub fn buffer_width(&self) -> f32
	{
		self.buffer1().get_width() as f32
	}

	pub fn buffer_height(&self) -> f32
	{
		self.buffer1().get_height() as f32
	}

	pub fn ui_font(&self) -> &Font
	{
		self.ui_font.as_ref().unwrap()
	}

	pub fn resize_display(&mut self, display: &Display) -> Result<()>
	{
		const FIXED_BUFFER: bool = true;

		let buffer_width;
		let buffer_height;
		if FIXED_BUFFER
		{
			buffer_width = 640;
			buffer_height = 480;
		}
		else
		{
			buffer_width = display.get_width();
			buffer_height = display.get_height();
		}

		self.display_width = display.get_width() as f32;
		self.display_height = display.get_height() as f32;
		self.draw_scale = utils::min(
			(display.get_width() as f32) / (buffer_width as f32),
			(display.get_height() as f32) / (buffer_height as f32),
		);
		if !self.options.frac_scale
		{
			self.draw_scale = self.draw_scale.floor();
		}

		if self.buffer1.is_none() || !FIXED_BUFFER
		{
			self.buffer1 = Some(Bitmap::new(&self.core, buffer_width, buffer_height).unwrap());
			self.buffer2 = Some(Bitmap::new(&self.core, buffer_width, buffer_height).unwrap());
		}

		self.ui_font = Some(utils::load_ttf_font(
			&self.ttf,
			"data/Energon.ttf",
			(-24. * self.options.ui_scale) as i32,
		)?);
		Ok(())
	}

	pub fn transform_mouse(&self, x: f32, y: f32) -> (f32, f32)
	{
		let x = (x - self.display_width / 2.) / self.draw_scale + self.buffer_width() / 2.;
		let y = (y - self.display_height / 2.) / self.draw_scale + self.buffer_height() / 2.;
		(x, y)
	}

	pub fn cache_bitmap<'l>(&'l mut self, name: &str) -> Result<&'l Bitmap>
	{
		Ok(match self.bitmaps.entry(name.to_string())
		{
			Entry::Occupied(o) => o.into_mut(),
			Entry::Vacant(v) => v.insert(utils::load_bitmap(&self.core, name)?),
		})
	}

	pub fn cache_sprite<'l>(&'l mut self, name: &str) -> Result<&'l sprite::Sprite>
	{
		Ok(match self.sprites.entry(name.to_string())
		{
			Entry::Occupied(o) => o.into_mut(),
			Entry::Vacant(v) => v.insert(sprite::Sprite::load(name, &self.core, &mut self.atlas)?),
		})
	}

	pub fn get_bitmap<'l>(&'l self, name: &str) -> Result<&'l Bitmap>
	{
		Ok(self
			.bitmaps
			.get(name)
			.ok_or_else(|| format!("{name} is not cached!"))?)
	}

	pub fn get_sprite<'l>(&'l self, name: &str) -> Result<&'l sprite::Sprite>
	{
		Ok(self
			.sprites
			.get(name)
			.ok_or_else(|| format!("{name} is not cached!"))?)
	}

	pub fn time(&self) -> f64
	{
		self.tick as f64 * utils::DT as f64
	}
}
