use crate::error::Result;
use crate::{components, controls, game_state, ui, utils};

use allegro::*;
use allegro_font::*;
use allegro_sys::*;
use nalgebra::{Matrix4, Point2};
use rand::prelude::*;

pub struct Menu
{
	subscreens: ui::SubScreens,
}

fn to_f32(pos: Point2<i32>) -> Point2<f32>
{
	Point2::new(pos.x as f32, pos.y as f32)
}

impl Menu
{
	pub fn new(state: &mut game_state::GameState) -> Result<Self>
	{
		state.paused = false;
		state.sfx.cache_sample("data/ui1.ogg")?;
		state.sfx.cache_sample("data/ui2.ogg")?;
		state.cache_sprite("data/title.cfg")?;
		state
			.sfx
			.play_music("data/BANANAS - Menu 1.2.ogg", 0.25, &state.core);

		let mut subscreens = ui::SubScreens::new(state);
		subscreens.push(ui::SubScreen::MainMenu(ui::MainMenu::new(state)?));

		Ok(Self { subscreens })
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
		if let Some(action) = self.subscreens.input(state, event)?
		{
			match action
			{
				ui::Action::Start => return Ok(Some(game_state::NextScreen::Game)),
				ui::Action::Quit => return Ok(Some(game_state::NextScreen::Quit)),
				_ => (),
			}
		}
		Ok(None)
	}

	pub fn draw(&mut self, state: &mut game_state::GameState) -> Result<()>
	{
		state.hide_mouse = false;
		state.core.clear_to_color(Color::from_rgb_f(0., 0., 0.));
		self.subscreens.draw(state);
		if self.subscreens.subscreens.len() == 1
		{
			let sprite = state.get_sprite("data/title.cfg")?;
			sprite.draw_frame(
				Point2::new(state.buffer_width() / 2., state.buffer_height() / 2. - 100.),
				"Default",
				0,
				state,
			);
		}

		Ok(())
	}

	pub fn resize(&mut self, state: &game_state::GameState)
	{
		self.subscreens.resize(state);
	}
}
