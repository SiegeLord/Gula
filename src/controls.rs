use serde_derive::{Deserialize, Serialize};
use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap, VecDeque};
use std::fmt;

use crate::utils;

#[derive(PartialEq, Eq, Hash, Serialize, Deserialize, Copy, Clone, Debug, PartialOrd, Ord)]
pub enum Action
{
	MoveLeft,
	MoveRight,
	MoveForward,
	MoveBackward,
	LookLeft,
	LookRight,
	LookUp,
	LookDown,
	Jump,
	Reproduce,
	UILeft,
	UIRight,
	UIUp,
	UIDown,
	UIAccept,
	UICancel,
}

impl Action
{
	pub fn to_str(&self) -> &'static str
	{
		match self
		{
			Action::MoveLeft => "Move Left",
			Action::MoveRight => "Move Right",
			Action::MoveForward => "Move Forward",
			Action::MoveBackward => "Move Backward",
			Action::LookLeft => "Look Left",
			Action::LookRight => "Look Right",
			Action::LookUp => "Look Up",
			Action::LookDown => "Look Down",
			Action::Jump => "Jump",
			Action::Reproduce => "Reproduce",
			Action::UILeft => "UI Left",
			Action::UIRight => "UI Right",
			Action::UIUp => "UI Up",
			Action::UIDown => "UI Down",
			Action::UIAccept => "UI Accept",
			Action::UICancel => "UI Cancel",
		}
	}
}

#[derive(PartialEq, Eq, Hash, Copy, Clone, Debug, PartialOrd, Ord)]
pub enum Input
{
	Keyboard(allegro::KeyCode),
	JoystickPosAxis(allegro::JoystickStick, i32),
	JoystickNegAxis(allegro::JoystickStick, i32),
	JoystickButton(allegro::JoystickButton),

	MouseButton(i32),
	MouseXPos,
	MouseYPos,
	MouseZPos,
	MouseXNeg,
	MouseYNeg,
	MouseZNeg,
}

impl Input
{
	pub fn to_str(&self) -> &'static str
	{
		match self
		{
			Input::Keyboard(k) => match k
			{
				allegro::KeyCode::A => "A",
				allegro::KeyCode::B => "B",
				allegro::KeyCode::C => "C",
				allegro::KeyCode::D => "D",
				allegro::KeyCode::E => "E",
				allegro::KeyCode::F => "F",
				allegro::KeyCode::G => "G",
				allegro::KeyCode::H => "H",
				allegro::KeyCode::I => "I",
				allegro::KeyCode::J => "J",
				allegro::KeyCode::K => "K",
				allegro::KeyCode::L => "L",
				allegro::KeyCode::M => "M",
				allegro::KeyCode::N => "N",
				allegro::KeyCode::O => "O",
				allegro::KeyCode::P => "P",
				allegro::KeyCode::Q => "Q",
				allegro::KeyCode::R => "R",
				allegro::KeyCode::S => "S",
				allegro::KeyCode::T => "T",
				allegro::KeyCode::U => "U",
				allegro::KeyCode::V => "V",
				allegro::KeyCode::W => "W",
				allegro::KeyCode::X => "X",
				allegro::KeyCode::Y => "Y",
				allegro::KeyCode::Z => "Z",
				allegro::KeyCode::_0 => "0",
				allegro::KeyCode::_1 => "1",
				allegro::KeyCode::_2 => "2",
				allegro::KeyCode::_3 => "3",
				allegro::KeyCode::_4 => "4",
				allegro::KeyCode::_5 => "5",
				allegro::KeyCode::_6 => "6",
				allegro::KeyCode::_7 => "7",
				allegro::KeyCode::_8 => "8",
				allegro::KeyCode::_9 => "9",
				allegro::KeyCode::Pad0 => "Pad0",
				allegro::KeyCode::Pad1 => "Pad1",
				allegro::KeyCode::Pad2 => "Pad2",
				allegro::KeyCode::Pad3 => "Pad3",
				allegro::KeyCode::Pad4 => "Pad4",
				allegro::KeyCode::Pad5 => "Pad5",
				allegro::KeyCode::Pad6 => "Pad6",
				allegro::KeyCode::Pad7 => "Pad7",
				allegro::KeyCode::Pad8 => "Pad8",
				allegro::KeyCode::Pad9 => "Pad9",
				allegro::KeyCode::F1 => "F1",
				allegro::KeyCode::F2 => "F2",
				allegro::KeyCode::F3 => "F3",
				allegro::KeyCode::F4 => "F4",
				allegro::KeyCode::F5 => "F5",
				allegro::KeyCode::F6 => "F6",
				allegro::KeyCode::F7 => "F7",
				allegro::KeyCode::F8 => "F8",
				allegro::KeyCode::F9 => "F9",
				allegro::KeyCode::F10 => "F10",
				allegro::KeyCode::F11 => "F11",
				allegro::KeyCode::F12 => "F12",
				allegro::KeyCode::Escape => "Escape",
				allegro::KeyCode::Tilde => "Tilde",
				allegro::KeyCode::Minus => "Minus",
				allegro::KeyCode::Equals => "Equals",
				allegro::KeyCode::Backspace => "Backspace",
				allegro::KeyCode::Tab => "Tab",
				allegro::KeyCode::Openbrace => "Openbrace",
				allegro::KeyCode::Closebrace => "Closebrace",
				allegro::KeyCode::Enter => "Enter",
				allegro::KeyCode::Semicolon => "Semicolon",
				allegro::KeyCode::Quote => "Quote",
				allegro::KeyCode::Backslash => "Backslash",
				allegro::KeyCode::Backslash2 => "Backslash2",
				allegro::KeyCode::Comma => "Comma",
				allegro::KeyCode::Fullstop => "Fullstop",
				allegro::KeyCode::Slash => "Slash",
				allegro::KeyCode::Space => "Space",
				allegro::KeyCode::Insert => "Insert",
				allegro::KeyCode::Delete => "Delete",
				allegro::KeyCode::Home => "Home",
				allegro::KeyCode::End => "End",
				allegro::KeyCode::PgUp => "PgUp",
				allegro::KeyCode::PgDn => "PgDn",
				allegro::KeyCode::Left => "Left",
				allegro::KeyCode::Right => "Right",
				allegro::KeyCode::Up => "Up",
				allegro::KeyCode::Down => "Down",
				allegro::KeyCode::PadSlash => "PadSlash",
				allegro::KeyCode::PadAsterisk => "PadAsterisk",
				allegro::KeyCode::PadMinus => "PadMinus",
				allegro::KeyCode::PadPlus => "PadPlus",
				allegro::KeyCode::PadDelete => "PadDelete",
				allegro::KeyCode::PadEnter => "PadEnter",
				allegro::KeyCode::PrintScreen => "PrintScreen",
				allegro::KeyCode::Pause => "Pause",
				allegro::KeyCode::AbntC1 => "AbntC1",
				allegro::KeyCode::Yen => "Yen",
				allegro::KeyCode::Kana => "Kana",
				allegro::KeyCode::Convert => "Convert",
				allegro::KeyCode::NoConvert => "NoConvert",
				allegro::KeyCode::At => "At",
				allegro::KeyCode::Circumflex => "Circumflex",
				allegro::KeyCode::Colon2 => "Colon2",
				allegro::KeyCode::Kanji => "Kanji",
				allegro::KeyCode::PadEquals => "PadEquals",
				allegro::KeyCode::Backquote => "Backquote",
				allegro::KeyCode::Semicolon2 => "Semicolon2",
				allegro::KeyCode::Command => "Command",
				allegro::KeyCode::Unknown => "Unknown",
				allegro::KeyCode::LShift => "LShift",
				allegro::KeyCode::RShift => "RShift",
				allegro::KeyCode::LCtrl => "LCtrl",
				allegro::KeyCode::RCtrl => "RCtrl",
				allegro::KeyCode::Alt => "Alt",
				allegro::KeyCode::AltGr => "AltGr",
				allegro::KeyCode::LWin => "LWin",
				allegro::KeyCode::RWin => "RWin",
				allegro::KeyCode::Menu => "Menu",
				allegro::KeyCode::ScrollLock => "ScrollLock",
				allegro::KeyCode::NumLock => "NumLock",
				allegro::KeyCode::CapsLock => "CapsLock",
			},
			Input::MouseButton(b) => match b
			{
				0 => "Mouse0",
				1 => "Mouse Left",
				2 => "Mouse Right",
				3 => "Mouse Middle",
				4 => "Mouse4",
				5 => "Mouse5",
				6 => "Mouse6",
				7 => "Mouse7",
				8 => "Mouse8",
				9 => "Mouse9",
				10 => "Mouse10",
				11 => "Mouse11",
				12 => "Mouse12",
				13 => "Mouse13",
				14 => "Mouse14",
				15 => "Mouse15",
				16 => "Mouse16",
				17 => "Mouse17",
				18 => "Mouse18",
				b => panic!("button too high: {b}"),
			},
			Input::JoystickButton(b) => match b
			{
				allegro::JoystickButton::A => "A Btn",
				allegro::JoystickButton::B => "B Btn",
				allegro::JoystickButton::X => "X Btn",
				allegro::JoystickButton::Y => "Y Btn",
				allegro::JoystickButton::LeftShoulder => "Left Shoulder",
				allegro::JoystickButton::RightShoulder => "Right Shoulder",
				allegro::JoystickButton::Back => "Back",
				allegro::JoystickButton::Start => "Start",
				allegro::JoystickButton::Guide => "Guide",
				allegro::JoystickButton::LeftThumb => "Left Thumb",
				allegro::JoystickButton::RightThumb => "Right Thumb",
				_ => panic!("Joystick button not supported: {b:?}"),
			},
			Input::JoystickPosAxis(stick, axis) => match (stick, axis)
			{
				(allegro::JoystickStick::DPad, 0) => "DPad X+",
				(allegro::JoystickStick::DPad, 1) => "DPad Y+",
				(allegro::JoystickStick::LeftThumb, 0) => "Left Thumb X+",
				(allegro::JoystickStick::LeftThumb, 1) => "Left Thumb Y+",
				(allegro::JoystickStick::RightThumb, 0) => "Right Thumb X+",
				(allegro::JoystickStick::RightThumb, 1) => "Right Thumb Y+",
				(allegro::JoystickStick::LeftTrigger, 0) => "Left Trigger",
				(allegro::JoystickStick::RightTrigger, 0) => "Right Trigger",
				_ => panic!("Joystick axis not supported: {stick:?} {axis:?}"),
			},
			Input::JoystickNegAxis(stick, axis) => match (stick, axis)
			{
				(allegro::JoystickStick::DPad, 0) => "DPad X-",
				(allegro::JoystickStick::DPad, 1) => "DPad Y-",
				(allegro::JoystickStick::LeftThumb, 0) => "Left Thumb X-",
				(allegro::JoystickStick::LeftThumb, 1) => "Left Thumb Y-",
				(allegro::JoystickStick::RightThumb, 0) => "Right Thumb X-",
				(allegro::JoystickStick::RightThumb, 1) => "Right Thumb Y-",
				_ => panic!("Joystick axis not supported: {stick:?} {axis:?}"),
			},
			Input::MouseXNeg => "MouseX-",
			Input::MouseYNeg => "MouseY-",
			Input::MouseZNeg => "MouseZ-",
			Input::MouseXPos => "MouseX+",
			Input::MouseYPos => "MouseY+",
			Input::MouseZPos => "MouseZ+",
		}
	}

	pub fn from_str(s: &str) -> Option<Self>
	{
		let mut input = match s
		{
			"A" => Some(allegro::KeyCode::A),
			"B" => Some(allegro::KeyCode::B),
			"C" => Some(allegro::KeyCode::C),
			"D" => Some(allegro::KeyCode::D),
			"E" => Some(allegro::KeyCode::E),
			"F" => Some(allegro::KeyCode::F),
			"G" => Some(allegro::KeyCode::G),
			"H" => Some(allegro::KeyCode::H),
			"I" => Some(allegro::KeyCode::I),
			"J" => Some(allegro::KeyCode::J),
			"K" => Some(allegro::KeyCode::K),
			"L" => Some(allegro::KeyCode::L),
			"M" => Some(allegro::KeyCode::M),
			"N" => Some(allegro::KeyCode::N),
			"O" => Some(allegro::KeyCode::O),
			"P" => Some(allegro::KeyCode::P),
			"Q" => Some(allegro::KeyCode::Q),
			"R" => Some(allegro::KeyCode::R),
			"S" => Some(allegro::KeyCode::S),
			"T" => Some(allegro::KeyCode::T),
			"U" => Some(allegro::KeyCode::U),
			"V" => Some(allegro::KeyCode::V),
			"W" => Some(allegro::KeyCode::W),
			"X" => Some(allegro::KeyCode::X),
			"Y" => Some(allegro::KeyCode::Y),
			"Z" => Some(allegro::KeyCode::Z),
			"0" => Some(allegro::KeyCode::_0),
			"1" => Some(allegro::KeyCode::_1),
			"2" => Some(allegro::KeyCode::_2),
			"3" => Some(allegro::KeyCode::_3),
			"4" => Some(allegro::KeyCode::_4),
			"5" => Some(allegro::KeyCode::_5),
			"6" => Some(allegro::KeyCode::_6),
			"7" => Some(allegro::KeyCode::_7),
			"8" => Some(allegro::KeyCode::_8),
			"9" => Some(allegro::KeyCode::_9),
			"Pad0" => Some(allegro::KeyCode::Pad0),
			"Pad1" => Some(allegro::KeyCode::Pad1),
			"Pad2" => Some(allegro::KeyCode::Pad2),
			"Pad3" => Some(allegro::KeyCode::Pad3),
			"Pad4" => Some(allegro::KeyCode::Pad4),
			"Pad5" => Some(allegro::KeyCode::Pad5),
			"Pad6" => Some(allegro::KeyCode::Pad6),
			"Pad7" => Some(allegro::KeyCode::Pad7),
			"Pad8" => Some(allegro::KeyCode::Pad8),
			"Pad9" => Some(allegro::KeyCode::Pad9),
			"F1" => Some(allegro::KeyCode::F1),
			"F2" => Some(allegro::KeyCode::F2),
			"F3" => Some(allegro::KeyCode::F3),
			"F4" => Some(allegro::KeyCode::F4),
			"F5" => Some(allegro::KeyCode::F5),
			"F6" => Some(allegro::KeyCode::F6),
			"F7" => Some(allegro::KeyCode::F7),
			"F8" => Some(allegro::KeyCode::F8),
			"F9" => Some(allegro::KeyCode::F9),
			"F10" => Some(allegro::KeyCode::F10),
			"F11" => Some(allegro::KeyCode::F11),
			"F12" => Some(allegro::KeyCode::F12),
			"Escape" => Some(allegro::KeyCode::Escape),
			"Tilde" => Some(allegro::KeyCode::Tilde),
			"Minus" => Some(allegro::KeyCode::Minus),
			"Equals" => Some(allegro::KeyCode::Equals),
			"Backspace" => Some(allegro::KeyCode::Backspace),
			"Tab" => Some(allegro::KeyCode::Tab),
			"Openbrace" => Some(allegro::KeyCode::Openbrace),
			"Closebrace" => Some(allegro::KeyCode::Closebrace),
			"Enter" => Some(allegro::KeyCode::Enter),
			"Semicolon" => Some(allegro::KeyCode::Semicolon),
			"Quote" => Some(allegro::KeyCode::Quote),
			"Backslash" => Some(allegro::KeyCode::Backslash),
			"Backslash2" => Some(allegro::KeyCode::Backslash2),
			"Comma" => Some(allegro::KeyCode::Comma),
			"Fullstop" => Some(allegro::KeyCode::Fullstop),
			"Slash" => Some(allegro::KeyCode::Slash),
			"Space" => Some(allegro::KeyCode::Space),
			"Insert" => Some(allegro::KeyCode::Insert),
			"Delete" => Some(allegro::KeyCode::Delete),
			"Home" => Some(allegro::KeyCode::Home),
			"End" => Some(allegro::KeyCode::End),
			"PgUp" => Some(allegro::KeyCode::PgUp),
			"PgDn" => Some(allegro::KeyCode::PgDn),
			"Left" => Some(allegro::KeyCode::Left),
			"Right" => Some(allegro::KeyCode::Right),
			"Up" => Some(allegro::KeyCode::Up),
			"Down" => Some(allegro::KeyCode::Down),
			"PadSlash" => Some(allegro::KeyCode::PadSlash),
			"PadAsterisk" => Some(allegro::KeyCode::PadAsterisk),
			"PadMinus" => Some(allegro::KeyCode::PadMinus),
			"PadPlus" => Some(allegro::KeyCode::PadPlus),
			"PadDelete" => Some(allegro::KeyCode::PadDelete),
			"PadEnter" => Some(allegro::KeyCode::PadEnter),
			"PrintScreen" => Some(allegro::KeyCode::PrintScreen),
			"Pause" => Some(allegro::KeyCode::Pause),
			"AbntC1" => Some(allegro::KeyCode::AbntC1),
			"Yen" => Some(allegro::KeyCode::Yen),
			"Kana" => Some(allegro::KeyCode::Kana),
			"Convert" => Some(allegro::KeyCode::Convert),
			"NoConvert" => Some(allegro::KeyCode::NoConvert),
			"At" => Some(allegro::KeyCode::At),
			"Circumflex" => Some(allegro::KeyCode::Circumflex),
			"Colon2" => Some(allegro::KeyCode::Colon2),
			"Kanji" => Some(allegro::KeyCode::Kanji),
			"PadEquals" => Some(allegro::KeyCode::PadEquals),
			"Backquote" => Some(allegro::KeyCode::Backquote),
			"Semicolon2" => Some(allegro::KeyCode::Semicolon2),
			"Command" => Some(allegro::KeyCode::Command),
			"Unknown" => Some(allegro::KeyCode::Unknown),
			"LShift" => Some(allegro::KeyCode::LShift),
			"RShift" => Some(allegro::KeyCode::RShift),
			"LCtrl" => Some(allegro::KeyCode::LCtrl),
			"RCtrl" => Some(allegro::KeyCode::RCtrl),
			"Alt" => Some(allegro::KeyCode::Alt),
			"AltGr" => Some(allegro::KeyCode::AltGr),
			"LWin" => Some(allegro::KeyCode::LWin),
			"RWin" => Some(allegro::KeyCode::RWin),
			"Menu" => Some(allegro::KeyCode::Menu),
			"ScrollLock" => Some(allegro::KeyCode::ScrollLock),
			"NumLock" => Some(allegro::KeyCode::NumLock),
			"CapsLock" => Some(allegro::KeyCode::CapsLock),
			_ => None,
		}
		.map(Input::Keyboard);

		if input.is_none()
		{
			input = match s
			{
				"Mouse0" => Some(0),
				"Mouse Left" => Some(1),
				"Mouse Right" => Some(2),
				"Mouse Middle" => Some(3),
				"Mouse4" => Some(4),
				"Mouse5" => Some(5),
				"Mouse6" => Some(6),
				"Mouse7" => Some(7),
				"Mouse8" => Some(8),
				"Mouse9" => Some(9),
				"Mouse10" => Some(10),
				"Mouse11" => Some(11),
				"Mouse12" => Some(12),
				"Mouse13" => Some(13),
				"Mouse14" => Some(14),
				"Mouse15" => Some(15),
				"Mouse16" => Some(16),
				"Mouse17" => Some(17),
				"Mouse18" => Some(18),
				_ => None,
			}
			.map(Input::MouseButton);
		}

		if input.is_none()
		{
			input = match s
			{
				"MouseX-" => Some(Input::MouseXNeg),
				"MouseY-" => Some(Input::MouseYNeg),
				"MouseZ-" => Some(Input::MouseZNeg),
				"MouseX+" => Some(Input::MouseXPos),
				"MouseY+" => Some(Input::MouseYPos),
				"MouseZ+" => Some(Input::MouseZPos),
				_ => None,
			}
		}

		if input.is_none()
		{
			input = match s
			{
				"A Btn" => Some(Input::JoystickButton(allegro::JoystickButton::A)),
				"B Btn" => Some(Input::JoystickButton(allegro::JoystickButton::B)),
				"X Btn" => Some(Input::JoystickButton(allegro::JoystickButton::X)),
				"Y Btn" => Some(Input::JoystickButton(allegro::JoystickButton::Y)),
				"Left Shoulder" =>
				{
					Some(Input::JoystickButton(allegro::JoystickButton::LeftShoulder))
				}
				"Right Shoulder" => Some(Input::JoystickButton(
					allegro::JoystickButton::RightShoulder,
				)),
				"Back" => Some(Input::JoystickButton(allegro::JoystickButton::Back)),
				"Start" => Some(Input::JoystickButton(allegro::JoystickButton::Start)),
				"Guide" => Some(Input::JoystickButton(allegro::JoystickButton::Guide)),
				"Left Thumb" => Some(Input::JoystickButton(allegro::JoystickButton::LeftThumb)),
				"Right Thumb" => Some(Input::JoystickButton(allegro::JoystickButton::RightThumb)),
				_ => None,
			}
		}

		if input.is_none()
		{
			input = match s
			{
				"DPad X+" => Some(Input::JoystickPosAxis(allegro::JoystickStick::DPad, 0)),
				"DPad Y+" => Some(Input::JoystickPosAxis(allegro::JoystickStick::DPad, 1)),
				"Left Thumb X+" =>
				{
					Some(Input::JoystickPosAxis(allegro::JoystickStick::LeftThumb, 0))
				}
				"Left Thumb Y+" =>
				{
					Some(Input::JoystickPosAxis(allegro::JoystickStick::LeftThumb, 1))
				}
				"Right Thumb X+" => Some(Input::JoystickPosAxis(
					allegro::JoystickStick::RightThumb,
					0,
				)),
				"Right Thumb Y+" => Some(Input::JoystickPosAxis(
					allegro::JoystickStick::RightThumb,
					1,
				)),
				"Left Trigger" => Some(Input::JoystickPosAxis(
					allegro::JoystickStick::LeftTrigger,
					0,
				)),
				"Right Trigger" => Some(Input::JoystickPosAxis(
					allegro::JoystickStick::RightTrigger,
					0,
				)),
				_ => None,
			}
		}

		if input.is_none()
		{
			input = match s
			{
				"DPad X-" => Some(Input::JoystickNegAxis(allegro::JoystickStick::DPad, 0)),
				"DPad Y-" => Some(Input::JoystickNegAxis(allegro::JoystickStick::DPad, 1)),
				"Left Thumb X-" =>
				{
					Some(Input::JoystickNegAxis(allegro::JoystickStick::LeftThumb, 0))
				}
				"Left Thumb Y-" =>
				{
					Some(Input::JoystickNegAxis(allegro::JoystickStick::LeftThumb, 1))
				}
				"Right Thumb X-" => Some(Input::JoystickNegAxis(
					allegro::JoystickStick::RightThumb,
					0,
				)),
				"Right Thumb Y-" => Some(Input::JoystickNegAxis(
					allegro::JoystickStick::RightThumb,
					1,
				)),
				_ => None,
			}
		}

		input
	}
}

impl serde::Serialize for Input
{
	fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
	where
		S: serde::Serializer,
	{
		serializer.serialize_str(self.to_str())
	}
}

struct InputVisitor;

impl<'de> serde::de::Visitor<'de> for InputVisitor
{
	fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result
	{
		write!(formatter, "an Input")
	}

	type Value = Input;
	fn visit_str<E>(self, value: &str) -> Result<Self::Value, E>
	where
		E: serde::de::Error,
	{
		Input::from_str(&value).ok_or(serde::de::Error::invalid_value(
			serde::de::Unexpected::Str(value),
			&self,
		))
	}
}

impl<'de> serde::Deserialize<'de> for Input
{
	fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
	where
		D: serde::Deserializer<'de>,
	{
		deserializer.deserialize_str(InputVisitor)
	}
}

#[derive(Debug, Clone)]
struct InputState
{
	strength: f32,
	queue: Vec<f32>,
}

impl InputState
{
	fn new() -> Self
	{
		Self {
			strength: 0.,
			queue: Vec::new(),
		}
	}

	fn push(&mut self, strength: f32)
	{
		self.queue.push(strength);
	}

	fn get(&mut self) -> f32
	{
		if self.queue.is_empty()
		{
			self.strength
		}
		else
		{
			let mut current = 0.;
			let mut previous = 0.;
			for event in self.queue.drain(..)
			{
				self.strength = event;
				if event > 0.
				{
					current = utils::max(current, event);
				}
				else
				{
					previous += current;
					current = 0.;
				}
			}
			current + previous
		}
	}

	fn clear(&mut self)
	{
		self.strength = 0.;
		self.queue.clear();
	}
}

#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct Controls
{
	action_to_inputs: BTreeMap<Action, [Option<Input>; 2]>,
	mouse_sensitivity: f32,
}

impl Controls
{
	pub fn new_game() -> Self
	{
		let mut action_to_inputs = BTreeMap::new();
		action_to_inputs.insert(
			Action::MoveLeft,
			[
				Some(Input::Keyboard(allegro::KeyCode::A)),
				Some(Input::JoystickNegAxis(allegro::JoystickStick::LeftThumb, 0)),
			],
		);
		action_to_inputs.insert(
			Action::MoveRight,
			[
				Some(Input::Keyboard(allegro::KeyCode::D)),
				Some(Input::JoystickPosAxis(allegro::JoystickStick::LeftThumb, 0)),
			],
		);
		action_to_inputs.insert(
			Action::MoveForward,
			[
				Some(Input::Keyboard(allegro::KeyCode::W)),
				Some(Input::JoystickNegAxis(allegro::JoystickStick::LeftThumb, 1)),
			],
		);
		action_to_inputs.insert(
			Action::MoveBackward,
			[
				Some(Input::Keyboard(allegro::KeyCode::S)),
				Some(Input::JoystickPosAxis(allegro::JoystickStick::LeftThumb, 1)),
			],
		);
		action_to_inputs.insert(
			Action::LookLeft,
			[
				Some(Input::MouseXNeg),
				Some(Input::JoystickNegAxis(
					allegro::JoystickStick::RightThumb,
					0,
				)),
			],
		);
		action_to_inputs.insert(
			Action::LookRight,
			[
				Some(Input::MouseXPos),
				Some(Input::JoystickPosAxis(
					allegro::JoystickStick::RightThumb,
					0,
				)),
			],
		);
		action_to_inputs.insert(
			Action::LookUp,
			[
				Some(Input::MouseYPos),
				Some(Input::JoystickPosAxis(
					allegro::JoystickStick::RightThumb,
					1,
				)),
			],
		);
		action_to_inputs.insert(
			Action::LookDown,
			[
				Some(Input::MouseYNeg),
				Some(Input::JoystickNegAxis(
					allegro::JoystickStick::RightThumb,
					1,
				)),
			],
		);
		action_to_inputs.insert(
			Action::Jump,
			[
				Some(Input::Keyboard(allegro::KeyCode::Space)),
				Some(Input::JoystickButton(allegro::JoystickButton::A)),
			],
		);
		action_to_inputs.insert(
			Action::Reproduce,
			[
				Some(Input::Keyboard(allegro::KeyCode::R)),
				Some(Input::JoystickButton(allegro::JoystickButton::B)),
			],
		);

		Self {
			action_to_inputs: action_to_inputs,
			mouse_sensitivity: 0.1,
		}
	}

	pub fn new_menu() -> Self
	{
		let mut action_to_inputs = BTreeMap::new();
		action_to_inputs.insert(
			Action::UIUp,
			[
				Some(Input::Keyboard(allegro::KeyCode::Up)),
				Some(Input::JoystickNegAxis(allegro::JoystickStick::DPad, 1)),
			],
		);
		action_to_inputs.insert(
			Action::UIDown,
			[
				Some(Input::Keyboard(allegro::KeyCode::Down)),
				Some(Input::JoystickPosAxis(allegro::JoystickStick::DPad, 1)),
			],
		);
		action_to_inputs.insert(
			Action::UILeft,
			[
				Some(Input::Keyboard(allegro::KeyCode::Left)),
				Some(Input::JoystickNegAxis(allegro::JoystickStick::DPad, 0)),
			],
		);
		action_to_inputs.insert(
			Action::UIRight,
			[
				Some(Input::Keyboard(allegro::KeyCode::Right)),
				Some(Input::JoystickPosAxis(allegro::JoystickStick::DPad, 0)),
			],
		);
		action_to_inputs.insert(
			Action::UIAccept,
			[
				Some(Input::Keyboard(allegro::KeyCode::Enter)),
				Some(Input::JoystickButton(allegro::JoystickButton::A)),
			],
		);
		action_to_inputs.insert(
			Action::UICancel,
			[
				Some(Input::Keyboard(allegro::KeyCode::Escape)),
				Some(Input::JoystickButton(allegro::JoystickButton::B)),
			],
		);

		Self {
			action_to_inputs: action_to_inputs,
			mouse_sensitivity: 0.1,
		}
	}

	pub fn new_game_ui() -> Self
	{
		let mut action_to_inputs = BTreeMap::new();
		action_to_inputs.insert(
			Action::UIUp,
			[
				Some(Input::Keyboard(allegro::KeyCode::Up)),
				Some(Input::JoystickNegAxis(allegro::JoystickStick::DPad, 1)),
			],
		);
		action_to_inputs.insert(
			Action::UIDown,
			[
				Some(Input::Keyboard(allegro::KeyCode::Down)),
				Some(Input::JoystickPosAxis(allegro::JoystickStick::DPad, 1)),
			],
		);
		action_to_inputs.insert(
			Action::UILeft,
			[
				Some(Input::Keyboard(allegro::KeyCode::Left)),
				Some(Input::JoystickNegAxis(allegro::JoystickStick::DPad, 0)),
			],
		);
		action_to_inputs.insert(
			Action::UIRight,
			[
				Some(Input::Keyboard(allegro::KeyCode::Right)),
				Some(Input::JoystickPosAxis(allegro::JoystickStick::DPad, 0)),
			],
		);
		action_to_inputs.insert(
			Action::UIAccept,
			[
				Some(Input::Keyboard(allegro::KeyCode::Enter)),
				Some(Input::JoystickButton(allegro::JoystickButton::A)),
			],
		);
		action_to_inputs.insert(
			Action::UICancel,
			[
				Some(Input::Keyboard(allegro::KeyCode::Escape)),
				Some(Input::JoystickButton(allegro::JoystickButton::Start)),
			],
		);

		Self {
			action_to_inputs: action_to_inputs,
			mouse_sensitivity: 0.1,
		}
	}

	pub fn get_action_string(&self, action: Action) -> String
	{
		let mut inputs = vec![];
		for input in self.action_to_inputs.get(&action).unwrap()
		{
			if let Some(input) = input
			{
				inputs.push(input.to_str());
			}
		}
		inputs.join("/")
	}
}

#[derive(Clone, Debug)]
pub struct ControlsHandler
{
	controls: Controls,
	input_to_action: BTreeMap<Input, Action>,
	input_state: HashMap<Input, InputState>,
}

impl ControlsHandler
{
	pub fn new(controls: Controls) -> Self
	{
		let mut ret = Self {
			controls: controls,
			input_to_action: BTreeMap::new(),
			input_state: HashMap::new(),
		};
		ret.update_derived();
		ret
	}

	pub fn update_derived(&mut self)
	{
		self.input_state.clear();
		for inputs in self.controls.action_to_inputs.values()
		{
			for input in &inputs[..]
			{
				if let Some(input) = input
				{
					self.input_state.insert(*input, InputState::new());
				}
			}
		}
		self.input_to_action.clear();
		for (action, [input1, input2]) in &self.controls.action_to_inputs
		{
			if let Some(input1) = input1
			{
				self.input_to_action.insert(*input1, *action);
			}
			if let Some(input2) = input2
			{
				self.input_to_action.insert(*input2, *action);
			}
		}
	}

	pub fn get_controls(&self) -> &Controls
	{
		&self.controls
	}

	pub fn get_mouse_sensitivity(&self) -> f32
	{
		self.controls.mouse_sensitivity
	}

	pub fn set_mouse_sensitivity(&mut self, mouse_sensitivity: f32)
	{
		self.controls.mouse_sensitivity = mouse_sensitivity;
	}

	pub fn get_actions_to_inputs(&self) -> impl Iterator<Item = (&Action, &[Option<Input>; 2])>
	{
		self.controls.action_to_inputs.iter()
	}

	pub fn get_inputs(&self, action: Action) -> Option<&[Option<Input>; 2]>
	{
		self.controls.action_to_inputs.get(&action)
	}

	pub fn decode_event(&mut self, event: &allegro::Event) -> Vec<(f32, Action)>
	{
		match event
		{
			allegro::Event::KeyDown { keycode, .. } =>
			{
				if let Some(state) = self.input_state.get_mut(&Input::Keyboard(*keycode))
				{
					state.push(1.);
				}
			}
			allegro::Event::KeyUp { keycode, .. } =>
			{
				if let Some(state) = self.input_state.get_mut(&Input::Keyboard(*keycode))
				{
					state.push(0.);
				}
			}
			allegro::Event::MouseButtonDown { button, .. } =>
			{
				if let Some(state) = self
					.input_state
					.get_mut(&Input::MouseButton(*button as i32))
				{
					state.push(1.);
				}
			}
			allegro::Event::MouseButtonUp { button, .. } =>
			{
				if let Some(state) = self
					.input_state
					.get_mut(&Input::MouseButton(*button as i32))
				{
					state.push(0.);
				}
			}
			allegro::Event::MouseAxes { dx, dy, dz, .. } =>
			{
				if *dx < 0
				{
					if let Some(state) = self.input_state.get_mut(&Input::MouseXNeg)
					{
						state.push(self.controls.mouse_sensitivity * -*dx as f32);
						state.push(0.);
					}
				}
				else if *dx > 0
				{
					if let Some(state) = self.input_state.get_mut(&Input::MouseXPos)
					{
						state.push(self.controls.mouse_sensitivity * *dx as f32);
						state.push(0.);
					}
				}
				if *dy < 0
				{
					if let Some(state) = self.input_state.get_mut(&Input::MouseYNeg)
					{
						state.push(self.controls.mouse_sensitivity * -*dy as f32);
						state.push(0.);
					}
				}
				else if *dy > 0
				{
					if let Some(state) = self.input_state.get_mut(&Input::MouseYPos)
					{
						state.push(self.controls.mouse_sensitivity * *dy as f32);
						state.push(0.);
					}
				}
				if *dz < 0
				{
					if let Some(state) = self.input_state.get_mut(&Input::MouseZNeg)
					{
						state.push(-*dz as f32);
						state.push(0.);
					}
				}
				else if *dz > 0
				{
					if let Some(state) = self.input_state.get_mut(&Input::MouseZPos)
					{
						state.push(*dz as f32);
						state.push(0.);
					}
				}
			}
			allegro::Event::JoystickButtonDown { button, .. } =>
			{
				if let Some(state) = self.input_state.get_mut(&Input::JoystickButton(*button))
				{
					state.push(1.);
				}
			}
			allegro::Event::JoystickButtonUp { button, .. } =>
			{
				if let Some(state) = self.input_state.get_mut(&Input::JoystickButton(*button))
				{
					state.push(0.);
				}
			}
			allegro::Event::JoystickAxes {
				axis, stick, pos, ..
			} =>
			{
				let dead = 0.25;
				if *pos > dead
				{
					if let Some(state) = self
						.input_state
						.get_mut(&Input::JoystickPosAxis(*stick, *axis))
					{
						state.push(*pos);
					}
					if let Some(state) = self
						.input_state
						.get_mut(&Input::JoystickNegAxis(*stick, *axis))
					{
						state.push(0.);
					}
				}
				else if *pos < -dead
				{
					if let Some(state) = self
						.input_state
						.get_mut(&Input::JoystickPosAxis(*stick, *axis))
					{
						state.push(0.);
					}
					if let Some(state) = self
						.input_state
						.get_mut(&Input::JoystickNegAxis(*stick, *axis))
					{
						state.push(-*pos);
					}
				}
				else
				{
					if let Some(state) = self
						.input_state
						.get_mut(&Input::JoystickPosAxis(*stick, *axis))
					{
						state.push(0.);
					}
					if let Some(state) = self
						.input_state
						.get_mut(&Input::JoystickNegAxis(*stick, *axis))
					{
						state.push(0.);
					}
				}
			}
			_ => (),
		}
		vec![]
	}

	pub fn get_action_state(&mut self, action: Action) -> f32
	{
		let mut ret = 0.;
		if let Some(inputs) = self.controls.action_to_inputs.get(&action)
		{
			for input in &inputs[..]
			{
				if let Some(input) = input
				{
					ret += self.input_state.get_mut(input).unwrap().get();
				}
			}
		}
		return ret;
	}

	pub fn clear_action_state(&mut self, action: Action)
	{
		if let Some(inputs) = self.controls.action_to_inputs.get(&action)
		{
			for input in &inputs[..]
			{
				if let Some(input) = input
				{
					self.input_state.get_mut(input).unwrap().clear();
				}
			}
		}
	}

	pub fn clear_action_states(&mut self)
	{
		for inputs in self.controls.action_to_inputs.values()
		{
			for input in &inputs[..]
			{
				if let Some(input) = input
				{
					self.input_state.get_mut(input).unwrap().clear();
				}
			}
		}
	}

	pub fn clear_action(&mut self, action: Action, index: usize)
	{
		self.controls.action_to_inputs.get_mut(&action).unwrap()[index] = None;
		self.update_derived();
	}

	pub fn change_action(
		&mut self, action: Action, index: usize, event: &allegro::Event,
	) -> Option<bool>
	{
		let mut handled = false;
		let new_input = match event
		{
			allegro::Event::KeyDown { keycode, .. } =>
			{
				handled = true;
				match *keycode
				{
					allegro::KeyCode::Escape => None,
					keycode => Some(Input::Keyboard(keycode)),
				}
			}
			allegro::Event::MouseButtonUp { button, .. } =>
			{
				handled = true;
				Some(Input::MouseButton(*button as i32))
			}
			allegro::Event::MouseAxes { dx, dy, dz, .. } =>
			{
				handled = true;
				match (dx.cmp(&0), dy.cmp(&0), dz.cmp(&0))
				{
					(Ordering::Less, _, _) => Some(Input::MouseXNeg),
					(Ordering::Greater, _, _) => Some(Input::MouseXPos),
					(_, Ordering::Less, _) => Some(Input::MouseYNeg),
					(_, Ordering::Greater, _) => Some(Input::MouseYPos),
					(_, _, Ordering::Less) => Some(Input::MouseZNeg),
					(_, _, Ordering::Greater) => Some(Input::MouseZPos),
					_ => None,
				}
			}
			allegro::Event::JoystickButtonDown { button, .. } =>
			{
				handled = true;
				if let allegro::JoystickButton::Generic(_) = button
				{
					None
				}
				else
				{
					Some(Input::JoystickButton(*button))
				}
			}
			allegro::Event::JoystickAxes {
				stick, axis, pos, ..
			} =>
			{
				if let allegro::JoystickStick::Generic(_) = stick
				{
					None
				}
				else if pos.abs() < 0.5
				{
					None
				}
				else
				{
					handled = true;
					if *pos > 0.
					{
						Some(Input::JoystickPosAxis(*stick, *axis))
					}
					else
					{
						Some(Input::JoystickNegAxis(*stick, *axis))
					}
				}
			}
			_ => None,
		};
		if let Some(new_input) = new_input
		{
			if self.input_to_action.contains_key(&new_input)
			{
				let old_input = self.controls.action_to_inputs[&action][index];
				let other_action = self.input_to_action[&new_input];
				let other_inputs = self
					.controls
					.action_to_inputs
					.get_mut(&other_action)
					.unwrap();

				if other_inputs[0] == Some(new_input)
				{
					other_inputs[0] = old_input;
				}
				else if other_inputs[1] == Some(new_input)
				{
					other_inputs[1] = old_input;
				}
			}
			self.controls.action_to_inputs.get_mut(&action).unwrap()[index] = Some(new_input);
		}
		if handled
		{
			self.update_derived();
			Some(new_input.is_some())
		}
		else
		{
			None
		}
	}
}
