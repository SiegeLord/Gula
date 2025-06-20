use crate::astar;
use crate::error::Result;
use crate::utils;
use nalgebra::{Point3, Vector3};
use serde_derive::{Deserialize, Serialize};

use allegro::*;
use allegro_primitives::*;

use std::collections::HashMap;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(i32)]
pub enum MaterialKind
{
	Static = 0,
	Dynamic = 1,
	Fullbright = 2,
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct MaterialDesc
{
	pub texture: String,
	#[serde(default)]
	pub lightmap: String,
	pub material_kind: MaterialKind,
	#[serde(default)]
	pub two_sided: bool,
}

#[derive(Debug, Clone)]
pub struct Material
{
	pub name: String,
	pub desc: MaterialDesc,
}

pub struct Mesh
{
	pub vtxs: Vec<NormVertex>,
	pub idxs: Vec<i32>,
	pub vertex_buffer: VertexBuffer<NormVertex>,
	pub index_buffer: IndexBuffer<u32>,
	pub material: Option<Material>,
}

#[derive(Clone, Debug)]
pub struct NavNode
{
	pub triangle: [Point3<f32>; 3],
	pub neighbours: Vec<i32>,
}

pub enum ObjectKind
{
	MultiMesh
	{
		meshes: Vec<Mesh>,
	},
	NavMesh
	{
		nodes: Vec<NavNode>,
	},
	Light
	{
		intensity: f32,
		color: Color,
	},
	Empty,
}

pub struct Object
{
	pub name: String,
	pub position: Point3<f32>,
	pub kind: ObjectKind,
}

pub struct Scene
{
	pub objects: Vec<Object>,
}

impl Scene
{
	pub fn load(display: &mut Display, prim: &PrimitivesAddon, gltf_file: &str) -> Result<Self>
	{
		let (document, buffers, _) = gltf::import(gltf_file)?;
		let mut objects = vec![];
		for node in document.nodes()
		{
			let (translation, _rot, _scale) = node.transform().decomposed();
			let position = Point3::new(translation[0], translation[1], translation[2]);
			let object;

			if node.name().map_or(false, |n| n == "Navmesh")
			{
				object = Object {
					name: node.name().unwrap_or("").to_string(),
					position: position,
					kind: ObjectKind::NavMesh {
						nodes: get_navmesh(&node, &buffers)?,
					},
				}
			}
			else if let Some(light) = node.light()
			{
				let color = light.color();
				object = Object {
					name: node.name().unwrap_or("").to_string(),
					position: position,
					kind: ObjectKind::Light {
						intensity: light.intensity(),
						color: Color::from_rgb_f(color[0], color[1], color[2]),
					},
				};
			}
			else if let Some(mesh) = node.mesh()
			{
				let mut meshes = vec![];
				for primitive in mesh.primitives()
				{
					let mut vtxs = vec![];
					let mut idxs = vec![];
					let reader = primitive.reader(|buffer| Some(&buffers[buffer.index()]));
					if let (
						Some(pos_iter),
						Some(gltf::mesh::util::ReadTexCoords::F32(uv_iter)),
						Some(normal_iter),
					) = (
						reader.read_positions(),
						reader.read_tex_coords(0),
						reader.read_normals(),
					)
					{
						if let Some(gltf::mesh::util::ReadTexCoords::F32(uv2_iter)) =
							reader.read_tex_coords(1)
						{
							for (((pos, uv), uv2), normal) in
								pos_iter.zip(uv_iter).zip(uv2_iter).zip(normal_iter)
							{
								vtxs.push(NormVertex {
									x: pos[0],
									y: pos[1],
									z: pos[2],
									u: uv[0],
									v: 1. - uv[1],
									u2: uv2[0],
									v2: 1. - uv2[1],
									nx: normal[0],
									ny: normal[1],
									nz: normal[2],
									color: Color::from_rgb_f(1., 1., 1.),
								});
							}
						}
						else
						{
							for ((pos, uv), normal) in pos_iter.zip(uv_iter).zip(normal_iter)
							{
								vtxs.push(NormVertex {
									x: pos[0],
									y: pos[1],
									z: pos[2],
									u: uv[0],
									v: 1. - uv[1],
									u2: uv[0],
									v2: 1. - uv[1],
									nx: normal[0],
									ny: normal[1],
									nz: normal[2],
									color: Color::from_rgb_f(1., 1., 1.),
								});
							}
						}
					}

					if let Some(iter) = reader.read_indices()
					{
						for idx in iter.into_u32()
						{
							idxs.push(idx as i32)
						}
					}

					let material = primitive
						.material()
						.name()
						.map(|name| {
							(
								name.to_string(),
								utils::load_config(&format!("data/{}.cfg", name)),
							)
						})
						.map_or(Ok(None), |(name, desc)| {
							desc.map(|desc| {
								Some(Material {
									name: name,
									desc: desc,
								})
							})
						})?;

					let (vertex_buffer, index_buffer) =
						create_buffers(display, prim, &vtxs, &idxs)?;

					meshes.push(Mesh {
						vtxs: vtxs,
						idxs: idxs,
						vertex_buffer: vertex_buffer,
						index_buffer: index_buffer,
						material: material,
					});
				}
				object = Object {
					name: node.name().unwrap_or("").to_string(),
					position: position,
					kind: ObjectKind::MultiMesh { meshes: meshes },
				};
			}
			else
			{
				object = Object {
					name: node.name().unwrap_or("").to_string(),
					position: position,
					kind: ObjectKind::Empty,
				};
			}
			objects.push(object);
		}
		Ok(Self { objects: objects })
	}

	pub fn draw<'l, T: Fn(&Material, &str) -> Result<&'l Bitmap>>(
		&self, core: &Core, prim: &PrimitivesAddon, bitmap_fn: T,
	)
	{
		for object in self.objects.iter()
		{
			if let ObjectKind::MultiMesh { meshes } = &object.kind
			{
				for mesh in meshes
				{
					core.set_shader_uniform(
						"material",
						&[mesh
							.material
							.as_ref()
							.map(|m| m.desc.material_kind as i32)
							.unwrap_or(0)][..],
					)
					.ok();
					prim.draw_indexed_buffer(
						&mesh.vertex_buffer,
						mesh.material
							.as_ref()
							.and_then(|m| Some(bitmap_fn(&m, &m.desc.texture).unwrap())),
						&mesh.index_buffer,
						0,
						mesh.idxs.len() as u32,
						PrimType::TriangleList,
					);
				}
			}
		}
	}

	pub fn clip_meshes(
		&self, display: &mut Display, prim: &PrimitivesAddon,
		keep_triangle_fn: impl Fn(Vector3<f32>) -> bool,
	) -> Result<Scene>
	{
		let mut objects = vec![];

		for object in &self.objects
		{
			let kind = match &object.kind
			{
				ObjectKind::Empty => ObjectKind::Empty,
				ObjectKind::Light { intensity, color } => ObjectKind::Light {
					intensity: *intensity,
					color: *color,
				},
				ObjectKind::NavMesh { nodes } => ObjectKind::NavMesh {
					nodes: nodes.clone(),
				},
				ObjectKind::MultiMesh { meshes } =>
				{
					let mut new_meshes = vec![];
					for mesh in meshes
					{
						let mut new_indices = vec![];
						for triangle in mesh.idxs.chunks(3)
						{
							let v1 = &mesh.vtxs[triangle[0] as usize];
							let v2 = &mesh.vtxs[triangle[1] as usize];
							let v3 = &mesh.vtxs[triangle[2] as usize];

							let centre = (Vector3::new(v1.x, v1.y, v1.z)
								+ Vector3::new(v2.x, v2.y, v2.z)
								+ Vector3::new(v3.x, v3.y, v3.z))
								/ 3.;
							if keep_triangle_fn(centre)
							{
								new_indices.extend(triangle.iter().copied());
							}
						}

						let (vertex_buffer, index_buffer) =
							create_buffers(display, prim, &mesh.vtxs, &new_indices)?;

						new_meshes.push(Mesh {
							vtxs: mesh.vtxs.clone(),
							idxs: new_indices,
							material: mesh.material.clone(),
							vertex_buffer: vertex_buffer,
							index_buffer: index_buffer,
						});
					}
					ObjectKind::MultiMesh { meshes: new_meshes }
				}
			};
			objects.push(Object {
				name: object.name.clone(),
				position: object.position,
				kind: kind,
			})
		}

		Ok(Scene { objects: objects })
	}
}

#[derive(Clone, Debug)]
#[repr(C)]
pub struct NormVertex
{
	pub x: f32,
	pub y: f32,
	pub z: f32,
	pub u: f32,
	pub v: f32,
	pub u2: f32,
	pub v2: f32,
	pub nx: f32,
	pub ny: f32,
	pub nz: f32,
	pub color: Color,
}

unsafe impl VertexType for NormVertex
{
	fn get_decl(prim: &PrimitivesAddon) -> VertexDecl
	{
		fn make_builder() -> std::result::Result<VertexDeclBuilder, ()>
		{
			VertexDeclBuilder::new(std::mem::size_of::<NormVertex>())
				.pos(
					VertexAttrStorage::F32_3,
					memoffset::offset_of!(NormVertex, x),
				)?
				.uv(
					VertexAttrStorage::F32_2,
					memoffset::offset_of!(NormVertex, u),
				)?
				.color(memoffset::offset_of!(NormVertex, color))?
				.user_attr(
					VertexAttrStorage::F32_3,
					memoffset::offset_of!(NormVertex, nx),
				)?
				.user_attr(
					VertexAttrStorage::F32_2,
					memoffset::offset_of!(NormVertex, u2),
				)
		}

		VertexDecl::from_builder(prim, &make_builder().unwrap())
	}
}

fn get_navmesh(node: &gltf::Node, buffers: &[gltf::buffer::Data]) -> Result<Vec<NavNode>>
{
	let mesh = node.mesh();
	let mesh = mesh.as_ref().ok_or("No mesh in navmesh".to_string())?;
	let prim = mesh
		.primitives()
		.next()
		.ok_or("No prim in navmesh".to_string())?;

	let mut vtxs = vec![];
	let mut idxs = vec![];
	let reader = prim.reader(|buffer| Some(&buffers[buffer.index()]));
	if let Some(pos_iter) = reader.read_positions()
	{
		for pos in pos_iter
		{
			vtxs.push(Point3::new(pos[0], pos[1], pos[2]));
		}
	}

	if let Some(iter) = reader.read_indices()
	{
		for idx in iter.into_u32()
		{
			idxs.push(idx as i32)
		}
	}

	let point_to_idx = |pt1: Point3<f32>, pt2: Point3<f32>| {
		(
			(pt1[0] / 0.1) as i32 + (pt2[0] / 0.1) as i32,
			(pt1[1] / 0.1) as i32 + (pt2[1] / 0.1) as i32,
			(pt1[2] / 0.1) as i32 + (pt2[2] / 0.1) as i32,
		)
	};

	let mut idx_to_triangle = HashMap::new();
	for (i, triangle) in idxs.chunks(3).enumerate()
	{
		for (vtx1_idx, vtx2_idx) in [
			(triangle[0], triangle[1]),
			(triangle[1], triangle[2]),
			(triangle[2], triangle[0]),
		]
		{
			idx_to_triangle
				.entry(point_to_idx(
					vtxs[vtx1_idx as usize],
					vtxs[vtx2_idx as usize],
				))
				.or_insert(vec![])
				.push(i as i32);
		}
	}

	let mut ret = vec![];
	for (i, triangle) in idxs.chunks(3).enumerate()
	{
		let mut neighbours = vec![];
		for (vtx1_idx, vtx2_idx) in [
			(triangle[0], triangle[1]),
			(triangle[1], triangle[2]),
			(triangle[2], triangle[0]),
		]
		{
			for &other_triangle_idx in
				&idx_to_triangle[&point_to_idx(vtxs[vtx1_idx as usize], vtxs[vtx2_idx as usize])]
			{
				if other_triangle_idx == i as i32
				{
					continue;
				}
				neighbours.push(other_triangle_idx);
			}
		}
		neighbours.sort();
		neighbours.dedup();
		let node = NavNode {
			triangle: [
				vtxs[triangle[0] as usize],
				vtxs[triangle[1] as usize],
				vtxs[triangle[2] as usize],
			],
			neighbours: neighbours,
		};
		ret.push(node);
	}
	Ok(ret)
}

impl astar::Node for NavNode
{
	fn get_pos(&self) -> Point3<f32>
	{
		Point3::origin()
			+ (self.triangle[0].coords + self.triangle[1].coords + self.triangle[2].coords) / 3.
	}

	fn get_neighbours(&self) -> &[i32]
	{
		&self.neighbours
	}
}

fn create_buffers(
	display: &mut Display, prim: &PrimitivesAddon, vtxs: &[NormVertex], idxs: &[i32],
) -> Result<(VertexBuffer<NormVertex>, IndexBuffer<u32>)>
{
	let vertex_buffer =
		VertexBuffer::new(display, prim, Some(&vtxs), vtxs.len() as u32, BUFFER_STATIC)
			.map_err(|_| "Could not create vertex buffer".to_string())?;
	let index_buffer = IndexBuffer::new(
		display,
		prim,
		Some(&idxs.iter().map(|&i| i as u32).collect::<Vec<_>>()),
		idxs.len() as u32,
		BUFFER_STATIC,
	)
	.map_err(|_| "Could not create index buffer".to_string())?;
	Ok((vertex_buffer, index_buffer))
}
