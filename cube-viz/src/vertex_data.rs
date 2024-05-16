use std::mem;
use bytemuck::{Pod, Zeroable};

#[repr(C)]
#[derive(Copy, Clone, Debug, Pod, Zeroable)]
pub struct Vertex {
    position: [f32; 4],
    color: [f32; 4],
}

pub(crate) fn vertex(p: [i8; 3], c: [i8; 3]) -> Vertex {
    Vertex {
        position: [p[0] as f32, p[1] as f32, p[2] as f32, 1.0],
        color: [c[0] as f32, c[1] as f32, c[2] as f32, 1.0],
    }
}

pub(crate) fn create_vertices() -> Vec<Vertex> {
    let pos = cube_positions();
    let col = cube_colors();
    let mut data: Vec<Vertex> = Vec::with_capacity(pos.len());
    for i in 0..pos.len() {
        data.push(vertex(pos[i], col[i]));
    }
    data.to_vec()
}

impl Vertex {
    const ATTRIBUTES: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0=>Float32x4, 1=>Float32x4];
    pub fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBUTES,
        }
    }
}

pub fn cube_positions() -> Vec<[i8; 3]> {
    [
        // front (0, 0, 1)
        [-1, -1, 1], [1, -1, 1], [-1, 1, 1], [-1, 1, 1], [1, -1, 1], [1, 1, 1],

        // right (1, 0, 0)
        [1, -1, 1], [1, -1, -1], [1, 1, 1], [1, 1, 1], [1, -1, -1], [1, 1, -1],

        // back (0, 0, -1)
        [1, -1, -1], [-1, -1, -1], [1, 1, -1], [1, 1, -1], [-1, -1, -1], [-1, 1, -1],

        // left (-1, 0, 0)
        [-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, -1], [-1, -1, 1], [-1, 1, 1],

        // top (0, 1, 0)
        [-1, 1, 1], [1, 1, 1], [-1, 1, -1], [-1, 1, -1], [1, 1, 1], [1, 1, -1],

        // bottom (0, -1, 0)
        [-1, -1, -1], [1, -1, -1], [-1, -1, 1], [-1, -1, 1], [1, -1, -1], [1, -1, 1],
    ].to_vec()
}

pub fn cube_colors() -> Vec<[i8; 3]> {
    [
        // front
        [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1],

        // right
        [1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0],

        // back
        [0, 1, 1], [0, 1, 1], [0, 1, 1], [0, 1, 1], [0, 1, 1], [0, 1, 1],

        // left
        [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0],

        // top
        [1, 1, 0], [1, 1, 0], [1, 1, 0], [1, 1, 0], [1, 1, 0], [1, 1, 0],

        // bottom
        [1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1], [1, 1, 1],
    ].to_vec()
}