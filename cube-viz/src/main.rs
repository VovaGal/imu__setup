use std::io::Read;
use std::net::TcpStream;
use std::str::{from_utf8, FromStr};

use cgmath::*;
use regex::Regex;
use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

use crate::state::State;

mod vertex_data;
mod transforms;
mod state;

const IS_PERSPECTIVE: bool = true;

fn main() {
    env_logger::init();
    let event_loop = EventLoop::new();
    let window = WindowBuilder::new().build(&event_loop).unwrap();
    window.set_title("Cube");
    let mut state = pollster::block_on(State::new(&window));

    let start_time = std::time::Instant::now();

    // let mut socket = TcpStream::connect("99.22.0.1:9922").unwrap();
    let mut socket = TcpStream::connect("192.168.4.1:23").unwrap();

    let mut rt = Quaternion::new(1f32, 0., 0., 0.);

    event_loop.run(move |event, _, control_flow| {
        match event {
            Event::WindowEvent {
                ref event,
                window_id,
            } if window_id == window.id() => {
                if !state.input(event) {
                    match event {
                        WindowEvent::CloseRequested
                        | WindowEvent::KeyboardInput {
                            input:
                            KeyboardInput {
                                state: ElementState::Pressed,
                                virtual_keycode: Some(VirtualKeyCode::Escape),
                                ..
                            },
                            ..
                        } => *control_flow = ControlFlow::Exit,
                        WindowEvent::Resized(physical_size) => {
                            state.resize(*physical_size);
                        }
                        WindowEvent::ScaleFactorChanged { new_inner_size, .. } => {
                            state.resize(**new_inner_size);
                        }
                        _ => {}
                    }
                }
            }
            Event::RedrawRequested(_) => {
                let now = std::time::Instant::now();
                let dt = now - start_time;

                let mut buffer = [0; 1024];

                let sz = socket.read(&mut buffer).unwrap();
                let d_res = from_utf8(&buffer[..sz]);
                if let Ok(data) = d_res {
                    let regex = Regex::new(r"-?\d+\.\d+").unwrap();
                    let caps = &regex.find_iter(data)
                        .filter_map(|s| f64::from_str(s.as_str()).ok()).collect::<Vec<f64>>();
                    for slice in caps.chunks(4) {
                        let [qw, qx, qy, qz] = slice else { continue; };
                        let check = (qw * qw + qx * qx + qy * qy + qz * qz).sqrt();
                        if (check - 1.).abs() < 0.00001 {
                            rt = Quaternion::new(
                                *qw as f32,
                                *qx as f32, *qy as f32, *qz as f32,
                            ).normalize();
                        } else {
                            println!("Malformed data");
                        }
                    }
                }

                state.update(dt, rt);

                match state.render() {
                    Ok(_) => {}
                    Err(wgpu::SurfaceError::Lost) => state.resize(state.init.size),
                    Err(wgpu::SurfaceError::OutOfMemory) => *control_flow = ControlFlow::Exit,
                    Err(e) => eprintln!("{:?}", e),
                }
            }
            Event::MainEventsCleared => {
                window.request_redraw();
            }
            _ => {}
        }
    });
}
