use core::f32::consts::PI;

use libm::sincosf;
use nalgebra::{ComplexField, Vector3};

use crate::{
    camera::Camera,
    constants::{
        BlockType,
        player::{GRAVITY, JUMP_SPEED, MOVEMENT_SPEED, PLAYER_HEIGHT, PLAYER_RADIUS},
    },
    eadk,
    mesh::{Mesh, Quad, QuadDir},
    world::World,
};

pub struct Player {
    pub pos: Vector3<f32>,
    pub rotation: Vector3<f32>,
    velocity: Vector3<f32>,
    on_ground: bool,
    ray_cast_result: Option<RaycastResult>,
}

impl Player {
    pub fn new() -> Self {
        Player {
            pos: Vector3::new(0., 0., 0.),
            rotation: Vector3::new(0., 0., 0.),
            velocity: Vector3::new(0., 0., 0.),
            on_ground: false,
            ray_cast_result: None,
        }
    }

    pub fn get_block_marker(&self) -> (Mesh, Vector3<isize>) {
        let mut mesh = Mesh::new();

        if let Some(result) = &self.ray_cast_result {
            mesh.quads
                .push(Quad::new(Vector3::new(0, 0, 0), result.face_dir, 255, 0));
            (mesh, result.block_pos)
        } else {
            (mesh, Vector3::repeat(0))
        }
    }

    pub fn sync_with_camera(&mut self, camera: &mut Camera) {
        camera.update_pos(self.pos - Vector3::new(0., PLAYER_HEIGHT, 0.));
        self.rotation = *camera.get_rotation();
    }

    pub fn set_pos_rotation(&mut self, camera: &mut Camera, rotation: Vector3<f32>, pos: Vector3<f32>) {
        self.pos = pos;
        camera.set_rotation(rotation);
        self.sync_with_camera(camera);
    }

    pub fn update(
        &mut self,
        delta: f32,
        keyboard_state: eadk::input::KeyboardState,
        just_pressed_keyboard_state: eadk::input::KeyboardState,
        world: &mut World,
        camera: &mut Camera,
    ) {
        self.sync_with_camera(camera);
        self.rotation = *camera.get_rotation();

        self.ray_cast_result = self.ray_cast(camera, world, 10);

        // Movements
        let mut velocity = Vector3::new(0.0, self.velocity.y, 0.0);

        if keyboard_state.key_down(eadk::input::Key::Toolbox) {
            // Forward
            let translation = sincosf(self.rotation.y);
            velocity.x += translation.0 * MOVEMENT_SPEED;
            velocity.z += translation.1 * MOVEMENT_SPEED;
        }
        if keyboard_state.key_down(eadk::input::Key::Comma) {
            // Backward
            let translation = sincosf(self.rotation.y);
            velocity.x -= translation.0 * MOVEMENT_SPEED;
            velocity.z -= translation.1 * MOVEMENT_SPEED;
        }
        if keyboard_state.key_down(eadk::input::Key::Imaginary) {
            // Left
            let translation = sincosf(self.rotation.y + PI / 2.0);
            velocity.x -= translation.0 * MOVEMENT_SPEED;
            velocity.z -= translation.1 * MOVEMENT_SPEED;
        }
        if keyboard_state.key_down(eadk::input::Key::Power) {
            // Right
            let translation = sincosf(self.rotation.y + PI / 2.0);
            velocity.x += translation.0 * MOVEMENT_SPEED;
            velocity.z += translation.1 * MOVEMENT_SPEED;
        }

        if keyboard_state.key_down(eadk::input::Key::Shift) && self.on_ground {
            // Jump
            velocity.y = -JUMP_SPEED;
        }

        self.velocity.y += GRAVITY * delta;
        velocity.y += self.velocity.y;

        self.move_and_collide(velocity * delta, world);

        self.velocity.y = velocity.y;

        if just_pressed_keyboard_state.key_down(eadk::input::Key::Back) {
            // Break Block
            if let Some(result) = &self.ray_cast_result {
                world.set_block_in_world(result.block_pos, BlockType::Air);
            }
        }

        if just_pressed_keyboard_state.key_down(eadk::input::Key::Ok) {
            // Place Block
            if let Some(result) = &self.ray_cast_result {
                let block_pos = result.block_pos + result.face_dir.get_normal_vector();
                if world
                    .get_block_in_world(block_pos)
                    .is_some_and(|b| b.is_air())
                // Just in case
                {
                    world.set_block_in_world(block_pos, BlockType::Stone);
                }
            }
        }
    }

    fn is_colliding(&self, world: &World, pos: Vector3<f32>) -> bool {
        let min_x = (pos.x - PLAYER_RADIUS).floor() as isize;
        let max_x = (pos.x + PLAYER_RADIUS).floor() as isize;
        let min_y = pos.y.floor() as isize;
        let max_y = (pos.y + PLAYER_HEIGHT).floor() as isize;
        let min_z = (pos.z - PLAYER_RADIUS).floor() as isize;
        let max_z = (pos.z + PLAYER_RADIUS).floor() as isize;

        for x in min_x..=max_x {
            for y in min_y..=max_y {
                for z in min_z..=max_z {
                    if let Some(b) = world.get_block_in_world(Vector3::new(x, y, z)) {
                        if !b.is_air() {
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    fn move_and_collide(&mut self, movement: Vector3<f32>, world: &World) {
        let mut new_pos = self.pos;

        new_pos.x += movement.x;
        if self.is_colliding(world, new_pos) {
            new_pos.x = self.pos.x;
        }

        new_pos.z += movement.z;
        if self.is_colliding(world, new_pos) {
            new_pos.z = self.pos.z;
        }

        new_pos.y += movement.y;
        if self.is_colliding(world, new_pos) {
            if movement.y > 0.0 {
                new_pos.y = new_pos.y.floor();
                self.on_ground = true;
            } else {
                new_pos.y = self.pos.y;
            }
            self.velocity.y = 0.0;
        } else {
            self.on_ground = false;
        }

        self.pos = new_pos;
    }

    fn ray_cast(&self, camera: &Camera, world: &World, max_lenght: usize) -> Option<RaycastResult> {
        let start_pos = *camera.get_pos();
        let forward_vector = camera.get_forward_vector();

        let end_pos = start_pos + forward_vector.normalize() * (max_lenght as f32);

        let mut current_voxel_pos = start_pos;
        let mut step_dir = -1;

        let dx = (end_pos.x - start_pos.x).signum();
        let delta_x = if dx != 0. {
            (dx / (end_pos.x - start_pos.x)).min(10000000.0)
        } else {
            10000000.0
        };
        let mut max_x = if dx > 0. {
            delta_x * (1.0 - start_pos.x.fract())
        } else {
            delta_x * start_pos.x.fract()
        };

        let dy = (end_pos.y - start_pos.y).signum();
        let delta_y = if dy != 0. {
            (dy / (end_pos.y - start_pos.y)).min(10000000.0)
        } else {
            10000000.0
        };
        let mut max_y = if dy > 0. {
            delta_y * (1.0 - start_pos.y.fract())
        } else {
            delta_y * start_pos.y.fract()
        };

        let dz = (end_pos.z - start_pos.z).signum();
        let delta_z = if dz != 0. {
            (dz / (end_pos.z - start_pos.z)).min(10000000.0)
        } else {
            10000000.0
        };
        let mut max_z = if dz > 0. {
            delta_z * (1.0 - start_pos.z.fract())
        } else {
            delta_z * start_pos.z.fract()
        };

        while !(max_x > 1.0 && max_y > 1.0 && max_z > 1.0) {
            let current_voxel_pos_isize = current_voxel_pos.map(|x| x as isize);
            let result = world.get_block_in_world(current_voxel_pos_isize);
            if !result.is_none_or(|b| b == BlockType::Air) {
                let voxel_normal = if step_dir == 0 {
                    if dx < 0. {
                        QuadDir::Right
                    } else {
                        QuadDir::Left
                    }
                } else if step_dir == 1 {
                    if dy < 0. {
                        QuadDir::Bottom
                    } else {
                        QuadDir::Top
                    }
                } else if dz < 0. {
                    QuadDir::Back
                } else {
                    QuadDir::Front
                };
                return Some(RaycastResult {
                    block_pos: current_voxel_pos_isize,
                    face_dir: voxel_normal,
                });
            }

            if max_x < max_y {
                if max_x < max_z {
                    current_voxel_pos.x += dx;
                    max_x += delta_x;
                    step_dir = 0;
                } else {
                    current_voxel_pos.z += dz;
                    max_z += delta_z;
                    step_dir = 2;
                }
            } else if max_y < max_z {
                current_voxel_pos.y += dy;
                max_y += delta_y;
                step_dir = 1;
            } else {
                current_voxel_pos.z += dz;
                max_z += delta_z;
                step_dir = 2
            }
        }
        None
    }
}

struct RaycastResult {
    pub block_pos: Vector3<isize>,
    pub face_dir: QuadDir,
}
