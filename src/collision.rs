use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc}, time};

use ndarray::Array1;

use crate::drones::Drones;

const MINIMAL_DISTANCE2: f32 = 0.5 * 0.5;

pub struct CollisionDetector
{
    collision_checker: Option<thread::JoinHandle<()>>
}

impl CollisionDetector
{
    pub fn new(_drones: Arc<Mutex<Drones>>) -> Self
    {
        let collision_checker: JoinHandle<()> = thread::spawn(move ||
        {
            loop {
                let drones_lck = _drones.lock().unwrap();
                let pos = drones_lck.getPositions();
                drop(drones_lck);
                
                for i in 0..pos.len() {
                    for j in (i+1)..pos.len() {
                        let dist: Array1<f32> = pos.get(i).unwrap()-pos.get(j).unwrap();
                        if dist.dot(&dist).abs() < MINIMAL_DISTANCE2
                        {
                            println!("Collision detected between drone {} and {}", i,j);
                        }
                    }
                }

                thread::sleep(time::Duration::from_millis(100));
            }
        });
        CollisionDetector { collision_checker: Some(collision_checker) }
    }
}

impl Drop for CollisionDetector{
    fn drop(&mut self) {
        self.collision_checker.take().unwrap().join().expect("Join error");
    }
}