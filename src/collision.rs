use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};

use ndarray::Array1;

use crate::drones::Drones;

const MINIMAL_DISTANCE2: f32 = 0.5 * 0.5;

pub struct CollisionDetector
{
    running: Arc<AtomicBool>,
    collision_checker: Option<thread::JoinHandle<()>>
}

impl CollisionDetector
{
    pub fn new(_drones: Arc<Mutex<Drones>>) -> Self
    {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        let collision_checker: JoinHandle<()> = thread::spawn(move ||
        {
            while r.load(Ordering::SeqCst) {
                let drones_lck = _drones.lock().unwrap();
                let pos = drones_lck.getPositions();
                drop(drones_lck);
                
                for i in 0..pos.len() {
                    for j in (i+1)..pos.len() {
                        let obj1 = pos.get(i).unwrap();
                        let obj2 = pos.get(j).unwrap();
                        let dist: Array1<f32> = obj1.1.clone()-obj2.1.clone();
                        if dist.dot(&dist).abs() < MINIMAL_DISTANCE2
                        {
                            println!("Collision detected between drone {} and {}", obj1.0,obj2.0);
                        }
                    }
                }
                thread::sleep(time::Duration::from_millis(100));
            }
        });
        CollisionDetector {running: running, collision_checker: Some(collision_checker)}
    }
}

impl Drop for CollisionDetector{
    fn drop(&mut self) {
        println!("Dropping collision detector instance");
        self.running.store(false, Ordering::SeqCst);
        self.collision_checker.take().unwrap().join().expect("Join error");
        println!("Collision detector instance dropped");
    }
}