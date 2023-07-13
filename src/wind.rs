use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};
use nalgebra::Vector3;

use crate::{drones::Drones, objects::Objects};


pub struct Wind
{
    running: Arc<AtomicBool>,
    wind_reqester: Option<thread::JoinHandle<()>>
}

impl Wind
{
    pub fn new(drones: Arc<Mutex<Drones>>,objects: Arc<Mutex<Objects>>) -> Self
    {   
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        let wind_reqester: JoinHandle<()> = thread::spawn(move ||
        {
            while r.load(Ordering::SeqCst) {
                let drones_lck = drones.lock().unwrap();
                let pos = drones_lck.getPositions();
                let wind: Vec<(usize,Vector3<f32>)> = pos.iter().map(|p| (p.0, Wind::calcWind(&p.1))).collect();
                thread::sleep(time::Duration::from_millis(50));
                for (id,w) in wind.iter()
                {
                    let d = drones_lck.drones.lock().unwrap();
                    if let Some(drone) = d.iter().find(|drone| drone.id == *id)
                    {
                        drone.sendWind(w);
                    }

                }
                drop(drones_lck);
                thread::sleep(time::Duration::from_millis(50));
                let objects_lck = objects.lock().unwrap();
                let pos = objects_lck.getPositions();
                drop(objects_lck);
                let wind: Vec<(usize,Vector3<f32>)> = pos.iter().map(|p| (p.0,Wind::calcWind(&p.1))).collect();
                if wind.is_empty()
                {
                    continue;
                }
                thread::sleep(time::Duration::from_millis(50));
                let objects_lck = objects.lock().unwrap();
                objects_lck.updateWind(wind);
                drop(objects_lck);
                thread::sleep(time::Duration::from_millis(50));
            }
        });
        Wind {running: running, wind_reqester: Some(wind_reqester) }
    }

    fn calcWind(pos: &Vector3<f32>) -> Vector3<f32>
    {
        let mut result = Vector3::zeros();
        result[0] = -0.05 * pos[2] + 2.0;
        result[1] = -0.02 * pos[2] + 1.0;
        result
    }
}

impl Drop for Wind{
    fn drop(&mut self) {
        println!("Dropping wind instance");
        self.running.store(false, Ordering::SeqCst);
        self.wind_reqester.take().unwrap().join().expect("Join error");
        println!("Wind instance dropped");
    }
}