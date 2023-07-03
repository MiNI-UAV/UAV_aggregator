use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};
use ndarray::{Array1,arr1};

use crate::{drones::Drones, objects::{Objects}};


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
                drop(drones_lck);
                let wind: Vec<Array1<f32>> = pos.iter().map(|p| Wind::calcWind(p)).collect();
                thread::sleep(time::Duration::from_millis(50));
                println!("A");
                let drones_lck = drones.lock().unwrap();
                for (i,w) in wind.iter().enumerate()
                {
                    if let Some(d) = drones_lck.drones.get(i)
                    {
                        d.sendWind(w);
                    }
                }
                drop(drones_lck);
                thread::sleep(time::Duration::from_millis(50));
                println!("B");
                let objects_lck = objects.lock().unwrap();
                let pos = objects_lck.getPositions();
                drop(objects_lck);
                let wind: Vec<(usize,Array1<f32>)> = pos.iter().map(|p| (p.0,Wind::calcWind(&p.1))).collect();
                if wind.is_empty()
                {
                    continue;
                }
                thread::sleep(time::Duration::from_millis(50));
                println!("C");
                let objects_lck = objects.lock().unwrap();
                objects_lck.updateWind(wind);
                drop(objects_lck);
                thread::sleep(time::Duration::from_millis(50));
                println!("D");
            }
        });
        Wind {running: running, wind_reqester: Some(wind_reqester) }
    }

    fn calcWind(pos: &Array1<f32>) -> Array1<f32>
    {
        let mut result = arr1(&[0.0,0.0,0.0]);
        result[0] = -0.5 * pos[2] + 20.0;
        result[1] = -0.2 * pos[2] + 10.0;
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