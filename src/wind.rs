use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc}, time};
use ndarray::{Array1,arr1};

use crate::drones::Drones;


pub struct Wind
{
    wind_reqester: Option<thread::JoinHandle<()>>
}

impl Wind
{
    pub fn new(drones: Arc<Mutex<Drones>>) -> Self
    {
        let wind_reqester: JoinHandle<()> = thread::spawn(move ||
        {
            loop {
                let drones_lck = drones.lock().unwrap();
                let pos = drones_lck.getPositions();
                drop(drones_lck);
                let wind: Vec<Array1<f32>> = pos.iter().map(|p| Wind::calcWind(p)).collect();
                thread::sleep(time::Duration::from_millis(100));
                let drones_lck = drones.lock().unwrap();
                for (i,w) in wind.iter().enumerate()
                {
                    if let Some(d) = drones_lck.drones.get(i)
                    {
                        d.sendWind(w);
                    }
                }
                drop(drones_lck);
                thread::sleep(time::Duration::from_millis(100));
            }
        });
        Wind { wind_reqester: Some(wind_reqester) }
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
        self.wind_reqester.take().unwrap().join().expect("Join error");
    }
}