use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc}};
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
                for (i,state) in drones_lck.states.lock().unwrap().iter().enumerate() {
                    let state_lck = state.lock().unwrap();
                    let pos = state_lck.getPos();
                    drop(state_lck);
                    if let Some(d) = drones_lck.drones.get(i)
                    {
                        d.sendWind(Wind::calcWind(pos));
                    }
                }

            }
        });
        Wind { wind_reqester: Some(wind_reqester) }
    }

    fn calcWind(pos: Array1<f32>) -> Array1<f32>
    {
        let mut result = arr1(&[0.0,0.0,0.0]);
        result[0] = -0.5 * pos[2];
        result[1] = -0.2 * pos[2];
        result
    }
}

impl Drop for Wind{
    fn drop(&mut self) {
        self.wind_reqester.take().unwrap().join().expect("Join error");
    }
}