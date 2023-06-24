use std::sync::{Arc, Mutex};

use crate::uav::{UAV,DroneState};


pub struct Drones
{
    ctx: zmq::Context,
    drones: Vec<UAV>,
    states: Vec<Arc<Mutex<DroneState>>> 
}

impl Drones
{
    pub fn new() -> Self {
        Drones {ctx: zmq::Context::new(), drones: Vec::new(), states: Vec::new() }
    }

    pub fn startUAV(&mut self, name: &str)
    {
        let state = Arc::new(Mutex::new(DroneState::new()));
        self.states.push(state.clone());
        self.drones.push(UAV::new(&mut self.ctx, name,state));
    }

    pub fn printState(&self)
    {
        for (i, item) in self.states.iter().enumerate() {
            let state = item.lock();
            println!("{}: {}",i,state.unwrap().to_string());
        }
    }
}
