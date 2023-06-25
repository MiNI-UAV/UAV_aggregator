use std::{sync::{Arc, Mutex}, thread::{JoinHandle, self}, time};

use crate::uav::{UAV,DroneState};
pub struct Drones
{

    ctx: zmq::Context,
    drones: Vec<UAV>,
    states: Arc<Mutex<Vec<Arc<Mutex<DroneState>>>>>,
    _state_publisher: JoinHandle<()>
}

impl Drones
{
    pub fn new(_ctx: zmq::Context) -> Self {
        let states = Arc::new(Mutex::new(Vec::<Arc<Mutex<DroneState>>>::new()));
        let state_arc = states.clone();
        let publisher_socket = _ctx.socket(zmq::PUB).expect("Pub socket error");
        let publisher: JoinHandle<()> = thread::spawn(move ||
        {
            publisher_socket.bind("tcp://127.0.0.1:9090").expect("Bind error tcp 9090");
            loop {
                let state = state_arc.lock().unwrap();
                if !state.is_empty()
                {
                    let mut result = String::with_capacity(state.len()*300);
                    for elem in state.iter()  {
                        let drone = elem.lock().unwrap();
                        result.push_str(&drone.to_string());
                        result.push(';');
                    }
                    publisher_socket.send(&result, 0).unwrap();
                    //println!("{}",result);
                }
                drop(state);

                thread::sleep(time::Duration::from_millis(15));
            }
        });
        println!("Created");
        Drones {ctx: _ctx, drones: Vec::new(), states: states, _state_publisher: publisher}
    }

    pub fn startUAV(&mut self, name: &str)
    {
        let state = Arc::new(Mutex::new(DroneState::new()));
        self.states.lock().unwrap().push(state.clone());
        self.drones.push(UAV::new(&mut self.ctx, name,state));
    }

    pub fn printState(&self)
    {
        for (i, item) in self.states.lock().unwrap().iter().enumerate() {
            let state = item.lock();
            println!("{}:{}",i,state.unwrap().to_string());
        }
    }



}

