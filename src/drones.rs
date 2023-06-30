use std::{sync::{Arc, Mutex}, thread::{JoinHandle, self}, time};
use ndarray::Array1;
use crate::uav::{UAV,DroneState};
use crate::objects::Objects;

pub struct Drones
{

    ctx: zmq::Context,
    pub drones: Vec<UAV>,
    pub states: Arc<Mutex<Vec<Arc<Mutex<DroneState>>>>>,
    objects: Arc<Mutex<Objects>>,
    _state_publisher: JoinHandle<()>
}

impl Drones
{
    pub fn new(_ctx: zmq::Context,objects: Arc<Mutex<Objects>>) -> Self {
        let states = Arc::new(Mutex::new(Vec::<Arc<Mutex<DroneState>>>::new()));
        let state_arc = states.clone();
        let publisher_socket = _ctx.socket(zmq::PUB).expect("Pub socket error");
        let publisher: JoinHandle<()> = thread::spawn(move ||
        {
            publisher_socket.bind("tcp://127.0.0.1:9090").expect("Bind error tcp 9090");
            println!("State publisher started on TCP: {}", 9090);
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
        Drones {ctx: _ctx, drones: Vec::new(), states: states, objects: objects, _state_publisher: publisher}
    }

    pub fn startUAV(&mut self, name: &str) -> (usize,String)
    {
        let state = Arc::new(Mutex::new(DroneState::new()));
        self.states.lock().unwrap().push(state.clone());
        self.drones.push(UAV::new(&mut self.ctx, name,state,self.objects.clone()));
        (self.drones.len()-1,format!("ipc:///tmp/{}/steer", name))
    }

    pub fn printState(&self)
    {
        for (i, item) in self.states.lock().unwrap().iter().enumerate() {
            let state = item.lock();
            println!("{}:{}",i,state.unwrap().to_string());
        }
    }

    pub fn getPositions(&self) -> Vec<Array1<f32>>
    {
        let mut pos = Vec::new();
        let state = self.states.lock().unwrap();
        if !state.is_empty()
        {
            for elem in state.iter()  {
                pos.push(elem.lock().unwrap().getPos());
            }
        }
        drop(state);
        pos
    }



}

