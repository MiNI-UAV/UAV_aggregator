use std::{sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}}, thread::{JoinHandle, self}, time};
use ndarray::Array1;
use crate::uav::{UAV,DroneState};
use crate::objects::Objects;

pub struct Drones
{
    ctx: zmq::Context,
    running: Arc<AtomicBool>,
    pub drones: Arc<Mutex<Vec<UAV>>>,
    objects: Arc<Mutex<Objects>>,
    _state_publisher: Option<thread::JoinHandle<()>>,
    nextID: usize
}

impl Drones
{
    pub fn new(_ctx: zmq::Context,objects: Arc<Mutex<Objects>>) -> Self {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        let drones = Arc::new(Mutex::new(Vec::<UAV>::new()));
        let drones_arc = drones.clone();
        let publisher_socket = _ctx.socket(zmq::PUB).expect("Pub socket error");
        let publisher: JoinHandle<()> = thread::spawn(move ||
        {
            publisher_socket.bind("tcp://*:9090").expect("Bind error tcp 9090");
            println!("State publisher started on TCP: {}", 9090);
            while r.load(Ordering::SeqCst) {
                let drones = drones_arc.lock().unwrap();
                if !drones.is_empty()
                {
                    let mut result = String::with_capacity(drones.len()*320);
                    for elem in drones.iter()  {
                        result.push_str(&elem.id.to_string());
                        result.push(',');
                        let state = elem.state_arc.lock().unwrap();
                        result.push_str(&state.to_string());
                        drop(state);
                        result.push(';');
                    }
                    publisher_socket.send(&result, 0).unwrap();
                    //println!("{}",result);
                }
                drop(drones);
                thread::sleep(time::Duration::from_millis(15));
            }
        });
        Drones {ctx: _ctx, running: running, drones: drones, objects: objects, _state_publisher: Some(publisher), nextID: 0}
    }

    pub fn startUAV(&mut self, name: &str) -> (usize,String)
    {
        let state = Arc::new(Mutex::new(DroneState::new()));
        let mut drone = self.drones.lock().unwrap();
        let id = self.nextID;
        self.nextID += 1;
        drone.push(UAV::new(&mut self.ctx,id, name,state,self.objects.clone()));
        drop(drone);
        (id,format!("ipc:///tmp/{}/steer", name))
    }

    pub fn removeUAV(&mut self, id: usize)
    {
        let mut drone = self.drones.lock().unwrap();
        drone.retain_mut(|d| d.id != id);
        drop(drone);
    }

    pub fn removeAllUAV(&mut self)
    {
        let mut drone = self.drones.lock().unwrap();
        drone.clear();
        drop(drone);
    }

    pub fn printState(&self)
    {
        for (i, item) in self.drones.lock().unwrap().iter().enumerate() {
            let state = item.state_arc.lock().unwrap();
            println!("{}:{}",i,state.to_string());
        }
    }

    pub fn getPositions(&self) -> Vec<(usize,Array1<f32>)>
    {
        let mut pos = Vec::new();
        let drone = self.drones.lock().unwrap();
        if !drone.is_empty()
        {

            for elem in drone.iter()  {
                pos.push((elem.id,elem.state_arc.lock().unwrap().getPos()));
            }
        }
        drop(drone);
        pos
    }

}

impl Drop for Drones{
    fn drop(&mut self) {
        println!("Dropping drones instance");
        self.running.store(false, Ordering::SeqCst);
        self._state_publisher.take().unwrap().join().expect("Join error");
        self.removeAllUAV();
        println!("Drones instance dropped");
    }
}
