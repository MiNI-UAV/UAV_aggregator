use std::{sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}}, thread::{JoinHandle, self}, time};
use nalgebra::{Vector3,Vector4, Matrix3xX, DVector};
use crate::{uav::{UAV,DroneState}, notification::Notification};
use crate::objects::Objects;
use crate::config::ServerConfig;
use crate::printLog;


pub struct Drones
{
    ctx: zmq::Context,
    running: Arc<AtomicBool>,
    pub drones: Arc<Mutex<Vec<UAV>>>,
    objects: Arc<Mutex<Objects>>,
    _state_publisher: Option<thread::JoinHandle<()>>,
    nextID: usize,
    slots: DVector<usize>
}

impl Drones
{
    pub fn new(_ctx: zmq::Context,objects: Arc<Mutex<Objects>>) -> Self {
        let port: usize = ServerConfig::get_usize("drones_port"); 
        let client_limit: usize = ServerConfig::get_usize("client_limit");
        let notify_type_scaler: usize = ServerConfig::get_usize("notifyTypeScaler");
        let mut counter: usize = 0;

        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        let drones = Arc::new(Mutex::new(Vec::<UAV>::new()));
        let drones_arc = drones.clone();
        let publisher_socket = _ctx.socket(zmq::PUB).expect("Pub socket error");
        let slots: DVector<usize> = DVector::zeros(client_limit);
        let publisher: JoinHandle<()> = thread::spawn(move ||
        {
            publisher_socket.bind(format!("tcp://*:{}",port).as_str()).expect(format!("Bind error tcp {}",port).as_str());
            printLog!("State publisher started on TCP: {}", port);
            while r.load(Ordering::SeqCst) {
                let drones = drones_arc.lock().unwrap();
                if !drones.is_empty()
                {
                    let mut timeToNotify = false;
                    let mut notifyTypesMsg = String::new();
                    counter += 1;
                    if counter == notify_type_scaler
                    {
                        timeToNotify = true;
                        counter = 0;
                        notifyTypesMsg.reserve(drones.len()*50);
                        notifyTypesMsg.push_str("t:");
                    }
                    let mut result = String::with_capacity(drones.len()*320);

                    for elem in drones.iter()  {
                        result.push_str(&elem.id.to_string());
                        result.push(',');
                        let state = elem.state_arc.lock().unwrap();
                        result.push_str(&state.to_string());
                        drop(state);
                        result.push(';');
                        if timeToNotify
                        {
                            notifyTypesMsg.push_str(&elem.id.to_string());
                            notifyTypesMsg.push(',');
                            notifyTypesMsg.push_str(&elem.config.drone_type);
                            notifyTypesMsg.push(';');
                        }
                    }
                    publisher_socket.send(&result, 0).unwrap();
                    if timeToNotify
                    {
                        Notification::sendMsg(&notifyTypesMsg);
                    }
                    //printLog!("{}",result);
                }
                drop(drones);
                thread::sleep(time::Duration::from_millis(10));
            }
        });
        Drones {ctx: _ctx, running: running, drones: drones, objects: objects,
             _state_publisher: Some(publisher), nextID: 1, slots }
    }

    fn getSlot(&mut self, id: usize) -> Option<usize>
    {
        for (i, slot) in self.slots.iter_mut().enumerate()
        {
            if *slot == 0
            {
                *slot = id;
                return Some(i)
            }
        }
        None
    }
    fn freeSlot(&mut self, id: usize)
    {
        
        for (i,slot) in self.slots.iter_mut().enumerate()
        {
            if *slot == id
            {
                Self::sendTerminate(self.ctx.clone(),i);
                *slot = 0;
            }
        }
    }

    fn sendTerminate(ctx: zmq::Context, slot_no: usize)
    {
        let stopSocket = ctx.socket(zmq::SocketType::PUB).unwrap();
        stopSocket.bind(&format!("inproc://stop{}",slot_no)).unwrap();
        stopSocket.send("TERMINATE", 0).unwrap();
        drop(stopSocket);
    }

    pub fn startUAV(&mut self, name: &str, config_path: &str) -> (usize,usize,String)
    {
        let slot = self.getSlot(self.nextID);
        if slot.is_none()
        {
            return (0,0,"".to_owned())
        }
        let slot = slot.unwrap();
        let state = Arc::new(Mutex::new(DroneState::new()));
        let mut drone = self.drones.lock().unwrap();
        let id = self.nextID;
        self.nextID += 1;
        drone.push(UAV::new(&mut self.ctx,id, name, config_path, state,self.objects.clone()));
        drop(drone);
        (id,slot,format!("ipc:///tmp/{}/steer", name))
    }

    pub fn removeUAV(&mut self, id: usize)
    {
        self.freeSlot(id);
        let mut drone = self.drones.lock().unwrap();
        drone.retain_mut(|d| d.id != id);
        drop(drone);
    }

    pub fn removeAllUAV(&mut self)
    {
        let mut drone = self.drones.lock().unwrap();
        drone.clear();
        self.slots.apply(|x| *x = 0);
        drop(drone);
    }

    pub fn printState(&self)
    {
        for (i, item) in self.drones.lock().unwrap().iter().enumerate() {
            let state = item.state_arc.lock().unwrap();
            printLog!("{}:{}",i,state.to_string());
        }
    }

    pub fn getPositions(&self) -> Vec<(usize,Vector3<f32>)>
    {
        let mut pos = Vec::new();
        let drone = self.drones.lock().unwrap();
        if !drone.is_empty()
        {

            for elem in drone.iter()  {
                pos.push((elem.id,elem.state_arc.lock().unwrap().getPos3()));
            }
        }
        drop(drone);
        pos
    }

    pub fn getPosOriVels(&self) -> Vec<(usize,Vector3<f32>,Vector4<f32>,Vector3<f32>,Vector3<f32>)>
    {
        let mut pos = Vec::new();
        let drone = self.drones.lock().unwrap();
        if !drone.is_empty()
        {

            for elem in drone.iter()  {
                let state_lck = elem.state_arc.lock().unwrap();
                pos.push((elem.id,state_lck.getPos3(),state_lck.getOri(),state_lck.getVel(),state_lck.getAngVel()));
            }
        }
        drop(drone);
        pos
    }

    pub fn getPosOriRPYVels(&self) -> Vec<(usize,Vector3<f32>,Vector3<f32>,Vector3<f32>,Vector3<f32>)>
    {
        let mut pos = Vec::new();
        let drone = self.drones.lock().unwrap();
        if !drone.is_empty()
        {

            for elem in drone.iter()  {
                let state_lck = elem.state_arc.lock().unwrap();
                pos.push((elem.id,state_lck.getPos3(),state_lck.getOriRPY(),state_lck.getVel(),state_lck.getAngVel()));
            }
        }
        drop(drone);
        pos
    }

    pub fn getRotorPos(&self) -> Vec<Matrix3xX<f32>>
    {
        let mut rotor_pos = Vec::<Matrix3xX<f32>>::new();
        let drone = self.drones.lock().unwrap();
        if !drone.is_empty()
        {
            for elem in drone.iter()  {
                rotor_pos.push(elem.config.rotors.positions.clone());
            }
        }
        drop(drone);
        rotor_pos
    }

    pub fn updateForce(&self, id: &usize, force: &Vector3<f32>, torque: &Vector3<f32>)
    {
        let drone_lck = self.drones.lock().unwrap();
        if let Some(uav) = drone_lck.iter().find(|uav| uav.id == *id)
        {
            uav.updateForce(force,torque);
        }
        drop(drone_lck);
    }

    pub fn sendSurfaceCollison(&self, id: &usize, COR: f32, mi_s: f32, mi_d: f32, collisionPoint: &Vector3<f32>, normalVector: &Vector3<f32>)
    {
        let drone_lck = self.drones.lock().unwrap();
        if let Some(uav) = drone_lck.iter().find(|uav| uav.id == *id)
        {
            uav.sendSurfaceCollison(COR, mi_s, mi_d, collisionPoint, normalVector);
        }
        drop(drone_lck);
    }
}

impl Drop for Drones{
    fn drop(&mut self) {
        printLog!("Dropping drones instance");
        self.running.store(false, Ordering::SeqCst);
        self._state_publisher.take().unwrap().join().expect("Join error");
        self.removeAllUAV();
        printLog!("Drones instance dropped");
    }
}
