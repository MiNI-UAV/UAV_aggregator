use std::{sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}}, thread::{JoinHandle, self}, time::{self, Instant}};
use nalgebra::{Vector3,Vector4, DVector};
use crate::{uav::{UAV,DroneState}, notification::{Notification, PromptColor, PromptCategory}, atmosphere::GRAVITY_ACCELERATION};
use crate::objects::Objects;
use crate::config::ServerConfig;
use crate::printLog;

/// Control all UAVs in air. Communicate with simulation processes and visualizations
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
    /// Constructor. Start listener and publisher threads
    pub fn new(_ctx: zmq::Context,objects: Arc<Mutex<Objects>>) -> Self {
        let port: usize = ServerConfig::get_usize("drones_port"); 
        let client_limit: usize = ServerConfig::get_usize("client_limit");
        let mut last_notify = Instant::now();
        let notify_period = ServerConfig::get_usize("notify_period").try_into().unwrap();
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
                    if last_notify.elapsed().as_millis() > notify_period
                    {
                        last_notify = Instant::now();
                        timeToNotify = true;
                        notifyTypesMsg.reserve(drones.len()*50);
                        notifyTypesMsg.push_str("t:");
                    }
                    let mut result = String::with_capacity(drones.len()*320);

                    for elem in drones.iter()  {
                        result.push_str(&elem.id.to_string());
                        result.push(',');
                        let state = elem.state_arc.lock().unwrap();
                        result.push_str(&state.to_string());
                        if timeToNotify
                        {
                            check_acceleration(elem.id, state.getAcc(), notify_period);
                        }
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
                else
                {
                    publisher_socket.send(&";", 0).unwrap();
                }
                drop(drones);
                thread::sleep(time::Duration::from_millis(10));
            }
        });
        Drones {ctx: _ctx, running: running, drones: drones, objects: objects,
             _state_publisher: Some(publisher), nextID: 1, slots }
    }


    /// Returns first free slot, if available 
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

    /// Free specified slot - remove process in slot
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

    /// Sends terminate command to proxies on specified slot
    fn sendTerminate(ctx: zmq::Context, slot_no: usize)
    {
        let stopSocket = ctx.socket(zmq::SocketType::PUB).unwrap();
        stopSocket.bind(&format!("inproc://stop{}",slot_no)).unwrap();
        stopSocket.send("TERMINATE", 0).unwrap();
        drop(stopSocket);
    }

    /// Starts new UAV and all requiered process & threads
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

    /// Remove UAV specified by id
    pub fn removeUAV(&mut self, id: usize)
    {
        self.freeSlot(id);
        let mut drone = self.drones.lock().unwrap();
        drone.retain_mut(|d| d.id != id);
        drop(drone);
    }

    /// Remove all UAVs
    pub fn removeAllUAV(&mut self)
    {
        let mut drone = self.drones.lock().unwrap();
        drone.clear();
        self.slots.apply(|x| *x = 0);
        drop(drone);
    }

    /// Serializes all active UAV's states to string
    pub fn printState(&self)
    {
        for (i, item) in self.drones.lock().unwrap().iter().enumerate() {
            let state = item.state_arc.lock().unwrap();
            printLog!("{}:{}",i,state.to_string());
        }
    }

    /// Get position of active UAVs
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

    /// Get position & orientation & velocities of active UAVs. Orientation is given by quaterion
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

    // Get position & orientation & velocities of active UAVs. Orientation is given by Euler angles
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

    /// Get types of active UAVs
    pub fn getTypes(&self) -> Vec<String>
    {
        let mut types = Vec::new();
        let drone = self.drones.lock().unwrap();
        if !drone.is_empty()
        {
            for elem in drone.iter()  {

                types.push(elem.config.drone_type.to_owned());
            }
        }
        drop(drone);
        types
    }

    /// Update outer force for UAV specified by id
    pub fn updateForce(&self, id: &usize, force: &Vector3<f32>, torque: &Vector3<f32>)
    {
        let drone_lck = self.drones.lock().unwrap();
        if let Some(uav) = drone_lck.iter().find(|uav| uav.id == *id)
        {
            uav.updateForce(force,torque);
        }
        drop(drone_lck);
    }

    /// Sends information about collsion with surface to UAV specified by id
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

fn check_acceleration(id: usize, acceleration: Vector3<f32> , notify_period: u128)
{
    let accel = acceleration.norm()/GRAVITY_ACCELERATION;
    if accel < 4.0f32
    {
        return;
    }
    let rounded = (accel * 2.0).round() / 2.0;
    let message = format!("OVERLOAD {:.1}G", rounded);
    Notification::sendPrompt(id as isize, PromptCategory::OVERLOAD,
        if rounded >= 6.0f32 { PromptColor::RED } else { PromptColor::ORANGE },
        2* notify_period as usize, &message)
}

/// Deconstructor
impl Drop for Drones{
    fn drop(&mut self) {
        printLog!("Dropping drones instance");
        self.running.store(false, Ordering::SeqCst);
        self._state_publisher.take().unwrap().join().expect("Join error");
        self.removeAllUAV();
        printLog!("Drones instance dropped");
    }
}
