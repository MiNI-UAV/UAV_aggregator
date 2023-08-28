use std::{process::{Command, Child, Stdio}, thread::{self, JoinHandle}, time, sync::{Mutex, Arc}};
use nalgebra::{Vector3,Vector6, SVector, Vector4};
use crate::objects::Objects;
use crate::config::DroneConfig;
use crate::printLog;



pub struct DroneState
{
    time: f32,
    pos: SVector<f32,7>,
    vel: Vector6<f32>,
    om: Vec<f32>,
}

impl DroneState {
    pub fn new() -> Self {
        DroneState {time: -1.0, pos: SVector::repeat(-1.0f32), vel: Vector6::repeat(-1.0f32), om: Vec::new()}
    }

    pub fn getPos3(&self) -> Vector3<f32>
    {
        self.pos.fixed_view::<3, 1>(0, 0).into()
    }

    pub fn getOri(&self) -> Vector4<f32>
    {
        self.pos.fixed_view::<4, 1>(3, 0).into()
    }

    pub fn getOriRPY(&self) -> Vector3<f32>
    {
        let q: Vector4<f32> = self.pos.fixed_view::<4, 1>(3, 0).into();
        Self::quaterionsToRPY(q)
    }

    pub fn getVel(&self) -> Vector3<f32>
    {
        self.vel.fixed_view::<3, 1>(0, 0).into()
    }

    pub fn getAngVel(&self) -> Vector3<f32>
    {
        self.vel.fixed_view::<3, 1>(3, 0).into()
    }

    fn quaterionsToRPY(e: Vector4<f32>) -> Vector3<f32>
    {
        let mut RPY = Vector3::<f32>::zeros();
        RPY[0] = (2.0*(e[0]*e[1]+e[2]*e[3])).atan2(e[0]*e[0]-e[1]*e[1]-e[2]*e[2]+e[3]*e[3]);
        RPY[1] = (2.0*(e[0]*e[2]-e[1]*e[3])).asin();
        RPY[2] = (2.0*(e[0]*e[3]+e[1]*e[2])).atan2(e[0]*e[0]+e[1]*e[1]-e[2]*e[2]-e[3]*e[3]);
        RPY
    }
}

impl ToString for DroneState {
    fn to_string(&self) -> String {
        let mut result = String::with_capacity(200);
        result.push_str(&self.time.to_string());
        result.push(',');
        for i in 0..7 {
            result.push_str(&self.pos[i].to_string());
            result.push(','); 
        }
        for i in 0..6 {
            result.push_str(&self.vel[i].to_string());
            result.push(','); 
        }
        for elem in &self.om {
            result.push_str(&elem.to_string());
            result.push(','); 
        }
        result.pop();
        result
    }
}

pub struct UAV
{
    pub id: usize,
    pub name: String,
    pub state_arc: Arc<Mutex<DroneState>>,
    pub config : DroneConfig,

    objects_arc: Arc<Mutex<Objects>>,  
    simulation: Child,
    controller: Child,
    steer_socket: zmq::Socket,
    control_socket: zmq::Socket,
    state_listener: Option<JoinHandle<()>>
}

impl UAV
{
    pub fn new(_ctx: &mut zmq::Context,id : usize , name: &str, config_path: &str, state: Arc<Mutex<DroneState>>, objects: Arc<Mutex<Objects>>) -> Self {
        let config = DroneConfig::parse(&config_path).expect("Config file error");
        let mut uav = UAV 
        {
            id,

            name: name.to_string(),

            state_arc: state.clone(),

            config,

            objects_arc: objects,

            simulation: Command::new("../UAV_physics_engine/build/uav")
            .arg("-c").arg(&config_path)
            .arg("-n").arg(name)
            .stdout(Stdio::null())
            .spawn()
            .expect("failed to execute simulation process"),

            controller: Command::new("../UAV_controller/build/controller")
            .arg("-c").arg(&config_path)
            .arg("-n").arg(name)
            .stdout(Stdio::null())
            .spawn()
            .expect("failed to execute controller process"),

            steer_socket:  _ctx.socket(zmq::REQ)
                                .expect("creating socket error"),

            control_socket:  _ctx.socket(zmq::REQ)
                                .expect("creating socket error"),

            state_listener: Option::None
        };

        uav.steer_socket.connect(&format!("ipc:///tmp/{}/steer",uav.name.to_owned()))
                        .expect("steer connect error");
        uav.control_socket.connect(&format!("ipc:///tmp/{}/control",uav.name.to_owned()))
                        .expect("control connect error");

        UAV::startListeners(_ctx, &mut uav, state);
        printLog!("Created new drone: {}!", uav.name);      

        uav
    }

    fn startListeners(_ctx: &mut zmq::Context, uav: &mut UAV, state: Arc<Mutex<DroneState>>)
    {
        let state_address = format!("ipc:///tmp/{}/state",uav.name.to_owned());

        let buildSocket = |topic: &str|
        {
            let socket = _ctx.socket(zmq::SUB).expect("state sub error");
            socket.set_conflate(true).expect("socket setting error");
            socket.set_subscribe(topic.as_bytes()).unwrap();
            socket.connect(&state_address).expect("state connect error");
            socket
        };

        let parseToArray = |msg: &str, start: usize|
        {
            let mut array =  Vector6::repeat(-1.0f32);
            let trimmed = msg.chars().skip(start).collect::<String>();
            let items = trimmed.split(',').take(6);

            for (i, item) in items.enumerate() {
                if let Ok(parsed_value) = item.trim().parse::<f32>() {
                    array[i] = parsed_value;
                }
            }
            array
        };

        let parseToArray7 = |msg: &str, start: usize|
        {
            let mut array =  SVector::<f32,7>::repeat(-1.0f32);
            let trimmed = msg.chars().skip(start).collect::<String>();
            let items = trimmed.split(',').take(7);

            for (i, item) in items.enumerate() {
                if let Ok(parsed_value) = item.trim().parse::<f32>() {
                    array[i] = parsed_value;
                }
            }
            array
        };
        
        let t_socket = buildSocket("t");
        let pos_socket = buildSocket("pos");
        let vel_socket = buildSocket("vn");
        let om_socket = buildSocket("om");

        uav.state_listener = Option::Some(thread::spawn(move || {
            let mut msg = zmq::Message::new();
            loop {

                t_socket.recv(&mut msg, 0).unwrap();
                let s = msg.as_str().unwrap();
                //printLog!("{}", s);
                let t = s[2..].parse::<f32>().expect("parse t error");

                pos_socket.recv(&mut msg, 0).unwrap();
                let s = msg.as_str().unwrap();
                //printLog!("{}", s);
                let pos = parseToArray7(s,4);

                vel_socket.recv(&mut msg, 0).unwrap();
                let s = msg.as_str().unwrap();
                //printLog!("{}", s);
                let vel = parseToArray(s,3);

                om_socket.recv(&mut msg, 0).unwrap();
                let s = msg.as_str().unwrap();
                let trimmed_input = &s[3..];
                let om: Vec<f32> = trimmed_input
                        .split(',')
                        .map(|item| item.trim().parse::<f32>())
                        .filter_map(Result::ok)
                        .collect();
                
                let mut state = state.lock().unwrap();
                state.time = t;
                state.pos = pos;
                state.vel = vel;
                state.om = om;
                drop(state);
                thread::sleep(time::Duration::from_millis(10));
            }
        }));
    }

    fn _sendSteeringMsg(&self, msg: &str)
    {
        self.steer_socket.send(&msg, 0).unwrap();
    }

    fn _sendControlMsg(&self, msg_str: &str) -> String
    {
        self.control_socket.send(&msg_str, 0).unwrap();
        let mut msg = zmq::Message::new();
        if self.control_socket.recv(&mut msg, 0).is_ok()
        {
            let rep = msg.as_str().unwrap();
            //printLog!("{}", msg_str);
            assert!(rep.contains("ok"));
            rep.to_string()
        }
        else {
            printLog!("Error while sending: {}", msg_str);
            String::new()
        }      
    }

    pub fn sendWind(&self, wind: &Vector3<f32>)
    {
        let mut command = String::with_capacity(30);
        command.push_str("w:");
        command.push_str(&wind[0].to_string());
        command.push(',');
        command.push_str(&wind[1].to_string());
        command.push(',');
        command.push_str(&wind[2].to_string());
        self._sendControlMsg(&command);
    }

    pub fn updateForce(&self, force: &Vector3<f32>, torque: &Vector3<f32>)
    {
        let mut command = String::with_capacity(30);
        command.push_str("f:");
        command.push_str(&force[0].to_string());
        command.push(',');
        command.push_str(&force[1].to_string());
        command.push(',');
        command.push_str(&force[2].to_string());
        command.push(',');
        command.push_str(&torque[0].to_string());
        command.push(',');
        command.push_str(&torque[1].to_string());
        command.push(',');
        command.push_str(&torque[2].to_string());

        self._sendControlMsg(&command);
    }


    pub fn dropOrShot(&self, mut mass: Option<f32>, mut speed: Option<f32>, mut CS: Option<f32>, mut r: Option<[f32;3]>) -> isize
    {
        //9mm bullet
        // let mass = mass.get_or_insert(0.008);
        // let speed = speed.get_or_insert(350.0);
        // let CS = CS.get_or_insert(0.00001876708);
        // let r = r.get_or_insert([0.0,0.0,0.1]);

        //paintball
        let mass = mass.get_or_insert(0.003);
        let speed = speed.get_or_insert(90.0);
        let CS = CS.get_or_insert(0.47*0.000126645);
        let r = r.get_or_insert([0.0,0.0,0.1]);

        let mut command = String::with_capacity(30);
        command.push_str("d:");
        command.push_str(&mass.to_string());
        command.push(',');
        command.push_str(&speed.to_string());
        command.push(',');
        command.push_str(&r[0].to_string());
        command.push(',');
        command.push_str(&r[1].to_string());
        command.push(',');
        command.push_str(&r[2].to_string());
        let rep = self._sendControlMsg(&command);
        let mut vel = Vector3::zeros();
        for (i,elem) in rep.split(';').skip(1).next().get_or_insert("0.0,0.0,0.0").split(",").enumerate()
        {
            vel[i] = elem.parse::<f32>().unwrap();
        }
        let state = self.state_arc.lock().unwrap();
        let pos = state.getPos3();
        drop(state);
        let objects = self.objects_arc.lock().unwrap();
        let id = objects.addObj(*mass, *CS, pos, vel);
        drop(objects);
        id
    }

    pub fn sendSurfaceCollison(&self, COR: f32, mi_s: f32, mi_d: f32,
        collisionPoint: &Vector3<f32>, normalVector: &Vector3<f32>)
    {
        let mut command = String::with_capacity(30);
        command.push_str("j:");
        command.push_str(&COR.to_string());
        command.push(',');
        command.push_str(&mi_s.to_string());
        command.push(',');
        command.push_str(&mi_d.to_string());
        command.push(',');
        command.push_str(&collisionPoint[0].to_string());
        command.push(',');
        command.push_str(&collisionPoint[1].to_string());
        command.push(',');
        command.push_str(&collisionPoint[2].to_string());
        command.push(',');
        command.push_str(&normalVector[0].to_string());
        command.push(',');
        command.push_str(&normalVector[1].to_string());
        command.push(',');
        command.push_str(&normalVector[2].to_string());
        //printLog!("{}", command);
        self._sendControlMsg(&command);
    }

}

impl Drop for UAV {
    fn drop(&mut self) {
        printLog!("Dropping drone: {}", self.name);
        self._sendSteeringMsg("c:exit");
        self.simulation.wait().expect("sim wait");
        self.controller.wait().expect("controller wait");
        printLog!("Drone eliminated: {}!", self.name); 
    } 
}