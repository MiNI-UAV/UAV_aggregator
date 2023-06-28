use std::{process::{Command, Child, Stdio}, thread::{self, JoinHandle}, time, sync::{Mutex, Arc}};
use ndarray::{Array1,arr1};


pub struct DroneState
{
    time: f32,
    pos: Array1<f32>,
    vel: Array1<f32>,
    om: Vec<f32>,
}

impl DroneState {
    pub fn new() -> Self {
        DroneState {time: -1.0, pos: arr1(&[-1.0; 6]), vel: arr1(&[-1.0; 6]), om: Vec::new()}
    }

    pub fn getPos(&self) -> Array1<f32>
    {
        self.pos.clone()
    }
}

impl ToString for DroneState {
    fn to_string(&self) -> String {
        let mut result = String::with_capacity(200);
        result.push_str(&self.time.to_string());
        result.push(',');
        for i in 0..6 {
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
    name: String,
    simulation: Child,
    controller: Child,
    steer_socket: zmq::Socket,
    control_socket: zmq::Socket,
    state_listener: Option<JoinHandle<()>>
}

impl UAV
{
    pub fn new(_ctx: &mut zmq::Context, name: &str, state: Arc<Mutex<DroneState>>) -> Self {
        let mut uav = UAV 
        {
            name: name.to_string(),

            simulation: Command::new("../UAV_physics_engine/build/uav")
            .arg("-c").arg("/home/wgajda/Desktop/Development/UAV/UAV_aggregator/config.xml")
            .arg("-n").arg(name)
            .stdout(Stdio::null())
            .spawn()
            .expect("failed to execute simulation process"),

            controller: Command::new("../UAV_controller/build/controller")
            .arg("-c").arg("/home/wgajda/Desktop/Development/UAV/UAV_aggregator/config.xml")
            .arg("-n").arg(name)
            .stdout(Stdio::null())
            .spawn()
            .expect("failed to execute controller process"),

            steer_socket:  _ctx.socket(zmq::PUB)
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
        println!("Created new drone: {}!", uav.name);      

        uav
    }

    fn startListeners(_ctx: &mut zmq::Context, uav: &mut UAV, state: Arc<Mutex<DroneState>>)
    {
        //thread::sleep(time::Duration::from_secs(3));
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
            let mut array: Array1<f32> = arr1(&[-1.0; 6]);
            let trimmed = msg.chars().skip(start).collect::<String>();
            let items = trimmed.split(',').take(6);

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
                //println!("{}", s);
                let t = s[2..].parse::<f32>().expect("parse t error");

                pos_socket.recv(&mut msg, 0).unwrap();
                let s = msg.as_str().unwrap();
                //println!("{}", s);
                let pos = parseToArray(s,4);

                vel_socket.recv(&mut msg, 0).unwrap();
                let s = msg.as_str().unwrap();
                //println!("{}", s);
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
                thread::sleep(time::Duration::from_millis(10));
            }
        }));
    }

    fn _sendSteeringMsg(&self, msg: &str)
    {
        self.steer_socket.send(&msg, 0).unwrap();
    }

    fn _sendControlMsg(&self, msg: &str)
    {
        self.control_socket.send(&msg, 0).unwrap();
        let mut msg = zmq::Message::new();
        self.control_socket.recv(&mut msg, 0).unwrap();
        assert!(msg.as_str().unwrap().contains("ok"));
    }

    pub fn sendWind(&self, wind: &Array1<f32>)
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

    pub fn dropOrShot(&self, mut mass: Option<f32>, mut speed: Option<f32>, mut r: Option<[f32;3]>)
    {
        let mut command = String::with_capacity(30);
        command.push_str("d:");
        command.push_str(&mass.get_or_insert(0.03).to_string());
        command.push(',');
        command.push_str(&speed.get_or_insert(150.0).to_string());
        command.push(',');
        let r = r.get_or_insert([0.0,0.0,0.1]);
        command.push_str(&r[0].to_string());
        command.push(',');
        command.push_str(&r[1].to_string());
        command.push(',');
        command.push_str(&r[2].to_string());
        self._sendControlMsg(&command);
    }

}

impl Drop for UAV {
    fn drop(&mut self) {
        self.simulation.wait().expect("sim wait");
        self.controller.wait().expect("controller wait");
        println!("Drone eliminated: {}!", self.name); 
    } 
}