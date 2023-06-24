use std::{process::{Command, Child, Stdio}, thread::{self, JoinHandle}, time, sync::{Mutex, Arc}};


pub struct DroneState
{
    time: f32,
    pos: [f32; 6],
    vel: [f32; 6],
    om: Vec<f32>,
}

impl DroneState {
    pub fn new() -> Self {
        DroneState {time: -1.0, pos: [-1.0; 6], vel: [-1.0; 6], om: Vec::new()}
    }
}

impl ToString for DroneState {
    fn to_string(&self) -> String {
        format!("t: {}", self.time)
    }
}

pub struct UAV
{
    name: String,
    simulation: Child,
    controller: Child,
    steer_socket: zmq::Socket,
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

            state_listener: Option::None
        };

        uav.steer_socket.connect(&format!("ipc:///tmp/{}/steer",uav.name.to_owned()))
                        .expect("steer connect error");

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
            let mut array: [f32;6] = [-1.0; 6];
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

    pub fn sendSteeringMsg(self, msg: &str)
    {
        println!("Msg: {}", msg);
    }
}

impl Drop for UAV {
    fn drop(&mut self) {
        self.simulation.wait().expect("sim wait");
        self.controller.wait().expect("controller wait");
        println!("Drone eliminated: {}!", self.name); 
    } 
}