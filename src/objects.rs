use std::{thread::{self, JoinHandle}, sync::{Arc, Mutex}, process::{Command, Stdio, Child}};
use ndarray::{Array1,arr1};


pub struct ObjectState
{
    id: usize,
    pos: Array1<f32>,
    vel: Array1<f32>,
}

impl ObjectState {
    pub fn new() -> Self {
        ObjectState {id: 0, pos: arr1(&[-1.0; 6]), vel: arr1(&[-1.0; 6])}
    }

    pub fn fromInfo(info: &str) -> Self {
        let mut id: usize = 0;
        let mut pos: Array1<f32> = arr1(&[-1.0;3]);
        let mut vel: Array1<f32> = arr1(&[-1.0;3]);
        let list = info.split(";");
        for (i,elem) in list.into_iter().enumerate()
        {
            match i {
                0 =>{ id = elem.parse().unwrap();},
                1..=3 =>{ pos[i-1] = elem.parse().unwrap();},
                4..=6 =>{ vel[i-4] = elem.parse().unwrap();},
                _ => {}
            }
        }
        ObjectState {id: id, pos: pos, vel: vel}
    }

    pub fn getPos(&self) -> Array1<f32>
    {
        self.pos.clone()
    }
}

pub struct Objects
{
    _ctx: zmq::Context,
    pub _time: Arc<Mutex<f32>>,
    pub states: Arc<Mutex<Vec<ObjectState>>>,

    _drop_physic: Child,
    _state_proxy: JoinHandle<()>,
    _state_cupturer: JoinHandle<()>
}

impl Objects
{
    pub fn new(_ctx: zmq::Context) -> Self {
        let drop_physic = Command::new("../UAV_drop_physic/build/drop")
        .stdout(Stdio::null())
        .spawn()
        .expect("failed to execute drop physic process");
        let states = Arc::new(Mutex::new(Vec::new()));
        let time = Arc::new(Mutex::new(0.0));
        let ctx = _ctx.clone();
        let proxy: JoinHandle<()> = thread::spawn(move ||
        {
            let mut capture_socket = ctx.socket(zmq::PAIR).expect("Capture socket error");
            capture_socket.bind("inproc://obj_state").unwrap();
            let mut listener_socket = ctx.socket(zmq::XSUB).expect("Sub socket error");
            listener_socket.set_subscribe(b"").unwrap();
            listener_socket.connect("ipc:///tmp/drop_shot/state").unwrap();
            let mut publisher_socket = ctx.socket(zmq::XPUB).expect("Pub socket error");
            publisher_socket.bind("tcp://127.0.0.1:9100").expect("Bind error tcp 9100");
            println!("Object state proxy started on TCP: {}", 9100);
            zmq::proxy_with_capture(&mut listener_socket,&mut publisher_socket,&mut capture_socket).unwrap();
        });
        let ctx = _ctx.clone();
        let time_access = time.clone();
        let states_access = states.clone();
        let capture: JoinHandle<()> = thread::spawn(move ||
        {
            let capture_socket = ctx.socket(zmq::PAIR).expect("Capture socket error");
            capture_socket.connect("inproc://obj_state").unwrap();
            let mut obj_states_msg =  zmq::Message::new();
            capture_socket.recv(&mut obj_states_msg, 0).unwrap();
            let obj_info = obj_states_msg.as_str().unwrap().to_string();
            Self::parseInfo(time_access,states_access,obj_info);
        });
        Objects {_ctx: _ctx,_time: time,states: states,_drop_physic: drop_physic, _state_proxy: proxy, _state_cupturer: capture}
    }

    fn parseInfo(time: Arc<Mutex<f32>>, states: Arc<Mutex<Vec<ObjectState>>>, info: String)
    {
        let mut newStates = Vec::new();
        let list = info.split(";");
        for (i,elem) in list.into_iter().enumerate()
        {
            if i == 0
            {
                let mut time_lck = time.lock().unwrap() ;
                *time_lck = elem.parse::<f32>().unwrap();
                drop(time_lck);
                continue;
            }
            newStates.push(ObjectState::fromInfo(elem));
        }
        let mut state_lck = states.lock().unwrap();
        *state_lck = newStates;
        drop(state_lck);
    }

}

impl Drop for Objects {
    fn drop(&mut self) {
        self._drop_physic.wait().expect("drop wait");
    } 
}