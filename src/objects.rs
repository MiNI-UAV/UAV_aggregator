use std::{thread::{self, JoinHandle}, sync::{Arc, Mutex, atomic::{Ordering, AtomicBool}}, process::{Command, Stdio, Child}, time};
use ndarray::{Array1,arr1};


#[derive(Debug)]
pub struct ObjectState
{
    pub id: usize,
    pub pos: Array1<f32>,
    pub vel: Array1<f32>,
}

impl ObjectState {
    pub fn new() -> Self {
        ObjectState {id: 0, pos: arr1(&[-1.0; 6]), vel: arr1(&[-1.0; 6])}
    }

    pub fn fromInfo(info: &str) -> Self {
        let mut id: usize = 0;
        let mut pos: Array1<f32> = arr1(&[-1.0;3]);
        let mut vel: Array1<f32> = arr1(&[-1.0;3]);
        let list = info.split(",");
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
}

pub struct Objects
{
    _ctx: zmq::Context,
    pub _time: Arc<Mutex<f32>>,
    pub states: Arc<Mutex<Vec<ObjectState>>>,
    running: Arc<AtomicBool>,

    control_socket: zmq::Socket,
    _drop_physic: Child,
    _state_proxy: Option<JoinHandle<()>>,
    _state_cupturer: Option<JoinHandle<()>>
}

impl Objects
{
    pub fn new(_ctx: zmq::Context) -> Self {
        let drop_physic = Command::new("../UAV_drop_physic/build/drop")
        .stdout(Stdio::null())
        .spawn()
        .expect("failed to execute drop physic process");
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        let states = Arc::new(Mutex::new(Vec::new()));
        let time = Arc::new(Mutex::new(0.0));
        let ctx = _ctx.clone();
        let proxy: JoinHandle<()> = thread::spawn(move ||
        {
            let mut listener_socket = ctx.socket(zmq::XSUB).expect("Sub socket error");
            listener_socket.connect("ipc:///tmp/drop_shot/state").unwrap();
            let mut publisher_socket = ctx.socket(zmq::XPUB).expect("Pub socket error");
            publisher_socket.bind("tcp://*:9100").expect("Bind error tcp 9100");
            println!("Object state proxy started on TCP: {}", 9100);
            let mut stop_sub_socket = ctx.socket(zmq::SUB).unwrap();
            stop_sub_socket.set_subscribe(b"").unwrap();
            stop_sub_socket.connect("inproc://stop").unwrap();
            zmq::proxy_steerable(&mut listener_socket,&mut publisher_socket, &mut stop_sub_socket).unwrap();
            println!("Closing objects proxy");
        });
        let ctx = _ctx.clone();
        let time_access = time.clone();
        let states_access = states.clone();
        let capture: JoinHandle<()> = thread::spawn(move ||
        {
            let capture_socket = ctx.socket(zmq::SUB).expect("Capture socket error");
            capture_socket.set_subscribe(b"").unwrap();
            capture_socket.set_rcvtimeo(1000).unwrap();
            capture_socket.set_conflate(true).unwrap();
            capture_socket.connect("ipc:///tmp/drop_shot/state").unwrap();
            while r.load(Ordering::SeqCst) {
                let mut obj_states_msg =  zmq::Message::new();
                if let Err(_) = capture_socket.recv(&mut obj_states_msg, 0)
                {
                    continue;
                }
                let obj_info = obj_states_msg.as_str().unwrap().to_string();
                Self::parseInfo(&time_access,&states_access,obj_info);
            }
        });
        let control_socket  =_ctx.socket(zmq::REQ).expect("creating socket error");
        control_socket.connect("ipc:///tmp/drop_shot/control").expect("control connect error");
        Objects {_ctx: _ctx,_time: time,states: states, running: running, control_socket: control_socket,
             _drop_physic: drop_physic, _state_proxy: Some(proxy), _state_cupturer: Some(capture)}
    }

    fn parseInfo(time: &Arc<Mutex<f32>>, states: &Arc<Mutex<Vec<ObjectState>>>, info: String)
    {
        let mut newStates = Vec::new();
        let list = info.split(";");
        for (i,elem) in list.into_iter().enumerate()
        {
            if elem.len() <= 1
            {
                continue;
            }
            if i == 0
            {
                let mut time_lck = time.lock().unwrap();
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

    fn _sendControlMsg(&self, msg: &str) -> String
    {
        self.control_socket.send(&msg, 0).unwrap();
        let mut msg = zmq::Message::new();
        self.control_socket.recv(&mut msg, 0).unwrap();
        let rep = msg.as_str().unwrap();
        assert!(rep.contains("ok"));
        rep.to_string()
    }

    pub fn addObj(&self, mass: f32, CS: f32, pos: Array1<f32>, vel: Array1<f32>)
    {
        let mut command = String::with_capacity(60);
        command.push_str("a:");
        command.push_str(&mass.to_string());
        command.push(',');
        command.push_str(&CS.to_string());
        command.push(',');
        command.push_str(&pos[0].to_string());
        command.push(',');
        command.push_str(&pos[1].to_string());
        command.push(',');
        command.push_str(&pos[2].to_string());
        command.push(',');
        command.push_str(&vel[0].to_string());
        command.push(',');
        command.push_str(&vel[1].to_string());
        command.push(',');
        command.push_str(&vel[2].to_string());
        self._sendControlMsg(&command);
    }

    pub fn removeObj(&self, id: usize)
    {
        self._sendControlMsg(&format!("r:{}",id.to_string()));
    }

    pub fn updateWind(&self, wind: Vec<(usize,Array1<f32>)>)
    {
        let mut command = String::with_capacity(30*wind.len());
        command.push_str("w:");
        for (id,wind_vec) in wind {
            command.push_str(&id.to_string());
            command.push(',');
            command.push_str(&wind_vec[0].to_string());
            command.push(',');
            command.push_str(&wind_vec[1].to_string());
            command.push(',');
            command.push_str(&wind_vec[2].to_string());
            command.push(';');
        }
        self._sendControlMsg(&command);
    }

    pub fn getPositions(&self) -> Vec<(usize,Array1<f32>)>
    {
        let mut pos = Vec::<(usize,Array1<f32>)>::new();
        let state = self.states.lock().unwrap();
        if !state.is_empty()
        {
            for elem in state.iter()  {
                pos.push((elem.id,elem.pos.clone()));
            }
        }
        drop(state);
        pos
    }

    pub fn getVelocities(&self) -> Vec<(usize,Array1<f32>)>
    {
        let mut vel = Vec::<(usize,Array1<f32>)>::new();
        let state = self.states.lock().unwrap();
        if !state.is_empty()
        {
            for elem in state.iter()  {
                vel.push((elem.id,elem.vel.clone()));
            }
        }
        drop(state);
        vel
    }

    pub fn getPosVels(&self) -> Vec<(usize,Array1<f32>, Array1<f32>)>
    {
        let mut posvel = Vec::<(usize,Array1<f32>,Array1<f32>)>::new();
        let state = self.states.lock().unwrap();
        if !state.is_empty()
        {
            for elem in state.iter()  {
                posvel.push((elem.id,elem.pos.clone(),elem.vel.clone()));
            }
        }
        drop(state);
        posvel
    }

}

impl Drop for Objects {
    fn drop(&mut self) {
        println!("Dropping objects instance");
        self.running.store(false, Ordering::SeqCst);
        loop {
            match self._drop_physic.try_wait() {
                Err(E) => println!("{}", E),
                Ok(None) =>
                {
                    self._sendControlMsg("s");
                    thread::sleep(time::Duration::from_millis(50));
                }
                Ok(_) =>
                {
                    break;
                }
            }
        }
        self._state_proxy.take().unwrap().join().expect("Join error");
        self._state_cupturer.take().unwrap().join().expect("Join error");
        println!("Objects instance dropped")
    } 
}