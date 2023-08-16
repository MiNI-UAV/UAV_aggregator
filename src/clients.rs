use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, collections::HashSet, io::Write};
use nalgebra::Vector3;
use std::fs::File;
use std::path::Path;
use std::str;
use sha1::{Sha1, Digest};

use crate::{drones::Drones, cargo::Cargo, config::ServerConfig};

const DRONE_CONFIGS_PATH: &str = "./configs/drones_configs/";

pub struct Clients
{
    running: Arc<AtomicBool>,
    _proxies: Arc<Mutex<Vec<Option<thread::JoinHandle<()>>>>>,
    _control: Arc<Mutex<Vec<Option<thread::JoinHandle<()>>>>>,
    _replyer: Option<thread::JoinHandle<()>>,
}

impl Clients
{
    pub fn new(_ctx: zmq::Context, drones: Arc<Mutex<Drones>>, cargo: Arc<Mutex<Cargo>>) -> Self {
        let hb_disconnect: usize = ServerConfig::get_usize("hb_disconnect");
        let replyer_port: usize = ServerConfig::get_usize("replyer_port");
        let first_port: usize = ServerConfig::get_usize("first_port");
        
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        let replyer_socket = _ctx.socket(zmq::REP).expect("REP socket error");
        let mut taken_name =  HashSet::<String>::new();
        let proxies = Arc::new(Mutex::new(Vec::<Option::<JoinHandle<()>>>::new()));
        let p = proxies.clone();
        let control = Arc::new(Mutex::new(Vec::<Option::<JoinHandle<()>>>::new()));
        let c = proxies.clone();
        let replyer: JoinHandle<()> = thread::spawn(move ||
        {
            replyer_socket.set_rcvtimeo(1000).unwrap();
            replyer_socket.bind(format!("tcp://*:{}",replyer_port).as_str()).expect(format!("Bind error tcp {}",replyer_port).as_str());
            println!("Replyer started on TCP: {}", replyer_port);
            while r.load(Ordering::SeqCst) {
                let mut request =  zmq::Message::new();
                if let Err(_) = replyer_socket.recv(&mut request, 0)
                {
                    continue;
                }
                let request = request.as_str().unwrap().to_string();
                match request.chars().next().unwrap(){
                    's' => {
                    let mut command = request[2..].splitn(2,';');
                    let mut drone_name = command.next().unwrap().to_owned();
                    let config_name = command.next().get_or_insert("config").to_owned();
                    let mut config_path = DRONE_CONFIGS_PATH.to_string();
                    config_path.push_str(&config_name);
                    config_path.push_str(".xml");
                    if drone_name.is_empty(){
                        let mut reply = String::new();
                        reply.push_str("-1");
                        replyer_socket.send(&reply, 0).unwrap(); 
                        continue;
                    }
                    if !Path::new(&config_path).exists()
                    {
                        let mut reply = String::new();
                        reply.push_str("-2");
                        replyer_socket.send(&reply, 0).unwrap(); 
                        continue;
                    }
                    let no = taken_name.iter().map(|name|  if name.contains(&drone_name) {1} else {0}).count();
                    if no > 0
                    {
                        drone_name.push('_');
                        drone_name.push_str(&no.to_string());
                    }
                    taken_name.insert(drone_name.to_string());
                    let mut drones_lck = drones.lock().unwrap();
                    let (drone_no, slot, uav_address) = drones_lck.startUAV(&drone_name,&config_path);
                    if drone_no == 0
                    {
                        let mut reply = String::new();
                        reply.push_str("-3");
                        replyer_socket.send(&reply, 0).unwrap(); 
                        continue;
                    }
                    drop(drones_lck);
                    println!("Started new drone with name: {}", drone_name);
                    let mut steer_pair_socket = _ctx.socket(zmq::PAIR).unwrap();
                    let address = format!("tcp://*:{}", first_port+slot);
                    steer_pair_socket.bind(&address).unwrap();
                    let mut steer_xpub_socket = _ctx.socket(zmq::XPUB).unwrap();
                    steer_xpub_socket.connect(&uav_address).unwrap();
                    let mut stop_sub_socket = _ctx.socket(zmq::SUB).unwrap();
                    stop_sub_socket.set_subscribe(b"").unwrap();
                    stop_sub_socket.connect(&format!("inproc://stop{}",slot)).unwrap();
                    let mut proxy = p.lock().unwrap();
                    proxy.push(Some(
                        thread::spawn(move ||
                        {
                            zmq::proxy_steerable(&mut steer_pair_socket, &mut steer_xpub_socket,&mut stop_sub_socket).expect("Proxy err");
                            println!("Closing client proxy");

                        }))
                    );
                    drop(proxy);
                    println!("Ready to connect steer client on TCP: {}", first_port+slot);

                    let control_pair_socket = _ctx.socket(zmq::PAIR).unwrap();
                    let mut control = c.lock().unwrap();
                    let r2 = r.clone();
                    let d2 = drones.clone();
                    let c2 = cargo.clone();
                    control.push(Some(
                        thread::spawn(move ||
                        {
                            let mut skipedHeartbeats: usize = 0;
                            let mut local_running = true;
                            control_pair_socket.set_rcvtimeo(1000).unwrap();
                            let address = format!("tcp://*:{}", first_port+slot+1000);
                            control_pair_socket.bind(&address).unwrap();
                            while r2.load(Ordering::SeqCst) && local_running {
                                let mut request =  zmq::Message::new();
                                if let Err(_) = control_pair_socket.recv(&mut request, 0)
                                {
                                    skipedHeartbeats += 1;
                                    if skipedHeartbeats == hb_disconnect
                                    {
                                        let mut d_lck = d2.lock().unwrap();
                                        local_running = false;
                                        d_lck.removeUAV(drone_no);
                                        drop(d_lck);
                                    }
                                    continue;
                                }
                                let mut d_lck = d2.lock().unwrap();
                                let mut cargo_lck = c2.lock().unwrap();
                                Clients::handleControlMsg(request.as_str().unwrap(), drone_no, &mut d_lck, &mut cargo_lck, &mut skipedHeartbeats);
                                drop(cargo_lck);
                                drop(d_lck);
                            }
                        })
                    ));
                    drop(control);
                    println!("Ready to connect control client on TCP: {}", first_port+slot+1000);

                    let mut reply = String::with_capacity(30);
                    reply.push_str(&drone_no.to_string());
                    reply.push(',');
                    reply.push_str(&(first_port+slot).to_string());
                    reply.push(',');
                    reply.push_str(&(first_port+slot+1000).to_string());
                    replyer_socket.send(&reply, 0).unwrap();
                },
                'c' => {
                    let content = &request[2..];
                    let mut hasher = Sha1::new();
                    hasher.update(content.as_bytes());
                    let hash_val = hasher.finalize();
                    let hash_val = hex::encode(&hash_val[..]);
                    let hash_val = &hash_val[0..8];
                    println!("Creating/updateing file {}.xml", &hash_val);
                    let mut file_name = DRONE_CONFIGS_PATH.to_string();
                    file_name.push_str(&hash_val);
                    file_name.push_str(".xml");
                    let mut file = File::create(file_name).unwrap();
                    file.write_all(content.as_bytes()).expect("Unable to write config");
                    drop(file);
                    replyer_socket.send("ok", 0).unwrap();
                },
                _ => println!("Unknown command: {}", request)
            }
            }
        });
        Clients{running: running, _proxies: proxies, _control: control, _replyer: Some(replyer)}
    }

    fn handleControlMsg(msg: &str, drone_no: usize, drones: &mut Drones, cargo: &mut Cargo,  skipedHeartbeats: &mut usize)
    {
        let mut d = drones.drones.lock().unwrap();
        if let Some(drone) = d.iter().find(|drone| drone.id == drone_no)
        {
            match msg {
                "beep" => {  
                    *skipedHeartbeats = 0;
                }
                "shot" => {  
                    drone.dropOrShot(None, None, None, None);
                }
                "drop" => {
                    // 20cm ball
                    let id = drone.dropOrShot(Some(0.2), Some(0.0), Some(0.015), None);
                    if id >= 0
                    {
                        cargo.addLink(drone_no, id as usize, 2.0, 5.0, Vector3::zeros());
                    }
                }
                "kill" => {  
                    d.retain_mut(|d| d.id != drone_no);
                }
                _ => {
                    println!("Unknown command: {}", msg);
                }
            }
        }
        drop(d);
    }
}

impl Drop for Clients{
    fn drop(&mut self) {
        println!("Dropping clients instance");
        self.running.store(false, Ordering::SeqCst);
        self._replyer.take().unwrap().join().expect("Join error");
        println!("Main client thread dropped");
        let mut proxy = self._proxies.lock().unwrap();
        while let Some(handler) = proxy.pop()
        {
            if let Some(h) = handler
            {
                h.join().expect("Proxy joining error");
            }
        }
        drop(proxy);
        let mut control = self._control.lock().unwrap();
        while let Some(handler) = control.pop()
        {
            if let Some(h) = handler
            {
                h.join().expect("Control joining error");
            }
        }
        drop(control); 
        println!("Clients instance dropped");
    }
}