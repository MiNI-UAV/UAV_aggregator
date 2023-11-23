use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, collections::HashSet, io::Write};
use std::fs::{File,read_dir};
use std::path::Path;
use std::str;
use sha1::{Sha1, Digest};
use serde_json::json;
use regex::Regex;

use crate::{drones::Drones, cargo::Cargo, config::ServerConfig, checksum::getChecksum};
use crate::printLog;

/// Path to folder containing UAV's configurations
const DRONE_CONFIGS_PATH: &str = "./configs/drones_configs/";

/// Handle simulation clients - visualizations
pub struct Clients
{
    running: Arc<AtomicBool>,
    _proxies: Arc<Mutex<Vec<Option<thread::JoinHandle<()>>>>>,
    _control: Arc<Mutex<Vec<Option<thread::JoinHandle<()>>>>>,
    _replyer: Option<thread::JoinHandle<()>>,
}

impl Clients
{
    /// Contstuctor. Starts new process that handle incoming requests
    pub fn new(_ctx: zmq::Context, drones: Arc<Mutex<Drones>>, cargo: Arc<Mutex<Cargo>>) -> Self {
        let hb_disconnect: usize = ServerConfig::get_usize("hb_disconnect");
        let replyer_port: usize = ServerConfig::get_usize("replyer_port");
        let first_port: usize = ServerConfig::get_usize("first_port");
        Self::check_config_folder();
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
            printLog!("Replyer started on TCP: {}", replyer_port);
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
                        printLog!("Started new drone with name: {}", drone_name);
                        let mut steer_router_socket = _ctx.socket(zmq::ROUTER).unwrap();
                        let address = format!("tcp://*:{}", first_port+slot);
                        steer_router_socket.bind(&address).unwrap();
                        let mut steer_dealer_socket = _ctx.socket(zmq::DEALER).unwrap();
                        steer_dealer_socket.connect(&uav_address).unwrap();
                        let mut stop_sub_socket = _ctx.socket(zmq::SUB).unwrap();
                        stop_sub_socket.set_subscribe(b"").unwrap();
                        stop_sub_socket.connect(&format!("inproc://stop{}",slot)).unwrap();
                        let mut proxy = p.lock().unwrap();
                        proxy.push(Some(
                            thread::spawn(move ||
                            {
                                zmq::proxy_steerable(&mut steer_router_socket, &mut steer_dealer_socket,&mut stop_sub_socket).expect("Proxy err");
                                printLog!("Closing client proxy");

                            }))
                        );
                        drop(proxy);
                        printLog!("Ready to connect steer client on TCP: {}", first_port+slot);

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
                                    let rep = Clients::handleControlMsg(request.as_str().unwrap(), drone_no, &mut d_lck, &mut cargo_lck, &mut skipedHeartbeats);
                                    drop(cargo_lck);
                                    drop(d_lck);
                                    control_pair_socket.send(&rep, 0).unwrap();
                                }
                            })
                        ));
                        drop(control);
                        printLog!("Ready to connect control client on TCP: {}", first_port+slot+1000);

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
                        printLog!("Creating/updateing file {}.xml", &hash_val);
                        let mut file_name = DRONE_CONFIGS_PATH.to_string();
                        file_name.push_str(&hash_val);
                        file_name.push_str(".xml");
                        let mut file = File::create(file_name).unwrap();
                        let content_without_comments = Self::remove_xml_comments(content);
                        file.write_all(content_without_comments.as_bytes()).expect("Unable to write config");
                        drop(file);
                        let mut reply = String::with_capacity(12);
                        reply.push_str("ok;");
                        reply.push_str(&hash_val);
                        replyer_socket.send(&reply, 0).unwrap();
                    },
                
                    'i' => {
                        replyer_socket.send(&Self::getServerInfo(), 0).unwrap();
                    },
                    _ => printLog!("Unknown command: {}", request)
                }
            }
        });
        Clients{running: running, _proxies: proxies, _control: control, _replyer: Some(replyer)}
    }

    fn remove_xml_comments(xml: &str) -> String {
        // Create a regular expression to match XML comments
        let comment_regex = Regex::new(r"<!--(.*?)-->").unwrap();

        // Use the `replace_all` method to replace all matches with an empty string
        let without_comments = comment_regex.replace_all(xml, "");

        // Remove empty lines by filtering and joining non-empty lines
        let without_empty_lines = without_comments
            .lines()
            .filter(|line| !line.trim().is_empty())
            .collect::<Vec<&str>>()
            .join("\n");

        without_empty_lines
    }

    /// Returns information of running server as JSON
    fn getServerInfo() -> String
    {
        let mut configs = Vec::<String>::new();

        if let Ok(dir) = read_dir(DRONE_CONFIGS_PATH)
        {
            configs = dir.map(|p| p.unwrap().file_name().to_str().unwrap().split(".").next().unwrap().to_string()).collect();
        }
        else 
        {
            printLog!("Error: config directory can not be open.");    
        }
            
        let info = json!({
            "checksum": getChecksum(),
            "map": ServerConfig::get_str("map"),
            "configs": configs 

        });
        serde_json::to_string(&info).unwrap()
    }

    /// Handle incomming control message
    fn handleControlMsg(msg: &str, drone_no: usize, drones: &mut Drones, cargo: &mut Cargo,  skipedHeartbeats: &mut usize) -> String
    {
        let mut splited = msg.split(";");
        let action = splited.next().unwrap();
        let mut params = Vec::<&str>::new();
        if let Some(params_string) =  splited.next()
        {
            params_string.split(",").for_each(|s| params.push(s));
        }
        let mut d = drones.drones.lock().unwrap();
        let mut rep = String::with_capacity(30);
        rep.push_str("ok");
        if let Some(drone) = d.iter().find(|drone| drone.id == drone_no)
        {
            match action {
                "beep" => {  
                    *skipedHeartbeats = 0;
                }
                "shoot" => { 
                    let index = params.first().get_or_insert(&"0").parse().unwrap();
                    let (res,id) = drone.shootAmmo(index);
                    if id < 0
                    {
                        rep = "error".to_string();   
                    }
                    rep.push(';');
                    rep.push_str(&res.to_string());
                    rep.push(',');
                    rep.push_str(&id.to_string());
                }
                "drop" => {
                    let index = params.first().get_or_insert(&"0").parse().unwrap();
                    let (res,id) = drone.releaseCargo(index);
                    if id < 0
                    {
                        rep = "error".to_string();   
                    }
                    else if res >= 0
                    {
                        let params = drone.config.cargo.get(index).unwrap();
                        cargo.addLink(drone_no,
                            id as usize,
                            params.length,
                            params.k,
                            params.b,
                            params.hook);
                    }
                    rep.push(';');
                    rep.push_str(&res.to_string());
                    rep.push(',');
                    rep.push_str(&id.to_string());
                }
                "release" => {
                    cargo.removeLink(drone_no);
                }
                "kill" => {  
                    d.retain_mut(|d| d.id != drone_no);
                }
                _ => {
                    rep = "error".to_string();
                    printLog!("Unknown command: {}", msg);
                }
            }
        }
        drop(d);
        rep
    }

    fn check_config_folder()
    {
        if !std::fs::metadata(&DRONE_CONFIGS_PATH).is_ok() 
        {
            match std::fs::create_dir(&DRONE_CONFIGS_PATH) {
                Ok(_) => printLog!("Drones config directory created"),
                Err(_) => printLog!("Cannot create drones config directory"),
            }
        }
    }
}

/// Deconstructor
impl Drop for Clients{
    fn drop(&mut self) {
        printLog!("Dropping clients instance");
        self.running.store(false, Ordering::SeqCst);
        self._replyer.take().unwrap().join().expect("Join error");
        printLog!("Main client thread dropped");
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
        printLog!("Clients instance dropped");
    }
}
