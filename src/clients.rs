use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc}, collections::HashSet};

use crate::drones::Drones;

pub struct Clients
{
    _replyer: Option<thread::JoinHandle<()>>
}

impl Clients
{
    pub fn new(_ctx: zmq::Context, drones: Arc<Mutex<Drones>>) -> Self {
        let mut next_port = 10000;
        let replyer_socket = _ctx.socket(zmq::REP).expect("REP socket error");
        let mut taken_name =  HashSet::<String>::new();
        let mut proxies = Vec::<JoinHandle<()>>::new();
        let replyer: JoinHandle<()> = thread::spawn(move ||
        {
            replyer_socket.bind("tcp://127.0.0.1:9000").expect("Bind error tcp 9000");
            println!("Replyer started on TCP: {}", 9000);
            loop {
                let mut request =  zmq::Message::new();
                replyer_socket.recv(&mut request, 0).unwrap();
                let mut drone_name = request.as_str().unwrap().to_string();
                if drone_name.is_empty(){
                    let mut reply = String::new();
                    reply.push_str("-1");
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
                let mut drones = drones.lock().unwrap();
                let (drone_no,uav_address) = drones.startUAV(&drone_name);
                println!("Started new drone with name: {}", drone_name);
                let steer_pair_socket = _ctx.socket(zmq::PAIR).unwrap();
                let address = format!("tcp://127.0.0.1:{}", next_port);
                steer_pair_socket.bind(&address).unwrap();
                let steer_xpub_socket = _ctx.socket(zmq::XPUB).unwrap();
                steer_xpub_socket.connect(&uav_address).unwrap();
                proxies.push(
                    thread::spawn(move ||
                    {
                        zmq::proxy(&steer_pair_socket, &steer_xpub_socket).expect("Proxy err");
                        println!("Closing proxy");
                    })
                );       
                println!("Ready to connect client on TCP: {}", next_port);
                let mut reply = String::new();
                reply.push_str(&drone_no.to_string());
                reply.push(',');
                reply.push_str(&next_port.to_string());
                replyer_socket.send(&reply, 0).unwrap();
                next_port = next_port + 1;
            }
        });
        Clients{_replyer: Some(replyer)}
    }
}

impl Drop for Clients{
    fn drop(&mut self) {
        self._replyer.take().unwrap().join().expect("Join error");
    }
}