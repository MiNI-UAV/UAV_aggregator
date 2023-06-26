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
                let msg = request.as_str().unwrap();
                if msg.is_empty() || taken_name.contains(msg) {
                    let mut reply = String::new();
                    reply.push_str("-1");
                    replyer_socket.send(&reply, 0).unwrap(); 
                    continue;
                }

                taken_name.insert(msg.to_string());
                let mut drones = drones.lock().unwrap();
                let uav_address = drones.startUAV(msg);
                println!("Started new drone with name: {}", msg);
                let steer_xsub_socket = _ctx.socket(zmq::XSUB).unwrap();
                let address = format!("tcp://127.0.0.1:{}", next_port);
                steer_xsub_socket.bind(&address).unwrap();
                let steer_xpub_socket = _ctx.socket(zmq::XPUB).unwrap();
                steer_xpub_socket.connect(&uav_address).unwrap();

                proxies.push(
                    thread::spawn(move ||
                    {
                        zmq::proxy(&steer_xsub_socket, &steer_xpub_socket).expect("Proxy err");
                    })
                );
                
                println!("Ready to connect client on TCP: {}", next_port);
                let mut reply = String::new();
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