use std::sync::atomic::{AtomicBool, self};
use std::sync::Mutex;
use zmq::Socket;
use crate::printLog;



static READY: AtomicBool = AtomicBool::new(false);
static NOTIFY_SOCKET: Mutex<Option<Socket>> = Mutex::new(None);

pub struct Notification
{
}

impl Notification
{
    pub fn init(_ctx: zmq::Context, port: &usize)
    {
        let pub_socket = _ctx.socket(zmq::PUB).expect("PUB socket error");
        pub_socket.bind(format!("tcp://*:{}",port).as_str()).expect(format!("Bind error tcp {}",port).as_str());
        printLog!("Notification publisher started on TCP: {}", port);
        let mut socket_lck = NOTIFY_SOCKET.lock().unwrap();
        *socket_lck = Some(pub_socket);
        READY.store(true, atomic::Ordering::Relaxed)
    }

    pub fn sendMsg(msg: &str)
    {
        if !READY.load(atomic::Ordering::Relaxed)
        {
            return;
        }
        let socket_lck = NOTIFY_SOCKET.lock().unwrap();
        socket_lck.as_ref().unwrap().send(msg, 0).unwrap(); 
    }
}

