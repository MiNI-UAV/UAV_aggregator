use std::sync::atomic::{AtomicBool, self};
use std::sync::Mutex;
use zmq::Socket;
use crate::printLog;


/// static variable to check if Notification was initialized
static READY: AtomicBool = AtomicBool::new(false);
/// static notify socket used by Notification methods
static NOTIFY_SOCKET: Mutex<Option<Socket>> = Mutex::new(None);

/// Notify subscribers about simulation less importants events and statuses.
/// Contains static method to send message and not require class instance to use.
pub struct Notification
{
}

impl Notification
{
    /// Initialize Notification class
    pub fn init(_ctx: zmq::Context, port: &usize)
    {
        let pub_socket = _ctx.socket(zmq::PUB).expect("PUB socket error");
        pub_socket.bind(format!("tcp://*:{}",port).as_str()).expect(format!("Bind error tcp {}",port).as_str());
        printLog!("Notification publisher started on TCP: {}", port);
        let mut socket_lck = NOTIFY_SOCKET.lock().unwrap();
        *socket_lck = Some(pub_socket);
        READY.store(true, atomic::Ordering::Relaxed)
    }

    /// Send notification. Instance is not requiered.
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

