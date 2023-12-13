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

#[derive(Copy, Clone)]
pub enum PromptCategory
{
    TEST = 0,
    OVERLOAD = 1,
    COLLISION = 2,
}

impl PromptCategory {
    pub fn to_usize(&self) -> usize
    {
        *self as usize
    }
}

pub enum PromptColor
{
    BLACK,
    WHITE,
    RED,
    GREEN,
    BLUE,
    ORANGE,
}

impl PromptColor {
    pub fn to_hex(&self) -> Option<&str>
    {
        match self {
            PromptColor::BLACK  => Some("000000"),
            PromptColor::WHITE  => Some("FFFFFF"),
            PromptColor::RED    => Some("FF0000"),
            PromptColor::GREEN  => Some("00FF00"),
            PromptColor::BLUE   => Some("0000FF"),
            PromptColor::ORANGE => Some("FF8000"),

            #[allow(unreachable_patterns)]
            _      => None,
        }
    }
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

    
    /// Send notification about prompt. Notification schema:
    /// p:TARGET,CATEGORY,COLOR,SHOW_TIME,MESSAGE;
    /// TARGET -- id of UAV that should show prompt. Negative value means broadcast, all UAV should show that prompt.
    /// CATEGORY -- category identificator, string. Aggregator use natural numbers as category names.
    /// COLOR -- RGB 8bit color as HEX: rrggbb. For example: ORANGE -- FF8000
    /// SHOW_TIME -- how long prompt should be displayed, in miliseconds. Zero means unlimited time, until replaced.
    /// MESSAGE - prompt content. May be empty what mean that prompt in this category should be cleared.
    pub fn sendPrompt(target_id: isize, category: PromptCategory, color: PromptColor, show_time_ms: usize ,  message: &str)
    {
        if let Some(color) = color.to_hex()
        {
            let mut promptMsg = String::with_capacity(20 + message.len());
            promptMsg.push_str("p:");
            promptMsg.push_str(target_id.to_string().as_str());
            promptMsg.push(',');
            promptMsg.push_str(category.to_usize().to_string().as_str());
            promptMsg.push(',');
            promptMsg.push_str(color);
            promptMsg.push(',');
            promptMsg.push_str(show_time_ms.to_string().as_str());
            promptMsg.push(',');
            promptMsg.push_str(message);
            promptMsg.push(';');
            Self::sendMsg(&promptMsg);
        }
    }
}

