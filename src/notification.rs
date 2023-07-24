pub struct Notification
{
    pub_socket: zmq::Socket
}

impl Notification
{
    pub fn new(_ctx: zmq::Context) -> Self
    {
        let pub_socket = _ctx.socket(zmq::PUB).expect("PUB socket error");
        pub_socket.bind("tcp://*:8000").expect("Bind error tcp 8000");
        println!("Notification publisher started on TCP: {}", 8000);
        Notification{pub_socket}
    }

    pub fn sendMsg(&self,msg: &str)
    {
        self.pub_socket.send(msg, 0).unwrap(); 
    }
}

