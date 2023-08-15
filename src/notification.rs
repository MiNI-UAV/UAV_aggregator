pub struct Notification
{
    pub_socket: zmq::Socket
}

impl Notification
{
    pub fn new(_ctx: zmq::Context, port: &usize) -> Self
    {
        let pub_socket = _ctx.socket(zmq::PUB).expect("PUB socket error");
        pub_socket.bind(format!("tcp://*:{}",port).as_str()).expect(format!("Bind error tcp {}",port).as_str());
        println!("Notification publisher started on TCP: {}", port);
        Notification{pub_socket}
    }

    pub fn sendMsg(&self,msg: &str)
    {
        self.pub_socket.send(msg, 0).unwrap(); 
    }
}

