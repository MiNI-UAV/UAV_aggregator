use zmq::{Context, SocketType};

fn main() {
    // Create a ZMQ context
    let context = Context::new();

    // Create a subscriber socket
    let subscriber = context.socket(SocketType::SUB).unwrap();

    // Connect the subscriber socket to the IPC address
    let ipc_address = "ipc:///tmp/mini/state";
    subscriber.connect(ipc_address).unwrap();

    // Subscribe to all messages (empty prefix)
    let topic_filter = "";
    subscriber.set_subscribe(topic_filter.as_bytes()).unwrap();

    loop {
        // Receive a message from the subscriber socket
        let message = subscriber.recv_msg(0).unwrap();

        // Convert the message to a string
        let message_str = message.as_str().unwrap();

        // Display the received message
        println!("Received: {}", message_str);
    }
}
