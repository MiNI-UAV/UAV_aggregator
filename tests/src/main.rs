use core::time;
use std::panic;
use std::process::{Command, Child};
use std::sync::Mutex;
use libc;
use regex::Regex;
use zmq::Socket;
use serde_json::Value;

static MAIN_PROCESS: Mutex<Option<Child>> = Mutex::new(Option::None);

fn setup() {
    let mut process = MAIN_PROCESS.lock().expect("Can not lock main process variable");

    if process.is_some()
    {
        panic!("Main process variable is not empty! Test before may not clean up");
    }

    let thread = Command::new("cargo")
        .arg("run")
        .spawn()
        .expect("Can not run main program");

    *process = Some(thread);
    std::thread::sleep(time::Duration::from_secs(5));
}

fn teardown() {
    let mut process = MAIN_PROCESS.lock().expect("Can not lock main process variable");

    if process.is_none()
    {
        panic!("Main process variable is empty! Maybe missing setup?");
    }

    let mut thread = process.take().unwrap();

    if let Ok(status) = thread.try_wait()
    {
        if let Some(code) = status
        {
            eprintln!("Main process ended before excepted, and exit with status {}", code);
        } 
    }

    let child_pid = thread.id() as libc::pid_t;
    unsafe { libc::kill(child_pid, libc::SIGINT) };

    for _ in 0..5 {
        if let Ok(status) = thread.try_wait()
        {
            if let Some(code) = status
            {
                println!("Main process exitted correctly with status code {}", code);
                return;
            } 
        }
        std::thread::sleep(time::Duration::from_secs(2));
    }

    unsafe { libc::kill(child_pid, libc::SIGQUIT) };
    eprintln!("Main process does not exit in 5s and was terminated!"); 
}


fn create_req_sock() -> Socket
{
    let ctx: zmq::Context = zmq::Context::new();
    let request_socket = ctx.socket(zmq::REQ).expect("REQ socket error");
    request_socket.connect("tcp://127.0.0.1:9000").expect("Connect error tcp 9000");
    request_socket
}

#[test]
fn program_run_and_end() {
    setup();
    teardown();
}

#[test]
fn server_reply_info_correctly() {
    setup();
    let result = panic::catch_unwind(|| {

        let request_socket = create_req_sock();
        request_socket.send(&"i", 0).expect("Can not send command");

        let responce = request_socket.recv_string(0).expect("Can not recv message").expect("Excepted string");
        println!("Response: {}", responce);
        let parsed_json: Result<Value, _> = serde_json::from_str(&responce);

        if let Ok(parsed_json) = parsed_json
        {
            let obj = parsed_json.as_object().expect("Response should be object");
            if !obj.contains_key("checksum") || !obj.get("checksum").unwrap().is_string()
            {
                panic!("Response not contains correct field checksum");
            }
            if !obj.contains_key("map") || !obj.get("map").unwrap().is_string()
            {
                panic!("Response not contains correct field map");
            }

            if !obj.contains_key("configs") || !obj.get("configs").unwrap().is_array()
            {
                panic!("Response not contains correct field checksum");
            }
        }
        else
        { 
            panic!("Can not parse json");
        }
    });
    teardown();
    assert!(result.is_ok());
}

#[test]
fn server_reply_on_config_send_correctly() {
    setup();
    let result = panic::catch_unwind(|| {

        let request_socket = create_req_sock();
        request_socket.send(&"c:abc", 0).expect("Can not send command");

        let responce = request_socket.recv_string(0).expect("Can not recv message").expect("Excepted string");
        println!("Response: {}", responce);
        let pattern = r"^ok;[A-Za-z0-9]{8}$";
        let regex = Regex::new(pattern).unwrap();
        assert!(regex.is_match(&responce));
        
    });
    teardown();
    assert!(result.is_ok());
}