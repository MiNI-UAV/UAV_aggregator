#![allow(non_snake_case)]

use std::{time, thread, sync::{Mutex, Arc}};
use std::sync::atomic::{AtomicBool, Ordering};
use device_query::{DeviceEvents, DeviceState};



pub mod clients;
pub mod drones;
pub mod uav;
pub mod wind;
pub mod collision;
pub mod objects;
pub mod config;

fn main() {
    let drone_config = Arc::new(config::DroneConfig::parse("config.xml").expect("Config file error"));
    let ctx: zmq::Context = zmq::Context::new();
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    let r2 = running.clone();

    let stopSocket = ctx.socket(zmq::SocketType::PUB).unwrap();
    stopSocket.bind("inproc://stop").unwrap();

    let device_state = DeviceState::new();

    let _guardQ = device_state.on_key_down(move |key| {
        if key.eq(&device_query::Keycode::Q)
        {
            r.store(false, Ordering::SeqCst);
        }
     });

    ctrlc::set_handler(move || {
        r2.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    let _objects = Arc::new(Mutex::new(objects::Objects::new(ctx.clone())));
    let _drones = Arc::new(Mutex::new(drones::Drones::new(ctx.clone(),_objects.clone())));
    let _clients = clients::Clients::new(ctx.clone(),_drones.clone()); 
    let _wind = wind::Wind::new(_drones.clone(),_objects.clone());
    let _colision_detector = collision::CollisionDetector::new(_drones.clone(),_objects.clone(),drone_config.clone());

    while running.load(Ordering::SeqCst) {
        thread::sleep(time::Duration::from_millis(300));
    }
    println!("Bye!");
    stopSocket.send("TERMINATE", 0).unwrap();
    let mut drones_lck = _drones.lock().unwrap();
    drones_lck.removeAllUAV();
    println!("All drone killed!");
    drop(drones_lck);
    drop(_colision_detector);
    drop(_wind);
    drop(_clients);
    drop(_drones);
    drop(_objects);
    drop(ctx);
}
