#![allow(non_snake_case)]

use std::{time, thread, sync::{Mutex, Arc}};
use std::sync::atomic::{AtomicBool, Ordering};


pub mod clients;
pub mod drones;
pub mod uav;
pub mod wind;
pub mod collision;
pub mod objects;

fn main() {
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    }).expect("Error setting Ctrl-C handler");

    let ctx = zmq::Context::new();
    let _objects = Arc::new(Mutex::new(objects::Objects::new(ctx.clone())));
    let _drones = Arc::new(Mutex::new(drones::Drones::new(ctx.clone(),_objects.clone())));
    let _clients = clients::Clients::new(ctx.clone(),_drones.clone()); 
    let _wind = wind::Wind::new(_drones.clone(),_objects.clone());
    let _colision_detector = collision::CollisionDetector::new(_drones.clone());

    while running.load(Ordering::SeqCst) {
        // let drone = _drones.lock().unwrap();
        // if let Some(d) = drone.drones.get(0)
        // {
        //     for _ in 0..2 {
        //         d.dropOrShot(None,None,None,Some([0.0,0.1,0.0]));
        //         thread::sleep(time::Duration::from_millis(300));
        //         d.dropOrShot(None,None,None,Some([0.0,-0.1,0.0]));
        //         thread::sleep(time::Duration::from_millis(300));
        //     }        
        // }
        // drop(drone);    
        thread::sleep(time::Duration::from_millis(100));
    }
    println!("Bye!");
}
