#![allow(non_snake_case)]

use std::{time, thread, sync::{Mutex, Arc}};

pub mod clients;
pub mod drones;
pub mod uav;
pub mod wind;
pub mod collision;

fn main() {
    let ctx = zmq::Context::new();
    
    let _drones = Arc::new(Mutex::new(drones::Drones::new(ctx.clone())));
    let _clients = clients::Clients::new(ctx.clone(),_drones.clone());
    let _wind = wind::Wind::new(_drones.clone());
    let _colision_detector = collision::CollisionDetector::new(_drones.clone());

    loop {
        thread::sleep(time::Duration::from_millis(1000));
    }
}
