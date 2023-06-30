#![allow(non_snake_case)]

use std::{time, thread, sync::{Mutex, Arc}};

pub mod clients;
pub mod drones;
pub mod uav;
pub mod wind;
pub mod collision;
pub mod objects;

fn main() {
    let ctx = zmq::Context::new();
    let _objects = Arc::new(Mutex::new(objects::Objects::new(ctx.clone())));
    let _drones = Arc::new(Mutex::new(drones::Drones::new(ctx.clone(),_objects.clone())));
    let _clients = clients::Clients::new(ctx.clone(),_drones.clone()); 
    let _wind = wind::Wind::new(_drones.clone(),_objects.clone());
    let _colision_detector = collision::CollisionDetector::new(_drones.clone());

    loop {
        let drone = _drones.lock().unwrap();
        if let Some(d) = drone.drones.get(0)
        {
            d.dropOrShot(None,None,None,None);

        }
        drop(drone);
        let obj = _objects.lock().unwrap();
        println!("{}",obj._time.lock().unwrap().to_string());

        thread::sleep(time::Duration::from_millis(3000));
    }
}
