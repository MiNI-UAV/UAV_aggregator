#![allow(non_snake_case)]

use std::{time, thread, sync::{Mutex, Arc}};

pub mod clients;
pub mod drones;
pub mod uav;

fn main() {
    let ctx = zmq::Context::new();
    
    let mut _drones = Arc::new(Mutex::new(drones::Drones::new(ctx.clone())));
    let mut _clients = clients::Clients::new(ctx.clone(),_drones.clone());

    loop {
        thread::sleep(time::Duration::from_millis(100));
    }
}
