#![allow(non_snake_case)]

use std::{time, thread};

pub mod drones;
pub mod uav;

fn main() {
    let mut drones = drones::Drones::new();
    drones.startUAV("mini3");
    drones.startUAV("mini4");

    loop {
        drones.printState();
        thread::sleep(time::Duration::from_millis(100));
    }
}
