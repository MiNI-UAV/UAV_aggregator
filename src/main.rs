#![allow(non_snake_case)]

use std::process::Command;

fn main() {
    print!("Simulation:");
    let mut simulation = Command::new("../UAV_physics_engine/build/uav")
                     .arg("-c").arg("/home/wgajda/Desktop/Development/UAV/UAV_aggregator/config.xml")
                     .arg("-n").arg("test1")
                     .spawn()
                     .expect("failed to execute simulation process");

    print!("Controller:");
    let mut controller = Command::new("../UAV_controller/build/controller")
                     .arg("-c").arg("/home/wgajda/Desktop/Development/UAV/UAV_aggregator/config.xml")
                     .arg("-n").arg("test1")
                     .spawn()
                     .expect("failed to execute controller process");

    
    let mut visualization = Command::new("gradle")
                     .arg("-p")
                     .arg("../UAV_visualization")
                     .arg("run")
                     .spawn()
                     .expect("failed to execute visualization process");

    


    let ecode = simulation.wait()
                     .expect("failed to wait on child");
    
    assert!(ecode.success());

    let ecode = controller.wait()
                     .expect("failed to wait on child");
    
    assert!(ecode.success());

    let ecode = visualization.wait()
                     .expect("failed to wait on child");
    
    assert!(ecode.success());
}
