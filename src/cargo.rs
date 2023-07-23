use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time, collections::HashMap};
use nalgebra::{Vector3,geometry::Rotation3};
use std::time::Instant;
use crate::{drones::Drones, objects::Objects};

const TIMEOUT_LIMIT: usize = 10;

struct Link
{
    timeout: usize,
    length: f32,
    k: f32,
    hook_offset: Vector3<f32>
}

pub struct Cargo
{
    running: Arc<AtomicBool>,
    collision_checker: Option<thread::JoinHandle<()>>,
    links: Arc<Mutex<HashMap<(usize,usize),Link>>>
}

impl Cargo
{
    pub fn new(_drones: Arc<Mutex<Drones>>, _objects: Arc<Mutex<Objects>>) -> Self
    {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        let links = Arc::new(Mutex::new(HashMap::<(usize,usize),Link>::new()));
        let l = links.clone();
        let collision_checker: JoinHandle<()> = thread::spawn(move ||
        {
            while r.load(Ordering::SeqCst) {
                let links_lck = l.lock().unwrap();
                let skip = links_lck.is_empty();
                drop(links_lck);
                if skip
                {
                    thread::sleep(time::Duration::from_millis(2));
                }

                let mut forceToSend = Vec::new();

                let start = Instant::now();
                let drones_lck = _drones.lock().unwrap();
                let drones_pos_vel = drones_lck.getPosOriVels();
                drop(drones_lck);

                let obj_lck = _objects.lock().unwrap();
                let objs_pos_vels = obj_lck.getPosVels();
                drop(obj_lck);

                let mut links_lck = l.lock().unwrap();
                for ((drone_id, obj_id),link) in links_lck.iter_mut()
                {
                    if let (Some(drone), Some(obj)) 
                        =  (drones_pos_vel.iter().find(|d|d.0 == *drone_id),
                            objs_pos_vels.iter().find(|o|o.0 == *obj_id))
                    {
                        let mut force = Vector3::<f32>::zeros();
                        let mut torque = Vector3::<f32>::zeros();
                        
                        let offset = Rotation3::from_euler_angles(drone.2.x, drone.2.y, drone.2.z)*link.hook_offset;
                        let dist = obj.1 - (drone.1+offset);
                        let length = dist.norm();
                        if length > link.length
                        {

                            force = link.k*(length-link.length)*dist.normalize();
                            torque = offset.cross(&force);
                        }
                        forceToSend.push((drone.0, obj.0, force, torque));
                    }
                    else {
                        link.timeout += 1;
                    }
                }
                links_lck.retain(|_,v| v.timeout < TIMEOUT_LIMIT);
                drop(links_lck);

                if !forceToSend.is_empty()
                {
                    let obj_lck = _objects.lock().unwrap();
                    for (_, obj_id, force, _) in &forceToSend {
                        obj_lck.setForce(*obj_id,-force);
                    }
                    drop(obj_lck);

                    let drone_lck = _drones.lock().unwrap();
                    for (drone_id, _, force, torque) in &forceToSend {
                        drone_lck.updateForce(drone_id,force,torque);
                    }
                    drop(drone_lck);
                }
                        
                #[allow(unused_variables)]
                let elapsed = start.elapsed();
                //println!("Cargo calc time: {} ms", elapsed.as_millis());
                thread::sleep(time::Duration::from_millis(5));
            }
        });
        Cargo {running, collision_checker: Some(collision_checker), links}
    }

    pub fn addLink(&self,drone_id: usize, obj_id: usize, length: f32, k: f32, hook_offset: Vector3<f32>)
    {
        let mut links_lck = self.links.lock().unwrap();
        links_lck.insert((drone_id,obj_id), Link { timeout: 0, length, k, hook_offset});
        drop(links_lck);
    }

    pub fn removeLink(&self, drone_id: usize, obj_id: usize)
    {
        let mut links_lck = self.links.lock().unwrap();
        links_lck.remove(&(drone_id, obj_id));
        drop(links_lck);
    }


}

impl Drop for Cargo{
    fn drop(&mut self) {
        println!("Dropping cargo instance");
        self.running.store(false, Ordering::SeqCst);
        self.collision_checker.take().unwrap().join().expect("Join error");
        println!("Cargo instance dropped");
    }
}
