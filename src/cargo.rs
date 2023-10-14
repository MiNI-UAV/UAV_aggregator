use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time, collections::HashMap};
use nalgebra::{Vector3,geometry::Rotation3};
use std::time::Instant;
use crate::{drones::Drones, objects::Objects, config::ServerConfig, notification::Notification};
use crate::printLog;


struct Link
{
    timeout: usize,
    length: f32,
    k: f32,
    b: f32,
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
        let timeout_limit = ServerConfig::get_usize("timeout_limit");
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        let links = Arc::new(Mutex::new(HashMap::<(usize,usize),Link>::new()));
        let l = links.clone();
        let collision_checker: JoinHandle<()> = thread::spawn(move ||
        {
            let mut counter: usize = 0;
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
                        let mut dist = obj.1 - (drone.1+offset);
                        let length = dist.norm();
                        dist = dist.normalize();
                        let relative_vel = obj.2.dot(&dist) - drone.3.dot(&dist);
                        
                        if length > link.length
                        {
                            force = (link.k*(length-link.length) + link.b*relative_vel)*dist;
                            torque = offset.cross(&force);
                        }
                        forceToSend.push((drone.0, obj.0, force, torque));
                    }
                    else {
                        link.timeout += 1;
                    }
                }
                links_lck.retain(|_,v| v.timeout < timeout_limit);

                if counter % 500 == 0
                {
                    notifyAboutLinks(&links_lck);
                }

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
                //printLog!("Cargo calc time: {} ms", elapsed.as_millis());
                counter = (counter+1)%10000;
                thread::sleep(time::Duration::from_millis(5));
            }
        });
        Cargo {running, collision_checker: Some(collision_checker), links}
    }

    pub fn addLink(&self,drone_id: usize, obj_id: usize, length: f32, k: f32, b: f32, hook_offset: Vector3<f32>)
    {
        let mut links_lck = self.links.lock().unwrap();
        links_lck.insert((drone_id,obj_id), Link { timeout: 0, length, k, b, hook_offset});
        notifyAboutLinks(&links_lck);
        drop(links_lck);
    }

    pub fn removeLink(&self, drone_id: usize)
    {
        let mut links_lck = self.links.lock().unwrap();
        links_lck.retain(|(d_id,_),_| *d_id != drone_id);
        notifyAboutLinks(&links_lck);
        drop(links_lck);
    }
}

fn notifyAboutLinks(links: &HashMap<(usize, usize), Link>){
    let mut msg = String::with_capacity(5 + 15*links.len());
    msg.push_str("l:");
    for ((drone_id,obj_id),link) in links
    {
        msg.push_str(&drone_id.to_string());
        msg.push(',');
        msg.push_str(&obj_id.to_string());
        msg.push(',');
        msg.push_str(&link.length.to_string());
        msg.push(',');
        msg.push_str(&link.hook_offset.x.to_string());
        msg.push(',');
        msg.push_str(&link.hook_offset.y.to_string());
        msg.push(',');
        msg.push_str(&link.hook_offset.z.to_string());
        msg.push(';');
    }
    Notification::sendMsg(&msg);
}


impl Drop for Cargo{
    fn drop(&mut self) {
        printLog!("Dropping cargo instance");
        self.running.store(false, Ordering::SeqCst);
        self.collision_checker.take().unwrap().join().expect("Join error");
        printLog!("Cargo instance dropped");
    }
}
