use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};

use ndarray::Array1;

use crate::{drones::Drones, objects:: Objects};

const MINIMAL_DISTANCE2: f32 = 1.0 * 1.0;

//BOUNDING BOX
const BOUNDMAXX: f32 = 200.0;
const BOUNDMINX: f32 = -200.0;
const BOUNDMAXY: f32 = 200.0;
const BOUNDMINY: f32 = -200.0;
const BOUNDMAXZ: f32 = 200.0;
const BOUNDMINZ: f32 = -200.0;

pub struct CollisionDetector
{
    running: Arc<AtomicBool>,
    collision_checker: Option<thread::JoinHandle<()>>
}

impl CollisionDetector
{
    pub fn new(_drones: Arc<Mutex<Drones>>, _objects: Arc<Mutex<Objects>>) -> Self
    {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        let collision_checker: JoinHandle<()> = thread::spawn(move ||
        {
            // let upperLimit = arr1(&[BOUNDMAXX, BOUNDMAXY, BOUNDMAXZ]);
            // let lowerLimit = arr1(&[BOUNDMINX,BOUNDMINY, BOUNDMINZ]);
            while r.load(Ordering::SeqCst) {
                let drones_lck = _drones.lock().unwrap();
                let drones_pos = drones_lck.getPositions();
                drop(drones_lck);

                let obj_lck = _objects.lock().unwrap();
                let objs_pos_vels = obj_lck.getPosVels();
                drop(obj_lck);
                
                //Colisions between drones
                for i in 0..drones_pos.len() {
                    for j in (i+1)..drones_pos.len() {
                        let obj1 = drones_pos.get(i).unwrap();
                        let obj2 = drones_pos.get(j).unwrap();
                        let dist: Array1<f32> = &obj1.1-&obj2.1;
                        if dist.dot(&dist).abs() < MINIMAL_DISTANCE2
                        {
                            println!("Collision detected between drone {} and {}", obj1.0,obj2.0);
                        }
                    }
                }

                //Colision between objects are negligible

                //Drone-object colisions
                for obj1 in drones_pos.iter() {
                    for obj2 in objs_pos_vels.iter() {
                        let dist: Array1<f32> = &obj1.1-&obj2.1;
                        if dist.dot(&obj2.2) > 0.0 && dist.dot(&dist).abs() < MINIMAL_DISTANCE2
                        {
                            println!("Collision detected between drone {} and object {}", obj1.0,obj2.0);
                        }
                    }
                }

                //Boundary box for objects
                let mut objToKill = Vec::new();
                for obj in objs_pos_vels.iter() 
                {
                    if obj.1[0] > BOUNDMAXX || obj.1[1] > BOUNDMAXY || obj.1[2] > BOUNDMAXZ
                        || obj.1[0] < BOUNDMINX || obj.1[1] < BOUNDMINY || obj.1[2] < BOUNDMINZ
                    {
                        //println!("Object {} is outside the boundary box", obj.0);
                        objToKill.push(obj.0);
                    }
                }
                if !objToKill.is_empty()
                {
                    let obj_lck = _objects.lock().unwrap();
                    for id in objToKill {
                        obj_lck.removeObj(id);
                    }
                    drop(obj_lck);
                }

                //TEST ground
                let k = 0.05f32;
                let b = 0.2f32;
                let mut forceToSend = Vec::<(usize,Array1<f32>)>::new();
                for obj in objs_pos_vels.iter() {
                    if obj.1[2] > 0.0
                    {
                        let mut force = Array1::<f32>::zeros(3);
                        force[2] = -k*obj.1[2] - b*obj.2[2];
                        forceToSend.push((obj.0,force.clone()));
                    }
                }
                if !forceToSend.is_empty()
                {
                    let obj_lck = _objects.lock().unwrap();
                    for elem in forceToSend {
                        obj_lck.setForce(elem.0,elem.1);
                    }
                    drop(obj_lck);
                }

                thread::sleep(time::Duration::from_millis(10));
            }
        });
        CollisionDetector {running: running, collision_checker: Some(collision_checker)}
    }
}

impl Drop for CollisionDetector{
    fn drop(&mut self) {
        println!("Dropping collision detector instance");
        self.running.store(false, Ordering::SeqCst);
        self.collision_checker.take().unwrap().join().expect("Join error");
        println!("Collision detector instance dropped");
    }
}