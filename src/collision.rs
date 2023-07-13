use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};

use nalgebra::{Vector3,geometry::Rotation3};

use crate::{drones::Drones, objects::Objects, config::DroneConfig};

const MINIMAL_DISTANCE2: f32 = 1.0 * 1.0;

//BOUNDING BOX
const BOUNDMAXX: f32 = 200.0;
const BOUNDMINX: f32 = -200.0;
const BOUNDMAXY: f32 = 200.0;
const BOUNDMINY: f32 = -200.0;
const BOUNDMAXZ: f32 = 200.0;
const BOUNDMINZ: f32 = -200.0;

//DRONE SPHERES
const SPHERE_RADIUS: f32 = 0.1;

pub struct CollisionDetector
{
    running: Arc<AtomicBool>,
    collision_checker: Option<thread::JoinHandle<()>>
}

impl CollisionDetector
{
    pub fn new(_drones: Arc<Mutex<Drones>>, _objects: Arc<Mutex<Objects>>, _config: Arc<DroneConfig>) -> Self
    {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        let collision_checker: JoinHandle<()> = thread::spawn(move ||
        {
            let rotorsPositions = _config.rotors.positions.clone();

            while r.load(Ordering::SeqCst) {
                let drones_lck = _drones.lock().unwrap();
                let drones_pos_vel = drones_lck.getPosOriVels();
                drop(drones_lck);

                let obj_lck = _objects.lock().unwrap();
                let objs_pos_vels = obj_lck.getPosVels();
                drop(obj_lck);
                
                //Colision between objects are negligible
                Self::colisions_between_drones(&drones_pos_vel);
                Self::colisions_drones_obj(&drones_pos_vel, &objs_pos_vels);
                Self::boundary_box_obj(&objs_pos_vels, &_objects);
                

                //TEST
                Self::ground_objs(&objs_pos_vels, &_objects);
                Self::ground_drones(&drones_pos_vel,&_drones, &rotorsPositions);

                thread::sleep(time::Duration::from_millis(5));
            }
        });
        CollisionDetector {running, collision_checker: Some(collision_checker)}
    }

    fn colisions_between_drones(drones_pos_vel: &Vec<(usize,Vector3<f32>,Vector3<f32>,Vector3<f32>,Vector3<f32>)>)
    {
        for i in 0..drones_pos_vel.len() {
            for j in (i+1)..drones_pos_vel.len() {
                let obj1 = drones_pos_vel.get(i).unwrap();
                let obj2 = drones_pos_vel.get(j).unwrap();
                let dist: Vector3<f32> = obj1.1-obj2.1;
                if dist.dot(&dist).abs() < MINIMAL_DISTANCE2
                {
                    println!("Collision detected between drone {} and {}", obj1.0,obj2.0);
                }
            }
        }
    }

    fn colisions_drones_obj(drones_pos_vel: &Vec<(usize,Vector3<f32>,Vector3<f32>,Vector3<f32>,Vector3<f32>)>,
        objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>)>)
    {
        for obj1 in drones_pos_vel.iter() {
            for obj2 in objs_pos_vels.iter() {
                let dist: Vector3<f32> = obj1.1-obj2.1;
                if dist.dot(&obj2.2) > 0.0 && dist.dot(&dist).abs() < MINIMAL_DISTANCE2
                {
                    println!("Collision detected between drone {} and object {}", obj1.0,obj2.0);
                }
            }
        }
    }

    fn boundary_box_obj(objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>)>,
        objects: &Arc<Mutex<Objects>>)
    {
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
            let obj_lck = objects.lock().unwrap();
            for id in objToKill {
                obj_lck.removeObj(id);
            }
            drop(obj_lck);
        }
    }

    fn ground_objs(objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>)>,
        objects: &Arc<Mutex<Objects>>)
    {
        let k = 0.4f32;
        let b = 0.2f32;
        let mut forceToSend = Vec::<(usize,Vector3<f32>)>::new();
        for obj in objs_pos_vels.iter() {
            if obj.1[2] > 0.0
            {
                let mut force = Vector3::<f32>::zeros();
                force[2] = -k*obj.1[2] - b*obj.2[2];
                forceToSend.push((obj.0,force.clone()));
            }
        }
        if !forceToSend.is_empty()
        {
            let obj_lck = objects.lock().unwrap();
            for elem in forceToSend {
                obj_lck.setForce(elem.0,elem.1);
            }
            drop(obj_lck);
        }
    }

    fn ground_drones(objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>,Vector3<f32>,Vector3<f32>)>,
        drones: &Arc<Mutex<Drones>>, rotorPos: &Vec<Vector3<f32>>)
    {
        let k = 0.4f32;
        let b = 0.2f32;
        let mut forceToSend = Vec::<(usize,Vector3<f32>, Vector3<f32>)>::new();
        for (id, pos, ori, vel, om) in objs_pos_vels.iter()
        {
            let worldPosVel = rotorPos.iter().map(|r| 
                {
                    (r, Rotation3::from_euler_angles(ori.x, ori.y, ori.z)* r + pos, vel + om.cross(r))
                }
            );
            for (r, pos,vel) in worldPosVel
            {
                if pos[2] + SPHERE_RADIUS > 0.0
                {
                    let mut force = Vector3::<f32>::zeros();
                    force[2] = -k*(pos[2] + SPHERE_RADIUS) - b*vel[2];
                    forceToSend.push((*id, force, r.cross(&force)));
                }
            }
        }
        if !forceToSend.is_empty()
        {
            let drones_lck = drones.lock().unwrap();
            for (id,force,torque) in &forceToSend {
                drones_lck.updateForce(id,force,torque);
            }
            drop(drones_lck);
        }
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