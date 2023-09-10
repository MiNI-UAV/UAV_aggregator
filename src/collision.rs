use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};
use nalgebra::{Vector3,Vector4,geometry::Rotation3, Matrix3};
use std::time::Instant;
use crate::{drones::Drones, objects::Objects, map::Map, config::ServerConfig};
use crate::printLog;


pub struct CollisionDetector
{
    running: Arc<AtomicBool>,
    collision_checker: Option<thread::JoinHandle<()>>,
}

impl CollisionDetector
{
    pub fn new(_drones: Arc<Mutex<Drones>>, _objects: Arc<Mutex<Objects>>) -> Self
    {
        let map_offset = ServerConfig::get_f32("map_offset");
        let mut map_path = "assets/maps/".to_string();
        map_path.push_str(ServerConfig::get_str("map").as_str());
        map_path.push_str("/model/model.obj");

        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        let grid = ServerConfig::get_str("grid").as_str().split(',')
        .map(|component| component.trim().parse())
        .collect::<Result<Vec<f32>, _>>()
        .map(|components| {
            if components.len() != 3 {
                panic!("Input does not have three components.");
            }
            Vector3::new(components[0], components[1], components[2])
        }).unwrap();

        let map = Map::new(&map_path,
            ServerConfig::get_f32("collisionPlusEps"),
            ServerConfig::get_f32("collisionMinusEps"),
            grid,
            ServerConfig::get_f32("sphereRadius"),
            ServerConfig::get_f32("projectileRadius"),
            ServerConfig::get_f32("COR"),
            ServerConfig::get_f32("mi_s"),
            ServerConfig::get_f32("mi_d"),
            ServerConfig::get_f32("minimalDist"),
        );
        let (mut box_min, mut box_max) = map.getMinMax();
        box_min.add_scalar_mut(-map_offset);
        box_max.add_scalar_mut(map_offset);

        let collision_checker: JoinHandle<()> = thread::spawn(move ||
        {
            while r.load(Ordering::SeqCst) {
                let start = Instant::now();
                let drones_lck = _drones.lock().unwrap();
                let drones_pos_vel = drones_lck.getPosOriVels();
                let drones_rotor_pos = drones_lck.getRotorPos();
                drop(drones_lck);

                let obj_lck = _objects.lock().unwrap();
                let objs_pos_vels = obj_lck.getPosVels();
                drop(obj_lck);
                
                //Colision between objects are negligible
                Self::colisions_between_drones(&drones_pos_vel,map.minimalDist);
                Self::colisions_drones_obj(&drones_pos_vel, &objs_pos_vels,map.minimalDist);
                
                
                //Drone collision with map3
                Self::impulse_collision_drone(&drones_pos_vel,&_drones, &drones_rotor_pos,&map);
                Self::impulse_collision_projectiles(&objs_pos_vels,&_objects,&map);
                //Second box
                Self::boundary_box_obj(&objs_pos_vels, &_objects, box_min, box_max);
                
                //TEST
                //Self::ground_objs(&objs_pos_vels, &_objects);
                //Self::spring_dumper_drone(&drones_pos_vel,&_drones, &rotorsPositions);
                
                #[allow(unused_variables)]
                let elapsed = start.elapsed();
                //printLog!("Collision calc time: {} ms", elapsed.as_millis());
                thread::sleep(time::Duration::from_millis(1));

            }
        });
        CollisionDetector {running,
             collision_checker: Some(collision_checker),
            }
    }

    fn colisions_between_drones(drones_pos_vel: &Vec<(usize,Vector3<f32>,Vector4<f32>,Vector3<f32>,Vector3<f32>)>, minimal_dist: f32)
    {
        for i in 0..drones_pos_vel.len() {
            for j in (i+1)..drones_pos_vel.len() {
                let obj1 = drones_pos_vel.get(i).unwrap();
                let obj2 = drones_pos_vel.get(j).unwrap();
                let dist: Vector3<f32> = obj1.1-obj2.1;
                if dist.dot(&dist).abs() < minimal_dist
                {
                    printLog!("Collision detected between drone {} and {}", obj1.0,obj2.0);
                }
            }
        }
    }

    fn colisions_drones_obj(drones_pos_vel: &Vec<(usize,Vector3<f32>,Vector4<f32>,Vector3<f32>,Vector3<f32>)>,
        objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>)>, minimal_dist: f32)
    {
        for obj1 in drones_pos_vel.iter() {
            for obj2 in objs_pos_vels.iter() {
                let dist: Vector3<f32> = obj1.1-obj2.1;
                if dist.dot(&obj2.2) > 0.0 && dist.dot(&dist).abs() < minimal_dist
                {
                    //printLog!("Collision detected between drone {} and object {}", obj1.0,obj2.0);
                }
            }
        }
    }

    #[allow(dead_code)]
    fn boundary_box_obj(objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>)>,
        objects: &Arc<Mutex<Objects>>, box_min: Vector3<f32>, box_max: Vector3<f32>)
    {
        let mut objToKill = Vec::new();
        for (id,pos,_) in objs_pos_vels.iter() 
        {
            if box_min.inf(pos) != box_min || box_max.sup(pos) != box_max
            {
                printLog!("Object {} is outside the boundary box", id);
                objToKill.push(id);
            }
        }
        if !objToKill.is_empty()
        {
            let obj_lck = objects.lock().unwrap();
            for id in objToKill {
                obj_lck.removeObj(*id);
            }
            drop(obj_lck);
        }
    }


    #[allow(dead_code)]
    fn spring_dumper_drone(objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>,Vector3<f32>,Vector3<f32>)>,
        drones: &Arc<Mutex<Drones>>, rotorPos: &Vec<Vector3<f32>>, map: &Map)
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
            let mut forceSum = Vector3::<f32>::zeros();
            let mut torqueSum = Vector3::<f32>::zeros();
            for (r, pos,vel) in worldPosVel
            {
                if pos[2] + map.sphereRadius > 10.0
                {
                    let mut force = Vector3::<f32>::zeros();
                    force[2] = -k*(pos[2] + map.sphereRadius) - b*vel[2];
                    forceSum += force;
                    torqueSum += r.cross(&force);
                }
            }
            forceToSend.push((*id, forceSum,torqueSum));
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

    #[allow(dead_code)]
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

    fn impulse_collision_projectiles(objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>)>,
    objects: &Arc<Mutex<Objects>>, map: &Map)
    {
        let mut collisionsToSend = Vec::<(usize, Vector3<f32>)>::new();

        //For every drone
        for (id, pos, _) in objs_pos_vels.iter()
        {
            collisionsToSend.extend(map.checkWalls(*pos,map.projectileRadius).iter().map(|n| (*id,n.clone())));
        }
        if !collisionsToSend.is_empty()
        {
            let objects_lck = objects.lock().unwrap();
            for (id, normalVector) in &collisionsToSend {
                objects_lck.sendSurfaceCollison(*id, map.COR, map.mi_s, map.mi_d, normalVector);
            }
            drop(objects_lck);
        }
    }

    fn quaterionToRot3(q: &Vector4<f32>) -> Matrix3<f32>
    {
        Matrix3::new(q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3], 2.0f32*(q[1]*q[2]-q[0]*q[3])           , 2.0f32*(q[1]*q[3]+q[0]*q[2]),
         2.0f32*(q[1]*q[2]+q[0]*q[3])                       , q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3], 2.0f32*(q[2]*q[3]-q[0]*q[1]),
        2.0f32*(q[1]*q[3]-q[0]*q[2])                        , 2.0f32*(q[2]*q[3]+q[0]*q[1])           , q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])
    }

    fn impulse_collision_drone(objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector4<f32>,Vector3<f32>,Vector3<f32>)>,
        drones: &Arc<Mutex<Drones>>, drones_rotor_pos: &Vec<Vec<Vector3<f32>>>, map: &Map)
    {
        let mut collisionsToSend = Vec::<(usize, Vector3<f32>, Vector3<f32>)>::new();
        let offset: nalgebra::Matrix<f32, nalgebra::Const<3>, nalgebra::Const<1>, nalgebra::ArrayStorage<f32, 3, 1>> = Vector3::new(0.0,0.0,map.sphereRadius);
        
        //For every drone
        for ((id, pos, ori, _, _),rotor_pos) in objs_pos_vels.iter().zip(drones_rotor_pos.iter())
        {
            let rot = Self::quaterionToRot3(ori);
            //For every point of drone
            for point in rotor_pos.iter()
            .map(|r| rot*(r-offset)  + pos)
            {  
                collisionsToSend.extend(map.checkWalls(point,0.0).iter().map(|n| (*id,point,n.clone())));
            }
            for point in rotor_pos.iter()
            .map(|r| rot*  (r-offset) + pos)
            {  
                collisionsToSend.extend(map.checkWalls(point,0.0).iter().map(|n| (*id,point,n.clone())));
            }
        }
        if !collisionsToSend.is_empty()
        {
            let drones_lck = drones.lock().unwrap();
            for (id,colisionPoint, normalVector) in &collisionsToSend {
                drones_lck.sendSurfaceCollison(id, map.COR, map.mi_s, map.mi_d, colisionPoint, normalVector);
            }
            drop(drones_lck);
        }
    }

}

impl Drop for CollisionDetector{
    fn drop(&mut self) {
        printLog!("Dropping collision detector instance");
        self.running.store(false, Ordering::SeqCst);
        self.collision_checker.take().unwrap().join().expect("Join error");
        printLog!("Collision detector instance dropped");
    }
}
