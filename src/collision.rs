use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time, collections::HashMap};
use nalgebra::{Vector3,Vector4, Matrix3, DMatrix};
use std::time::Instant;
use crate::{drones::Drones, objects::Objects, map::Map, config::ServerConfig, obj::Obj};
use crate::printLog;

/// Detect collision in simulation. Checks collision uav-map, obj-map uav-uav and uav-obj.
pub struct CollisionDetector
{
    running: Arc<AtomicBool>,
    collision_checker: Option<thread::JoinHandle<()>>
}

impl CollisionDetector
{
    /// Constructor
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
            let mut loop_time = ServerConfig::get_f32("collisionLoopTime");
            let nominal_loop_time  = time::Duration::from_secs_f32(loop_time);
            let mut meshes = HashMap::<String,DMatrix<f32>>::new();
            while r.load(Ordering::SeqCst) {
                let start = Instant::now();
                let drones_lck = _drones.lock().unwrap();
                let drones_pos_vel = drones_lck.getPosOriVels();
                let types = drones_lck.getTypes();
                drop(drones_lck);

                let obj_lck = _objects.lock().unwrap();
                let objs_pos_vels_radius = obj_lck.getPosVelsRadius();
                drop(obj_lck);
                
                //Drone collision with map
                Self::impulse_collision_drone(&drones_pos_vel,&_drones,&mut meshes, &types,&map,loop_time);
                Self::impulse_collision_projectiles(&objs_pos_vels_radius,&_objects,&map,loop_time);


                //Collision between objects are negligible
                Self::colisions_between_drones(&drones_pos_vel,map.minimalDist);
                Self::colisions_drones_obj(&drones_pos_vel, &objs_pos_vels_radius,map.minimalDist);

                //Eliminate objects outside boundary box
                Self::boundary_box_obj(&objs_pos_vels_radius, &_objects, box_min, box_max);
                              
                #[allow(unused_variables)]
                let elapsed = start.elapsed();
                if elapsed < nominal_loop_time
                {
                    thread::sleep(nominal_loop_time - elapsed);
                }
                let final_elapsed = start.elapsed();
                loop_time = 0.7 * loop_time + 0.3 * final_elapsed.as_secs_f32();
                //printLog!("Loop time: {} ms", loop_time * 1000f32); 
            }
        });
        CollisionDetector {running,
             collision_checker: Some(collision_checker)
            }
    }

    /// Find collisions between pair of UAVs
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

    /// Find collisions between UAV and object
    fn colisions_drones_obj(drones_pos_vel: &Vec<(usize,Vector3<f32>,Vector4<f32>,Vector3<f32>,Vector3<f32>)>,
        objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>,f32)>, minimal_dist: f32)
    {
        for obj1 in drones_pos_vel.iter() {
            for obj2 in objs_pos_vels.iter() {
                let dist: Vector3<f32> = obj1.1-obj2.1;
                if dist.dot(&obj2.2) > 0.0 && dist.dot(&dist).abs() < minimal_dist
                {
                    printLog!("Collision detected between drone {} and object {}", obj1.0,obj2.0);
                }
            }
        }
    }

    #[allow(dead_code)]
    /// Find object outside boundary box and remove them
    fn boundary_box_obj(objs_pos_vels: &Vec<(usize,Vector3<f32>,Vector3<f32>,f32)>,
        objects: &Arc<Mutex<Objects>>, box_min: Vector3<f32>, box_max: Vector3<f32>)
    {
        let mut objToKill = Vec::new();
        for (id,pos,_,_) in objs_pos_vels.iter() 
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

    /// Find all collision between Object and map walls. Handles collision
    fn impulse_collision_projectiles(objs_pos_vels_radius: &Vec<(usize,Vector3<f32>,Vector3<f32>,f32)>,
    objects: &Arc<Mutex<Objects>>, map: &Map, loop_time: f32)
    {
        let mut collisionsToSend = Vec::<(usize, Vector3<f32>)>::new();

        //For every drone
        for (id, pos, vel,radius) in objs_pos_vels_radius.iter()
        {
            collisionsToSend.extend(map.checkWalls2(*pos,*vel,loop_time, *radius).iter().map(|n| (*id,n.clone())));
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

    /// Converts quaterion to rotation matrix
    fn quaterionToRot3(q: &Vector4<f32>) -> Matrix3<f32>
    {
        Matrix3::new(q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3], 2.0f32*(q[1]*q[2]-q[0]*q[3])           , 2.0f32*(q[1]*q[3]+q[0]*q[2]),
         2.0f32*(q[1]*q[2]+q[0]*q[3])                       , q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3], 2.0f32*(q[2]*q[3]-q[0]*q[1]),
        2.0f32*(q[1]*q[3]-q[0]*q[2])                        , 2.0f32*(q[2]*q[3]+q[0]*q[1])           , q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])
    }

    /// Find all collision between UAV and map walls. Handles collision
    fn impulse_collision_drone(uav_pos_vels: &Vec<(usize,Vector3<f32>,Vector4<f32>,Vector3<f32>,Vector3<f32>)>,
        drones: &Arc<Mutex<Drones>>,meshes: &mut HashMap<String,DMatrix<f32>>, types: &Vec<String>, map: &Map, loop_time: f32)
    {
        let mut collisionsToSend = Vec::<(usize, Vector3<f32>, Vector3<f32>)>::new();
        
        //For every drone
        for ((id, pos, ori, vel, angVel),drone_type) in uav_pos_vels.iter().zip(types.iter())
        {
            let mut best_depth = f32::MAX;
            let mut best_point: Vector3<f32> = Vector3::zeros();
            let mut best_normal: Vector3<f32> = Vector3::zeros();
            let rot = Self::quaterionToRot3(ori);
            let mesh = getMesh(meshes,drone_type);
            mesh.column_iter().for_each(|col| {
                let point = rot * col + pos;
                let point_vel =  rot * (vel + angVel.cross(&col));
                if let Some((depth,normal)) = 
                    map.checkWallsBest2(point,point_vel, loop_time)
                {
                    if depth < best_depth
                    {
                        best_depth = depth;
                        best_normal = normal;
                        best_point = point;
                    }
                }
            });
            if best_depth < f32::MAX
            {
                collisionsToSend.push((*id,best_point,best_normal));
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

/// Returns mesh of UAV. Load UAV OBJ file on first call.
fn getMesh<'a>(meshes: &'a mut HashMap<String, DMatrix<f32>>, drone_type: & str) -> &'a DMatrix<f32> {
    if !meshes.contains_key(drone_type)
    {
        let drone_model = Obj::load_from_file(format!("./assets/drones/{}/model/model.obj", drone_type).as_str(),true);
        let mesh = drone_model.getMesh();
        meshes.insert(drone_type.to_string(), mesh);
    }
    meshes.get(drone_type).unwrap()
}

/// Deconstructor
impl Drop for CollisionDetector{
    fn drop(&mut self) {
        printLog!("Dropping collision detector instance");
        self.running.store(false, Ordering::SeqCst);
        self.collision_checker.take().unwrap().join().expect("Join error");
        printLog!("Collision detector instance dropped");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static EPS: f32 = 1e-3f32;

    #[test]
    fn create_rot_matrix_from_quaterion() {
        // Example from MATLAB docs
        // https://www.mathworks.com/help/nav/ref/quaternion.rotmat.html
        let q = Vector4::new(0.8924, 0.23912,0.36964,-0.099046);
        let rot = CollisionDetector::quaterionToRot3(&q);
        //assert!((rot.m12 - 0.0).abs() < EPS);
        assert!((rot.m23 + 0.5).abs() < EPS);
        assert!((rot.m32 - 0.3536).abs() < EPS);
        assert!((rot.m21 - 0.0).abs() < EPS);
    }
}
