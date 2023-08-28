use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};
use nalgebra::{Vector3, Matrix3};
use crate::{drones::Drones, objects::Objects, config::ServerConfig};
use crate::printLog;

pub struct Wind
{
    running: Arc<AtomicBool>,
    wind_reqester: Option<thread::JoinHandle<()>>
}

impl Wind
{
    pub fn new(drones: Arc<Mutex<Drones>>,objects: Arc<Mutex<Objects>>) -> Self
    {   
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        let (wind_matrix, wind_bias) =  
            parseWindFunction(&ServerConfig::get_str("wind_matrix"),
            &ServerConfig::get_str("wind_bias"));


        let wind_reqester: JoinHandle<()> = thread::spawn(move ||
        {
            while r.load(Ordering::SeqCst) {
                let drones_lck = drones.lock().unwrap();
                let pos = drones_lck.getPositions();
                let wind: Vec<(usize,Vector3<f32>)> = pos.iter().map(|p| (p.0, Wind::calcWind(&p.1,&wind_matrix,&wind_bias))).collect();
                thread::sleep(time::Duration::from_millis(50));
                for (id,w) in wind.iter()
                {
                    let d = drones_lck.drones.lock().unwrap();
                    if let Some(drone) = d.iter().find(|drone| drone.id == *id)
                    {
                        drone.sendWind(w);
                    }

                }
                drop(drones_lck);
                thread::sleep(time::Duration::from_millis(50));
                let objects_lck = objects.lock().unwrap();
                let pos = objects_lck.getPositions();
                drop(objects_lck);
                let wind: Vec<(usize,Vector3<f32>)> = pos.iter().map(|p| (p.0,Wind::calcWind(&p.1,&wind_matrix,&wind_bias))).collect();
                if wind.is_empty()
                {
                    continue;
                }
                thread::sleep(time::Duration::from_millis(50));
                let objects_lck = objects.lock().unwrap();
                objects_lck.updateWind(wind);
                drop(objects_lck);
                thread::sleep(time::Duration::from_millis(50));
            }
        });
        Wind {running: running, wind_reqester: Some(wind_reqester) }
    }

    fn calcWind(pos: &Vector3<f32>, windMatrix: &Matrix3<f32>, windBias: &Vector3<f32>) -> Vector3<f32>
    {
        windBias + windMatrix * pos
    }
}

fn parseWindFunction(wind_matrix: &str, wind_bias: &str) -> (Matrix3<f32>, Vector3<f32>) {
    
    let wind_matrix = wind_matrix.split(';')
    .map(|row| {
        row.split(',')
            .map(|component| component.trim().parse::<f32>())
            .collect::<Result<Vec<f32>, _>>()
    })
    .collect::<Result<Vec<Vec<f32>>, _>>()
    .map(|rows| {
        if rows.len() != 3 {
            panic!("Input does not have three rows.");
        }
        Matrix3::new(
            rows[0][0], rows[0][1], rows[0][2],
            rows[1][0], rows[1][1], rows[1][2],
            rows[2][0], rows[2][1], rows[2][2],
        )
    }).unwrap();
    
    let wind_bias = wind_bias.split(',')
        .map(|component| component.trim().parse())
        .collect::<Result<Vec<f32>, _>>()
        .map(|components| {
            if components.len() != 3 {
                panic!("Input does not have three components.");
            }
            Vector3::new(components[0], components[1], components[2])
        }).unwrap();

    (wind_matrix,wind_bias)
}

impl Drop for Wind{
    fn drop(&mut self) {
        printLog!("Dropping wind instance");
        self.running.store(false, Ordering::SeqCst);
        self.wind_reqester.take().unwrap().join().expect("Join error");
        printLog!("Wind instance dropped");
    }
}