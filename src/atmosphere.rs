use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};
use nalgebra::{Vector3, Matrix3};
use crate::{drones::Drones, objects::Objects, config::ServerConfig};
use crate::printLog;

const R_AIR_CONSTANT: f32 = 287.052874;
const TEMP_ALTITUDE_RATE: f32 = 6.5e-3;
const GRAVITY_ACCELERATION: f32 = 9.8067;

pub struct Atmosphere
{
    running: Arc<AtomicBool>,
    atmosphere_reqester: Option<thread::JoinHandle<()>>
}

#[derive(Debug)]
pub struct AtmosphereInfo
{
    pub wind: Vector3<f32>,
    pub air_temperature: f32,
    pub air_pressure: f32,
    pub air_density: f32,
}

impl Atmosphere
{
    pub fn new(drones: Arc<Mutex<Drones>>,objects: Arc<Mutex<Objects>>) -> Self
    {   
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        let (wind_matrix, wind_bias) =  
            Atmosphere::parseWindFunction(&ServerConfig::get_str("wind_matrix"),
            &ServerConfig::get_str("wind_bias"));
        let T0 = ServerConfig::get_f32("temperature");
        let p0 = ServerConfig::get_f32("pressure");


        let atmosphere_reqester: JoinHandle<()> = thread::spawn(move ||
        {
            while r.load(Ordering::SeqCst) {
                //Update aircrafts
                let drones_lck = drones.lock().unwrap();
                let pos = drones_lck.getPositions();
                let mut infos = Vec::new();
                for p in pos
                {
                    let (air_temperature,air_pressure, air_density) = calcAirInfo(&p.1,T0,p0);
                    let info = 
                        AtmosphereInfo{
                            wind: Atmosphere::calcWind(&p.1,&wind_matrix,&wind_bias),
                            air_temperature,
                            air_pressure,
                            air_density
                        };
                    //println!("{:?}", info);
                    infos.push((p.0,info));
                }
                thread::sleep(time::Duration::from_millis(50));
                for (id,info) in infos.iter()
                {
                    let d = drones_lck.drones.lock().unwrap();
                    if let Some(drone) = d.iter().find(|drone| drone.id == *id)
                    {
                        drone.sendAtmosphereInfo(info);
                    }
                }
                drop(drones_lck);
                thread::sleep(time::Duration::from_millis(50));

                //Update objects
                let objects_lck = objects.lock().unwrap();
                let pos = objects_lck.getPositions();
                drop(objects_lck);
                let wind: Vec<(usize,Vector3<f32>)> = pos.iter().map(|p| 
                    (p.0,Atmosphere::calcWind(&p.1,&wind_matrix,&wind_bias))
                ).collect();
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
        Atmosphere {running: running, atmosphere_reqester: Some(atmosphere_reqester) }
    }

    fn calcWind(pos: &Vector3<f32>, windMatrix: &Matrix3<f32>, windBias: &Vector3<f32>) -> Vector3<f32>
    {
        windBias + windMatrix * pos
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
}

fn calcAirInfo(pos: &Vector3<f32>, temp0: f32, pressure0: f32) -> (f32, f32, f32) {
    let h = -pos[2];
    let temp = calcTemperature(h,temp0);
    let pressure = calcPressure(h,pressure0, temp0);
    let density = calcDensity(temp, pressure);
    return (temp,pressure,density);
}

fn calcDensity(temp: f32, pressure: f32) -> f32 {
    pressure/(temp*R_AIR_CONSTANT)
}

fn calcPressure(h: f32, pressure0: f32, temp0: f32) -> f32 {
    pressure0 * (1.0 - TEMP_ALTITUDE_RATE * (h/temp0)).powf(GRAVITY_ACCELERATION/(R_AIR_CONSTANT*TEMP_ALTITUDE_RATE))
}

fn calcTemperature(h: f32, temp0: f32) -> f32 {
    temp0 - h * TEMP_ALTITUDE_RATE
}

impl Drop for Atmosphere{
    fn drop(&mut self) {
        printLog!("Dropping wind instance");
        self.running.store(false, Ordering::SeqCst);
        self.atmosphere_reqester.take().unwrap().join().expect("Join error");
        printLog!("Wind instance dropped");
    }
}