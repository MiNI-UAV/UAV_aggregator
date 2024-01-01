use std::{thread::{JoinHandle, self}, sync::{Mutex, Arc, atomic::{AtomicBool, Ordering}}, time};
use nalgebra::{Vector3, Matrix3};
use rand::{rngs::ThreadRng, Rng};
use crate::{drones::Drones, objects::Objects, config::ServerConfig};
use crate::printLog;

/// Air thermodynamic gas constant for dry air
const R_AIR_CONSTANT: f32 = 287.052874;
/// Temperature drop due to altitude rise
const TEMP_ALTITUDE_RATE: f32 = 6.5e-3;
/// Gravity constant on Earth
pub const GRAVITY_ACCELERATION: f32 = 9.8067;

/// Atmosphere simulation. Notify UAVs and Objects about atmosphere state.
pub struct Atmosphere
{
    running: Arc<AtomicBool>,
    atmosphere_reqester: Option<thread::JoinHandle<()>>
}

/// Atmosphere state DTO
#[derive(Debug)]
pub struct AtmosphereInfo
{
    /// wind vector in m/s
    pub wind: Vector3<f32>,
    /// air temperature in K
    pub air_temperature: f32,
    /// air pressure Pa
    pub air_pressure: f32,
    /// air_density kg/m3
    pub air_density: f32,
}

impl Atmosphere
{
    /// Construct atmosphere instance. Require arcs to drones and objects to inform them.
    pub fn new(drones: Arc<Mutex<Drones>>,objects: Arc<Mutex<Objects>>) -> Self
    {   
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();

        let (wind_matrix, wind_bias) =  
            Atmosphere::parseWindFunction(&ServerConfig::get_str("wind_matrix"),
            &ServerConfig::get_str("wind_bias"));
        let wind_turbulence_scale = ServerConfig::get_f32("wind_turbulence");

        let T0 = ServerConfig::get_f32("temperature");
        let p0 = ServerConfig::get_f32("pressure");


        let atmosphere_reqester: JoinHandle<()> = thread::spawn(move ||
        {
            let mut wind_turbulence_rng: [rand::rngs::ThreadRng; 3] = Default::default();
            let mut turbulence = Vector3::zeros();
            while r.load(Ordering::SeqCst) {
                //Update aircrafts
                let drones_lck = drones.lock().unwrap();
                let pos = drones_lck.getPositions();
                let mut infos = Vec::new();
                Atmosphere::calcWindTurbulance(&mut turbulence,wind_turbulence_scale, &mut wind_turbulence_rng);
                for p in pos
                {
                    let (air_temperature,air_pressure, air_density) = calcAirInfo(&p.1,T0,p0);
                    let info = 
                        AtmosphereInfo{
                            wind: Atmosphere::calcWind(&p.1,&wind_matrix,&wind_bias) + turbulence,
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

    /// Returns wind vector for specified position
    fn calcWind(pos: &Vector3<f32>, windMatrix: &Matrix3<f32>, windBias: &Vector3<f32>) -> Vector3<f32>
    {
        windBias + windMatrix * pos
    }

    /// Returns next turbulance vector
    fn calcWindTurbulance(turbulence: &mut Vector3<f32>, turbulence_scale: f32, rng: &mut [ThreadRng; 3])
    {
        if turbulence_scale < f32::EPSILON
        {
            return;
        }
        for i in 0..turbulence.len()
        {
            turbulence[i] = turbulence[i] + rng[i].gen_range(-turbulence_scale..turbulence_scale);
            turbulence[i] = turbulence[i].clamp(-3.0 * turbulence_scale, 3.0 * turbulence_scale);
        }
    }


    /// Parses wind matrix and wind bias from string from configuration file.
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


/// Calc air information for specified position
fn calcAirInfo(pos: &Vector3<f32>, temp0: f32, pressure0: f32) -> (f32, f32, f32) {
    let h = -pos[2];
    let temp = calcTemperature(h,temp0);
    let pressure = calcPressure(h,pressure0, temp0);
    let density = calcDensity(temp, pressure);
    return (temp,pressure,density);
}

/// Calculates air density in kg/m3 for specified temperature and pressure
fn calcDensity(temp: f32, pressure: f32) -> f32 {
    pressure/(temp*R_AIR_CONSTANT)
}

/// Calculates air pressure in Pa for specified altitude
fn calcPressure(h: f32, pressure0: f32, temp0: f32) -> f32 {
    pressure0 * (1.0 - TEMP_ALTITUDE_RATE * (h/temp0)).powf(GRAVITY_ACCELERATION/(R_AIR_CONSTANT*TEMP_ALTITUDE_RATE))
}

/// Calculates air temperature in K for specified altitude
fn calcTemperature(h: f32, temp0: f32) -> f32 {
    temp0 - h * TEMP_ALTITUDE_RATE
}

/// Deconstructor
impl Drop for Atmosphere{
    fn drop(&mut self) {
        printLog!("Dropping wind instance");
        self.running.store(false, Ordering::SeqCst);
        self.atmosphere_reqester.take().unwrap().join().expect("Join error");
        printLog!("Wind instance dropped");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static EPS: f32 = 1e-2f32;

    #[test]
    fn calc_temp_test() {
        assert!((calcTemperature(0.0,0.0) - 0.0).abs() < EPS);
        assert!((calcTemperature(0.0,50.0) - 50.0).abs() < EPS);
        assert!((calcTemperature(0.0,-50.0) + 50.0).abs() < EPS);
        assert!((calcTemperature(1000.0,0.0) + 6.5).abs() < EPS);
        assert!((calcTemperature(2000.0,0.0) + 13.0).abs() < EPS);
    }

    #[test]
    fn calc_pressure_test() {
        assert!((calcPressure(0.0,101325.0,288.15) - 101325.0).abs() < EPS);
        assert!((calcPressure(0.0,101325.0,273.15) - 101325.0).abs() < EPS);
        assert!((calcPressure(1000.0,101325.0,288.15) - 89874.52).abs() < EPS);
    }

    #[test]
    fn calc_density_test() {
        assert!((calcDensity(273.15,0.0) - 0.0).abs() < EPS);
        assert!((calcDensity(273.15 + 20.0,101325.0) - 1.204).abs() < EPS);
        assert!((calcDensity(273.15 + 15.0,101325.0) - 1.225).abs() < EPS);
    }
}