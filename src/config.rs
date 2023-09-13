use std::sync::atomic::{self, AtomicBool};
use std::{fs::File, sync::Mutex};
use std::io::Read;
use nalgebra::DMatrix;
use xmltree::Element;
use crate::obj::Obj;

const CONFIG_FILE_PATH: &str = "configs/config.yaml";

static CONFIG: Mutex<Option<serde_yaml::Value>> = Mutex::new(None);
static READY: AtomicBool = AtomicBool::new(false);

pub struct ServerConfig
{

}

impl ServerConfig {
    fn readConfig()
    {
        let f = std::fs::File::open(CONFIG_FILE_PATH).unwrap();
        let mut config_lck = CONFIG.lock().unwrap();
        *config_lck = Some(serde_yaml::from_reader(f).unwrap());
        READY.store(true, atomic::Ordering::Relaxed)
    }

    pub fn get_usize(key :&str) -> usize
    {
        if !READY.load(atomic::Ordering::Relaxed)
        {
            Self::readConfig();
        }
        let config_lck = CONFIG.lock().unwrap();
        let config_data = config_lck.clone().unwrap();
        config_data[key].as_u64().unwrap() as usize
    }

    pub fn get_f32(key :&str) -> f32
    {
        if !READY.load(atomic::Ordering::Relaxed)
        {
            Self::readConfig();
        }
        let config_lck = CONFIG.lock().unwrap();
        let config_data = config_lck.clone().unwrap();
        config_data[key].as_f64().unwrap() as f32
    }

    pub fn get_str(key :&str) -> String
    {
        if !READY.load(atomic::Ordering::Relaxed)
        {
            Self::readConfig();
        }
        let config_lck = CONFIG.lock().unwrap();
        let config_data = config_lck.clone().unwrap();
        config_data[key].as_str().unwrap().to_owned()
    }

    pub fn get_bool(key :&str) -> bool
    {
        if !READY.load(atomic::Ordering::Relaxed)
        {
            Self::readConfig();
        }
        let config_lck = CONFIG.lock().unwrap();
        let config_data = config_lck.clone().unwrap();
        config_data[key].as_bool().unwrap()
    }
}

#[derive(Clone)]
pub struct DroneConfig {
    pub name: String,
    pub drone_type: String,
    pub mesh: DMatrix<f32>
}


impl DroneConfig {
    pub fn parse(file_path: &str) -> Result<DroneConfig, Box<dyn std::error::Error>> {
        let mut file = File::open(file_path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;
        let root = Element::parse(contents.as_bytes())?;
        let name = root.get_child("name").unwrap().get_text().unwrap().to_string();
        let drone_type = root.get_child("type").unwrap().get_text().unwrap().to_string();
        let drone_model = Obj::load_from_file(format!("./assets/drones/{}/model/model.obj", &drone_type.as_str()).as_str(),true);
        let mesh = drone_model.getMesh();

        let config = DroneConfig {
            name,
            drone_type,
            mesh
        };

        Ok(config)
    }
}