use std::f32::consts::PI;
use std::sync::atomic::{self, AtomicBool};
use std::{fs::File, sync::Mutex};
use std::io::Read;
use nalgebra::{DMatrix,Vector3};
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

#[derive(Debug)]
#[derive(Clone)]
#[allow(dead_code)]
pub struct AmmoParams {
    pub name: String,
    pub model: String,
    pub V0: Vector3<f32>,
    pub position: Vector3<f32>,
    pub reload: f32,
    pub radius: f32,
    pub C0: f32,
    pub mass: f32,
    pub ammount: usize,

    pub CS: f32,
}

#[derive(Debug)]
#[derive(Clone)]
#[allow(dead_code)]
pub struct CargoParams {
    pub name: String,
    pub model: String,
    pub hook: Vector3<f32>,
    pub length: f32,
    pub k: f32,
    pub b: f32,
    pub reload: f32,
    pub radius: f32,
    pub C0: f32,
    pub mass: f32,
    pub ammount: usize,

    pub CS: f32,
}

fn parse_ammo(element: &Element) -> AmmoParams {
    let name = element.name.clone();
    let model = element.get_child("model").unwrap().get_text().unwrap().to_string();
    let V0 = Vector3::from_vec(element.get_child("V0").unwrap().get_text().unwrap().split(", ")
        .map(|s| s.parse::<f32>().unwrap())
        .collect::<Vec<f32>>());
    let position = Vector3::from_vec(element.get_child("position").unwrap().get_text().unwrap().split(", ")
        .map(|s| s.parse::<f32>().unwrap())
        .collect::<Vec<f32>>());
    let reload = element.get_child("reload").unwrap().get_text().unwrap().parse().unwrap();
    let radius = element.get_child("radius").unwrap().get_text().unwrap().parse().unwrap();
    let C0 = element.get_child("C0").unwrap().get_text().unwrap().parse().unwrap();
    let mass = element.get_child("mass").unwrap().get_text().unwrap().parse().unwrap();
    let ammount = element.get_child("ammount").unwrap().get_text().unwrap().parse().unwrap();

    AmmoParams {
        name,
        model,
        V0,
        position,
        reload,
        radius,
        C0,
        mass,
        ammount,
        CS: C0 * PI * radius* radius
    }
}

fn parse_cargo(element: &Element) -> CargoParams {
    let name = element.name.clone();
    let model = element.get_child("model").unwrap().get_text().unwrap().to_string();
    let hook = Vector3::from_vec(element.get_child("hook").unwrap().get_text().unwrap().split(", ")
        .map(|s| s.parse::<f32>().unwrap())
        .collect::<Vec<f32>>());
    let length = element.get_child("length").unwrap().get_text().unwrap().parse().unwrap();
    let k = element.get_child("k").unwrap().get_text().unwrap().parse().unwrap();
    let b = element.get_child("b").unwrap().get_text().unwrap().parse().unwrap();
    let reload = element.get_child("reload").unwrap().get_text().unwrap().parse().unwrap();
    let radius = element.get_child("radius").unwrap().get_text().unwrap().parse().unwrap();
    let C0 = element.get_child("C0").unwrap().get_text().unwrap().parse().unwrap();
    let mass = element.get_child("mass").unwrap().get_text().unwrap().parse().unwrap();
    let ammount = element.get_child("ammount").unwrap().get_text().unwrap().parse().unwrap();

    CargoParams {
        name,
        model,
        hook,
        length,
        k,
        b,
        reload,
        radius,
        C0,
        mass,
        ammount,
        CS: C0 * PI * radius* radius
    }
}

#[derive(Clone)]
pub struct DroneConfig {
    pub name: String,
    pub drone_type: String,
    pub cargo: Vec<CargoParams>,
    pub ammo: Vec<AmmoParams>,
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

        let cargo = root.get_child("cargo").unwrap().children.iter()
        .map(|child| parse_cargo(child.as_element().unwrap())).collect();
        let ammo = root.get_child("ammo").unwrap().children.iter()
        .map(|child| parse_ammo(child.as_element().unwrap())).collect();

        let drone_model = Obj::load_from_file(format!("./assets/drones/{}/model/model.obj", &drone_type.as_str()).as_str(),true);
        let mesh = drone_model.getMesh();

        let config = DroneConfig {
            name,
            drone_type,
            cargo,
            ammo,
            mesh
        };

        Ok(config)
    }
}