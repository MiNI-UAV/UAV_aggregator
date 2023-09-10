use std::sync::atomic::{self, AtomicBool};
use std::{fs::File, sync::Mutex};
use std::io::Read;
use xmltree::Element;
use nalgebra::Vector3;

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
    pub mass: f32,
    pub inertia: Inertia,
    pub noOfRotors: usize,
    pub rotors: Vec<Rotor>,
    pub aero: Aero,
    pub pid: PID,
    pub control: Control,
    pub mixer: Vec<f32>,
}

#[derive(Clone)]
pub struct Inertia {
    pub mass: f32,
    pub ix: f32,
    pub iy: f32,
    pub iz: f32,
    pub ixy: f32,
    pub ixz: f32,
    pub iyz: f32,
}

#[derive(Clone)]
pub struct Rotor {
    pub force_coeff: f32,
    pub torque_coeff: f32,
    pub position: Vector3<f32>,
    pub direction: isize,
    pub time_constant: f32,
}

#[derive(Clone)]
pub struct Aero {
    pub s: f32,
    pub d: f32,
    pub c: Vec<f32>,
}

#[derive(Clone)]
pub struct PID {
    pub x: AxisPID,
    pub y: AxisPID,
    pub z: AxisPID,
    pub u: AxisPID,
    pub v: AxisPID,
    pub w: AxisPID,
    pub fi: AxisPID,
    pub theta: AxisPID,
    pub psi: AxisPID,
    pub roll: AxisPID,
    pub pitch: AxisPID,
    pub yaw: AxisPID,
}

#[derive(Clone)]
pub struct AxisPID {
    pub p: f32,
    pub i: f32,
    pub d: f32,
    pub min: f32,
    pub max: f32,
}

#[derive(Clone)]
pub struct Control {
    pub max_speed: f32,
    pub hover_speed: f32,
}

impl DroneConfig {
    pub fn parse(file_path: &str) -> Result<DroneConfig, Box<dyn std::error::Error>> {
        // Read the XML configuration file
        let mut file = File::open(file_path)?;
        let mut contents = String::new();
        file.read_to_string(&mut contents)?;

        // Parse the XML content
        let root = Element::parse(contents.as_bytes())?;

        let rotors = parse_rotors(root.get_child("rotors").unwrap());

        // Create a DroneConfig instance and populate its fields
        let config = DroneConfig {
            name: root.get_child("name").unwrap().get_text().unwrap().to_string(),
            drone_type: root.get_child("type").unwrap().get_text().unwrap().to_string(),
            mass: root.get_child("ineria").unwrap().get_child("mass").unwrap().get_text().unwrap().parse()?,
            inertia: parse_inertia(root.get_child("ineria").unwrap()),
            noOfRotors: rotors.len(),
            rotors,
            aero: parse_aero(root.get_child("aero").unwrap()),
            pid: parse_pid(root.get_child("PID").unwrap()),
            control: parse_control(root.get_child("control").unwrap()),
            mixer: parse_mixer(&root.get_child("mixer").unwrap().get_text().unwrap()),
        };

        Ok(config)
    }
}

fn parse_inertia(inertia_elem: &Element) -> Inertia {
    Inertia {
        mass: inertia_elem.get_child("mass").unwrap().get_text().unwrap().parse().unwrap(),
        ix: inertia_elem.get_child("Ix").unwrap().get_text().unwrap().parse().unwrap(),
        iy: inertia_elem.get_child("Iy").unwrap().get_text().unwrap().parse().unwrap(),
        iz: inertia_elem.get_child("Iz").unwrap().get_text().unwrap().parse().unwrap(),
        ixy: inertia_elem.get_child("Ixy").unwrap().get_text().unwrap().parse().unwrap(),
        ixz: inertia_elem.get_child("Ixz").unwrap().get_text().unwrap().parse().unwrap(),
        iyz: inertia_elem.get_child("Iyz").unwrap().get_text().unwrap().parse().unwrap(),
    }
}

fn parse_rotors(rotors_elem: &Element) -> Vec<Rotor> {

    let mut rotors = Vec::new();



    for rotor_node in &rotors_elem.children
    {
        let rotor = Rotor { 
            position: Vector3::<f32>::from_vec( 
                rotor_node.as_element().unwrap().get_child("position").unwrap().get_text().unwrap()
                .split_whitespace()
                .map(|value| value.parse().unwrap())
                .collect()),
            force_coeff: rotor_node.as_element().unwrap().get_child("forceCoff").unwrap().get_text().unwrap().parse().unwrap(),
            torque_coeff: rotor_node.as_element().unwrap().get_child("torqueCoff").unwrap().get_text().unwrap().parse().unwrap(),
            direction: rotor_node.as_element().unwrap().get_child("direction").unwrap().get_text().unwrap().parse().unwrap(),
            time_constant: rotor_node.as_element().unwrap().get_child("timeConstant").unwrap().get_text().unwrap().parse().unwrap(),
        };
        rotors.push(rotor);
    } rotors
}

fn parse_aero(_aero_elem: &Element) -> Aero {
    Aero {
        s: 0.0,
        d: 0.0,
        c: Vec::new()
    }
}

fn parse_pid(pid_elem: &Element) -> PID {
    fn parse_axis_pid(axis_elem: &Element) -> AxisPID {
        AxisPID {
            p: axis_elem.get_child("P").unwrap().get_text().unwrap().parse().unwrap(),
            i: axis_elem.get_child("I").unwrap().get_text().unwrap().parse().unwrap(),
            d: axis_elem.get_child("D").unwrap().get_text().unwrap().parse().unwrap(),
            min: axis_elem.get_child("min").unwrap().get_text().unwrap().parse().unwrap(),
            max: axis_elem.get_child("max").unwrap().get_text().unwrap().parse().unwrap(),
        }
    }

    PID {
        x: parse_axis_pid(pid_elem.get_child("X").unwrap()),
        y: parse_axis_pid(pid_elem.get_child("Y").unwrap()),
        z: parse_axis_pid(pid_elem.get_child("Z").unwrap()),
        u: parse_axis_pid(pid_elem.get_child("U").unwrap()),
        v: parse_axis_pid(pid_elem.get_child("V").unwrap()),
        w: parse_axis_pid(pid_elem.get_child("W").unwrap()),
        fi: parse_axis_pid(pid_elem.get_child("Fi").unwrap()),
        theta: parse_axis_pid(pid_elem.get_child("Theta").unwrap()),
        psi: parse_axis_pid(pid_elem.get_child("Psi").unwrap()),
        roll: parse_axis_pid(pid_elem.get_child("Roll").unwrap()),
        pitch: parse_axis_pid(pid_elem.get_child("Pitch").unwrap()),
        yaw: parse_axis_pid(pid_elem.get_child("Yaw").unwrap()),
    }
}

fn parse_control(control_elem: &Element) -> Control {
    Control {
        max_speed: control_elem.get_child("maxSpeed").unwrap().get_text().unwrap().parse().unwrap(),
        hover_speed: control_elem.get_child("hoverSpeed").unwrap().get_text().unwrap().parse().unwrap(),
    }
}

fn parse_mixer(mixer_text: &str) -> Vec<f32> {
    mixer_text
        .split(',')
        .map(|value| value.trim().parse().unwrap())
        .collect()
}
