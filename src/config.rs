use std::fs::File;
use std::io::Read;
use xmltree::Element;
use nalgebra::{Vector3, Matrix3xX};

#[derive(Clone)]
pub struct DroneConfig {
    pub name: String,
    pub mass: f32,
    pub inertia: Inertia,
    pub rotors: Rotors,
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
pub struct Rotors {
    pub num: u32,
    pub force_coeff: f32,
    pub torque_coeff: f32,
    pub positions: Matrix3xX<f32>,
    pub direction: Vec<i32>,
    pub time_constants: Vec<f32>,
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

        // Create a DroneConfig instance and populate its fields
        let config = DroneConfig {
            name: root.get_child("name").unwrap().get_text().unwrap().to_string(),
            mass: root.get_child("ineria").unwrap().get_child("mass").unwrap().get_text().unwrap().parse()?,
            inertia: parse_inertia(root.get_child("ineria").unwrap()),
            rotors: parse_rotors(root.get_child("rotors").unwrap()),
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

fn parse_rotors(rotors_elem: &Element) -> Rotors {
    let num = rotors_elem.get_child("no").unwrap().get_text().unwrap().parse().unwrap();
    let force_coeff = rotors_elem.get_child("forceCoff").unwrap().get_text().unwrap().parse().unwrap();
    let torque_coeff = rotors_elem.get_child("torqueCoff").unwrap().get_text().unwrap().parse().unwrap();

    let positions_elem = rotors_elem.get_child("positions").unwrap();
    let positions = positions_elem
        .children
        .iter()
        .map(|item| {
            let values: Vec<f32> = item
                .as_element()
                .unwrap()
                .get_text()
                .unwrap()
                .split_whitespace()
                .map(|value| value.parse().unwrap())
                .collect();
            Vector3::<f32>::from_vec(values)
        });
    let mut posistion_matrix = Matrix3xX::<f32>::zeros(positions.len());
    for (i,elem) in positions.enumerate()    
    {
        posistion_matrix.column_mut(i).copy_from(&elem);
    }
    let direction = rotors_elem
        .get_child("direction")
        .unwrap()
        .children
        .iter()
        .map(|item| item.as_element().unwrap().get_text().unwrap().parse().unwrap())
        .collect();

    let time_constants = rotors_elem
        .get_child("timeConstants")
        .unwrap()
        .children
        .iter()
        .map(|item| item.as_element().unwrap().get_text().unwrap().parse().unwrap())
        .collect();

    Rotors {
        num,
        force_coeff,
        torque_coeff,
        positions: posistion_matrix,
        direction,
        time_constants,
    }
}

fn parse_aero(aero_elem: &Element) -> Aero {
    Aero {
        s: aero_elem.get_child("S").unwrap().get_text().unwrap().parse().unwrap(),
        d: aero_elem.get_child("d").unwrap().get_text().unwrap().parse().unwrap(),
        c: aero_elem
            .get_child("C")
            .unwrap()
            .get_text()
            .unwrap()
            .split_whitespace()
            .map(|value| value.parse().unwrap())
            .collect(),
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
