use nalgebra::{Vector3, Matrix3};
use std::fs::File;
use std::io::{BufRead, BufReader};

pub struct Map
{
    walls: Obj,
    collisionPlusEps: f32,
    collisionMinusEps: f32
}

impl Map
{
    pub fn new(path: &str, collisionPlusEps: f32, collisionMinusEps: f32) -> Self
    {
        let walls = Obj::from_file(path);
        Map{walls, collisionPlusEps, collisionMinusEps}
    }

    pub fn checkWalls(&self, point: Vector3<f32>) -> Vec<Vector3<f32>>
    {
        let mut normalsInColisionPoint = Vec::new();
        for face in &self.walls.faces {
            if let (true, dist) = face.projectPoint(point)
            {
                if dist <= self.collisionPlusEps && dist >= self.collisionMinusEps
                {
                    normalsInColisionPoint.push(face.normal)
                }
            }
        }
        normalsInColisionPoint
    }
}

struct Obj
{
    _vertices: Vec<Vector3<f32>>,
    _normals: Vec<Vector3<f32>>,
    faces: Vec<Face>
}

impl Obj
{
    fn from_file(file_path: &str) -> Self {
        let file = File::open(file_path).expect("Can not open file");
        let reader = BufReader::new(file);

        let mut vertices = Vec::<Vector3<f32>>::new();
        let mut normals = Vec::<Vector3<f32>>::new();
        let mut faces = Vec::<Face>::new();

        for line in reader.lines() 
        {
            let line = line.expect("Can not read line");
            let elements: Vec<&str> = line.trim().split_whitespace().collect();
            
            match elements[0] {
                "v" => {
                    assert_eq!(elements.len(), 4);
                    let x: f32 = elements[1].parse().expect("Invalid vertex coordinate");
                    let y: f32 = elements[2].parse().expect("Invalid vertex coordinate");
                    let z: f32 = elements[3].parse().expect("Invalid vertex coordinate");
                    vertices.push(Vector3::new(x, y, z));
                }
                "vn" => {
                    assert_eq!(elements.len(), 4);
                    let x: f32 = elements[1].parse().expect("Invalid normal coordinate");
                    let y: f32 = elements[2].parse().expect("Invalid normal coordinate");
                    let z: f32 = elements[3].parse().expect("Invalid normal coordinate");
                    normals.push(Vector3::new(x, y, z).normalize());
                }
                "f" => {
                    assert_eq!(elements.len(), 4);

                    let mut face_vertices = [Vector3::zeros();3];
                    let mut face_normals= [Vector3::zeros();3];

                    for (i, element) in elements[1..].iter().enumerate() {
                        let items: Vec<String> = element.split('/').map(|s| s.to_string()).collect();
                        assert_eq!(items.len(), 3);
                        face_vertices[i] = vertices[items[0].parse::<usize>().unwrap()-1];
                        if !items[2].is_empty()
                        {
                            face_normals[i] = normals[items[2].parse::<usize>().unwrap()-1];
                        }
                    }
                    faces.push(Face::new(face_vertices,face_normals));
                }
                _ => continue,
            }
        }

        Obj{_vertices: vertices, _normals: normals,faces} 
    }
}

struct Face
{
    _vertices: [Vector3<f32>;3],
    _normals: [Vector3<f32>;3],
    normal: Vector3<f32>,
    projectMatrix: Matrix3<f32>,
    base: Vector3<f32>
}

impl Face {

    fn new(vertices: [Vector3<f32>;3],normals: [Vector3<f32>;3]) -> Self
    {
        let s = vertices[1]-vertices[0];
        let t = vertices[2]-vertices[0];
        let v_mean = (normals[0] + normals[1] + normals[2])/3.0;
        let mut n = s.cross(&t);
        if n.dot(&v_mean) < 0.0
        {
            n *= -1.0;
        }
        n.normalize_mut();
        let invProjectMatrix = Matrix3::from_columns(&[s,t,n]);
        //println!("Inv: {}", invProjectMatrix);

        Face{_vertices: vertices, _normals: normals, normal: n,
                projectMatrix: invProjectMatrix.try_inverse().expect("Can not inverse project matrix"),
                base: vertices[0].clone()}
    }

    fn projectPoint(&self, point: Vector3<f32>) -> (bool, f32)
    {

        let res = self.projectMatrix* (point-self.base);
        if res[0] >= 0.0 && res[0] <= 1.0 && res[1] >= 0.0 && res[1] <= 1.0 && res[0]+res[1] <= 1.0
        {
            if res[2].abs() < 0.1
            {

                println!("Point: {}", point);
                println!("Face vertex: {:?}", self._vertices);
                println!("Face normal: {:?}", self.normal);
                println!("Res: {}", res);
                            
            }

            return (true,res[2])
        } 
        (false, 0.0)
    }
}