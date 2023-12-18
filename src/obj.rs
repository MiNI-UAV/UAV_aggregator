use nalgebra::{Vector3, Matrix3, DMatrix};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::hash::{Hash, Hasher};

use crate::printLog;

/// Parsed OBJ file
pub struct Obj
{
    pub _vertices: Vec<Vector3<f32>>,
    pub _normals: Vec<Vector3<f32>>,
    pub faces: Vec<Face>
}

impl Obj
{
    /// Read obj from file
    pub fn from_file(file_path: &str)-> Self
    {
        Self::load_from_file(file_path, false)
    }

    /// Read obj from file. Parsing normals and faces may be disable by setting verticesOnly to true.
    pub fn load_from_file(file_path: &str, verticesOnly: bool) -> Self {
        let mut vertices = Vec::<Vector3<f32>>::new();
        let mut normals = Vec::<Vector3<f32>>::new();
        let mut faces = Vec::<Face>::new();

        let file = File::open(file_path);
        if file.is_err()
        {
            printLog!("Can not open file: {}. Assumed empty obj.", file_path);
            return Obj{_vertices: vertices, _normals: normals,faces};
        }
        let reader = BufReader::new(file.unwrap());

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
                    if verticesOnly
                    {
                        continue;
                    }
                    assert_eq!(elements.len(), 4);
                    let x: f32 = elements[1].parse().expect("Invalid normal coordinate");
                    let y: f32 = elements[2].parse().expect("Invalid normal coordinate");
                    let z: f32 = elements[3].parse().expect("Invalid normal coordinate");
                    normals.push(Vector3::new(x, y, z).normalize());
                }
                "f" => {
                    if verticesOnly
                    {
                        continue;
                    }

                    if elements.len() != 4
                    {
                        printLog!("Face {} is not triangle. Skipped", line);
                        continue;
                    }

                    let mut face_vertices = [Vector3::zeros();3];
                    let mut face_normals= [Vector3::zeros();3];

                    for (i, element) in elements[1..].iter().enumerate() {
                        let items: Vec<String> = element.split('/').map(|s| s.to_string()).collect();
                        face_vertices[i] = vertices[items[0].parse::<usize>().unwrap()-1];
                        if items.len() < 3 || items[2].is_empty()
                        {
                            printLog!("Face {} is invalid. Skipped", line);
                            continue;
                        }
                        face_normals[i] = normals[items[2].parse::<usize>().unwrap()-1];
                    }
                    faces.push(Face::new(faces.len(),face_vertices,face_normals));
                }
                _ => continue,
            }
        }

        Obj{_vertices: vertices, _normals: normals,faces} 
    }

    /// Finds the coordinates of minimal cuboid that contains all vertices
    pub fn boundingBox(&self) -> (Vector3<f32>, Vector3<f32>)
    {
        let mut min = Vector3::repeat(f32::MAX);
        let mut max = Vector3::repeat(f32::MIN);
        for v in &self._vertices 
        {
            min = min.inf(v);
            max = max.sup(v);
        }
        (min,max)
    }
    /// Get mesh. Mesh is matrix 3xN which columns are vertex's coordinates 
    pub fn getMesh(&self) -> DMatrix<f32>
    {
        let mut mesh = DMatrix::<f32>::zeros(3, self._vertices.len());
        for (col, vector) in self._vertices.iter().enumerate() {
            mesh.set_column(col, vector);
        }
        mesh
    }


}

#[derive(Clone)]
/// Parsed single face in OBJ file.
pub struct Face
{
    pub id:usize,
    pub _vertices: [Vector3<f32>;3],
    pub _normals: [Vector3<f32>;3],
    pub normal: Vector3<f32>,
    pub projectMatrix: Matrix3<f32>,
    pub s: Vector3<f32>,
    pub t: Vector3<f32>,
    pub base: Vector3<f32>
}

impl Face {

    /// Constructor
    pub fn new(id : usize , vertices: [Vector3<f32>;3],normals: [Vector3<f32>;3]) -> Self
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
        //printLog!("Inv: {}", invProjectMatrix);

        Face{id, _vertices: vertices, _normals: normals, normal: n,
                projectMatrix: invProjectMatrix.try_inverse().expect("Can not inverse project matrix"),
                s, t,
                base: vertices[0].clone()}
    }

    /// Projects point on face. Return true if projection is inside triangle. 
    /// If true second field in tuple is distance from face.
    pub fn projectPoint(&self, point: Vector3<f32>) -> (bool, f32)
    {

        let res = self.projectMatrix* (point-self.base);
        if res[0] >= 0.0 && res[0] <= 1.0 && res[1] >= 0.0 && res[1] <= 1.0 && res[0]+res[1] <= 1.0
        {
            return (true,res[2])
        } 
        (false, 0.0)
    }

    // Check if ray comes across triangle. Möller-Trumbore algorithm.
    pub fn rayIntersection (&self, point: Vector3<f32>, dir: Vector3<f32>) -> (bool, f32)
    {
        static EPS: f32 = 1e-3f32;

        let ray_t_cross = dir.cross(&self.t);
        let det = self.s.dot(&ray_t_cross);
        if det.abs() < EPS
        {
            return (false,0.0);
        }
        let invDet = 1.0f32/det;
        let P = point - self.base;
        let u = invDet * P.dot(&ray_t_cross);
        if u < 0.0 || u > 1.0
        {
            return (false,0.0);
        }
        let P_cross_s = P.cross(&self.s);
        let v = invDet * dir.dot(&P_cross_s);
        if v < 0.0 || u + v > 1.0
        {
            return (false,0.0);
        }
        let t = invDet * self.t.dot(&P_cross_s);
        if t > EPS
        {
            return (true,t);
        }
        (false,0.0)
    }
}

impl PartialEq for Face {
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id
    }
}
impl Eq for Face {}

impl Hash for Face {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.id.hash(state);
    }
}