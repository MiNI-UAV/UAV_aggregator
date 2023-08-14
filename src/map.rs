use nalgebra::{Vector3, Matrix3};
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::collections::{HashMap,HashSet};
use std::hash::{Hash, Hasher};

pub struct Map
{
    _walls: Obj,
    _min: Vector3<f32>,
    _max: Vector3<f32>,
    _step: Vector3<f32>,
    facesInChunk: HashMap<Vector3<usize>,HashSet<Face>>,

    pub collisionPlusEps: f32,
    pub collisionMinusEps: f32,
    pub sphereRadius: f32,
    pub projectileRadius: f32,
    pub COR: f32,
    pub mi_s: f32,
    pub mi_d: f32,
    pub minimalDist: f32
}

impl Map
{
    pub fn new(path: &str, collisionPlusEps: f32, collisionMinusEps: f32, grid: Vector3<f32>,
        sphereRadius: f32, projectileRadius: f32, COR: f32, mi_s: f32, mi_d: f32, minimalDist: f32) -> Self
    {
        let walls = Obj::from_file(path);
        let (min,max) = walls.boundingBox();
        let step = (max-min).component_div(&grid);

        println!("Min: {} Max: {}", min,max);
        println!("Chunk size: {}", step);

        let facesInChunk =  HashMap::<Vector3<usize>,HashSet<Face>>::new();

        let mut map = Map{_walls: walls, _min: min, _max: max, _step: step,
            facesInChunk, collisionPlusEps, collisionMinusEps,
            sphereRadius,
            projectileRadius,
            COR,
            mi_s,
            mi_d,
            minimalDist
        };
        map.insertFace();
        map
    }

    pub fn checkWalls(&self, point: Vector3<f32>, radius: f32) -> Vec<Vector3<f32>>
    {
        let mut normalsInColisionPoint = Vec::new();
        let chunk = self.calcChunk(point);
        if let Some(faces) = self.facesInChunk.get(&chunk)
        {
            for face in faces {
                if let (true, mut dist) = face.projectPoint(point)
                {
                    dist -= radius;
                    if dist <= self.collisionPlusEps && dist >= self.collisionMinusEps
                    {
                        normalsInColisionPoint.push(face.normal)
                    }
                }
            }
        }
        normalsInColisionPoint
    }

    fn calcChunk(&self, point: Vector3<f32>) -> Vector3<usize>
    {
        let pos =  (point -  self._min).component_div(&self._step);
        let chunk = Vector3::new(
            pos.x.floor() as usize,
            pos.y.floor() as usize,
            pos.z.floor() as usize,
        );
        chunk
    }

    fn insertFace(&mut self)
    {
        for face in &self._walls.faces
        {
            let chunks  = face._vertices.map(|p| self.calcChunk(p));
            let mut minChunks = chunks[0];
            let mut maxChunks = chunks[0];
            for chunk in chunks
            {
                minChunks = minChunks.inf(&chunk);
                maxChunks = maxChunks.sup(&chunk);
            }

            for i in minChunks[0]..=maxChunks[0]
            {
                for j in minChunks[1]..=maxChunks[1]
                {
                    for k in minChunks[2]..=maxChunks[2]
                    {
                        let chunk = Vector3::new(i,j,k);
                        if let Some(v) = self.facesInChunk.get_mut(&chunk)
                        {
                            v.insert(face.clone());
                        }
                        else
                        {
                            let mut set = HashSet::new();
                            set.insert(face.clone());
                            self.facesInChunk.insert(chunk, set);
                        }
                    }
                }
            }
        }

        // for elem in &self.facesInChunk
        // {
        //     println!("Chunk: {} - faces: {}", elem.0, elem.1.len());
        // }
    }

    pub fn getMinMax(&self) -> (Vector3<f32>,Vector3<f32>)
    {
        (self._min,self._max)
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
                    faces.push(Face::new(faces.len(),face_vertices,face_normals));
                }
                _ => continue,
            }
        }

        Obj{_vertices: vertices, _normals: normals,faces} 
    }

    fn boundingBox(&self) -> (Vector3<f32>, Vector3<f32>)
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


}

#[derive(Clone)]
struct Face
{
    id:usize,
    _vertices: [Vector3<f32>;3],
    _normals: [Vector3<f32>;3],
    normal: Vector3<f32>,
    projectMatrix: Matrix3<f32>,
    base: Vector3<f32>
}

impl Face {

    fn new(id : usize , vertices: [Vector3<f32>;3],normals: [Vector3<f32>;3]) -> Self
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

        Face{id, _vertices: vertices, _normals: normals, normal: n,
                projectMatrix: invProjectMatrix.try_inverse().expect("Can not inverse project matrix"),
                base: vertices[0].clone()}
    }

    fn projectPoint(&self, point: Vector3<f32>) -> (bool, f32)
    {

        let res = self.projectMatrix* (point-self.base);
        if res[0] >= 0.0 && res[0] <= 1.0 && res[1] >= 0.0 && res[1] <= 1.0 && res[0]+res[1] <= 1.0
        {
            // if res[2].abs() < 0.1
            // {
            //     println!("Point: {}", point);
            //     println!("Face vertex: {:?}", self._vertices);
            //     println!("Face normal: {:?}", self.normal);
            //     println!("Res: {}", res);            
            // }
            return (true,res[2])
        } 
        (false, 0.0)
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