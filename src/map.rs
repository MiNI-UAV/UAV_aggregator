use nalgebra::Vector3;
use std::collections::{HashMap,HashSet};
use crate::obj::{Obj,Face};
use crate::printLog;


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

        printLog!("Min: {} Max: {}", min,max);
        printLog!("Chunk size: {}", step);

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
        //     printLog!("Chunk: {} - faces: {}", elem.0, elem.1.len());
        // }
    }

    pub fn getMinMax(&self) -> (Vector3<f32>,Vector3<f32>)
    {
        (self._min,self._max)
    }
}
