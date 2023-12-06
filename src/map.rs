use nalgebra::Vector3;
use std::collections::{HashMap,HashSet};
use crate::obj::{Obj,Face};
use crate::printLog;

/// Simulation map
pub struct Map
{
    _walls: Obj,
    _min: Vector3<f32>,
    _max: Vector3<f32>,
    _step: Vector3<f32>,
    facesInChunk: HashMap<Vector3<usize>,HashSet<Face>>,

    pub collisionPlusEps: f32,
    pub collisionMinusEps: f32,
    pub COR: f32,
    pub mi_s: f32,
    pub mi_d: f32,
    pub minimalDist: f32
}

impl Map
{
    /// Constructor
    pub fn new(path: &str, collisionPlusEps: f32, collisionMinusEps: f32, grid: Vector3<f32>,
        COR: f32, mi_s: f32, mi_d: f32, minimalDist: f32) -> Self
    {
        let walls = Obj::from_file(path);
        let (min,max) = walls.boundingBox();
        let step = (max-min).component_div(&grid);

        printLog!("Min: {} Max: {}", min,max);
        printLog!("Chunk size: {}", step);

        let facesInChunk =  HashMap::<Vector3<usize>,HashSet<Face>>::new();

        let mut map = Map{_walls: walls, _min: min, _max: max, _step: step,
            facesInChunk, collisionPlusEps, collisionMinusEps,
            COR,
            mi_s,
            mi_d,
            minimalDist
        };
        map.insertFace();
        map
    }

    /// Checks if in specified point there is collision with map walls. 
    /// Returns colection of normal vectors of face that point colide with.
    /// If there is no collisions, return colection length is equal 0.
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

    /// Modified checkWalls, predict position in looptime
    pub fn checkWalls2(&self, start_point: Vector3<f32>,velocity: Vector3<f32>, dt: f32, radius: f32) -> Vec<Vector3<f32>>
    {
        let mut normalsInColisionPoint = Vec::new();

        for s in [0.0f32, 0.33f32, 0.66f32, 1.0f32]
        {
            let point = start_point + s * dt * velocity;
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
        }
        normalsInColisionPoint
    }

    /// Checks if in specified point there is collision with map walls. 
    /// Returns normal vector of wall that is the closest to point
    /// If there is no collisions, return None
    pub fn checkWallsBest(&self, point: Vector3<f32>) -> Option<(f32,Vector3<f32>)>
    {
        let mut bestNormal = Vector3::<f32>::zeros();
        let mut bestDepth = self.collisionPlusEps;
        let chunk = self.calcChunk(point);
        if let Some(faces) = self.facesInChunk.get(&chunk)
        {
            for face in faces {
                if let (true, dist) = face.projectPoint(point)
                {
                    if dist <= self.collisionPlusEps && dist >= self.collisionMinusEps
                    {
                        if dist < bestDepth
                        {
                            bestDepth = dist;
                            bestNormal = face.normal;
                        }
                    }
                }
            }
        }
        if bestDepth < self.collisionPlusEps
        {
            return Some((bestDepth,bestNormal));
        }
        None
    }

    /// Checks if in specified point there will collide with map walls in specificed dt. 
    /// Returns normal vector of wall that will be cross in next dt
    /// If there is no collisions, return None
    pub fn checkWallsBest2(&self, point: Vector3<f32>, velocity: Vector3<f32>, dt: f32) -> Option<(f32,Vector3<f32>)>
    {
        let inRange = velocity.norm() * dt;
        let dir = velocity.normalize();
        let mut bestNormal = Vector3::<f32>::zeros();
        let mut bestDist = inRange;

        let chunk = self.calcChunk(point);
        if let Some(faces) = self.facesInChunk.get(&chunk)
        {
            for face in faces {
                if let (true, dist) = face.rayIntersection(point,dir)
                {
                    if dist < bestDist
                    {
                        bestDist = dist;
                        bestNormal = face.normal;
                    }
                }
            }
        }
        if bestDist < inRange
        {
            return Some((bestDist,bestNormal));
        }
        None
    }

    /// Calculates chunk that for specified point
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

    /// Splits faces into chunks
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
    }

    /// Get minimal and maximal coordinate of cubiod that contains all map's vertices
    pub fn getMinMax(&self) -> (Vector3<f32>,Vector3<f32>)
    {
        (self._min,self._max)
    }
}
