use core::ops;

pub const CUBE_SIZE: usize = 12;

#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct LedMatrix {
    pub cells: [[[Color; CUBE_SIZE]; CUBE_SIZE]; CUBE_SIZE],
}

#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

#[derive(Clone, Copy, Debug, Default, Eq, Ord, PartialEq, PartialOrd)]
pub struct Coord {
    pub x: usize,
    pub y: usize,
    pub z: usize,
}

impl LedMatrix {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn clear(&mut self) {
        self.cells = Default::default();
    }

    pub fn fill_all(&mut self, cell: Color) {
        self.cells = [[[cell; CUBE_SIZE]; CUBE_SIZE]; CUBE_SIZE];
    }

    pub fn fill_plane_x(&mut self, x: usize, cell: Color) {
        for z in 0..CUBE_SIZE {
            for y in 0..CUBE_SIZE {
                self.cells[z][y][x] = cell;
            }
        }
    }

    pub fn fill_plane_y(&mut self, y: usize, cell: Color) {
        for z in 0..CUBE_SIZE {
            self.cells[z][y] = [cell; CUBE_SIZE];
        }
    }

    pub fn fill_plane_z(&mut self, z: usize, cell: Color) {
        self.cells[z] = [[cell; CUBE_SIZE]; CUBE_SIZE];
    }

    pub fn xyz(&self, x: usize, y: usize, z: usize) -> &Color {
        &self.cells[z][y][x]
    }

    pub fn xyz_mut(&mut self, x: usize, y: usize, z: usize) -> &mut Color {
        &mut self.cells[z][y][x]
    }

    pub unsafe fn xyz_unchecked(&self, x: usize, y: usize, z: usize) -> Color {
        *self
            .cells
            .get_unchecked(z)
            .get_unchecked(y)
            .get_unchecked(x)
    }
}

impl ops::Index<Coord> for LedMatrix {
    type Output = Color;

    fn index(&self, index: Coord) -> &Self::Output {
        self.xyz(index.x, index.y, index.z)
    }
}

impl ops::IndexMut<Coord> for LedMatrix {
    fn index_mut(&mut self, index: Coord) -> &mut Self::Output {
        self.xyz_mut(index.x, index.y, index.z)
    }
}

impl Color {
    pub fn rgb(r: u8, g: u8, b: u8) -> Self {
        Self { r, g, b }
    }
}

impl Coord {
    pub fn xyz(x: usize, y: usize, z: usize) -> Self {
        assert!((x as usize) < CUBE_SIZE);
        assert!((y as usize) < CUBE_SIZE);
        assert!((z as usize) < CUBE_SIZE);
        Self { x, y, z }
    }
}
