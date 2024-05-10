const Q: f32 = 0.1; // шум процесса (чем ближе к 0, тем плавнее)
const R: f32 = 0.1; // шум измерения (чем больше, тем плавнее)

#[derive(Clone)]
pub struct Filter {
    x: f32,
    p: f32,
}

impl Filter {
    pub fn new() -> Self {
        Self {
            x: f32::default(),
            p: f32::default(),
        }
    }

    pub fn filter(&mut self, val: f32) -> f32 {
        self.p += Q;
        let k = self.p / (self.p + R);
        self.x = self.x + k * (val - self.x);
        self.p *= 1.0 - k;
        self.x
    }
}
