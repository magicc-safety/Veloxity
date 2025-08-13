#[derive(Default)]
pub struct Messages {
    pub param_set: Option<SmallBaroMsg>,
}

#[derive(Default)]
pub struct SmallBaroMsg {
    pub altitude: f32,
    pub pressure: f32,
    pub temperature: f32,
}
