#![warn(clippy::all, rust_2018_idioms)]
use egui::{CentralPanel, ScrollArea};

pub const PADDING: f32 = 5.0;

pub struct ParamTunerApp {
    node: rclrs::Node,
    parameters: Vec<Parameter>,
}

pub struct Parameter {
    name: String,
    value: i32,
}

impl ParamTunerApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let dummy_parameters = (1..20).map(|i| Parameter {
            name: format!("parameter {}", i),
            value: i,
        });

        let context = rclrs::Context::new(std::env::args()).unwrap();
        let node = rclrs::Node::new(&context, "param_tuner").unwrap();

        Self {
            node,
            parameters: Vec::from_iter(dummy_parameters),
        }
    }
}

impl eframe::App for ParamTunerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        CentralPanel::default().show(ctx, |ui| {
            ScrollArea::vertical().show(ui, |ui| {
                for param in &mut self.parameters {
                    ui.add_space(PADDING);
                    if param.value <= 0 {
                        let min = param.value - 5;
                        ui.add(egui::Slider::new(&mut param.value, min..=20).text(&param.name));
                    } else if param.value >= 20 {
                        let max = param.value + 5;
                        ui.add(egui::Slider::new(&mut param.value, 0..=max).text(&param.name));
                    } else {
                        ui.add(egui::Slider::new(&mut param.value, 0..=20).text(&param.name));
                    }
                }
            });
        });
    }
}

fn main() -> eframe::Result<()> {
    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "param tuner",
        native_options,
        Box::new(|cc| Box::new(ParamTunerApp::new(cc))),
    )
}
