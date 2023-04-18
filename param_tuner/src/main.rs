#![warn(clippy::all, rust_2018_idioms)]
use egui::{CentralPanel, ScrollArea};

// fn list_parameters(node: &mut rclrs::Node) -> Vec<String> {
//     let client = node
//         .create_client::<rcl_interfaces::srv::ListParameters>("/example_tunable/list_parameters");
//     let request = rcl_interfaces::srv::ListParameters_Request {};
//     let future = client.call_async(&request);
//     let rclrs_spin = tokio::task::spawn_blocking(move || rclrs::spin(&node));
//     let response = future.await?;
//     response.names.collect()
// }

struct ParamTunerApp {
    node: rclrs::Node,
    parameters: Vec<Parameter>,
}

struct Parameter {
    name: String,
    value: i32,
}

impl ParamTunerApp {
    fn new(_cc: &eframe::CreationContext<'_>) -> Self {
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
                    ui.add_space(5.0);
                    ui.add(egui::Slider::new(&mut param.value, 0..=20).text(&param.name));
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
