use egui::{CentralPanel, ScrollArea};

pub const PADDING: f32 = 5.0;

/// We derive Deserialize/Serialize so we can persist app state on shutdown.
#[derive(serde::Deserialize, serde::Serialize)]
#[serde(default)] // if we add new fields, give them default values when deserializing old state
pub struct ParamTuner {
    parameters: Vec<Parameter>,
}

#[derive(serde::Deserialize, serde::Serialize)]
pub struct Parameter {
    name: String,
    value: i32,
}

impl Default for ParamTuner {
    fn default() -> Self {
        let dummy_parameters = (0..20).map(|i| Parameter {
            name: format!("parameter {}", i),
            value: i,
        });

        Self {
            parameters: Vec::from_iter(dummy_parameters),
        }
    }
}

impl ParamTuner {
    /// Called once before the first frame.
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // This is also where you can customize the look and feel of egui using
        // `cc.egui_ctx.set_visuals` and `cc.egui_ctx.set_fonts`.

        // Load previous app state (if any).
        // Note that you must enable the `persistence` feature for this to work.
        if let Some(storage) = cc.storage {
            return eframe::get_value(storage, eframe::APP_KEY).unwrap_or_default();
        }

        Default::default()
    }
}

impl eframe::App for ParamTuner {
    /// Called by the frame work to save state before shutdown.
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, self);
    }

    /// Called each time the UI needs repainting, which may be many times per second.
    /// Put your widgets into a `SidePanel`, `TopPanel`, `CentralPanel`, `Window` or `Area`.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        CentralPanel::default().show(ctx, |ui| {
            ScrollArea::vertical().show(ui, |ui| {
                for param in &mut self.parameters {
                    ui.add_space(PADDING);
                    if param.value < 0 {
                        let min = param.value - 5;
                        ui.add(egui::Slider::new(&mut param.value, min..=20).text(&param.name));
                    } else if param.value > 20 {
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
