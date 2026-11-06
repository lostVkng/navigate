//! Draw a Stanley simulation against four synthetic routes.
//!
//! Run with `cargo run --release --example route_follow_demo`.
//!
//! Writes `assets/route_follow_demo.svg` and `assets/route_follow_demo.png`.
//! The process exits with an error if a route is stale or the plant leaves the
//! tracking band. Wheelbase, steering limits, footprint, and the controller
//! gains are placeholders. The picture is not a stopping test.

use std::fs;
use std::io::Write;
use std::path::PathBuf;

use navigate::planning::synthetic::{self, RouteQuery};
use navigate::prelude::*;

struct Scene {
    title: &'static str,
    query: RouteQuery,
    path: PlannedPath,
    report: TrackingReport,
}

fn main() {
    let scenarios = [
        ("Open field", synthetic::open_field()),
        ("Narrow aisle", synthetic::narrow_aisle()),
        ("Reverse park", synthetic::parking_bay()),
        (
            "U-turn",
            synthetic::uturn_course(synthetic::EXAMPLE_MAX_STEERING_RAD),
        ),
    ];
    let config = FollowConfig::placeholder();
    println!(
        "{:<16} {:>10} {:>10} {:>10} {:>8} {:>8}",
        "route", "cross_m", "head_rad", "goal_m", "gears", "seconds"
    );
    let mut scenes = Vec::with_capacity(scenarios.len());
    for (title, query) in scenarios {
        let plan = plan_route(
            &query.map,
            &RouteRequest {
                start: query.start,
                goal: query.goal,
                model: &query.model,
                footprint: &query.footprint,
                config: &query.config,
                clearance: 0.0,
            },
        )
        .unwrap_or_else(|error| panic!("{title}: {error}"));
        let report = follow_route(&FollowRequest {
            plan: &plan,
            map: &query.map,
            model: &query.model,
            footprint: &query.footprint,
            clearance: 0.0,
            sweep_tolerance: query.config.sweep_tolerance,
            config: &config,
        })
        .unwrap_or_else(|error| panic!("{title} tracking failed: {error}"));
        println!(
            "{:<16} {:>10.4} {:>10.4} {:>10.4} {:>8} {:>8.2}",
            title,
            report.max_cross_track,
            report.max_heading_error,
            report.goal_position_error,
            report.direction_changes,
            report.duration
        );
        scenes.push(Scene {
            title,
            query,
            path: plan.path().clone(),
            report,
        });
    }

    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let svg_path = root.join("assets/route_follow_demo.svg");
    let png_path = root.join("assets/route_follow_demo.png");
    fs::write(&svg_path, render_svg(&scenes)).expect("write svg");
    write_png(&png_path, &render_png(&scenes)).expect("write png");
    println!("wrote {}", svg_path.display());
    println!("wrote {}", png_path.display());
}

fn render_svg(scenes: &[Scene]) -> String {
    let panel_w = 560.0;
    let panel_h = 460.0;
    let margin = 28.0;
    let gap = 18.0;
    let header = 96.0;
    let width = margin * 2.0 + panel_w * 2.0 + gap;
    let height = margin + header + panel_h * 2.0 + gap + margin;
    let mut svg = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">"##
    );
    svg.push_str(&format!(
        r##"<rect width="{width}" height="{height}" fill="#121820"/>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{margin}" y="36" fill="#f2f4f8" font-family="ui-sans-serif, sans-serif" font-size="20">Offline route following</text>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{margin}" y="58" fill="#b7c0cc" font-family="ui-sans-serif, sans-serif" font-size="13">Stanley tracker on the placeholder bicycle. Gains, delay, and limits are not measurements. A clear run is not a physical stop.</text>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{margin}" y="82" fill="#e2b15a" font-family="ui-sans-serif, sans-serif" font-size="13">Planned forward</text>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{}" y="82" fill="#3ec4d6" font-family="ui-sans-serif, sans-serif" font-size="13">Planned reverse</text>"##,
        margin + 140.0
    ));
    svg.push_str(&format!(
        r##"<text x="{}" y="82" fill="#f4f7fb" font-family="ui-sans-serif, sans-serif" font-size="13">Simulated</text>"##,
        margin + 280.0
    ));
    for (index, scene) in scenes.iter().enumerate() {
        let col = (index % 2) as f64;
        let row = (index / 2) as f64;
        let x = margin + col * (panel_w + gap);
        let y = margin + header + row * (panel_h + gap);
        draw_svg_panel(&mut svg, scene, x, y, panel_w, panel_h);
    }
    svg.push_str("</svg>\n");
    svg
}

fn draw_svg_panel(svg: &mut String, scene: &Scene, x: f64, y: f64, w: f64, h: f64) {
    svg.push_str(&format!(
        r##"<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#1b2430" stroke="#2c3a4d"/>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{}" y="{}" fill="#f2f4f8" font-family="ui-sans-serif, sans-serif" font-size="16">{}</text>"##,
        x + 14.0,
        y + 24.0,
        scene.title
    ));
    svg.push_str(&format!(
        r##"<text x="{}" y="{}" fill="#9aa6b5" font-family="ui-sans-serif, sans-serif" font-size="12">cross {:.3} m, heading {:.3} rad, goal {:.3} m / {:.3} rad, {} gear changes</text>"##,
        x + 14.0,
        y + 42.0,
        scene.report.max_cross_track,
        scene.report.max_heading_error,
        scene.report.goal_position_error,
        scene.report.goal_heading_error,
        scene.report.direction_changes
    ));
    let map_h = h - 148.0;
    let frame = Frame::new(&scene.query, x + 12.0, y + 54.0, w - 24.0, map_h);
    svg.push_str(&format!(
        r##"<rect x="{:.2}" y="{:.2}" width="{:.2}" height="{:.2}" fill="#243044" stroke="#314155"/>"##,
        frame.x, frame.y, frame.width, frame.height
    ));
    draw_svg_grid(svg, &scene.query, &frame);
    draw_svg_plan(svg, scene, &frame);
    draw_svg_sim(svg, scene, &frame);
    draw_svg_marker(svg, scene.query.start, "#8dce7a", &frame);
    draw_svg_marker(svg, scene.query.goal, "#d58ad4", &frame);
    draw_svg_error(svg, scene, x + 12.0, y + h - 78.0, w - 24.0, 62.0);
}

fn draw_svg_grid(svg: &mut String, query: &RouteQuery, frame: &Frame) {
    let resolution = query.map.resolution();
    let cell = (resolution * frame.scale).max(0.4);
    for iy in 0..query.map.height() {
        for ix in 0..query.map.width() {
            if query.map.occupancy(ix, iy) == Some(Occupancy::Free) {
                continue;
            }
            let (cx, cy) = query.map.cell_center(ix, iy);
            let (px, py) = frame.project(cx - resolution * 0.5, cy + resolution * 0.5);
            svg.push_str(&format!(
                r##"<rect x="{px:.2}" y="{py:.2}" width="{cell:.2}" height="{cell:.2}" fill="#7d4542"/>"##
            ));
        }
    }
}

fn draw_svg_plan(svg: &mut String, scene: &Scene, frame: &Frame) {
    for index in 1..scene.path.poses.len() {
        let from = scene.path.poses[index - 1];
        let to = scene.path.poses[index];
        let (x0, y0) = frame.project(from.x, from.y);
        let (x1, y1) = frame.project(to.x, to.y);
        let color = if scene.path.segment_length[index] < 0.0 {
            "#3ec4d6"
        } else {
            "#e2b15a"
        };
        svg.push_str(&format!(
            r##"<line x1="{x0:.2}" y1="{y0:.2}" x2="{x1:.2}" y2="{y1:.2}" stroke="{color}" stroke-width="2.5" stroke-linecap="round"/>"##
        ));
    }
}

fn draw_svg_sim(svg: &mut String, scene: &Scene, frame: &Frame) {
    let mut points = String::new();
    for sample in &scene.report.samples {
        let (x, y) = frame.project(sample.pose.x, sample.pose.y);
        points.push_str(&format!("{x:.2},{y:.2} "));
    }
    svg.push_str(&format!(
        r##"<polyline points="{points}" fill="none" stroke="#f4f7fb" stroke-width="1.6" stroke-linejoin="round"/>"##
    ));
}

fn draw_svg_marker(svg: &mut String, pose: Pose2d, color: &str, frame: &Frame) {
    let (px, py) = frame.project(pose.x, pose.y);
    let (hx, hy) = frame.project(
        pose.x + 0.22 * pose.theta.cos(),
        pose.y + 0.22 * pose.theta.sin(),
    );
    svg.push_str(&format!(
        r##"<circle cx="{px:.2}" cy="{py:.2}" r="3.5" fill="{color}"/>"##
    ));
    svg.push_str(&format!(
        r##"<line x1="{px:.2}" y1="{py:.2}" x2="{hx:.2}" y2="{hy:.2}" stroke="{color}" stroke-width="1.5"/>"##
    ));
}

fn draw_svg_error(svg: &mut String, scene: &Scene, x: f64, y: f64, w: f64, h: f64) {
    svg.push_str(&format!(
        r##"<rect x="{x:.2}" y="{y:.2}" width="{w:.2}" height="{h:.2}" fill="#18202b" stroke="#314155"/>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{}" y="{}" fill="#9aa6b5" font-family="ui-sans-serif, sans-serif" font-size="11">Cross-track, meters</text>"##,
        x + 8.0,
        y + 14.0
    ));
    let plot_x = x + 8.0;
    let plot_y = y + 20.0;
    let plot_w = w - 16.0;
    let plot_h = h - 28.0;
    let mid = plot_y + plot_h * 0.5;
    svg.push_str(&format!(
        r##"<line x1="{plot_x:.2}" y1="{mid:.2}" x2="{:.2}" y2="{mid:.2}" stroke="#3a4a5e" stroke-width="1"/>"##,
        plot_x + plot_w
    ));
    let duration = scene.report.duration.max(1e-6);
    let scale = scene
        .report
        .samples
        .iter()
        .map(|sample| sample.cross_track.abs())
        .fold(0.02_f64, f64::max)
        * 1.15;
    let mut points = String::new();
    for sample in &scene.report.samples {
        let px = plot_x + plot_w * (sample.time / duration);
        let py = mid - plot_h * 0.5 * (sample.cross_track / scale);
        points.push_str(&format!("{px:.2},{py:.2} "));
    }
    svg.push_str(&format!(
        r##"<polyline points="{points}" fill="none" stroke="#e07a7a" stroke-width="1.4"/>"##
    ));
}

struct Frame {
    x: f64,
    y: f64,
    width: f64,
    height: f64,
    origin_x: f64,
    origin_y: f64,
    scale: f64,
}

impl Frame {
    fn new(query: &RouteQuery, x: f64, y: f64, width: f64, height: f64) -> Self {
        let (origin_x, origin_y) = query.map.origin();
        let world_w = query.map.width() as f64 * query.map.resolution();
        let world_h = query.map.height() as f64 * query.map.resolution();
        let scale = (width / world_w).min(height / world_h);
        let used_w = world_w * scale;
        let used_h = world_h * scale;
        Self {
            x: x + (width - used_w) * 0.5,
            y: y + (height - used_h) * 0.5,
            width: used_w,
            height: used_h,
            origin_x,
            origin_y,
            scale,
        }
    }

    fn project(&self, wx: f64, wy: f64) -> (f64, f64) {
        let px = self.x + (wx - self.origin_x) * self.scale;
        let py = self.y + self.height - (wy - self.origin_y) * self.scale;
        (px, py)
    }
}

fn render_png(scenes: &[Scene]) -> Raster {
    let width = 1200;
    let height = 1100;
    let mut image = Raster::new(width, height, [18, 24, 32]);
    image.text(28, 16, "OFFLINE ROUTE FOLLOWING", [242, 244, 248], 2);
    image.text(
        28,
        40,
        "PLACEHOLDER GAINS. NOT A PHYSICAL STOP.",
        [183, 192, 204],
        1,
    );
    image.text(
        28,
        58,
        "GOLD FORWARD  CYAN REVERSE  WHITE SIMULATED",
        [183, 192, 204],
        1,
    );
    for (index, scene) in scenes.iter().enumerate() {
        let col = index % 2;
        let row = index / 2;
        let x = 28 + col * (570 + 16);
        let y = 84 + row * (490 + 16);
        draw_png_panel(&mut image, scene, x as i32, y as i32, 570, 490);
    }
    image
}

fn draw_png_panel(image: &mut Raster, scene: &Scene, x: i32, y: i32, w: i32, h: i32) {
    image.fill_rect(x, y, w, h, [27, 36, 48]);
    image.text(
        x + 14,
        y + 12,
        &scene.title.to_ascii_uppercase(),
        [242, 244, 248],
        2,
    );
    let caption = format!(
        "CROSS {:.3} M  HEAD {:.3} RAD  GOAL {:.3} M",
        scene.report.max_cross_track,
        scene.report.max_heading_error,
        scene.report.goal_position_error
    );
    image.text(x + 14, y + 34, &caption, [154, 166, 181], 1);
    let frame = Frame::new(
        &scene.query,
        (x + 12) as f64,
        (y + 52) as f64,
        (w - 24) as f64,
        (h - 150) as f64,
    );
    image.fill_rect(
        frame.x.round() as i32,
        frame.y.round() as i32,
        frame.width.round() as i32,
        frame.height.round() as i32,
        [36, 48, 68],
    );
    let resolution = scene.query.map.resolution();
    let cell = (resolution * frame.scale).round().max(1.0) as i32;
    for iy in 0..scene.query.map.height() {
        for ix in 0..scene.query.map.width() {
            if scene.query.map.occupancy(ix, iy) == Some(Occupancy::Free) {
                continue;
            }
            let (cx, cy) = scene.query.map.cell_center(ix, iy);
            let (px, py) = frame.project(cx - resolution * 0.5, cy + resolution * 0.5);
            image.fill_rect(
                px.round() as i32,
                py.round() as i32,
                cell + 1,
                cell + 1,
                [125, 69, 66],
            );
        }
    }
    for index in 1..scene.path.poses.len() {
        let (x0, y0) = frame.project(scene.path.poses[index - 1].x, scene.path.poses[index - 1].y);
        let (x1, y1) = frame.project(scene.path.poses[index].x, scene.path.poses[index].y);
        let color = if scene.path.segment_length[index] < 0.0 {
            [62, 196, 214]
        } else {
            [226, 177, 90]
        };
        image.line(
            x0.round() as i32,
            y0.round() as i32,
            x1.round() as i32,
            y1.round() as i32,
            color,
            1,
        );
    }
    let mut previous: Option<(i32, i32)> = None;
    for sample in &scene.report.samples {
        let (px, py) = frame.project(sample.pose.x, sample.pose.y);
        let point = (px.round() as i32, py.round() as i32);
        if let Some(prev) = previous {
            image.line(prev.0, prev.1, point.0, point.1, [244, 247, 251], 0);
        }
        previous = Some(point);
    }
    draw_png_marker(image, scene.query.start, [141, 206, 122], &frame);
    draw_png_marker(image, scene.query.goal, [213, 138, 212], &frame);
    draw_png_error(image, scene, x + 12, y + h - 84, w - 24, 70);
}

fn draw_png_error(image: &mut Raster, scene: &Scene, x: i32, y: i32, w: i32, h: i32) {
    image.fill_rect(x, y, w, h, [24, 32, 43]);
    image.text(x + 8, y + 6, "CROSS-TRACK M", [154, 166, 181], 1);
    let plot_x = x + 8;
    let plot_y = y + 20;
    let plot_w = w - 16;
    let plot_h = h - 28;
    let mid = plot_y + plot_h / 2;
    image.line(plot_x, mid, plot_x + plot_w, mid, [58, 74, 94], 0);
    let duration = scene.report.duration.max(1e-6);
    let scale = scene
        .report
        .samples
        .iter()
        .map(|sample| sample.cross_track.abs())
        .fold(0.02_f64, f64::max)
        * 1.15;
    let mut previous: Option<(i32, i32)> = None;
    for sample in &scene.report.samples {
        let px = plot_x as f64 + plot_w as f64 * (sample.time / duration);
        let py = mid as f64 - plot_h as f64 * 0.5 * (sample.cross_track / scale);
        let point = (px.round() as i32, py.round() as i32);
        if let Some(prev) = previous {
            image.line(prev.0, prev.1, point.0, point.1, [224, 122, 122], 0);
        }
        previous = Some(point);
    }
}

fn draw_png_marker(image: &mut Raster, pose: Pose2d, color: [u8; 3], frame: &Frame) {
    let (px, py) = frame.project(pose.x, pose.y);
    let (hx, hy) = frame.project(
        pose.x + 0.22 * pose.theta.cos(),
        pose.y + 0.22 * pose.theta.sin(),
    );
    image.line(
        px.round() as i32,
        py.round() as i32,
        hx.round() as i32,
        hy.round() as i32,
        color,
        1,
    );
    let cx = px.round() as i32;
    let cy = py.round() as i32;
    for oy in -3..=3 {
        for ox in -3..=3 {
            if ox * ox + oy * oy <= 9 {
                image.set(cx + ox, cy + oy, color);
            }
        }
    }
}

struct Raster {
    width: usize,
    height: usize,
    rgb: Vec<u8>,
}

impl Raster {
    fn new(width: usize, height: usize, color: [u8; 3]) -> Self {
        let mut rgb = vec![0; width * height * 3];
        for pixel in rgb.chunks_exact_mut(3) {
            pixel.copy_from_slice(&color);
        }
        Self { width, height, rgb }
    }

    fn set(&mut self, x: i32, y: i32, color: [u8; 3]) {
        if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
            return;
        }
        let index = (y as usize * self.width + x as usize) * 3;
        self.rgb[index..index + 3].copy_from_slice(&color);
    }

    fn fill_rect(&mut self, x: i32, y: i32, w: i32, h: i32, color: [u8; 3]) {
        for py in y..(y + h).max(y) {
            for px in x..(x + w).max(x) {
                self.set(px, py, color);
            }
        }
    }

    fn line(&mut self, x0: i32, y0: i32, x1: i32, y1: i32, color: [u8; 3], radius: i32) {
        let dx = (x1 - x0).abs();
        let dy = (y1 - y0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx - dy;
        let mut x = x0;
        let mut y = y0;
        loop {
            for oy in -radius..=radius {
                for ox in -radius..=radius {
                    if ox * ox + oy * oy <= radius * radius + radius {
                        self.set(x + ox, y + oy, color);
                    }
                }
            }
            if x == x1 && y == y1 {
                break;
            }
            let e2 = 2 * err;
            if e2 > -dy {
                err -= dy;
                x += sx;
            }
            if e2 < dx {
                err += dx;
                y += sy;
            }
        }
    }

    fn text(&mut self, x: i32, y: i32, text: &str, color: [u8; 3], scale: i32) {
        let mut cursor = x;
        for ch in text.chars() {
            let glyph = glyph(ch);
            for (row, bits) in glyph.iter().enumerate() {
                for col in 0..5 {
                    if bits & (1 << (4 - col)) != 0 {
                        self.fill_rect(
                            cursor + col * scale,
                            y + i32::try_from(row).unwrap_or(0) * scale,
                            scale,
                            scale,
                            color,
                        );
                    }
                }
            }
            cursor += 6 * scale;
        }
    }
}

fn glyph(ch: char) -> [u8; 7] {
    match ch {
        'A' => [0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'B' => [0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E],
        'C' => [0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E],
        'D' => [0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E],
        'E' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F],
        'F' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10],
        'G' => [0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F],
        'H' => [0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'I' => [0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E],
        'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F],
        'M' => [0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11],
        'N' => [0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11],
        'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'P' => [0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10],
        'R' => [0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11],
        'S' => [0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E],
        'T' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04],
        'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11],
        'X' => [0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11],
        'Y' => [0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04],
        '0' => [0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E],
        '1' => [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
        '2' => [0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F],
        '3' => [0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E],
        '4' => [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
        '5' => [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
        '6' => [0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E],
        '7' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
        '9' => [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C],
        '.' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x06],
        '-' => [0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00],
        ' ' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
        _ => [0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F],
    }
}

fn write_png(path: &std::path::Path, image: &Raster) -> std::io::Result<()> {
    let mut raw = Vec::with_capacity(image.height * (1 + image.width * 3));
    for row in image.rgb.chunks_exact(image.width * 3) {
        raw.push(0);
        raw.extend_from_slice(row);
    }
    let compressed = zlib_store(&raw);
    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&(image.width as u32).to_be_bytes());
    ihdr.extend_from_slice(&(image.height as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    let mut file = Vec::new();
    file.extend_from_slice(&[137, 80, 78, 71, 13, 10, 26, 10]);
    file.extend(chunk(b"IHDR", &ihdr));
    file.extend(chunk(b"IDAT", &compressed));
    file.extend(chunk(b"IEND", &[]));
    let mut out = fs::File::create(path)?;
    out.write_all(&file)
}

fn chunk(kind: &[u8; 4], data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(12 + data.len());
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let mut crc_input = Vec::with_capacity(4 + data.len());
    crc_input.extend_from_slice(kind);
    crc_input.extend_from_slice(data);
    out.extend_from_slice(&crc32(&crc_input).to_be_bytes());
    out
}

fn zlib_store(data: &[u8]) -> Vec<u8> {
    let mut out = vec![0x78, 0x01];
    let mut offset = 0;
    while offset < data.len() || data.is_empty() {
        let take = (data.len() - offset).min(65_535);
        let last = offset + take == data.len();
        out.push(if last { 0x01 } else { 0x00 });
        out.extend_from_slice(&(take as u16).to_le_bytes());
        out.extend_from_slice(&(!(take as u16)).to_le_bytes());
        out.extend_from_slice(&data[offset..offset + take]);
        offset += take;
        if last {
            break;
        }
    }
    out.extend_from_slice(&adler32(data).to_be_bytes());
    out
}

fn adler32(data: &[u8]) -> u32 {
    let mut a = 1_u32;
    let mut b = 0_u32;
    for byte in data {
        a = (a + u32::from(*byte)) % 65_521;
        b = (b + a) % 65_521;
    }
    (b << 16) | a
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff_u32;
    for byte in data {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}
