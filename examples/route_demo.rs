//! Draw and time four synthetic hybrid-A* routes.
//!
//! Run with `cargo run --release --example route_demo`.
//!
//! Writes `assets/route_demo.svg` and `assets/route_demo.png`. Both are
//! pictures of the same routes the integration tests check. The process exits
//! with an error if a route fails the swept-footprint feasibility check.
//! Wheelbase, steering limit, and footprint are placeholders, not measurements.

use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

use navigate::planning::synthetic::{self, RouteQuery};
use navigate::prelude::*;

struct Solved {
    title: &'static str,
    query: RouteQuery,
    path: PlannedPath,
    elapsed_ms: f64,
}

fn main() {
    let scenarios = [
        ("Open field", synthetic::open_field()),
        ("Narrow aisle", synthetic::narrow_aisle()),
        ("Obstacle bypass", synthetic::obstacle_bypass()),
        ("Reverse park", synthetic::parking_bay()),
    ];
    let mut solved = Vec::with_capacity(scenarios.len());
    println!(
        "{:<18} {:>8} {:>8} {:>8} {:>10} {:>8}",
        "route", "length_m", "cost", "expand", "reverse_m", "ms"
    );
    for (title, query) in scenarios {
        let started = Instant::now();
        let path = query
            .plan()
            .unwrap_or_else(|error| panic!("{title}: {error}"));
        let elapsed_ms = started.elapsed().as_secs_f64() * 1_000.0;
        query
            .check(&path)
            .unwrap_or_else(|error| panic!("{title} failed the feasibility check: {error}"));
        let reverse: f64 = path
            .segment_length
            .iter()
            .filter(|length| **length < 0.0)
            .map(|length| length.abs())
            .sum::<f64>()
            .abs();
        println!(
            "{:<18} {:>8.3} {:>8.3} {:>8} {:>10.3} {:>8.2}",
            title, path.length, path.cost, path.expansions, reverse, elapsed_ms
        );
        solved.push(Solved {
            title,
            query,
            path,
            elapsed_ms,
        });
    }

    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let svg_path = root.join("assets/route_demo.svg");
    let png_path = root.join("assets/route_demo.png");
    fs::write(&svg_path, render_svg(&solved)).expect("write svg");
    write_png(&png_path, &render_png(&solved)).expect("write png");
    println!("wrote {}", svg_path.display());
    println!("wrote {}", png_path.display());
}

fn render_svg(solved: &[Solved]) -> String {
    let panel_w = 560.0;
    let panel_h = 420.0;
    let margin = 28.0;
    let gap = 18.0;
    let header = 78.0;
    let width = margin * 2.0 + panel_w * 2.0 + gap;
    let height = margin + header + panel_h * 2.0 + gap + margin;
    let mut svg = String::new();
    svg.push_str(&format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">"##
    ));
    svg.push_str(&format!(
        r##"<rect width="{width}" height="{height}" fill="#121820"/>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{margin}" y="36" fill="#f2f4f8" font-family="ui-sans-serif, sans-serif" font-size="20">Synthetic steering-limited routes</text>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{margin}" y="58" fill="#b7c0cc" font-family="ui-sans-serif, sans-serif" font-size="13">Placeholder vehicle, not a calibrated measurement. Each route was integrated and swept for collisions before this picture was written.</text>"##
    ));
    svg.push_str(&format!(
        r##"<text x="{margin}" y="76" fill="#8fd0c8" font-family="ui-sans-serif, sans-serif" font-size="13">Forward</text>"##
    ));
    svg.push_str(&format!(
        r##"<line x1="{}" y1="72" x2="{}" y2="72" stroke="#e2b15a" stroke-width="3"/>"##,
        margin + 62.0,
        margin + 110.0
    ));
    svg.push_str(&format!(
        r##"<text x="{}" y="76" fill="#8fd0c8" font-family="ui-sans-serif, sans-serif" font-size="13">Reverse</text>"##,
        margin + 124.0
    ));
    svg.push_str(&format!(
        r##"<line x1="{}" y1="72" x2="{}" y2="72" stroke="#3ec4d6" stroke-width="3"/>"##,
        margin + 186.0,
        margin + 234.0
    ));
    for (index, scene) in solved.iter().enumerate() {
        let col = (index % 2) as f64;
        let row = (index / 2) as f64;
        let x = margin + col * (panel_w + gap);
        let y = margin + header + row * (panel_h + gap);
        draw_svg_panel(&mut svg, scene, x, y, panel_w, panel_h);
    }
    svg.push_str("</svg>\n");
    svg
}

fn draw_svg_panel(svg: &mut String, scene: &Solved, x: f64, y: f64, w: f64, h: f64) {
    svg.push_str(&format!(
        r##"<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#1b2430" stroke="#2c3a4d"/>"##
    ));
    let radius = scene.query.model.min_turning_radius();
    svg.push_str(&format!(
        r##"<text x="{}" y="{}" fill="#f2f4f8" font-family="ui-sans-serif, sans-serif" font-size="16">{}</text>"##,
        x + 14.0,
        y + 24.0,
        scene.title
    ));
    svg.push_str(&format!(
        r##"<text x="{}" y="{}" fill="#9aa6b5" font-family="ui-sans-serif, sans-serif" font-size="12">{:.2} m, {} expansions, min radius {:.2} m, {:.2} ms</text>"##,
        x + 14.0,
        y + 42.0,
        scene.path.length,
        scene.path.expansions,
        radius,
        scene.elapsed_ms
    ));
    let frame = Frame::new(&scene.query, x + 12.0, y + 54.0, w - 24.0, h - 68.0);
    svg.push_str(&format!(
        r##"<rect x="{:.2}" y="{:.2}" width="{:.2}" height="{:.2}" fill="#243044" stroke="#314155"/>"##,
        frame.x, frame.y, frame.width, frame.height
    ));
    draw_svg_grid(svg, &scene.query, &frame);
    draw_svg_footprints(svg, scene, &frame);
    draw_svg_path(svg, scene, &frame);
    draw_svg_pose_marker(svg, scene.query.start, "#8dce7a", &frame);
    draw_svg_pose_marker(svg, scene.query.goal, "#d58ad4", &frame);
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

fn draw_svg_grid(svg: &mut String, query: &RouteQuery, frame: &Frame) {
    let resolution = query.map.resolution();
    let cell = resolution * frame.scale;
    for iy in 0..query.map.height() {
        for ix in 0..query.map.width() {
            let occupied = query.map.occupancy(ix, iy) != Some(Occupancy::Free);
            if !occupied {
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

fn draw_svg_path(svg: &mut String, scene: &Solved, frame: &Frame) {
    let mut forward = String::new();
    let mut reverse = String::new();
    for index in 1..scene.path.poses.len() {
        let from = scene.path.poses[index - 1];
        let to = scene.path.poses[index];
        let (x0, y0) = frame.project(from.x, from.y);
        let (x1, y1) = frame.project(to.x, to.y);
        let piece = format!("M{x0:.2},{y0:.2} L{x1:.2},{y1:.2} ");
        if scene.path.segment_length[index] < 0.0 {
            reverse.push_str(&piece);
        } else {
            forward.push_str(&piece);
        }
    }
    svg.push_str(&format!(
        r##"<path d="{forward}" fill="none" stroke="#e2b15a" stroke-width="2.4" stroke-linecap="round"/>"##
    ));
    svg.push_str(&format!(
        r##"<path d="{reverse}" fill="none" stroke="#3ec4d6" stroke-width="2.4" stroke-linecap="round"/>"##
    ));
}

fn draw_svg_footprints(svg: &mut String, scene: &Solved, frame: &Frame) {
    let step = (scene.path.poses.len() / 14).max(1);
    for (index, pose) in scene.path.poses.iter().enumerate() {
        if index % step != 0 && index + 1 != scene.path.poses.len() {
            continue;
        }
        let Some(corners) = scene.query.footprint.rectangle_corners(*pose) else {
            continue;
        };
        let mut points = String::new();
        for corner in corners {
            let (px, py) = frame.project(corner.x, corner.y);
            points.push_str(&format!("{px:.2},{py:.2} "));
        }
        svg.push_str(&format!(
            r##"<polygon points="{points}" fill="none" stroke="#d5deea" stroke-width="0.8" opacity="0.85"/>"##
        ));
    }
}

fn draw_svg_pose_marker(svg: &mut String, pose: Pose2d, color: &str, frame: &Frame) {
    let (px, py) = frame.project(pose.x, pose.y);
    let (hx, hy) = frame.project(
        pose.x + 0.28 * pose.theta.cos(),
        pose.y + 0.28 * pose.theta.sin(),
    );
    svg.push_str(&format!(
        r##"<circle cx="{px:.2}" cy="{py:.2}" r="4.5" fill="{color}"/>"##
    ));
    svg.push_str(&format!(
        r##"<line x1="{px:.2}" y1="{py:.2}" x2="{hx:.2}" y2="{hy:.2}" stroke="{color}" stroke-width="2"/>"##
    ));
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

fn render_png(solved: &[Solved]) -> Raster {
    let width = 1184;
    let height = 980;
    let mut image = Raster::new(width, height, [18, 24, 32]);
    image.text(
        28,
        18,
        "SYNTHETIC STEERING-LIMITED ROUTES",
        [242, 244, 248],
        2,
    );
    image.text(
        28,
        42,
        "PLACEHOLDER VEHICLE, NOT A CALIBRATED MEASUREMENT",
        [183, 192, 204],
        1,
    );
    image.fill_rect(28, 62, 28, 4, [226, 177, 90]);
    image.text(62, 58, "FORWARD", [183, 192, 204], 1);
    image.fill_rect(130, 62, 28, 4, [62, 196, 214]);
    image.text(164, 58, "REVERSE", [183, 192, 204], 1);
    image.text(230, 58, "OUTLINE = SWEPT BODY", [183, 192, 204], 1);
    for (index, scene) in solved.iter().enumerate() {
        let col = index % 2;
        let row = index / 2;
        let x = 28 + col * (560 + 18);
        let y = 84 + row * (420 + 18);
        draw_png_panel(&mut image, scene, x as i32, y as i32, 560, 420);
    }
    image
}

fn draw_png_panel(image: &mut Raster, scene: &Solved, x: i32, y: i32, w: i32, h: i32) {
    image.fill_rect(x, y, w, h, [27, 36, 48]);
    image.text(
        x + 14,
        y + 12,
        &scene.title.to_ascii_uppercase(),
        [242, 244, 248],
        2,
    );
    let caption = format!(
        "{:.2} M   {} EXPANSIONS   R {:.2} M",
        scene.path.length,
        scene.path.expansions,
        scene.query.model.min_turning_radius()
    );
    image.text(x + 14, y + 32, &caption, [154, 166, 181], 1);
    let frame = Frame::new(
        &scene.query,
        (x + 12) as f64,
        (y + 50) as f64,
        (w - 24) as f64,
        (h - 64) as f64,
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
    let stride = (scene.path.poses.len() / 12).max(1);
    for (index, pose) in scene.path.poses.iter().enumerate() {
        if index % stride != 0 && index + 1 != scene.path.poses.len() {
            continue;
        }
        let Some(corners) = scene.query.footprint.rectangle_corners(*pose) else {
            continue;
        };
        for edge in 0..4 {
            let (x0, y0) = frame.project(corners[edge].x, corners[edge].y);
            let next = corners[(edge + 1) % 4];
            let (x1, y1) = frame.project(next.x, next.y);
            image.line(
                x0.round() as i32,
                y0.round() as i32,
                x1.round() as i32,
                y1.round() as i32,
                [213, 222, 234],
                0,
            );
        }
    }
    for index in 1..scene.path.poses.len() {
        let from = scene.path.poses[index - 1];
        let to = scene.path.poses[index];
        let (x0, y0) = frame.project(from.x, from.y);
        let (x1, y1) = frame.project(to.x, to.y);
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
    draw_png_marker(image, scene.query.start, [141, 206, 122], &frame);
    draw_png_marker(image, scene.query.goal, [213, 138, 212], &frame);
}

fn draw_png_marker(image: &mut Raster, pose: Pose2d, color: [u8; 3], frame: &Frame) {
    let (px, py) = frame.project(pose.x, pose.y);
    let (hx, hy) = frame.project(
        pose.x + 0.28 * pose.theta.cos(),
        pose.y + 0.28 * pose.theta.sin(),
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
    for oy in -4..=4 {
        for ox in -4..=4 {
            if ox * ox + oy * oy <= 16 {
                image.set(cx + ox, cy + oy, color);
            }
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
        ',' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x08],
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
