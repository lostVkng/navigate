//! Offline obstacle updates and replanning.
//!
//! Run with `cargo run --release --example replan_demo`.
//!
//! Writes `assets/replan_demo.svg` and `assets/replan_demo.png`. The process
//! exits with an error unless the initial route and the replan both pass
//! `check_planned_path`, the blocked snapshot reports no safe route, and the
//! earlier tickets are not still validated. Lengths are placeholder geometry.

use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::Instant;

use navigate::planning::synthetic::{self, RouteQuery};
use navigate::prelude::*;

struct Scene {
    title: &'static str,
    note: String,
    map: GridMap,
    path: Option<PlannedPath>,
    stale: bool,
    footprint: Footprint,
}

fn main() {
    let mut corridor = synthetic::replan_corridor();

    let started = Instant::now();
    let initial = plan_route(&corridor.map, &request_of(&corridor)).expect("initial route");
    let initial_ms = started.elapsed().as_secs_f64() * 1_000.0;
    corridor
        .check(initial.path())
        .expect("initial route failed the swept check");

    let plug = synthetic::replan_plug();
    corridor
        .map
        .apply_aabb(plug.0, plug.1, Occupancy::Occupied)
        .expect("plug");
    assert!(
        initial.validated_path(&corridor.map, 0.0).is_none(),
        "the pre-plug ticket must be stale"
    );
    let started = Instant::now();
    let refreshed =
        refresh_route(&corridor.map, &initial, &request_of(&corridor)).expect("refresh");
    let replan_ms = started.elapsed().as_secs_f64() * 1_000.0;
    let RouteRefresh::Replanned(replanned) = refreshed else {
        panic!("plug should force a replan, got {refreshed:?}");
    };
    corridor
        .check(replanned.path())
        .expect("replan failed the swept check");

    let mut gate = synthetic::gate_lane();
    let gate_plan = plan_route(&gate.map, &request_of(&gate)).expect("gate route");
    let seal = synthetic::gate_seal();
    gate.map
        .apply_aabb(seal.0, seal.1, Occupancy::Occupied)
        .expect("seal");
    let started = Instant::now();
    let closed = refresh_route(&gate.map, &gate_plan, &request_of(&gate)).expect("closed refresh");
    let closed_ms = started.elapsed().as_secs_f64() * 1_000.0;
    let RouteRefresh::NoSafeRoute(error) = &closed else {
        panic!("closed gate should not validate a route: {closed:?}");
    };
    assert!(matches!(error, PathPlannerError::NoPathFound), "{error}");
    assert!(gate_plan.validated_path(&gate.map, 0.0).is_none());

    println!(
        "{:<16} {:>8} {:>8} {:>10} {:>8}",
        "snapshot", "length_m", "expand", "revision", "ms"
    );
    println!(
        "{:<16} {:>8.3} {:>8} {:>10} {:>8.2}",
        "initial",
        initial.path().length,
        initial.path().expansions,
        initial.revision(),
        initial_ms
    );
    println!(
        "{:<16} {:>8.3} {:>8} {:>10} {:>8.2}",
        "replan",
        replanned.path().length,
        replanned.path().expansions,
        replanned.revision(),
        replan_ms
    );
    println!(
        "{:<16} {:>8} {:>8} {:>10} {:>8.2}",
        "no safe route",
        "-",
        "-",
        gate.map.revision(),
        closed_ms
    );

    let scenes = [
        Scene {
            title: "INITIAL",
            note: format!(
                "REV {}  {:.2} M  VALID",
                initial.revision(),
                initial.path().length
            ),
            map: {
                let mut map = synthetic::replan_corridor().map;
                let _ = map.apply_aabb(plug.0, plug.1, Occupancy::Free);
                map
            },
            path: Some(initial.path().clone()),
            stale: false,
            footprint: corridor.footprint,
        },
        Scene {
            title: "STALE",
            note: format!("REV {} TICKET IS NOT VALID", corridor.map.revision()),
            map: corridor.map.clone(),
            path: Some(initial.path().clone()),
            stale: true,
            footprint: corridor.footprint,
        },
        Scene {
            title: "REPLAN",
            note: format!(
                "REV {}  {:.2} M  VALID",
                replanned.revision(),
                replanned.path().length
            ),
            map: corridor.map.clone(),
            path: Some(replanned.path().clone()),
            stale: false,
            footprint: corridor.footprint,
        },
        Scene {
            title: "NO SAFE ROUTE",
            note: format!("REV {}  SEARCH FOUND NONE", gate.map.revision()),
            map: gate.map,
            path: None,
            stale: false,
            footprint: gate.footprint,
        },
    ];

    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let svg_path = root.join("assets/replan_demo.svg");
    let png_path = root.join("assets/replan_demo.png");
    fs::write(&svg_path, render_svg(&scenes)).expect("write svg");
    write_png(&png_path, &render_png(&scenes)).expect("write png");
    println!("wrote {}", svg_path.display());
    println!("wrote {}", png_path.display());
}

fn request_of(query: &RouteQuery) -> RouteRequest<'_> {
    RouteRequest {
        start: query.start,
        goal: query.goal,
        model: &query.model,
        footprint: &query.footprint,
        config: &query.config,
        clearance: 0.0,
    }
}

fn render_svg(scenes: &[Scene]) -> String {
    let width = 1200.0;
    let height = 760.0;
    let mut svg = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}"><rect width="{width}" height="{height}" fill="#121820"/>"##
    );
    svg.push_str(r##"<text x="28" y="36" fill="#f2f4f8" font-family="ui-sans-serif, sans-serif" font-size="20">Offline obstacle updates</text>"##);
    svg.push_str(r##"<text x="28" y="58" fill="#b7c0cc" font-family="ui-sans-serif, sans-serif" font-size="13">Placeholder geometry. A gray route is a stale ticket, not a validated path. Unknown and out-of-map space stay blocked. This is not a stopping-distance model.</text>"##);
    for (index, scene) in scenes.iter().enumerate() {
        let col = (index % 2) as f64;
        let row = (index / 2) as f64;
        draw_svg_panel(
            &mut svg,
            scene,
            28.0 + col * 580.0,
            78.0 + row * 330.0,
            560.0,
            310.0,
        );
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
        r##"<text x="{}" y="{}" fill="#9aa6b5" font-family="ui-sans-serif, sans-serif" font-size="12">{}</text>"##,
        x + 14.0,
        y + 42.0,
        scene.note
    ));
    let frame = frame_of(&scene.map, x + 12.0, y + 54.0, w - 24.0, h - 66.0);
    svg.push_str(&format!(
        r##"<rect x="{:.2}" y="{:.2}" width="{:.2}" height="{:.2}" fill="#243044"/>"##,
        frame.x, frame.y, frame.w, frame.h
    ));
    paint_svg_cells(svg, &scene.map, &frame);
    if let Some(path) = &scene.path {
        let color = if scene.stale { "#8b93a0" } else { "#e2b15a" };
        let mut d = String::new();
        for pair in path.poses.windows(2) {
            let (x0, y0) = frame.project(pair[0].x, pair[0].y);
            let (x1, y1) = frame.project(pair[1].x, pair[1].y);
            d.push_str(&format!("M{x0:.2},{y0:.2} L{x1:.2},{y1:.2} "));
        }
        svg.push_str(&format!(
            r##"<path d="{d}" fill="none" stroke="{color}" stroke-width="2.4" stroke-linecap="round"/>"##
        ));
        if !scene.stale {
            draw_boxes(svg, path, &scene.footprint, &frame);
        }
    }
}

fn paint_svg_cells(svg: &mut String, map: &GridMap, frame: &Frame) {
    let resolution = map.resolution();
    let cell = resolution * frame.scale;
    for iy in 0..map.height() {
        for ix in 0..map.width() {
            let kind = map.occupancy(ix, iy);
            let fill = match kind {
                Some(Occupancy::Occupied) => "#7d4542",
                Some(Occupancy::Unknown) => "#5c6570",
                _ => continue,
            };
            let (cx, cy) = map.cell_center(ix, iy);
            let (px, py) = frame.project(cx - resolution * 0.5, cy + resolution * 0.5);
            svg.push_str(&format!(
                r##"<rect x="{px:.2}" y="{py:.2}" width="{cell:.2}" height="{cell:.2}" fill="{fill}"/>"##
            ));
        }
    }
}

fn draw_boxes(svg: &mut String, path: &PlannedPath, footprint: &Footprint, frame: &Frame) {
    let step = (path.poses.len() / 10).max(1);
    for (index, pose) in path.poses.iter().enumerate() {
        if index % step != 0 && index + 1 != path.poses.len() {
            continue;
        }
        let Some(corners) = footprint.rectangle_corners(*pose) else {
            continue;
        };
        let mut points = String::new();
        for corner in corners {
            let (px, py) = frame.project(corner.x, corner.y);
            points.push_str(&format!("{px:.2},{py:.2} "));
        }
        svg.push_str(&format!(
            r##"<polygon points="{points}" fill="none" stroke="#d5deea" stroke-width="0.8"/>"##
        ));
    }
}

struct Frame {
    x: f64,
    y: f64,
    w: f64,
    h: f64,
    origin_x: f64,
    origin_y: f64,
    scale: f64,
}

fn frame_of(map: &GridMap, x: f64, y: f64, width: f64, height: f64) -> Frame {
    let (origin_x, origin_y) = map.origin();
    let world_w = map.width() as f64 * map.resolution();
    let world_h = map.height() as f64 * map.resolution();
    let scale = (width / world_w).min(height / world_h);
    Frame {
        x: x + (width - world_w * scale) * 0.5,
        y: y + (height - world_h * scale) * 0.5,
        w: world_w * scale,
        h: world_h * scale,
        origin_x,
        origin_y,
        scale,
    }
}

impl Frame {
    fn project(&self, wx: f64, wy: f64) -> (f64, f64) {
        (
            self.x + (wx - self.origin_x) * self.scale,
            self.y + self.h - (wy - self.origin_y) * self.scale,
        )
    }
}

struct Raster {
    width: usize,
    height: usize,
    rgb: Vec<u8>,
}

impl Raster {
    fn new(width: usize, height: usize) -> Self {
        Self {
            width,
            height,
            rgb: [18, 24, 32].repeat(width * height),
        }
    }

    fn set(&mut self, x: i32, y: i32, color: [u8; 3]) {
        if x < 0 || y < 0 || x >= self.width as i32 || y >= self.height as i32 {
            return;
        }
        let index = (y as usize * self.width + x as usize) * 3;
        self.rgb[index..index + 3].copy_from_slice(&color);
    }

    fn fill(&mut self, x: i32, y: i32, w: i32, h: i32, color: [u8; 3]) {
        for py in y..y + h {
            for px in x..x + w {
                self.set(px, py, color);
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
                        self.fill(
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

fn render_png(scenes: &[Scene]) -> Raster {
    let mut image = Raster::new(1200, 760);
    image.text(28, 16, "OFFLINE OBSTACLE UPDATES", [242, 244, 248], 2);
    image.text(
        28,
        40,
        "GRAY PATH IS A STALE TICKET, NOT A VALIDATED ROUTE",
        [183, 192, 204],
        1,
    );
    for (index, scene) in scenes.iter().enumerate() {
        let col = (index % 2) as i32;
        let row = (index / 2) as i32;
        draw_png_panel(&mut image, scene, 28 + col * 580, 64 + row * 340, 560, 320);
    }
    image
}

fn draw_png_panel(image: &mut Raster, scene: &Scene, x: i32, y: i32, w: i32, h: i32) {
    image.fill(x, y, w, h, [27, 36, 48]);
    image.text(x + 14, y + 12, scene.title, [242, 244, 248], 2);
    image.text(x + 14, y + 32, &scene.note, [154, 166, 181], 1);
    let frame = frame_of(
        &scene.map,
        (x + 12) as f64,
        (y + 50) as f64,
        (w - 24) as f64,
        (h - 62) as f64,
    );
    image.fill(
        frame.x.round() as i32,
        frame.y.round() as i32,
        frame.w.round() as i32,
        frame.h.round() as i32,
        [36, 48, 68],
    );
    let resolution = scene.map.resolution();
    let cell = (resolution * frame.scale).round().max(1.0) as i32;
    for iy in 0..scene.map.height() {
        for ix in 0..scene.map.width() {
            let color = match scene.map.occupancy(ix, iy) {
                Some(Occupancy::Occupied) => [125, 69, 66],
                Some(Occupancy::Unknown) => [92, 101, 112],
                _ => continue,
            };
            let (cx, cy) = scene.map.cell_center(ix, iy);
            let (px, py) = frame.project(cx - resolution * 0.5, cy + resolution * 0.5);
            image.fill(
                px.round() as i32,
                py.round() as i32,
                cell + 1,
                cell + 1,
                color,
            );
        }
    }
    if let Some(path) = &scene.path {
        let color = if scene.stale {
            [139, 147, 160]
        } else {
            [226, 177, 90]
        };
        for pair in path.poses.windows(2) {
            let (x0, y0) = frame.project(pair[0].x, pair[0].y);
            let (x1, y1) = frame.project(pair[1].x, pair[1].y);
            line(
                image,
                x0.round() as i32,
                y0.round() as i32,
                x1.round() as i32,
                y1.round() as i32,
                color,
            );
        }
    }
}

fn line(image: &mut Raster, mut x0: i32, mut y0: i32, x1: i32, y1: i32, color: [u8; 3]) {
    let dx = (x1 - x0).abs();
    let dy = (y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx - dy;
    loop {
        for oy in -1..=1 {
            for ox in -1..=1 {
                image.set(x0 + ox, y0 + oy, color);
            }
        }
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 > -dy {
            err -= dy;
            x0 += sx;
        }
        if e2 < dx {
            err += dx;
            y0 += sy;
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
        'M' => [0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F],
        'N' => [0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11],
        'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'P' => [0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10],
        'R' => [0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11],
        'S' => [0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E],
        'T' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x0A, 0x0A, 0x04],
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
        '.' => [0, 0, 0, 0, 0, 0x06, 0x06],
        ' ' => [0, 0, 0, 0, 0, 0, 0],
        _ => [0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F],
    }
}

fn write_png(path: &std::path::Path, image: &Raster) -> std::io::Result<()> {
    let mut raw = Vec::new();
    for row in image.rgb.chunks_exact(image.width * 3) {
        raw.push(0);
        raw.extend_from_slice(row);
    }
    let mut out = vec![137, 80, 78, 71, 13, 10, 26, 10];
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&(image.width as u32).to_be_bytes());
    ihdr.extend_from_slice(&(image.height as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    out.extend(png_chunk(b"IHDR", &ihdr));
    out.extend(png_chunk(b"IDAT", &zlib_store(&raw)));
    out.extend(png_chunk(b"IEND", &[]));
    let mut file = fs::File::create(path)?;
    file.write_all(&out)
}

fn png_chunk(kind: &[u8; 4], data: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    out.extend_from_slice(kind);
    out.extend_from_slice(data);
    let mut crc_input = kind.to_vec();
    crc_input.extend_from_slice(data);
    out.extend_from_slice(&crc32(&crc_input).to_be_bytes());
    out
}

fn zlib_store(data: &[u8]) -> Vec<u8> {
    let mut out = vec![0x78, 0x01];
    let mut offset = 0;
    while offset < data.len() {
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
