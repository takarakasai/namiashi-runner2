//! Build the かわさきロボット競技大会 ring and dump it for inspection, so its
//! dimensions can be checked against the rulebook drawing BEFORE anything is
//! built on top of them.
//!
//! Every number in `KawasakiRingCfg` was scaled off a figure, not read from
//! CAD (see that type's doc comment for which are dimensioned and which are
//! estimates). Getting them wrong is cheap to fix and expensive to discover
//! later, so this renders the field before it becomes a benchmark.
//!
//! Writes a top-down PGM of the heightfield (greyscale = elevation, so hole
//! rims and the bowl's dish are directly readable) plus the MJCF, and loads
//! the model to confirm MuJoCo accepts it and the grid transfers.
//!
//! Run: `cargo run --release --no-default-features --features mujoco \
//!   --example kawasaki_ring_check -- [out_dir]`

#[cfg(feature = "mujoco")]
fn main() {
    use articara::mjcf::MjcfExportOptions;
use namiashi_sim::ring::KawasakiRingCfg;
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;

    let out_dir = std::env::args().nth(1).unwrap_or_else(|| "/tmp/kawasaki_ring".into());
    std::fs::create_dir_all(&out_dir).expect("create out dir");

    let ring = KawasakiRingCfg::default();
    let (nrow, ncol) = ring.grid();
    let z_top = ring.z_top_m();
    println!("ring {:.2} m, grid {nrow}x{ncol} @ {:.1} mm, z_top {:.1} mm",
             ring.ring_m, ring.cell_m * 1000.0, z_top * 1000.0);

    let heights = ring.heights();

    // Top-down elevation map. PGM because it needs no image crate and any
    // viewer opens it; the point is to eyeball the layout, not to ship art.
    let mut pgm = format!("P2\n{ncol} {nrow}\n255\n");
    for r in (0..nrow).rev() {
        // Top row of the image is +y, so the picture matches the rulebook's
        // plan view rather than being flipped.
        for c in 0..ncol {
            let v = (heights[r * ncol + c] * 255.0).round().clamp(0.0, 255.0) as u32;
            pgm.push_str(&format!("{v} "));
        }
        pgm.push('\n');
    }
    let pgm_path = format!("{out_dir}/ring_elevation.pgm");
    std::fs::write(&pgm_path, pgm).expect("write pgm");
    println!("wrote {pgm_path}");

    // Feature heights at a few named points, as a numeric cross-check on
    // the picture: a map that looks right can still be scaled wrong.
    for (label, x, y) in [
        ("ring floor", 0.80, 0.80),
        ("bowl centre", 0.0, 0.0),
        ("bowl rim", 0.21, 0.0),
        ("round plate body", 0.65 + 0.13, 0.0),
        ("round plate hole", 0.65, 0.0),
        // The quad plate is a diamond, so its hole centres sit on the world
        // axes even though they are on a square pitch in the plate's own
        // frame: (+q,+q) in plate coords maps to (0, q*sqrt(2)) in world.
        // An earlier version probed straight up from the centre and called
        // that the plate body -- it is a hole.
        ("quad plate body", 0.0, 0.65),
        ("quad plate hole", 0.0, 0.65 + 0.075 * 1.41421),
        ("quad plate hole 2", 0.075 * 1.41421, 0.65),
    ] {
        println!("  {label:<20} ({x:+.3},{y:+.3}) -> {:.1} mm",
                 ring.height_at(x, y) * 1000.0);
    }

    // Load it in MuJoCo: an hfield the exporter emits but MuJoCo rejects,
    // or a grid whose size disagrees with the asset, only shows up here.
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_meas.misa");
    let robot = RobotModel::from_misa(&misa).expect("load namiashi");
    // Spawn on the red start platform, same as both teleop demos -- the
    // origin is the centre bowl.
    let (pw, pd) = ring.red_platform_m;
    let opts = MjcfExportOptions {
        base_xy: Some((
            -(ring.ring_m / 2.0 + pd / 2.0),
            -(ring.ring_m / 2.0 - pw / 2.0),
        )),
        extra_asset_xml: Some(ring.asset_xml("kawasaki") + articara::mjcf::scene_lighting_asset_xml().as_str()),
        extra_visual_xml: Some(articara::mjcf::scene_lighting_visual_xml()),
        extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
        add_actuators: true,
        ..MjcfExportOptions::default()
    };
    let xml_path = format!("{out_dir}/model.xml");
    std::fs::write(&xml_path, articara::mjcf::export_mjcf_with_options(&robot, opts.clone()))
        .expect("write xml");
    println!("wrote {xml_path}");

    let mut robot = robot;
    let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
    sim.set_hfield_data("kawasaki", &heights).expect("fill hfield");
    println!("MuJoCo loaded the ring and accepted the {}-value grid.", heights.len());

    // Loading is not standing. Step it: a heightfield the robot falls
    // through, or one whose contacts explode, is invisible until something
    // actually rests on it -- and the ring is the first hfield in this repo,
    // so nothing else has exercised that path.
    let dt = sim.timestep();
    let start = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
    for _ in 0..(2.0 / dt) as u32 {
        sim.step(&mut robot, dt, true);
    }
    let end = sim.body_world_position(&robot.root_link).unwrap_or([0.0; 3]);
    let contacts = sim.contacts().len();
    println!(
        "after 2 s holding stance: trunk ({:+.3},{:+.3},{:+.3}) -> ({:+.3},{:+.3},{:+.3}), \
         {contacts} contacts",
        start[0], start[1], start[2], end[0], end[1], end[2],
    );
    println!(
        "  verdict: {}",
        if end[2] < -0.10 {
            "FELL THROUGH the field"
        } else if !end[2].is_finite() || end[2].abs() > 5.0 {
            "DIVERGED"
        } else {
            "stands on the platform"
        }
    );

    // The floor exists to catch a ring-out. Checking that it does needs a
    // robot placed off the ring, because nothing on the ring ever reaches
    // it -- an absent or mis-placed floor looks identical from up here.
    if let Some(side) = ring.floor_size_m {
        let off = ring.ring_m / 2.0 + 0.4; // clear of the ring, well inside the floor
        assert!(off < side / 2.0, "test point must land ON the floor");
        let mut robot2 = RobotModel::from_misa(&misa).expect("load namiashi");
        let opts2 = MjcfExportOptions {
            base_xy: Some((off, off)),
            extra_asset_xml: Some(ring.asset_xml("kawasaki") + articara::mjcf::scene_lighting_asset_xml().as_str()),
        extra_visual_xml: Some(articara::mjcf::scene_lighting_visual_xml()),
            extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
            add_actuators: true,
            ..MjcfExportOptions::default()
        };
        let mut sim2 = MujocoSim::new(&robot2, opts2).expect("MujocoSim::new (off-ring)");
        sim2.set_hfield_data("kawasaki", &heights).expect("fill hfield");
        let dt2 = sim2.timestep();
        for _ in 0..(2.0 / dt2) as u32 {
            sim2.step(&mut robot2, dt2, true);
        }
        let p = sim2.body_world_position(&robot2.root_link).unwrap_or([0.0; 3]);
        // Spawned just above the ring surface, so it drops onto the floor
        // and should end one stance height above it.
        let above_floor = p[2] + ring.floor_top_z();
        println!(
            "off-ring drop at ({off:+.2},{off:+.2}): trunk z={:+.3} m, {:.3} m above the floor",
            p[2], above_floor,
        );
        // Two separate questions, because they used to be conflated: does
        // the floor CATCH it (the floor's whole job), and does it survive
        // the landing (not the floor's job at all). Spawning at ring height
        // means a 0.49 m drop, which this robot topples from about as often
        // as not -- reading that as a floor failure sent the first version
        // of this check looking in the wrong place.
        println!(
            "  verdict: {}{}",
            if above_floor > -0.02 && above_floor < 0.45 {
                "caught by the floor"
            } else if above_floor <= -0.02 {
                "FELL PAST the floor"
            } else {
                "did not reach the floor"
            },
            if above_floor > 0.15 { ", still upright" } else { ", toppled on landing" },
        );
    }
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs: cargo run --features mujoco --example kawasaki_ring_check");
    std::process::exit(2);
}
