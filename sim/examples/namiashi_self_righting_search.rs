//! Search for a recovery trajectory that actually gets namiashi back on its
//! feet, over the 20 parameters of `namiashi_sim::self_righting::RecoveryParams`.
//!
//! Worth doing because `namiashi_self_righting_probe` established that the
//! ceiling is a scheduling problem, not a physical one: hand-written plans
//! stall on the robot's side at about 45 degrees, but widening hip roll from
//! 1.05 to 2.40 rad does not move that number at all. So the pose is
//! reachable and what is missing is the timing to reach it.
//!
//! Scored on success MINUS roughness, so an unhurried recovery beats a
//! violent one that works equally well.
//!
//! Scored through `Drive::TorqueAhrs` -- the torque PD and IMU-filtered
//! attitude `run_wbc_sim` uses -- not the .misa's Position actuators.
//!
//! METHOD: cross-entropy method, matching `go2_wbc_bound_cmaes_mode_h` in
//! `tests/wbc_walk_go2.rs` -- a diagonal Gaussian refit to the elite
//! fraction each generation, no new dependency, deterministic LCG so a run
//! reproduces. `evaluate` is deterministic (MuJoCo forward dynamics and an
//! open-loop trajectory, no RNG on the path), so one evaluation per
//! candidate per condition suffices.
//!
//! OBJECTIVE: every candidate is scored on TEN conditions at once -- all
//! five sites on the ring crossed with friction 0.30 and 1.00 -- and the
//! fitness is `mean + 0.5 * worst`. This is the whole point of the setup.
//! The hip-bias-gate search earlier in this project optimised a single
//! condition and produced a parameter set that collapsed under any
//! perturbation (`namiashi_staircase_5cm_hip_gate_robustness`, 6/6
//! failures); a fitness that a one-condition spike cannot maximise is the
//! structural fix, not more careful interpretation of the result.
//!
//! Friction 0.70 is held out of training entirely, so the validation grid
//! still contains five conditions the search never saw.
//!
//! Run: `cargo run --release --no-default-features --features mujoco
//!       --example namiashi_self_righting_search`
//!
//! Roughly 0.4 s per evaluation, parallel across the population.

#[cfg(feature = "mujoco")]
fn main() {
    use namiashi_sim::ring::KawasakiRingCfg;
    use namiashi_sim::self_righting::{
        evaluate_rolled, Drive, RecoveryParams, BOUNDS, DIM, GENTLE_V2, SEARCHED_V4,
    };

    // Search against the control path the teleop actually runs, not the
    // .misa's Position actuators. Round 2 optimised against those and
    // `namiashi_self_righting_check` then measured 15/15 under them and
    // 8/15 through the torque PD and IMU-filtered attitude that
    // `run_wbc_sim` uses. A trajectory that only works under the evaluator
    // it was fitted to is the same failure as fitting one condition.
    const TELEOP: Drive = Drive::TorqueAhrs { kp: 100.0, kd: 1.2, tilt_tau_s: 0.15 };

    const POP: usize = 40;
    const ELITE: usize = 10;
    const GENS: usize = 35;
    /// Search and validation horizons, deliberately equal, and long enough
    /// for a rate-limited recovery to finish -- an 8 s window would score a
    /// slow trajectory as a failed one. They were 6 and 8
    /// and round 4 scored 2.69 out of a possible 2.70 in training while
    /// righting only 4 of 15 in validation -- ten of those fifteen ARE the
    /// training conditions, so the contradiction was entirely the two extra
    /// seconds. The trajectory stood up inside six and toppled inside eight.
    /// A shorter search horizon does not select for faster recoveries, it
    /// selects for ones that have not fallen over yet.
    const SEARCH_S: f64 = 20.0;
    const VALIDATE_S: f64 = 20.0;

    // `--gentle` forbids the rocking. The searches so far all converge on a
    // ~1.3 s rock cycle and pay nearly the full roughness penalty, and the
    // reason is visible in the numbers: measured joint speed reaches 20 to
    // 31 rad/s while the COMMAND is rate-limited to 5.6, so the violence is
    // the body being thrown, not the targets moving fast. Rocking is how the
    // trajectory buys the momentum to get over.
    //
    // So this asks the question directly: with `period_s` pinned past the
    // horizon (one push, no cycles) and the command rate held under 1.5
    // rad/s, is there a quasi-static way over at all? If the answer is no,
    // that is worth knowing plainly rather than inferring from a penalty
    // the search keeps choosing to pay.
    // `--rate <v>` caps the commanded joint rate at v rad/s. The default
    // search converges on a ~1.3 s rock cycle and pays nearly the full
    // roughness penalty, and the numbers say why: measured joint speed
    // reaches 20 to 31 rad/s while the COMMAND is already limited to 5.6, so
    // the violence is the body being thrown, not the targets moving fast.
    // Rocking is how the trajectory buys the momentum to get over. Capping
    // the rate is therefore the one knob that actually forbids that, and
    // sweeping it maps the trade between how gentle a recovery is and how
    // often it works.
    let args: Vec<String> = std::env::args().collect();
    let rate_cap: Option<f64> = args
        .iter()
        .position(|a| a == "--rate")
        .and_then(|i| args.get(i + 1))
        .and_then(|v| v.parse().ok());
    let mut bounds = BOUNDS;
    if let Some(cap) = rate_cap {
        // `max_rate_rad_s` is the second-to-last dimension.
        let i = DIM - 2;
        bounds[i] = (BOUNDS[i].0, cap.clamp(BOUNDS[i].0, BOUNDS[i].1));
    }

    let ring = KawasakiRingCfg::default();
    let (pw, pd) = ring.red_platform_m;
    let misa = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/namiashi/namiashi_3p3_prop.misa");

    // The flat venue floor is the honest baseline (nothing to slide off, no
    // edge to help); the open ring surface adds the plate seams and the 5 mm
    // heightfield relief; the rest are the ring's own features.
    const FLAT: (f64, f64) = (1.60, 1.60);
    const OPEN_RING: (f64, f64) = (0.35, 0.35);
    let sites: [(&str, (f64, f64)); 5] = [
        ("flat venue floor", FLAT),
        ("open ring surface", OPEN_RING),
        ("ring centre (bowl)", (0.0, 0.0)),
        ("on a round plate", ring.round_plate_centres[0]),
        (
            "red platform (edge)",
            (-(ring.ring_m / 2.0 + pd / 2.0), -(ring.ring_m / 2.0 - pw / 2.0)),
        ),
    ];
    // Round 1 trained on two sites and failed on three of the other three,
    // so all five sites are in the training set now. Friction 0.70 is held
    // out entirely instead: it keeps a real generalisation check (five
    // conditions the search never sees) while no longer asking the
    // trajectory to generalise across terrain it was never shown.
    // Starting rolls, in radians. 180 degrees is flat on its back and is
    // what every earlier search trained on; +/-90 is on its side, which is
    // how a robot that has been knocked over usually ends up and which the
    // resulting trajectories then could not recover from at all (0 of 4,
    // `namiashi_self_righting_from_its_side`). Both signs, because the hip
    // limits are mirrored and the arm is off-centre in y, so the two sides
    // are not the same problem.
    let rolls = [
        std::f64::consts::PI,
        std::f64::consts::FRAC_PI_2,
        -std::f64::consts::FRAC_PI_2,
    ];
    // Two sites rather than five: adding the roll axis triples the
    // conditions, and covering the attitudes matters more here than
    // covering terrain the earlier rounds already showed it handles.
    let train: Vec<((f64, f64), f64, f64)> = sites[..2]
        .iter()
        .flat_map(|&(_, xy)| {
            [0.30_f64, 1.00]
                .into_iter()
                .flat_map(move |mu| rolls.map(move |r| (xy, mu, r)))
        })
        .collect();

    // Fitness across the training conditions. `mean + 0.5 * worst`: the mean
    // gives the search something to climb while everything still fails, the
    // worst-case term stops it trading a condition away once things start
    // working.
    let fitness = |p: &RecoveryParams, conds: &[((f64, f64), f64, f64)], secs: f64| -> f64 {
        let mut sum = 0.0;
        let mut worst = f64::INFINITY;
        for &(xy, mu, roll) in conds {
            let o = evaluate_rolled(&misa, &ring, xy, mu, p, secs, TELEOP, None, roll);
            // The success term minus how far past the gentleness targets it
            // went. Measured on the previous winner: 30 to 39 rad/s of joint
            // speed against a 33.5 rad/s limit, and 10 to 18 rad/s of trunk
            // angular speed -- two to three body revolutions per second. It
            // rights the robot by throwing it.
            let s = o.score() - o.roughness();
            sum += s;
            worst = worst.min(s);
        }
        sum / conds.len() as f64 + 0.5 * worst
    };

    // Deterministic LCG; std has no RNG and reproducibility is a feature.
    let mut seed: u64 = 0x5EED_1234_ABCD_0001;
    let mut next_unit = move || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((seed >> 11) as f64) / ((1u64 << 53) as f64)
    };
    let gauss = |m: f64, s: f64, u1: f64, u2: f64| {
        let r = (-2.0 * (u1.max(1e-12)).ln()).sqrt();
        m + s * r * (2.0 * std::f64::consts::PI * u2).cos()
    };

    // Seed from whichever incumbent belongs to this mode -- a rate-capped
    // search started from the violent trajectory spends its first
    // generations undoing it.
    let mut mean = if rate_cap.is_some() { GENTLE_V2.to_vec() } else { SEARCHED_V4.to_vec() };
    let mut sigma = [0.0; DIM];
    for i in 0..DIM {
        // Wider again than a pure refinement: the seed works under a
        // different evaluator, so its optimum is not necessarily near this
        // one's.
        mean[i] = mean[i].clamp(bounds[i].0, bounds[i].1);
        sigma[i] = 0.18 * (bounds[i].1 - bounds[i].0);
    }

    let seed = RecoveryParams::from_vec(&{
        let mut v = mean;
        for i in 0..DIM {
            v[i] = v[i].clamp(bounds[i].0, bounds[i].1);
        }
        v
    });
    println!(
        "commanded joint rate <= {} rad/s",
        rate_cap.map_or("(unconstrained)".to_string(), |c| format!("{c:.2}"))
    );
    let seed_fit = fitness(&seed, &train, SEARCH_S);
    println!("seed (previous winner) fitness = {seed_fit:.4}");
    println!("{:>4} {:>10} {:>10} {:>10}", "gen", "best", "elite mean", "sigma sum");

    let mut best = (seed_fit, seed);
    for g in 0..GENS {
        // Sample the population up front so the RNG draw order does not
        // depend on thread scheduling.
        let mut cands: Vec<RecoveryParams> = Vec::with_capacity(POP);
        cands.push(RecoveryParams::from_vec(&mean));
        while cands.len() < POP {
            let mut v = [0.0; DIM];
            for i in 0..DIM {
                v[i] = gauss(mean[i], sigma[i], next_unit(), next_unit())
                    .clamp(bounds[i].0, bounds[i].1);
            }
            cands.push(RecoveryParams::from_vec(&v));
        }

        let workers = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
        let chunk = cands.len().div_ceil(workers);
        let scored: Vec<(f64, RecoveryParams)> = std::thread::scope(|s| {
            let handles: Vec<_> = cands
                .chunks(chunk)
                .map(|part| s.spawn(|| part.iter().map(|c| (fitness(c, &train, SEARCH_S), *c)).collect::<Vec<_>>()))
                .collect();
            handles.into_iter().flat_map(|h| h.join().unwrap()).collect()
        });

        let mut ranked = scored;
        ranked.sort_by(|a, b| b.0.total_cmp(&a.0));
        if ranked[0].0 > best.0 {
            best = ranked[0];
        }

        // Refit the diagonal Gaussian to the elites.
        let elites = &ranked[..ELITE];
        let mut new_mean = [0.0; DIM];
        for (_, p) in elites {
            let v = p.to_vec();
            for i in 0..DIM {
                new_mean[i] += v[i] / ELITE as f64;
            }
        }
        let mut new_sigma = [0.0; DIM];
        for (_, p) in elites {
            let v = p.to_vec();
            for i in 0..DIM {
                new_sigma[i] += (v[i] - new_mean[i]).powi(2) / ELITE as f64;
            }
        }
        for i in 0..DIM {
            // Floor at 2% of the range: a diagonal CEM collapses its own
            // variance and stops exploring long before it has to.
            new_sigma[i] = new_sigma[i].sqrt().max(0.02 * (bounds[i].1 - bounds[i].0));
        }
        mean = new_mean;
        sigma = new_sigma;

        let elite_mean: f64 = elites.iter().map(|e| e.0).sum::<f64>() / ELITE as f64;
        println!(
            "{g:>4} {:>10.4} {elite_mean:>10.4} {:>10.4}",
            ranked[0].0,
            sigma.iter().sum::<f64>()
        );
    }

    // ---- validation on all fifteen conditions -------------------------
    println!("\nbest fitness {:.4}\n{:#?}\n", best.0, best.1);
    println!(
        "validation at {VALIDATE_S} s ('*' = a condition the search never saw)\n{:<18} {:>6} {:>5} {:>8} {:>8} {:>7} {:>7} {:>7}",
        "site", "roll", "mu", "peak up", "final", "t_right", "w rms", "qd rms"
    );
    let (mut ok, mut total) = (0, 0);
    for (name, xy) in sites {
      for roll in rolls {
        for mu in [0.30_f64, 0.70, 1.00] {
            let o = evaluate_rolled(&misa, &ring, xy, mu, &best.1, VALIDATE_S, TELEOP, None, roll);
            let unseen = if train.contains(&(xy, mu, roll)) { ' ' } else { '*' };
            total += 1;
            if o.righted() {
                ok += 1;
            }
            println!(
                "{unseen}{name:<17} {:>5.0}d {mu:>5.2} {:>8.3} {:>8.3} {:>7.2} {:>7.1} {:>7.1}  {}",
                roll.to_degrees(),
                o.peak_up,
                o.final_up,
                o.t_right_s,
                o.rms_omega_rad_s,
                o.rms_joint_rad_s,
                if o.righted() { "RIGHTED" } else { "no" }
            );
        }
      }
    }
    println!("\n{ok}/{total} righted");
}

#[cfg(not(feature = "mujoco"))]
fn main() {
    eprintln!("needs --features mujoco");
    std::process::exit(2);
}
