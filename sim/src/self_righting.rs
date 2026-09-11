//! Getting namiashi off its back.
//!
//! An open-loop joint trajectory with three regimes, switched on one
//! measured quantity (the trunk's world +z) plus one sign (which side of the
//! trunk is underneath). Both come from the attitude estimate, so this needs
//! nothing the robot does not already have -- IMU and joint encoders, no
//! exteroception.
//!
//! The shape is not a guess. `examples/namiashi_self_righting_probe`
//! measured 18 hand-written plans across 5 sites on the ring and 3 friction
//! values and found:
//!
//!   * the arm, despite 6.865 N.m against a hip's 2.5 and a 0.29 m reach,
//!     never gets the trunk near vertical -- its axis is pitch, so it can
//!     only tip the body over its longest dimension (0.294 m hip to hip);
//!   * repeated leg splay pulses roll the body onto its side (about 45
//!     degrees) reliably, from every site and friction tried;
//!   * no hand-written plan crossed the remaining 45 degrees; and
//!   * that ceiling is NOT kinematic -- widening hip roll from 1.05 to 2.40
//!     rad left the result bit-identical.
//!
//! So the primitive works and the schedule is what is missing, which is what
//! [`RecoveryParams`] exposes to search. Front and rear legs are tied
//! together (FL=RL, FR=RR): the probe's working plans were all front/rear
//! symmetric, and halving the dimension is worth more than the asymmetry is
//! likely to buy.

/// Joint order used throughout: FL, FR, RL, RR, each `[hip, thigh, calf]`.
pub type LegTargets = [[f64; 3]; 4];

/// Hip roll limits from the .misa, per side. Left legs (FL, RL) and right
/// legs (FR, RR) are mirrored, so a "push left" and a "push right" of equal
/// magnitude are not equally available.
pub const HIP_LIMIT_L: (f64, f64) = (-0.785, 1.05);
pub const HIP_LIMIT_R: (f64, f64) = (-1.05, 0.785);
pub const THIGH_LIMIT: (f64, f64) = (-2.62, 2.62);
pub const CALF_LIMIT: (f64, f64) = (-2.62, 2.62);
pub const ARM_LIMIT: (f64, f64) = (-2.3, 0.85);

/// The pose the robot holds when it is not recovering -- the .misa's spawn
/// stance, i.e. what `respawn` restores.
pub const STANCE: [f64; 3] = [0.0, 0.9, -1.8];

/// Above this much trunk +z the recovery is over and the robot just stands.
pub const UPRIGHT_UP: f64 = 0.85;

/// An open-loop recovery trajectory.
///
/// Regime 1 (`up < handoff_up`): a rocking cycle, `push_frac` of each
/// `period_s` spent in the push pose and the rest back at the rest pose,
/// with `ramp_s` to move between them. Ramp matters more than it looks: the
/// arm's actuator kp is 5.0 against the legs' 100.0, so a ramped target
/// commands very little arm torque while a step saturates it.
///
/// Regime 2 (`handoff_up <= up < UPRIGHT_UP`): past vertical and still
/// rolling. The legs underneath extend to push the trunk across; the ones on
/// top fold so they cannot prop it and stall the roll. Which side is
/// underneath comes from the sign of gravity's y component in the trunk
/// frame.
///
/// Regime 3: [`STANCE`], and stand up.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RecoveryParams {
    pub period_s: f64,
    pub push_frac: f64,
    pub ramp_s: f64,
    /// Hip roll in the push pose, `[left, right]`.
    pub hip_push: [f64; 2],
    /// Hip roll in the rest pose, `[left, right]`.
    pub hip_rest: [f64; 2],
    pub thigh_push: f64,
    pub calf_push: f64,
    pub thigh_rest: f64,
    pub calf_rest: f64,
    pub arm_push: f64,
    pub arm_rest: f64,
    /// Trunk +z at which regime 1 hands over to regime 2.
    pub handoff_up: f64,
    /// Regime 2 hip roll magnitude for the legs underneath; the sign is set
    /// by which side that is.
    pub finish_hip: f64,
    pub finish_thigh: f64,
    pub finish_calf: f64,
    pub finish_arm: f64,
    /// Regime 3. No longer searched -- pinned to [`STANCE`], and these are
    /// kept only so existing constants still describe a whole trajectory.
    ///
    /// Searching them was tried, on the theory that a wider stance would
    /// have more margin against the momentum left over from the roll, and it
    /// backfired. With a corrected attitude signal the search chose
    /// `stand_calf` = +1.72, bent the opposite way to the stance's -1.8 and
    /// unable to carry the robot at all: it righted itself (peak trunk +z
    /// 1.000 in every condition) and immediately collapsed into that pose,
    /// so it never accumulated the second of upright attitude that ends the
    /// recovery, and settled at 0.474 in ten of fifteen. Once the robot is
    /// up, the right pose is the one it stands in.
    pub stand_hip: f64,
    pub stand_thigh: f64,
    pub stand_calf: f64,
    /// Ceiling on how fast any commanded joint angle may move (rad/s),
    /// applied by [`Slew`] to whatever [`targets`] returns.
    ///
    /// The parameters alone cannot express this. `ramp_s` only shapes the
    /// rock cycle -- the step into the rest pose at t=0, and both regime
    /// changes, are instantaneous target jumps whatever it is set to, and
    /// a position servo asked to jump moves as fast as it physically can.
    /// The first searched trajectory peaked at 30 to 39 rad/s of measured
    /// joint speed, against the .misa's own 33.5 rad/s limit.
    ///
    /// [`targets`]: RecoveryParams::targets
    pub max_rate_rad_s: f64,
    /// Rate limit for the return to a normal stance, once the recovery is
    /// over. Separate from `max_rate_rad_s` because they are different
    /// problems: the low rate during the roll is what stops the trajectory
    /// building momentum, but applying it to the stand-up as well cost every
    /// condition. Measured with both at 1.5 rad/s, the robot reached a trunk
    /// +z of 1.000 in all fifteen and then settled back to 0.477 in eleven
    /// of them -- it was righting itself and toppling during the two and a
    /// half seconds the legs needed to unfold. The RMS gentleness terms
    /// cover the whole run, so a stand-up violent enough to matter is still
    /// penalised.
    pub stand_rate_rad_s: f64,
}

/// Which way is down, from the accelerometer alone.
///
/// The recovery needs two numbers: the trunk's +z in world coordinates
/// (upright or not) and which side of the body is underneath. Both are just
/// the direction of gravity in the body frame, which a stationary or
/// slowly-moving accelerometer measures directly.
///
/// A full attitude filter was tried first and is the wrong tool here. The
/// Madgwick filter this project already carries starts from identity and
/// walks toward the measurement at a rate set by `beta`; at the 0.1 the
/// proprioceptive levelling uses -- correct for that job, which only ever
/// sees small angles -- a 180-degree initial error takes far longer than a
/// recovery lasts. Measured on a robot lying still on its back: after six
/// seconds the filter reported a trunk +z of +0.371 against a true -0.985,
/// and over a whole 12 s recovery it never got the sign right. Every regime
/// switch and the hand-back to a normal stance were keying off that.
///
/// The accelerometer has no convergence time. Its problem is the opposite --
/// it reads specific force, so it is wrong exactly when the body is being
/// accelerated -- hence the low pass. Still proprioception only, no
/// exteroception, and no gyro integration to drift.
#[derive(Clone, Copy, Debug)]
pub struct TiltEstimator {
    /// Filtered gravity direction in the trunk frame, unit-ish.
    g: [f64; 3],
    seeded: bool,
    /// Time constant of the low pass (s).
    pub tau_s: f64,
}

impl TiltEstimator {
    pub fn new(tau_s: f64) -> Self {
        Self { g: [0.0, 0.0, -1.0], seeded: false, tau_s }
    }

    /// Feed one accelerometer sample (m/s^2, trunk frame, specific force --
    /// so a robot standing still reads +9.81 on its own +z).
    pub fn update(&mut self, accel: [f64; 3], dt: f64) {
        let n = (accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]).sqrt();
        if n < 1e-6 {
            return;
        }
        // Gravity points opposite to the specific force.
        let m = [-accel[0] / n, -accel[1] / n, -accel[2] / n];
        let a = if self.seeded {
            (dt / self.tau_s.max(1e-6)).clamp(0.0, 1.0)
        } else {
            self.seeded = true;
            1.0
        };
        for k in 0..3 {
            self.g[k] += a * (m[k] - self.g[k]);
        }
    }

    /// Trunk +z expressed in world coordinates: +1 upright, -1 on its back.
    pub fn up(&self) -> f64 {
        let n = (self.g[0] * self.g[0] + self.g[1] * self.g[1] + self.g[2] * self.g[2]).sqrt();
        if n < 1e-9 {
            0.0
        } else {
            -self.g[2] / n
        }
    }

    /// Gravity's y component in the trunk frame; its sign says which side of
    /// the body is underneath.
    pub fn g_body_y(&self) -> f64 {
        self.g[1]
    }
}

/// Rate limiter for the commanded joint angles.
///
/// Seeded from where the joints actually are when the recovery starts, so
/// the first command is a continuation of the current pose rather than a
/// jump to the trajectory's.
#[derive(Clone, Copy, Debug)]
pub struct Slew {
    pub max_rate_rad_s: f64,
    arm: f64,
    legs: LegTargets,
}

impl Slew {
    pub fn new(max_rate_rad_s: f64, arm: f64, legs: LegTargets) -> Self {
        Self { max_rate_rad_s, arm, legs }
    }

    pub fn apply(&mut self, dt: f64, arm: f64, legs: LegTargets) -> (f64, LegTargets) {
        let step = (self.max_rate_rad_s * dt).max(0.0);
        let to = |from: f64, want: f64| from + (want - from).clamp(-step, step);
        self.arm = to(self.arm, arm);
        for l in 0..4 {
            for k in 0..3 {
                self.legs[l][k] = to(self.legs[l][k], legs[l][k]);
            }
        }
        (self.arm, self.legs)
    }
}

/// The best hand-written plan from the probe, as a starting point for
/// search: a 0.1 s symmetric splay pulse every 2 s with the arm stepped
/// down. Rolls onto its side every time and stops there.
pub const PROBE_BEST: RecoveryParams = RecoveryParams {
    period_s: 2.0,
    push_frac: 0.5,
    ramp_s: 0.10,
    hip_push: [1.05, 0.785],
    hip_rest: [0.0, 0.0],
    thigh_push: 0.0,
    calf_push: -0.5,
    thigh_rest: 0.9,
    calf_rest: -1.8,
    arm_push: -2.3,
    arm_rest: 0.85,
    handoff_up: 0.6,
    finish_hip: 1.05,
    finish_thigh: 0.2,
    finish_calf: -0.5,
    finish_arm: 0.85,
    stand_hip: 0.0,
    stand_thigh: STANCE[1],
    stand_calf: STANCE[2],
    max_rate_rad_s: 1000.0,
    stand_rate_rad_s: 4.0,
};

/// Round 1 of the CEM search in `examples/namiashi_self_righting_search`:
/// trained on 6 conditions (2 sites x 3 friction), 6/6 on those and 10/15
/// over the full grid, against 1/15 for the best hand-written plan.
pub const SEARCHED_V1: RecoveryParams = RecoveryParams {
    period_s: 1.8126687973621372,
    push_frac: 0.6944644628777423,
    ramp_s: 0.11522486352708783,
    hip_push: [0.9620204067463585, 0.7546771946344781],
    hip_rest: [-0.3918493133503482, 0.16646240728247277],
    thigh_push: -0.774712182335545,
    calf_push: -0.9340819340746747,
    thigh_rest: -2.0890661028677333,
    calf_rest: -1.9721905662017687,
    arm_push: -2.110021721638257,
    arm_rest: 0.15883274785937398,
    handoff_up: 0.7467059294643767,
    finish_hip: 0.6681007974414791,
    finish_thigh: -0.04819512389931907,
    finish_calf: -0.16762677828496886,
    finish_arm: 0.85,
    stand_hip: 0.0,
    stand_thigh: STANCE[1],
    stand_calf: STANCE[2],
    max_rate_rad_s: 1000.0,
    stand_rate_rad_s: 4.0,
};

/// Round 2: seeded from [`SEARCHED_V1`], trained on all five ring sites at
/// friction 0.30 and 1.00 with 0.70 held out entirely, and searching the
/// standing pose as well. Rights the robot in all fifteen validation
/// conditions with a final trunk +z of 1.000, the five held-out ones
/// included. For comparison: the best hand-written plan managed 1/15.
pub const SEARCHED_V2: RecoveryParams = RecoveryParams {
    period_s: 1.8826887588548362,
    push_frac: 0.7191825882571744,
    ramp_s: 0.14282697950355674,
    hip_push: [0.8474995305882992, 0.6768977968828486],
    hip_rest: [-0.5649890212703218, -0.17080952763284502],
    thigh_push: -1.583573075859423,
    calf_push: -0.8235622205674531,
    thigh_rest: -1.9596440886388187,
    calf_rest: -2.3176836121285898,
    arm_push: -2.0280483426417875,
    arm_rest: 0.1472016387694903,
    handoff_up: 0.6857466418347575,
    finish_hip: 0.6910705694248478,
    finish_thigh: -0.11530437093683112,
    finish_calf: -0.2417548547087312,
    finish_arm: 0.85,
    stand_hip: 0.2382010722464898,
    stand_thigh: 0.767670263584147,
    stand_calf: -2.0872727063104746,
    max_rate_rad_s: 1000.0,
    stand_rate_rad_s: 4.0,
};

/// Round 3: the first searched against the teleop's own drive law, which
/// lifted that path from 8/15 to 13/15 -- but against an evaluator that let
/// it park in whatever crouch was stable (`stand_thigh` sits at its limit
/// and it settled at 0.920) and that scored at 6 s while validating at 8.
/// Under the corrected objective it scores -0.48. Kept as the record of
/// where the numbers came from.
pub const SEARCHED_V3: RecoveryParams = RecoveryParams {
    period_s: 1.6350900029811668,
    push_frac: 0.6826834924476096,
    ramp_s: 0.1708444572306406,
    hip_push: [0.8073424223849209, 0.3597124689100646],
    hip_rest: [-0.7380442201104547, -0.3669480987556565],
    thigh_push: -1.7090575629311473,
    calf_push: -1.2671453644922914,
    thigh_rest: -1.9627428232936004,
    calf_rest: -1.947691045672705,
    arm_push: -1.9269753184094436,
    arm_rest: -0.20938094408744395,
    handoff_up: 0.7377810245183191,
    finish_hip: 0.8124492456070739,
    finish_thigh: -2.1024606633420597,
    finish_calf: -0.649774683660324,
    finish_arm: 0.85,
    stand_hip: 0.7607914621396008,
    stand_thigh: 2.62,
    stand_calf: -0.11086124631030347,
    max_rate_rad_s: 1000.0,
    stand_rate_rad_s: 4.0,
};

/// Round 5, and the one in use. Searched against the teleop's drive law with
/// the search horizon raised to match validation, and scored only after the
/// robot has been handed back to a normal stance. Rights the robot in 14 of
/// the 15 validation conditions with a final trunk +z of 0.999 to 1.000 --
/// actually standing, not parked in a stable crouch.
///
/// The one failure is the red platform at friction 0.70, a condition held
/// out of training. It is a 5 cm plinth barely wider than the robot, and the
/// trajectory travels 0.4 to 0.9 m while righting, so most of what happens
/// there is a fall off the edge.
///
/// For scale: the best hand-written plan righted 1 of 15, and rounds 2 and 3
/// scored 15/15 and 13/15 against evaluators that turned out to be measuring
/// something else (see [`SEARCHED_V2`] and [`SEARCHED_V3`]).
pub const SEARCHED_V4: RecoveryParams = RecoveryParams {
    period_s: 1.190846481137671,
    push_frac: 0.7023220869469812,
    ramp_s: 0.12504386005728735,
    hip_push: [0.8918040573406579, -0.02653376421509206],
    hip_rest: [-0.551193407817909, -0.6406002214040931],
    thigh_push: -1.9066134657923735,
    calf_push: -0.8779970260362576,
    thigh_rest: -2.2664264201522055,
    calf_rest: -2.62,
    arm_push: -1.7250151106095313,
    arm_rest: -0.07762321822155842,
    handoff_up: 0.6595217954031017,
    finish_hip: 0.7013438997158615,
    finish_thigh: -2.11912022264595,
    finish_calf: -1.107535850753163,
    finish_arm: 0.85,
    stand_hip: 0.7591406978703054,
    stand_thigh: 2.593111185239448,
    stand_calf: 1.9850827369063018,
    max_rate_rad_s: 1000.0,
    stand_rate_rad_s: 4.0,
};

/// First unhurried recovery: commanded joint rate held under 1.5 rad/s.
/// Scored 7/15 when it was found -- but that was against the broken
/// attitude signal described on [`TiltEstimator`], which never let the
/// recovery terminate, so what the number really measured was a pose that
/// happened to leave the robot upright. Under a working signal it scores
/// 0/15. Superseded by [`GENTLE_V2`]; kept as the record.
///
/// The reliability is the price: 7/15 against [`SEARCHED_V4`]'s 12/15. That
/// one rocks to build momentum and throws the body over -- its measured
/// joint speed reaches 20 to 31 rad/s while its commands are already limited
/// to 5.6, so the speed is the body being flung rather than the targets
/// moving, and no rate limit alone can take it out.
///
/// Where it fails is specific rather than random: every round plate and
/// every red-platform condition, both of which are raised features the robot
/// has to roll across rather than on.
pub const GENTLE_V1: RecoveryParams = RecoveryParams {
    period_s: 5.0,
    push_frac: 0.8312312855533875,
    ramp_s: 0.41276754458812226,
    hip_push: [1.0274136769414768, -0.3700724244386972],
    hip_rest: [-0.5872458267151148, -0.6538882786363448],
    thigh_push: -2.275770962972069,
    calf_push: -1.2909873761218726,
    thigh_rest: -1.928576928696653,
    calf_rest: -2.62,
    arm_push: -1.6709787462491992,
    arm_rest: 0.1943547238402507,
    handoff_up: 0.7954077396695414,
    finish_hip: 0.9281519870867532,
    finish_thigh: -2.1560857985365356,
    finish_calf: -0.27395062266247017,
    finish_arm: 0.85,
    stand_hip: 0.27908731891393374,
    stand_thigh: 2.300333872967274,
    stand_calf: 1.9745608936502057,
    max_rate_rad_s: 1.5,
    stand_rate_rad_s: 4.0,
};

/// Round 5 of the unhurried search, and the first run with a working
/// attitude signal (see [`TiltEstimator`]).
///
/// Rights the robot in 5 of 15, at an RMS trunk angular speed of 0.7 to 1.4
/// rad/s and RMS joint speed of 0.5 to 0.8 -- inside the gentleness targets,
/// and against 0.310 and 0.107 for a robot merely standing.
///
/// Every one of the fifteen reaches a peak trunk +z of 1.000, so it always
/// turns all the way over; the failures are all falling back down
/// afterwards, and they settle on the same handful of resting attitudes
/// (0.460, 0.474, 0.479) rather than anywhere random. That is the open
/// problem, and it is a catching problem rather than a turning-over one.
///
/// It also does not generalise off the pose it was searched from. Started
/// on its SIDE rather than flat on its back it recovers in 0 of 4 tried,
/// settling at the same 0.47 -- which matters, because being knocked onto a
/// side is the common way to end up down. [`SEARCHED_V4`] manages 3 of those
/// 4. See `namiashi_self_righting_from_its_side`.
///
/// [`GENTLE_V1`]'s 7/15 is not a regression from this: it was measured
/// against the broken attitude signal described on [`TiltEstimator`], where
/// the recovery never terminated and simply held a pose that happened to be
/// upright. Under a working signal it scores 0/15.
pub const GENTLE_V2: RecoveryParams = RecoveryParams {
    period_s: 4.881933523758614,
    push_frac: 0.8401655821182884,
    ramp_s: 0.24826637171419955,
    hip_push: [0.878523494576535, -0.21850959581930013],
    hip_rest: [-0.542827424478892, -0.40349497272430357],
    thigh_push: -2.4491616366093174,
    calf_push: -1.7592486832167968,
    thigh_rest: -1.3312088807102656,
    calf_rest: -2.2446429173056237,
    arm_push: -1.941360711242757,
    arm_rest: 0.2347805767090969,
    handoff_up: 0.7942872132722747,
    finish_hip: 1.0094822213574963,
    finish_thigh: -2.533666829939073,
    finish_calf: 0.2617418451917215,
    finish_arm: 0.85,
    stand_hip: STANCE[0],
    stand_thigh: STANCE[1],
    stand_calf: STANCE[2],
    max_rate_rad_s: 1.4359838992034755,
    stand_rate_rad_s: 4.968451683225109,
};

/// Round 6: the first unhurried search trained on side-lying starts as well
/// as on-its-back ones, and the first with [`RecoveryParams::targets_mirrored`]
/// available so the rock can adapt to which side is down.
///
/// It buys the side fall and pays for it on the back. Head to head over
/// three sites x three starting rolls x three friction values:
///
/// ```text
///                     GENTLE_V2   GENTLE_V3   RECOVERY_FAST
///     on its back        2 of 9      2 of 9          9 of 9
///     side, +90 deg      2 of 9      6 of 9          6 of 9
///     side, -90 deg      0 of 9      3 of 9          8 of 9
///     total             4 of 27    11 of 27        23 of 27
/// ```
///
/// On that grid it is nearly three times the coverage. But on the older
/// five-site ON-ITS-BACK grid -- the one `namiashi_self_righting_teleop_drive`
/// runs -- it manages 2 of 15 where [`GENTLE_V2`] managed 5. The extra ring
/// features are where it loses them, so the trade is real: what this buys in
/// starting attitudes it gives back in terrain.
///
/// It is on `V` because being knocked onto a side is the ordinary way to end
/// up down and 4 of 27 was not a usable number. It does not touch
/// [`SEARCHED_V4`], twice as reliable again at 23 of 27 and what `Shift`+`V`
/// runs; the unhurried one is for when the motion matters more than the odds.
pub const GENTLE_V3: RecoveryParams = RecoveryParams {
    period_s: 6.719088095889823,
    push_frac: 0.674526389495254,
    ramp_s: 0.9178268243245967,
    hip_push: [0.9301197553778584, -0.0681222284078224],
    hip_rest: [-0.785, -0.34188947179592155],
    thigh_push: -2.3997499832536673,
    calf_push: -2.0109146719418627,
    thigh_rest: -1.6667232875810047,
    calf_rest: -1.9850065244400934,
    arm_push: -2.14633135003746,
    arm_rest: 0.16991488960597312,
    handoff_up: 0.718046125057601,
    finish_hip: 0.7464098498232922,
    finish_thigh: -1.145790616178967,
    finish_calf: 0.4601861938981359,
    finish_arm: 0.85,
    stand_hip: STANCE[0],
    stand_thigh: STANCE[1],
    stand_calf: STANCE[2],
    max_rate_rad_s: 1.4320819304001957,
    stand_rate_rad_s: 6.81690861539927,
};

/// What the teleop's `V` key runs: the unhurried recovery.
///
/// The default because a recovery that throws a 3.3 kg robot around is not
/// one to run on hardware, and the numbers behind that judgement are
/// measured rather than aesthetic -- see [`GENTLE_V1`] against
/// [`SEARCHED_V4`]. The cost is reliability, 7 of 15 conditions against 14.
pub const RECOVERY_GENTLE: RecoveryParams = GENTLE_V3;

/// What `Shift`+`V` runs: the reliable one, which gets there by rocking up
/// momentum and throwing the body over.
///
/// Kept because 12/15 against 5/15 is a real difference, and because it is
/// the only one of the two that gets up from a SIDE fall: 3 of 4 side-lying
/// starts against 0 of 4 (`namiashi_self_righting_from_its_side`). Its one
/// failure there ends at -0.701, having rolled onto its back instead of onto
/// its feet.
///
/// The choice between them and the choice
/// between them is a judgement about the robot, not about the search.
pub const RECOVERY_FAST: RecoveryParams = SEARCHED_V4;

/// Number of searchable dimensions; see [`RecoveryParams::to_vec`].
pub const DIM: usize = 19;

/// Per-dimension search bounds, in the order [`RecoveryParams::to_vec`]
/// uses. Every one is a joint limit from the .misa or a duration that a
/// 3.3 kg body can plausibly act on, so no candidate is unphysical.
pub const BOUNDS: [(f64, f64); DIM] = [
    (0.40, 12.00), // period_s
    (0.10, 0.90),  // push_frac
    (0.00, 2.00),  // ramp_s
    HIP_LIMIT_L,   // hip_push[0]
    HIP_LIMIT_R,   // hip_push[1]
    HIP_LIMIT_L,   // hip_rest[0]
    HIP_LIMIT_R,   // hip_rest[1]
    THIGH_LIMIT,   // thigh_push
    CALF_LIMIT,    // calf_push
    THIGH_LIMIT,   // thigh_rest
    CALF_LIMIT,    // calf_rest
    ARM_LIMIT,     // arm_push
    ARM_LIMIT,     // arm_rest
    (-0.20, 0.80), // handoff_up
    HIP_LIMIT_L,   // finish_hip
    THIGH_LIMIT,   // finish_thigh
    CALF_LIMIT,    // finish_calf
    (0.30, 8.00),  // max_rate_rad_s
    (0.50, 8.00),  // stand_rate_rad_s
];

impl RecoveryParams {
    pub fn to_vec(&self) -> [f64; DIM] {
        [
            self.period_s,
            self.push_frac,
            self.ramp_s,
            self.hip_push[0],
            self.hip_push[1],
            self.hip_rest[0],
            self.hip_rest[1],
            self.thigh_push,
            self.calf_push,
            self.thigh_rest,
            self.calf_rest,
            self.arm_push,
            self.arm_rest,
            self.handoff_up,
            self.finish_hip,
            self.finish_thigh,
            self.finish_calf,
            self.max_rate_rad_s,
            self.stand_rate_rad_s,
        ]
    }

    /// Inverse of [`to_vec`], clamping each element into [`BOUNDS`] so a
    /// search that samples outside them still yields a legal trajectory.
    /// `finish_arm` is not searched -- the probe showed the arm contributes
    /// nothing once the body is past vertical -- and is fixed at its high
    /// limit, out of the way.
    ///
    /// [`to_vec`]: RecoveryParams::to_vec
    pub fn from_vec(v: &[f64; DIM]) -> Self {
        let c = |i: usize| v[i].clamp(BOUNDS[i].0, BOUNDS[i].1);
        Self {
            period_s: c(0),
            push_frac: c(1),
            ramp_s: c(2),
            hip_push: [c(3), c(4)],
            hip_rest: [c(5), c(6)],
            thigh_push: c(7),
            calf_push: c(8),
            thigh_rest: c(9),
            calf_rest: c(10),
            arm_push: c(11),
            arm_rest: c(12),
            handoff_up: c(13),
            finish_hip: c(14),
            finish_thigh: c(15),
            finish_calf: c(16),
            finish_arm: ARM_LIMIT.1,
            stand_hip: STANCE[0],
            stand_thigh: STANCE[1],
            stand_calf: STANCE[2],
            max_rate_rad_s: c(17),
            stand_rate_rad_s: c(18),
        }
    }

    /// Joint targets at time `t` into the recovery.
    ///
    /// `up` is the trunk's own +z expressed in world coordinates (+1
    /// upright, -1 on its back) and `g_body_y` is gravity's y component in
    /// the trunk frame, whose sign says which side is underneath. Both are
    /// available from an attitude estimate alone.
    /// `mirror` swaps the rock cycle left-for-right.
    ///
    /// The cycle's hip angles are fixed per side, so without this the robot
    /// always rocks the same way in its own frame -- fine from flat on its
    /// back, where either direction works, and wrong from one of the two
    /// sides, where it means pushing into the ground it is already lying on.
    /// The caller latches the decision once from which side is down rather
    /// than reading it live: on its back `g_body_y` is near zero and its
    /// sign is noise, and a cycle that flips direction mid-rock cancels
    /// itself out.
    pub fn targets_mirrored(
        &self,
        t: f64,
        up: f64,
        g_body_y: f64,
        mirror: bool,
    ) -> (f64, LegTargets) {
        if !mirror {
            return self.targets(t, up, g_body_y);
        }
        let m = Self {
            // Left takes the right's angle negated and vice versa: a mirror
            // in the body's yz plane, clamped back into each side's own
            // limits, which are not symmetric (-0.785..1.05 against
            // -1.05..0.785).
            hip_push: [
                (-self.hip_push[1]).clamp(HIP_LIMIT_L.0, HIP_LIMIT_L.1),
                (-self.hip_push[0]).clamp(HIP_LIMIT_R.0, HIP_LIMIT_R.1),
            ],
            hip_rest: [
                (-self.hip_rest[1]).clamp(HIP_LIMIT_L.0, HIP_LIMIT_L.1),
                (-self.hip_rest[0]).clamp(HIP_LIMIT_R.0, HIP_LIMIT_R.1),
            ],
            ..*self
        };
        m.targets(t, up, g_body_y)
    }

    pub fn targets(&self, t: f64, up: f64, g_body_y: f64) -> (f64, LegTargets) {
        if up >= UPRIGHT_UP {
            return (ARM_LIMIT.1, [STANCE; 4]);
        }
        if up >= self.handoff_up {
            let down_is_left = g_body_y > 0.0;
            let hip = self.finish_hip.abs();
            let push = if down_is_left {
                [hip.min(HIP_LIMIT_L.1), self.finish_thigh, self.finish_calf]
            } else {
                [(-hip).max(HIP_LIMIT_R.0), self.finish_thigh, self.finish_calf]
            };
            // Folded, so the top-side legs cannot prop the body up and stall
            // the roll short of standing.
            let fold = [0.0, 2.5, -2.6];
            let (l, r) = if down_is_left { (push, fold) } else { (fold, push) };
            return (self.finish_arm, [l, r, l, r]);
        }

        // Regime 1: rock. `s` is 0 at the rest pose, 1 at the push pose.
        let period = self.period_s.max(1e-3);
        let phase = t % period;
        let push_end = period * self.push_frac;
        let ramp = self.ramp_s.max(1e-4);
        let s = if phase < push_end {
            (phase / ramp).clamp(0.0, 1.0)
        } else {
            1.0 - ((phase - push_end) / ramp).clamp(0.0, 1.0)
        };
        let mix = |rest: f64, push: f64| rest + (push - rest) * s;
        let hip_l = mix(self.hip_rest[0], self.hip_push[0]);
        let hip_r = mix(self.hip_rest[1], self.hip_push[1]);
        let thigh = mix(self.thigh_rest, self.thigh_push);
        let calf = mix(self.calf_rest, self.calf_push);
        let arm = mix(self.arm_rest, self.arm_push);
        (
            arm,
            [
                [hip_l, thigh, calf],
                [hip_r, thigh, calf],
                [hip_l, thigh, calf],
                [hip_r, thigh, calf],
            ],
        )
    }
}

/// How the joints are driven during an evaluation.
///
/// Both exist because the two differ in ways that could each break the
/// recovery on their own, and only measuring says whether they do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Drive {
    /// The .misa's own Position actuators -- kp 100 / kv 1.2 on the legs,
    /// kp 5 / kv 0.5 on the arm -- with attitude read from the simulator.
    /// What the search optimises against.
    Position,
    /// What the WBC teleop actually does: leg joints in torque mode with a
    /// host-side PD plus gravity compensation, and which-way-is-down from
    /// the accelerometer via [`TiltEstimator`] instead of the simulator's
    /// pose. The arm stays on its Position actuator, as it does there.
    TorqueAhrs { kp: f64, kd: f64, tilt_tau_s: f64 },
}

/// What one run of a recovery trajectory did.
#[derive(Clone, Copy, Debug)]
pub struct Outcome {
    /// Highest trunk +z reached. Separates "nearly" from "wrong actuator".
    pub peak_up: f64,
    /// Trunk +z at the end. This is the one that counts.
    pub final_up: f64,
    /// When it first passed [`UPRIGHT_UP`], or NaN.
    pub t_right_s: f64,
    /// How far it travelled in xy. Large values mean it slid rather than
    /// rolled, which on the ring usually means it fell off something.
    pub travel_m: f64,
    /// Peak trunk angular speed (rad/s).
    pub peak_omega_rad_s: f64,
    /// Peak measured joint speed over the twelve leg joints (rad/s).
    pub peak_joint_rad_s: f64,
    /// RMS trunk angular speed over the run (rad/s).
    pub rms_omega_rad_s: f64,
    /// RMS joint speed over the run and over the twelve leg joints (rad/s).
    pub rms_joint_rad_s: f64,
}

/// Targets for what counts as an unhurried recovery. Not limits the robot
/// would break through -- the .misa allows 33.5 rad/s at the hips -- but the
/// speeds below which the motion behaves like a deliberate one rather than a
/// thrash.
///
/// RMS, not peak, and that distinction was not obvious: a robot doing
/// nothing but standing still on flat ground measures a PEAK of 3.28 rad/s
/// of trunk angular speed and 6.49 rad/s of joint speed, all of it stiff-PD
/// ringing and contact noise. A peak-based target of 3 rad/s was therefore
/// below the standing baseline -- unreachable by construction, and
/// measuring the controller's numerics rather than the motion. In RMS the
/// same standing robot measures 0.310 rad/s of trunk angular speed and
/// 0.107 rad/s of joint speed, so these targets sit about 3x and 14x above
/// the noise floor -- reachable, and far below the 2.6 to 4.7 and 4.2 to 6.5
/// the first searched trajectory produced.
pub const GENTLE_OMEGA_RAD_S: f64 = 1.0;
pub const GENTLE_JOINT_RAD_S: f64 = 1.5;
pub const GENTLE_TRAVEL_M: f64 = 0.25;

impl Outcome {
    pub fn righted(&self) -> bool {
        self.final_up > 0.9
    }

    /// A single number to search on. Mostly the final attitude, with a
    /// smaller credit for the peak so that a trajectory which gets partway
    /// and falls back still scores above one that never moves -- without
    /// that gradient the search sees a flat landscape of failures.
    pub fn score(&self) -> f64 {
        self.final_up + 0.3 * self.peak_up + if self.righted() { 0.5 } else { 0.0 }
    }

    /// How far past the gentleness targets this run went, as a penalty to
    /// subtract from [`score`].
    ///
    /// Hinged, so staying under a target buys nothing and there is no
    /// pressure to creep toward zero motion, and capped at 1.5 so that
    /// righting the robot violently still scores above never righting it at
    /// all (1.8 - 1.5 against -1.3). Without the cap the search can find
    /// that lying still is the tidiest option.
    ///
    /// [`score`]: Outcome::score
    pub fn roughness(&self) -> f64 {
        let over = |v: f64, target: f64| (v / target - 1.0).max(0.0);
        (0.6 * over(self.rms_omega_rad_s, GENTLE_OMEGA_RAD_S)
            + 0.6 * over(self.rms_joint_rad_s, GENTLE_JOINT_RAD_S)
            + 0.6 * over(self.travel_m, GENTLE_TRAVEL_M))
        .min(1.5)
    }
}

#[cfg(feature = "mujoco")]
mod sim {
    use super::*;
    use articara::mjcf::MjcfExportOptions;
    use crate::ring::KawasakiRingCfg;
    use articara::mujoco_sim::MujocoSim;
    use articara::robot::RobotModel;

    /// Drop the robot on its back at `xy` on the ring and run `params`.
    ///
    /// Deterministic: MuJoCo's forward dynamics and this trajectory contain
    /// no RNG, so one evaluation per candidate is enough and a search need
    /// not average over seeds.
    pub fn evaluate(
        misa: &std::path::Path,
        ring: &KawasakiRingCfg,
        xy: (f64, f64),
        mu: f64,
        params: &RecoveryParams,
        horizon_s: f64,
    ) -> Outcome {
        evaluate_with(misa, ring, xy, mu, params, horizon_s, Drive::Position)
    }

    /// One row of a replay trace: time, root pose as `[x, y, z, qw, qx, qy,
    /// qz]`, the twelve leg joints in FL/FR/RL/RR order followed by the arm,
    /// then two diagnostics -- the trunk +z the CONTROLLER believes (which
    /// under `Drive::TorqueAhrs` is the IMU filter's, not the truth the rest
    /// of the row carries) and whether the recovery has handed back to a
    /// normal stance.
    pub type TraceRow = (f64, [f64; 7], [f64; 13], f64, bool);

    pub fn evaluate_with(
        misa: &std::path::Path,
        ring: &KawasakiRingCfg,
        xy: (f64, f64),
        mu: f64,
        params: &RecoveryParams,
        horizon_s: f64,
        drive: Drive,
    ) -> Outcome {
        evaluate_traced(misa, ring, xy, mu, params, horizon_s, drive, None)
    }

    /// Same, plus a replay trace sampled every `trace.1` seconds. Kept as one
    /// function rather than two so a rendered video cannot drift from what
    /// the search and the tests measure.
    #[allow(clippy::too_many_arguments)]
    pub fn evaluate_traced(
        misa: &std::path::Path,
        ring: &KawasakiRingCfg,
        xy: (f64, f64),
        mu: f64,
        params: &RecoveryParams,
        horizon_s: f64,
        drive: Drive,
        mut trace: Option<(&mut Vec<TraceRow>, f64)>,
    ) -> Outcome {
        evaluate_rolled(
            misa,
            ring,
            xy,
            mu,
            params,
            horizon_s,
            drive,
            trace.as_mut().map(|(v, e)| (&mut **v, *e)),
            std::f64::consts::PI,
        )
    }

    /// The full-control entry point: everything the others take, plus the
    /// roll the robot starts at. `PI` is flat on its back; `PI/2` is on its
    /// side, which is where a robot that has been knocked over usually ends
    /// up and is a materially easier or harder start depending on which way
    /// it is leaning.
    #[allow(clippy::too_many_arguments)]
    pub fn evaluate_rolled(
        misa: &std::path::Path,
        ring: &KawasakiRingCfg,
        xy: (f64, f64),
        mu: f64,
        params: &RecoveryParams,
        horizon_s: f64,
        drive: Drive,
        mut trace: Option<(&mut Vec<TraceRow>, f64)>,
        start_roll_rad: f64,
    ) -> Outcome {
        let mut robot = RobotModel::from_misa(misa).expect("load robot");
        if let Drive::TorqueAhrs { .. } = drive {
            // Set before the sim is built, so the exported MJCF and the
            // control law agree -- the same ordering `run_wbc_sim` uses.
            for j in robot.joints.iter_mut() {
                if j.name.ends_with("_hip_joint")
                    || j.name.ends_with("_thigh_joint")
                    || j.name.ends_with("_calf_joint")
                {
                    j.actuator_mode = articara::rbd::model::ActuatorMode::Torque;
                }
            }
        }
        let opts = MjcfExportOptions {
            base_xy: Some(xy),
            extra_asset_xml: Some(ring.asset_xml("kawasaki")),
            extra_worldbody_xml: Some(ring.worldbody_xml("kawasaki")),
            add_actuators: true,
            ..MjcfExportOptions::default()
        };
        let mut sim = MujocoSim::new(&robot, opts).expect("MujocoSim::new");
        sim.set_hfield_data("kawasaki", &ring.heights()).expect("fill hfield");
        sim.set_slide_friction_all(mu);
        if let Drive::TorqueAhrs { .. } = drive {
            sim.set_gravity_compensation(true);
        }
        let dt = sim.timestep();
        let mut tilt = match drive {
            Drive::TorqueAhrs { tilt_tau_s, .. } => Some(TiltEstimator::new(tilt_tau_s)),
            Drive::Position => None,
        };
        let root = robot.root_link.clone();

        sim.respawn_rolled(&mut robot, 0.03, start_roll_rad);
        // Settle the drop before driving anything, so the trajectory is
        // scored on its own motion and not on the landing.
        for _ in 0..(0.5 / dt) as u32 {
            sim.step(&mut robot, dt, true);
        }
        let start = sim.body_world_position(&root).unwrap();

        let arm_ji = robot.joint_map.get("arm_pitch_joint").copied();
        let leg_ji: Vec<[Option<usize>; 3]> = ["FL", "FR", "RL", "RR"]
            .iter()
            .map(|p| {
                [
                    robot.joint_map.get(&format!("{p}_hip_joint")).copied(),
                    robot.joint_map.get(&format!("{p}_thigh_joint")).copied(),
                    robot.joint_map.get(&format!("{p}_calf_joint")).copied(),
                ]
            })
            .collect();

        // Seconds of continuous upright attitude before the recovery is
        // declared over and the robot is handed back to a normal stance --
        // exactly what `run_wbc_sim` does on the `V` key. Scoring without
        // this let a trajectory win by parking in whatever crouch happened
        // to be stable: round 3 drove `stand_thigh` to its limit and settled
        // at a trunk +z of 0.920, which passes a 0.9 threshold while being a
        // robot sitting down.
        const HOLD_S: f64 = 1.0;
        let mut upright_s = 0.0_f64;
        let mut handed_back = false;
        // Which side is down, decided once and held. `None` until the tilt
        // estimate has something to say; flat on its back it never will, and
        // `false` is then as good an answer as any.
        let mut mirror: Option<bool> = None;

        // Seeded from where the joints actually are, so the recovery starts
        // by continuing the current pose rather than jumping to its own.
        let mut spawn_legs = [[0.0_f64; 3]; 4];
        for (slot, leg) in leg_ji.iter().enumerate() {
            for (k, ji) in leg.iter().enumerate() {
                if let Some(ji) = ji {
                    spawn_legs[slot][k] = sim
                        .joint_q_qd(&robot.joints[*ji].name)
                        .map(|(q, _)| q)
                        .unwrap_or(STANCE[k]);
                }
            }
        }
        let spawn_arm = arm_ji
            .and_then(|ji| sim.joint_q_qd(&robot.joints[ji].name))
            .map(|(q, _)| q)
            .unwrap_or(ARM_LIMIT.1);
        let mut slew = Slew::new(params.max_rate_rad_s, spawn_arm, spawn_legs);

        let (mut peak_omega, mut peak_qd) = (0.0_f64, 0.0_f64);
        let (mut sum_omega2, mut sum_qd2, mut n_samp) = (0.0_f64, 0.0_f64, 0_u64);
        let (mut peak_up, mut t_right_s, mut t) = (-1.0_f64, f64::NAN, 0.0);
        while t < horizon_s {
            // Attitude, from whichever source this drive is entitled to.
            let (up, g_body_y) = match &mut tilt {
                Some(f) => {
                    if let Some(imu) = sim.imu_readings(&robot).first() {
                        f.update(imu.accel, dt);
                    }
                    (f.up(), f.g_body_y())
                }
                None => {
                    let r = sim.body_world_orientation(&root).unwrap();
                    (
                        (r * nalgebra::Vector3::z()).z,
                        (r.inverse() * nalgebra::Vector3::new(0.0, 0.0, -1.0)).y,
                    )
                }
            };
            upright_s = if up > UPRIGHT_UP { upright_s + dt } else { 0.0 };
            if !handed_back && upright_s > HOLD_S {
                handed_back = true;
                slew.max_rate_rad_s = params.stand_rate_rad_s;
            }
            if mirror.is_none() && g_body_y.abs() > 0.35 {
                mirror = Some(g_body_y < 0.0);
            }
            let want = if handed_back {
                (ARM_LIMIT.1, [STANCE; 4])
            } else {
                params.targets_mirrored(t, up, g_body_y, mirror.unwrap_or(false))
            };
            let (arm, legs) = slew.apply(dt, want.0, want.1);
            // The arm is on its Position actuator either way.
            if let Some(ji) = arm_ji {
                sim.set_position_target(ji, arm);
            }
            match drive {
                Drive::Position => {
                    for (leg, targets) in leg_ji.iter().zip(legs.iter()) {
                        for (ji, &q) in leg.iter().zip(targets.iter()) {
                            if let Some(ji) = ji {
                                sim.set_position_target(*ji, q);
                            }
                        }
                    }
                }
                Drive::TorqueAhrs { kp, kd, .. } => {
                    let grav = sim.gravity_torques(&robot);
                    for (leg, targets) in leg_ji.iter().zip(legs.iter()) {
                        for (ji, &q_star) in leg.iter().zip(targets.iter()) {
                            let Some(ji) = ji else { continue };
                            let (q, qd) = sim
                                .joint_q_qd(&robot.joints[*ji].name)
                                .unwrap_or((q_star, 0.0));
                            let tau = kp * (q_star - q) - kd * qd
                                + grav.get(*ji).copied().unwrap_or(0.0);
                            sim.set_torque_target(*ji, tau);
                        }
                    }
                }
            }
            sim.step(&mut robot, dt, true);
            t += dt;
            if let Some(w) = sim.body_world_angular_velocity(&root) {
                let w2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2];
                peak_omega = peak_omega.max(w2.sqrt());
                sum_omega2 += w2;
            }
            let mut qd2 = 0.0;
            for leg in leg_ji.iter() {
                for ji in leg.iter().flatten() {
                    if let Some((_, qd)) = sim.joint_q_qd(&robot.joints[*ji].name) {
                        peak_qd = peak_qd.max(qd.abs());
                        qd2 += qd * qd;
                    }
                }
            }
            sum_qd2 += qd2 / 12.0;
            n_samp += 1;
            if let Some((rows, every)) = trace.as_mut() {
                if rows.last().is_none_or(|(t_last, ..)| t - *t_last >= *every - 1e-9) {
                    let p = sim.body_world_position(&root).unwrap_or([0.0; 3]);
                    let q = sim
                        .body_world_orientation(&root)
                        .unwrap_or_else(nalgebra::UnitQuaternion::identity);
                    let mut qs = [0.0_f64; 13];
                    for (slot, leg) in leg_ji.iter().enumerate() {
                        for (k, ji) in leg.iter().enumerate() {
                            if let Some(ji) = ji {
                                qs[slot * 3 + k] = sim
                                    .joint_q_qd(&robot.joints[*ji].name)
                                    .map(|(q, _)| q)
                                    .unwrap_or(0.0);
                            }
                        }
                    }
                    if let Some(ji) = arm_ji {
                        qs[12] = sim
                            .joint_q_qd(&robot.joints[ji].name)
                            .map(|(q, _)| q)
                            .unwrap_or(0.0);
                    }
                    rows.push((t, [p[0], p[1], p[2], q.w, q.i, q.j, q.k], qs, up, handed_back));
                }
            }
            let up = (sim.body_world_orientation(&root).unwrap() * nalgebra::Vector3::z()).z;
            peak_up = peak_up.max(up);
            if up > UPRIGHT_UP && t_right_s.is_nan() {
                t_right_s = t;
            }
        }
        let final_up = (sim.body_world_orientation(&root).unwrap() * nalgebra::Vector3::z()).z;
        let end = sim.body_world_position(&root).unwrap();
        Outcome {
            peak_up,
            final_up,
            t_right_s,
            travel_m: ((end[0] - start[0]).powi(2) + (end[1] - start[1]).powi(2)).sqrt(),
            peak_omega_rad_s: peak_omega,
            peak_joint_rad_s: peak_qd,
            rms_omega_rad_s: (sum_omega2 / n_samp.max(1) as f64).sqrt(),
            rms_joint_rad_s: (sum_qd2 / n_samp.max(1) as f64).sqrt(),
        }
    }
}

#[cfg(feature = "mujoco")]
pub use sim::{evaluate, evaluate_rolled, evaluate_traced, evaluate_with, TraceRow};
