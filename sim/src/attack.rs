//! Tipping the opponent over with the arm.
//!
//! Three phases, in the order the move is actually performed:
//!
//! 1. **Set.** The front of the body crouches and the arm swings down. The
//!    arm's pitch axis puts its tip 0.217 m below its mount at the +0.85
//!    limit (measured), and the mount sits 0.066 m above the trunk, so from
//!    a normal 0.235 m stance the tip lands at about 0.084 m -- just under a
//!    crouched opponent's belly at 0.089 m. "Just under" is not a margin,
//!    which is what the crouch buys.
//! 2. **Creep.** A slow forward command slides the arm underneath. The gait
//!    controller does the walking; nothing here reimplements it.
//! 3. **Lift.** The front legs extend back to the standing pose while the
//!    arm swings up, so the body's rise and the arm's rotation add at the
//!    tip rather than one waiting for the other.
//!
//! The front crouch goes through the per-leg `nominal_foot_body.z` the
//! harness already carries for terrain levelling and trunk height, so the
//! WBC and the gait keep running throughout and the robot can still be
//! walked out of a failed attempt. Driving the front joints directly would
//! have meant taking the legs away from the controller that is holding the
//! robot up.

/// One tick of the attack: what to ask of the front legs, the arm and the
/// gait.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttackCmd {
    /// Added to the FRONT legs' `nominal_foot_body.z`. Positive crouches --
    /// the nominal measures the foot toward the body.
    pub front_nom_off_m: f64,
    /// Arm joint target (rad). +0.85 is fully down, -2.3 fully up.
    pub arm_rad: f64,
    /// Forward velocity command handed to the gait (m/s).
    pub cmd_vx: f64,
}

/// Timings and amplitudes of the move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttackParams {
    /// Seconds to reach the crouch and get the arm down.
    pub set_s: f64,
    /// How much lower the front nominal goes during the set (m).
    pub front_drop_m: f64,
    /// Arm angle for the set and the creep (rad).
    pub arm_down: f64,
    /// Seconds of creeping forward with the arm low.
    pub creep_s: f64,
    /// Forward command during the creep (m/s). Slow: the arm is a 0.29 m
    /// lever at knee height and walking into the opponent at speed pushes it
    /// away rather than under.
    pub creep_vx: f64,
    /// Seconds to extend the front and swing the arm up.
    pub lift_s: f64,
    /// Front nominal offset at the top of the lift (m). Negative extends the
    /// front legs past the standing nominal, which is what actually raises
    /// the arm's mount.
    pub front_rise_m: f64,
    /// Arm angle at the top of the lift (rad).
    pub arm_up: f64,
    /// Seconds to hold at the top before handing back.
    pub hold_s: f64,
}

impl AttackParams {
    /// Usable in a `const` initialiser, which `Default` is not.
    pub const DEFAULT: Self = Self {
        set_s: 1.2,
        front_drop_m: 0.055,
        arm_down: 0.85,
        creep_s: 2.5,
        creep_vx: 0.10,
        lift_s: 0.9,
        front_rise_m: -0.030,
        arm_up: -2.3,
        hold_s: 1.0,
    };
}

impl Default for AttackParams {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl AttackParams {
    pub fn total_s(&self) -> f64 {
        self.set_s + self.creep_s + self.lift_s + self.hold_s
    }

    /// The command `t` seconds into the move.
    pub fn at(&self, t: f64) -> AttackCmd {
        let (set, creep, lift) = (self.set_s.max(1e-6), self.creep_s, self.lift_s.max(1e-6));
        if t < set {
            let s = (t / set).clamp(0.0, 1.0);
            return AttackCmd {
                front_nom_off_m: self.front_drop_m * s,
                arm_rad: self.arm_down * s,
                cmd_vx: 0.0,
            };
        }
        if t < set + creep {
            return AttackCmd {
                front_nom_off_m: self.front_drop_m,
                arm_rad: self.arm_down,
                cmd_vx: self.creep_vx,
            };
        }
        let s = ((t - set - creep) / lift).clamp(0.0, 1.0);
        AttackCmd {
            // Stop creeping the moment the lift starts: the opponent is
            // already on the arm and continuing to walk into it just drags
            // it along the floor.
            front_nom_off_m: self.front_drop_m + (self.front_rise_m - self.front_drop_m) * s,
            arm_rad: self.arm_down + (self.arm_up - self.arm_down) * s,
            cmd_vx: 0.0,
        }
    }
}
