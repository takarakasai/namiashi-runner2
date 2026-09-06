//! `models/namiashi/` の 2 つのファイルがずれていないかを見張る。
//!
//! `models/namiashi/` は [`namiashi_description`] の submodule で、同じ情報が 2 箇所にある:
//!
//! | | |
//! |---|---|
//! | `namiashi.misa` | misarta のネイティブ形式。**アプリが実際に読むのはこちら** |
//! | `urdf/namiashi.misarta.toml` | URDF を補うサイドカー。ROS 側や再生成の種 |
//!
//! スキーマは別物（`.misa` は link / joint / material まで持つ自己完結形式、
//! サイドカーは持たない）だが、**pose / sequence / actuator / collision_pair は
//! 両方に書かれている**。片方だけ直すとずれる。
//!
//! 実際にずれていた。2026-08-21 に突き合わせたところ:
//!
//! - サイドカーに `arm_raise` / `arm_lower` / `arm_home` と `greeting` が無かった
//! - `constrain` / `extend` / `constrain_2` の **FL/RL の hip にだけ 0.005 rad**
//!   が乗っていた（`.misa` は 0.0）
//!
//! どちらも「気付ける仕掛けが無い」ことが原因で放置されていた。ここで落とす。
//!
//! [`namiashi_description`]: https://github.com/takarakasai/namiashi_description

use std::collections::BTreeMap;
use std::path::PathBuf;

/// 角度の許容差 [rad]。
///
/// 完全一致は要求しない。articara が float32 で往復させるため、同じ値でも
/// 1e-7 程度の差が残る（実測 4.77e-08）。一方で拾いたいのは上記 0.005 rad の
/// ような**人が入れた値**なので、その 100 倍下に閾値を置く。
const ANGLE_TOL: f64 = 1e-6;

fn models_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("models/namiashi")
}

fn load(rel: &str) -> toml::Value {
    let path = models_dir().join(rel);
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{} を読めません: {e}\n\
             models/namiashi は namiashi_description の submodule です。空なら\n\
                 git submodule update --init\n\
             を実行してください。",
            path.display()
        )
    });
    toml::from_str(&text)
        .unwrap_or_else(|e| panic!("{} の TOML が壊れています: {e}", path.display()))
}

/// `[[[名前]]]` の配列を名前で引ける形にする。順序は別途 [`names`] で見る。
fn by_name(root: &toml::Value, key: &str) -> BTreeMap<String, toml::Value> {
    root.get(key)
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|v| {
                    let name = v.get("name")?.as_str()?.to_string();
                    Some((name, v.clone()))
                })
                .collect()
        })
        .unwrap_or_default()
}

fn names(root: &toml::Value, key: &str) -> Vec<String> {
    root.get(key)
        .and_then(|v| v.as_array())
        .map(|a| {
            a.iter()
                .filter_map(|v| Some(v.get("name")?.as_str()?.to_string()))
                .collect()
        })
        .unwrap_or_default()
}

fn count(root: &toml::Value, key: &str) -> usize {
    root.get(key)
        .and_then(|v| v.as_array())
        .map(|a| a.len())
        .unwrap_or(0)
}

fn angles(pose: &toml::Value) -> BTreeMap<String, f64> {
    pose.get("angles")
        .and_then(|v| v.as_table())
        .map(|t| {
            t.iter()
                .filter_map(|(k, v)| Some((k.clone(), v.as_float()?)))
                .collect()
        })
        .unwrap_or_default()
}

#[test]
fn poses_agree_between_misa_and_sidecar() {
    let misa = load("namiashi.misa");
    let side = load("urdf/namiashi.misarta.toml");

    assert_eq!(
        names(&misa, "pose"),
        names(&side, "pose"),
        "pose の名前か順序がずれています（.misa 側が左、サイドカーが右）"
    );

    let a = by_name(&misa, "pose");
    let b = by_name(&side, "pose");
    for (name, pa) in &a {
        let pb = &b[name];
        let (ka, kb) = (angles(pa), angles(pb));
        let joints: std::collections::BTreeSet<_> = ka.keys().chain(kb.keys()).collect();
        for j in joints {
            let (va, vb) = (
                ka.get(j).copied().unwrap_or(0.0),
                kb.get(j).copied().unwrap_or(0.0),
            );
            assert!(
                (va - vb).abs() <= ANGLE_TOL,
                "pose '{name}' の {j} がずれています: .misa={va} サイドカー={vb} \
                 （差 {:.3e} rad, 許容 {ANGLE_TOL:.0e}）",
                (vb - va).abs()
            );
        }
        for (field, label) in [("duration", "duration"), ("kind", "補間")] {
            assert_eq!(
                pa.get(field),
                pb.get(field),
                "pose '{name}' の {label} がずれています"
            );
        }
    }
}

#[test]
fn sequences_agree_between_misa_and_sidecar() {
    let misa = load("namiashi.misa");
    let side = load("urdf/namiashi.misarta.toml");

    assert_eq!(
        names(&misa, "sequence"),
        names(&side, "sequence"),
        "sequence の名前か順序がずれています"
    );

    let a = by_name(&misa, "sequence");
    let b = by_name(&side, "sequence");
    for (name, sa) in &a {
        assert_eq!(
            sa.get("steps"),
            b[name].get("steps"),
            "sequence '{name}' の steps がずれています"
        );
    }
}

/// actuator と collision_pair は件数だけ見る。
///
/// 中身の突き合わせまではしない。名前で引ける形になっておらず
/// （`joint_name` / リンク名の対）、比較器を書く手間の割に、件数が合っていれば
/// 片方だけ足した / 消したという事故はほぼ拾えるため。
#[test]
fn actuator_and_collision_pair_counts_agree() {
    let misa = load("namiashi.misa");
    let side = load("urdf/namiashi.misarta.toml");
    for key in ["actuator", "collision_pair"] {
        assert_eq!(
            count(&misa, key),
            count(&side, key),
            "{key} の件数がずれています"
        );
    }
}
