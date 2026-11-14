// HYDRA-UMC-PATH-PLANNER-3D - src/zones.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// The danger zones of a zones file of HYDRA-UMC-SAFETY-ZONES, read as solid obstacles. A scenario names the file in `zones_file`; every zone of the
// level `danger` becomes a solid the plan and the independent validator keep away from: an aligned box (`min`, `max`), an oriented box (`obb`:
// `center`, `half_extents`, `rotation_deg`) or a capsule (`capsule`: `start`, `end`, `radius`). The zones of the level `warning` are not used yet (they
// would be a cost, not an obstacle). A zone of another shape (a cylinder) cannot be taken as one of these, so it is refused by name instead of being
// approximated: a danger zone that is quietly made smaller would be unsafe.

use crate::geometry::Vec3;
use crate::obstacle::{BoxObstacle, CapsuleObstacle, ObbObstacle, Rotation, Solid};
use serde_json::Value;

/// The solids of the danger zones in the text of a zones file.
pub fn danger_boxes(text: &str) -> Result<Vec<Solid>, String> {
    let document: Value =
        serde_json::from_str(text).map_err(|e| format!("the zones file is not JSON: {e}"))?;
    let zones = document
        .get("zones")
        .and_then(Value::as_array)
        .ok_or_else(|| "the zones file has no zones list".to_string())?;
    let mut boxes = Vec::new();
    for (index, zone) in zones.iter().enumerate() {
        let name = zone
            .get("id")
            .and_then(Value::as_str)
            .map_or_else(|| format!("#{index}"), str::to_string);
        match zone.get("level").and_then(Value::as_str) {
            Some("warning") => continue,
            Some("danger") => {}
            other => {
                return Err(format!(
                    "zone {name} has the level {other:?}, expected danger or warning"
                ))
            }
        }
        let shape = zone.get("shape").and_then(Value::as_str).unwrap_or("aabb");
        let point = |key: &str| -> Result<Vec3, String> {
            let value = zone
                .get(key)
                .ok_or_else(|| format!("danger zone {name} has no {key}"))?;
            serde_json::from_value::<Vec3>(value.clone())
                .map_err(|e| format!("danger zone {name}: {key} is not a point: {e}"))
        };
        let solid = match shape {
            "aabb" => Solid::Aabb(BoxObstacle {
                min: point("min")?,
                max: point("max")?,
            }),
            "obb" => {
                let rotation_deg = match zone.get("rotation_deg") {
                    None => Rotation::default(),
                    Some(value) => serde_json::from_value::<Rotation>(value.clone()).map_err(
                        |e| format!("danger zone {name}: rotation_deg is not roll, pitch and yaw: {e}"),
                    )?,
                };
                Solid::Obb(ObbObstacle {
                    center: point("center")?,
                    half_extents: point("half_extents")?,
                    rotation_deg,
                })
            }
            "capsule" => {
                let radius = zone
                    .get("radius")
                    .and_then(Value::as_f64)
                    .ok_or_else(|| format!("danger zone {name} has no radius"))?;
                Solid::Capsule(CapsuleObstacle {
                    start: point("start")?,
                    end: point("end")?,
                    radius,
                })
            }
            other => {
                return Err(format!(
                    "danger zone {name} has the shape {other}, which the planner cannot take as a solid (it takes aabb, obb and capsule)"
                ))
            }
        };
        boxes.push(solid);
    }
    Ok(boxes)
}

#[cfg(test)]
mod tests {
    use super::*;

    const ZONES: &str = r#"{"zones":[
        {"id":"warn1","level":"warning","min":{"x":0,"y":0,"z":0},"max":{"x":5,"y":5,"z":5}},
        {"id":"danger1","level":"danger","min":{"x":0,"y":0,"z":0},"max":{"x":1,"y":1,"z":1}}]}"#;

    #[test]
    fn only_the_danger_zones_become_boxes() {
        let boxes = danger_boxes(ZONES).unwrap();
        assert_eq!(boxes.len(), 1);
        match boxes[0] {
            Solid::Aabb(b) => assert_eq!(b.max.x, 1.0),
            other => panic!("an aligned box expected, got {other:?}"),
        }
    }

    #[test]
    fn a_zone_that_is_not_a_box_or_has_no_level_is_refused_by_name() {
        let cylinder = r#"{"zones":[{"id":"c1","level":"danger","shape":"cylinder"}]}"#;
        let error = danger_boxes(cylinder).unwrap_err();
        assert!(
            error.contains("c1") && error.contains("cylinder"),
            "{error}"
        );
        let nameless =
            r#"{"zones":[{"id":"z","min":{"x":0,"y":0,"z":0},"max":{"x":1,"y":1,"z":1}}]}"#;
        assert!(danger_boxes(nameless).unwrap_err().contains("level"));
        // a warning zone of another shape is not used, so it is not refused
        let warning = r#"{"zones":[{"id":"w","level":"warning","shape":"cylinder"}]}"#;
        assert!(danger_boxes(warning).unwrap().is_empty());
    }

    #[test]
    fn an_oriented_box_and_a_capsule_zone_become_solids() {
        let text = r#"{"zones":[
            {"id":"fence","level":"danger","shape":"obb","center":{"x":1,"y":2,"z":3},"half_extents":{"x":0.5,"y":2,"z":1},"rotation_deg":{"yaw":90}},
            {"id":"flat","level":"danger","shape":"obb","center":{"x":0,"y":0,"z":0},"half_extents":{"x":1,"y":1,"z":1}},
            {"id":"pole","level":"danger","shape":"capsule","start":{"x":0,"y":0,"z":0},"end":{"x":0,"y":0,"z":2},"radius":0.25}]}"#;
        let solids = danger_boxes(text).unwrap();
        assert_eq!(solids.len(), 3);
        match solids[0] {
            Solid::Obb(b) => {
                assert_eq!(b.center.y, 2.0);
                assert_eq!(b.half_extents.y, 2.0);
                assert_eq!(b.rotation_deg.yaw, 90.0);
                assert_eq!(b.rotation_deg.roll, 0.0, "a missing angle is 0");
            }
            other => panic!("an oriented box expected, got {other:?}"),
        }
        match solids[1] {
            Solid::Obb(b) => assert_eq!(b.rotation_deg.yaw, 0.0, "no rotation is no rotation"),
            other => panic!("an oriented box expected, got {other:?}"),
        }
        match solids[2] {
            Solid::Capsule(b) => {
                assert_eq!(b.end.z, 2.0);
                assert_eq!(b.radius, 0.25);
            }
            other => panic!("a capsule expected, got {other:?}"),
        }
    }

    #[test]
    fn a_shape_zone_with_a_missing_or_wrong_field_is_refused_by_name() {
        let no_half = r#"{"zones":[{"id":"o1","level":"danger","shape":"obb","center":{"x":0,"y":0,"z":0}}]}"#;
        let error = danger_boxes(no_half).unwrap_err();
        assert!(
            error.contains("o1") && error.contains("half_extents"),
            "{error}"
        );
        let no_radius = r#"{"zones":[{"id":"k1","level":"danger","shape":"capsule","start":{"x":0,"y":0,"z":0},"end":{"x":1,"y":0,"z":0}}]}"#;
        let error = danger_boxes(no_radius).unwrap_err();
        assert!(error.contains("k1") && error.contains("radius"), "{error}");
        let bad_angle = r#"{"zones":[{"id":"r1","level":"danger","shape":"obb","center":{"x":0,"y":0,"z":0},"half_extents":{"x":1,"y":1,"z":1},"rotation_deg":"left"}]}"#;
        let error = danger_boxes(bad_angle).unwrap_err();
        assert!(
            error.contains("r1") && error.contains("rotation_deg"),
            "{error}"
        );
    }

    #[test]
    fn a_file_that_is_not_a_zones_file_is_refused() {
        assert!(danger_boxes("{not json").unwrap_err().contains("not JSON"));
        assert!(danger_boxes("{}").unwrap_err().contains("no zones list"));
        let missing = r#"{"zones":[{"id":"d","level":"danger","min":{"x":0,"y":0,"z":0}}]}"#;
        assert!(danger_boxes(missing).unwrap_err().contains("max"));
    }
}
