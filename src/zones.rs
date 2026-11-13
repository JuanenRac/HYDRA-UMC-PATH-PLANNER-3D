// HYDRA-UMC-PATH-PLANNER-3D - src/zones.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
//
// The danger zones of a zones file of HYDRA-UMC-SAFETY-ZONES, read as box obstacles. A scenario names the file in `zones_file`; every zone of the
// level `danger` becomes a box the plan and the independent validator keep away from. The zones of the level `warning` are not used yet (they
// would be a cost, not an obstacle). A zone that is not a box (an oriented box, a capsule, a cylinder) cannot be a `BoxObstacle`, so it is
// refused by name instead of being approximated: a danger zone that is quietly made smaller would be unsafe.

use crate::geometry::Vec3;
use crate::obstacle::BoxObstacle;
use serde_json::Value;

/// The boxes of the danger zones in the text of a zones file.
pub fn danger_boxes(text: &str) -> Result<Vec<BoxObstacle>, String> {
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
        if shape != "aabb" {
            return Err(format!(
                "danger zone {name} has the shape {shape}, which the planner cannot take as a box"
            ));
        }
        let corner = |key: &str| -> Result<Vec3, String> {
            let value = zone
                .get(key)
                .ok_or_else(|| format!("danger zone {name} has no {key} corner"))?;
            serde_json::from_value::<Vec3>(value.clone())
                .map_err(|e| format!("danger zone {name}: {key} is not a point: {e}"))
        };
        boxes.push(BoxObstacle {
            min: corner("min")?,
            max: corner("max")?,
        });
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
        assert_eq!(boxes[0].max.x, 1.0);
    }

    #[test]
    fn a_zone_that_is_not_a_box_or_has_no_level_is_refused_by_name() {
        let capsule = r#"{"zones":[{"id":"c1","level":"danger","shape":"capsule"}]}"#;
        let error = danger_boxes(capsule).unwrap_err();
        assert!(error.contains("c1") && error.contains("capsule"), "{error}");
        let nameless =
            r#"{"zones":[{"id":"z","min":{"x":0,"y":0,"z":0},"max":{"x":1,"y":1,"z":1}}]}"#;
        assert!(danger_boxes(nameless).unwrap_err().contains("level"));
        // a warning zone of another shape is not used, so it is not refused
        let warning = r#"{"zones":[{"id":"w","level":"warning","shape":"capsule"}]}"#;
        assert!(danger_boxes(warning).unwrap().is_empty());
    }

    #[test]
    fn a_file_that_is_not_a_zones_file_is_refused() {
        assert!(danger_boxes("{not json").unwrap_err().contains("not JSON"));
        assert!(danger_boxes("{}").unwrap_err().contains("no zones list"));
        let missing = r#"{"zones":[{"id":"d","level":"danger","min":{"x":0,"y":0,"z":0}}]}"#;
        assert!(danger_boxes(missing).unwrap_err().contains("max"));
    }
}
