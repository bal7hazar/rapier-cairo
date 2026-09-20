//! Deterministic, diff-friendly JSON writer: a node is written on one line when it fits in
//! `WIDTH` columns, otherwise one child per line. Floats use serde_json's shortest round-trip
//! representation, object keys keep their insertion order.

use serde_json::Value;

const WIDTH: usize = 110;
const INDENT: usize = 2;

pub fn render(value: &Value) -> String {
    let mut out = String::new();
    write(value, 0, &mut out);
    out.push('\n');
    out
}

fn inline(value: &Value) -> String {
    match value {
        Value::Array(items) => {
            let parts: Vec<String> = items.iter().map(inline).collect();
            format!("[{}]", parts.join(", "))
        }
        Value::Object(map) => {
            if map.is_empty() {
                return "{}".to_string();
            }
            let parts: Vec<String> = map
                .iter()
                .map(|(k, v)| format!("{}: {}", Value::String(k.clone()), inline(v)))
                .collect();
            format!("{{ {} }}", parts.join(", "))
        }
        scalar => scalar.to_string(),
    }
}

fn write(value: &Value, depth: usize, out: &mut String) {
    let flat = inline(value);
    let current_line = out.len() - out.rfind('\n').map_or(0, |i| i + 1);
    let fits = current_line + flat.len() < WIDTH;
    match value {
        Value::Array(items) if !fits && !items.is_empty() => {
            out.push_str("[\n");
            for (i, item) in items.iter().enumerate() {
                pad(depth + 1, out);
                write(item, depth + 1, out);
                out.push_str(if i + 1 == items.len() { "\n" } else { ",\n" });
            }
            pad(depth, out);
            out.push(']');
        }
        Value::Object(map) if !fits && !map.is_empty() => {
            out.push_str("{\n");
            for (i, (key, item)) in map.iter().enumerate() {
                pad(depth + 1, out);
                out.push_str(&Value::String(key.clone()).to_string());
                out.push_str(": ");
                write(item, depth + 1, out);
                out.push_str(if i + 1 == map.len() { "\n" } else { ",\n" });
            }
            pad(depth, out);
            out.push('}');
        }
        _ => out.push_str(&flat),
    }
}

fn pad(depth: usize, out: &mut String) {
    for _ in 0..depth * INDENT {
        out.push(' ');
    }
}
