// =============================================================================
// HYDRA-UMC-PATH-PLANNER-3D - src/server.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The only network surface of this program, and it is off unless asked for: `serve` starts a small HTTP listener (`tiny_http`, blocking, no async
//! runtime) that answers two read-only routes and nothing else - `GET /health` (liveness, open) and `GET /metrics` (the numbers of the service in the
//! text format of Prometheus, off unless the configuration file opens it, and behind the token when one is set). It plans nothing and takes no scenario: planning stays a command of the command line.
//!
//! By default it listens on the loopback address only. A listener on another address is refused unless a token is set (`--token` or
//! `PATH_PLANNER_TOKEN`) or the operator says on purpose that it may be open (`--allow-anonymous`).

use std::io::Write as _;

use serde_json::json;
use tiny_http::{Header, Method, Request, Response, Server};

use crate::service_metrics::{is_metrics_request, ServiceMetrics};

const NAME: &str = "HYDRA-UMC-PATH-PLANNER-3D";
const VERSION: &str = env!("CARGO_PKG_VERSION");
pub const DEFAULT_ADDR: &str = "127.0.0.1:8117";
pub const TOKEN_ENV: &str = "PATH_PLANNER_TOKEN";
pub const CONFIG_ENV: &str = "PATH_PLANNER_CONFIG";

/// Whether the configuration file opens `GET /metrics`: `{"metrics": {"enabled": true}}`. Off when there is no file, no section or `enabled` is false;
/// an unknown key is an error.
#[allow(dead_code)]
pub fn metrics_enabled(text: &str) -> Result<bool, String> {
    parse_config(text).map(|(metrics, _)| metrics)
}

/// What the `plan` section of the configuration file says about `POST /plan`. The route does not exist (it answers 404) unless `enabled` is true.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlanSettings {
    pub enabled: bool,
    /// the largest request body accepted, in bytes (413 above it)
    pub max_body_bytes: usize,
    /// the longest a search may run, in milliseconds; a scenario cannot ask for more
    pub max_duration_ms: u64,
}

impl Default for PlanSettings {
    fn default() -> Self {
        PlanSettings {
            enabled: false,
            max_body_bytes: 1_048_576,
            max_duration_ms: 10_000,
        }
    }
}

/// The whole configuration file: `{"metrics": {"enabled": true}, "plan": {"enabled": true, "max_body_bytes": 1048576, "max_duration_ms": 10000}}`.
/// Both sections are optional and off when absent; an unknown key anywhere is an error.
pub fn parse_config(text: &str) -> Result<(bool, PlanSettings), String> {
    if text.trim().is_empty() {
        return Ok((false, PlanSettings::default()));
    }
    let value: serde_json::Value =
        serde_json::from_str(text).map_err(|e| format!("configuration: {e}"))?;
    let object = value
        .as_object()
        .ok_or("the configuration must be a JSON object")?;
    if let Some(k) = object
        .keys()
        .find(|k| k.as_str() != "metrics" && k.as_str() != "plan")
    {
        return Err(format!("the configuration has an unknown field: {k}"));
    }
    let metrics = match object.get("metrics") {
        None => false,
        Some(section) => {
            let section = section.as_object().ok_or("metrics must be an object")?;
            if let Some(k) = section.keys().find(|k| k.as_str() != "enabled") {
                return Err(format!(
                    "the metrics configuration has an unknown field: {k}"
                ));
            }
            match section.get("enabled") {
                None => false,
                Some(v) => v
                    .as_bool()
                    .ok_or_else(|| "metrics.enabled must be true or false".to_string())?,
            }
        }
    };
    let mut plan = PlanSettings::default();
    if let Some(section) = object.get("plan") {
        let section = section.as_object().ok_or("plan must be an object")?;
        if let Some(k) = section
            .keys()
            .find(|k| !["enabled", "max_body_bytes", "max_duration_ms"].contains(&k.as_str()))
        {
            return Err(format!("the plan configuration has an unknown field: {k}"));
        }
        if let Some(v) = section.get("enabled") {
            plan.enabled = v.as_bool().ok_or("plan.enabled must be true or false")?;
        }
        if let Some(v) = section.get("max_body_bytes") {
            let n = v
                .as_u64()
                .filter(|n| (1024..=16 * 1024 * 1024).contains(n))
                .ok_or("plan.max_body_bytes must be a whole number between 1024 and 16777216")?;
            plan.max_body_bytes = n as usize;
        }
        if let Some(v) = section.get("max_duration_ms") {
            plan.max_duration_ms = v
                .as_u64()
                .filter(|n| (100..=600_000).contains(n))
                .ok_or("plan.max_duration_ms must be a whole number between 100 and 600000")?;
        }
    }
    Ok((metrics, plan))
}

/// The numbers of this service for `GET /metrics` (see `service_metrics.rs`).
static METRICS: ServiceMetrics = ServiceMetrics::new("hydra-umc-path-planner-3d");

/// What `serve` was asked for.
#[derive(Debug, PartialEq)]
pub struct Options {
    pub addr: String,
    pub token: Option<String>,
    pub allow_anonymous: bool,
    /// The JSON file whose `metrics` section opens `GET /metrics` (None: it stays closed).
    pub config: Option<String>,
}

/// Reads the options after `serve`; `env_token` is the value of `PATH_PLANNER_TOKEN`, used when `--token` is not given.
pub fn parse_options(args: &[String], env_token: Option<String>) -> Result<Options, String> {
    let mut options = Options {
        addr: DEFAULT_ADDR.to_string(),
        token: env_token.filter(|t| !t.is_empty()),
        allow_anonymous: false,
        config: std::env::var(CONFIG_ENV).ok().filter(|p| !p.is_empty()),
    };
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        match arg.as_str() {
            "--addr" => {
                options.addr = iter.next().ok_or("--addr needs HOST:PORT")?.clone();
            }
            "--token" => {
                let token = iter.next().ok_or("--token needs a value")?;
                options.token = Some(token.clone()).filter(|t| !t.is_empty());
            }
            "--config" => {
                options.config = Some(iter.next().ok_or("--config needs a path")?.clone())
                    .filter(|p| !p.is_empty());
            }
            "--allow-anonymous" => options.allow_anonymous = true,
            other => return Err(format!("unknown option '{other}'")),
        }
    }
    Ok(options)
}

/// True for the addresses that cannot be reached from another machine (`127.0.0.1:8117`, `[::1]:8117`, `localhost:8117`).
pub fn is_loopback_addr(addr: &str) -> bool {
    if let Ok(socket) = addr.parse::<std::net::SocketAddr>() {
        return socket.ip().is_loopback();
    }
    let host = match addr.rsplit_once(':') {
        Some((host, port)) if !host.contains(':') && port.chars().all(|c| c.is_ascii_digit()) => {
            host
        }
        _ => addr,
    };
    if host.eq_ignore_ascii_case("localhost") {
        return true;
    }
    host.trim_matches(|c| c == '[' || c == ']')
        .parse::<std::net::IpAddr>()
        .map(|ip| ip.is_loopback())
        .unwrap_or(false)
}

/// Why the listener must not start: the numbers of the service must not be open on the network by accident.
pub fn startup_problem(options: &Options) -> Option<String> {
    if is_loopback_addr(&options.addr) || options.token.is_some() || options.allow_anonymous {
        return None;
    }
    Some(format!(
        "refusing to listen on {} without a token: set --token (or {TOKEN_ENV}), bind a loopback address, or pass --allow-anonymous to run an open listener on purpose",
        options.addr
    ))
}

/// `POST /plan` listens only where a token protects it: on another address than the loopback it needs a token, even with `--allow-anonymous`.
pub fn plan_startup_problem(options: &Options, plan: &PlanSettings) -> Option<String> {
    if plan.enabled && !is_loopback_addr(&options.addr) && options.token.is_none() {
        return Some(format!(
            "refusing to serve POST /plan on {} without a token: set --token (or {TOKEN_ENV}) or bind a loopback address",
            options.addr
        ));
    }
    None
}

/// The answers to scenarios already planned, by the text of the scenario: the search is deterministic for a scenario (its seed is part of the
/// text), so the same text always has the same answer. Only answers that do not depend on the clock are kept (not the time limit).
const CACHE_ENTRIES: usize = 32;
static CACHE: std::sync::Mutex<Vec<(String, u16, String)>> = std::sync::Mutex::new(Vec::new());

fn cached(body: &str) -> Option<(u16, String)> {
    let cache = CACHE.lock().ok()?;
    cache
        .iter()
        .find(|(key, _, _)| key == body)
        .map(|(_, status, answer)| (*status, answer.clone()))
}

fn remember(body: &str, status: u16, answer: &str) {
    if status == 504 {
        return;
    }
    if let Ok(mut cache) = CACHE.lock() {
        if cache.len() >= CACHE_ENTRIES {
            cache.remove(0);
        }
        cache.push((body.to_string(), status, answer.to_string()));
    }
}

/// `POST /plan`: the plan for the scenario in the body. Called only when the route is open.
fn handle_plan(mut request: Request, token: &Option<String>, plan: &PlanSettings) {
    let allowed = if token.is_some() {
        authorized(&request, token)
    } else {
        request.remote_addr().is_some_and(|a| a.ip().is_loopback())
    };
    if !allowed {
        write_json(request, 401, &json!({"error": "unauthorized"}));
        return;
    }
    if request
        .body_length()
        .is_some_and(|length| length > plan.max_body_bytes)
    {
        write_json(request, 413, &json!({"error": "request body too large"}));
        return;
    }
    let mut body = String::new();
    let limit = plan.max_body_bytes as u64 + 1;
    let read = std::io::Read::read_to_string(
        &mut std::io::Read::take(request.as_reader(), limit),
        &mut body,
    );
    if read.is_err() {
        write_json(request, 400, &json!({"error": "the body is not text"}));
        return;
    }
    if body.len() > plan.max_body_bytes {
        write_json(request, 413, &json!({"error": "request body too large"}));
        return;
    }
    let (status, answer) = match cached(&body) {
        Some(hit) => hit,
        None => {
            let fresh = crate::plan_scenario_text(&body, plan.max_duration_ms);
            remember(&body, fresh.0, &fresh.1);
            fresh
        }
    };
    METRICS.count(status);
    let header = Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]).unwrap();
    let _ = request.respond(
        Response::from_string(answer)
            .with_status_code(status)
            .with_header(header),
    );
}

fn constant_time_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    a.iter().zip(b).fold(0u8, |acc, (x, y)| acc | (x ^ y)) == 0
}

fn authorized(request: &Request, token: &Option<String>) -> bool {
    let Some(expected) = token else {
        return true;
    };
    request
        .headers()
        .iter()
        .find(|h| h.field.equiv("Authorization"))
        .and_then(|h| h.value.as_str().strip_prefix("Bearer "))
        .is_some_and(|given| constant_time_eq(given.as_bytes(), expected.as_bytes()))
}

fn write_json(request: Request, status: u16, body: &serde_json::Value) {
    METRICS.count(status);
    let header = Header::from_bytes(&b"Content-Type"[..], &b"application/json"[..]).unwrap();
    let response = Response::from_string(body.to_string())
        .with_status_code(status)
        .with_header(header);
    let _ = request.respond(response);
}

pub fn bind(addr: &str) -> std::io::Result<Server> {
    Server::http(addr).map_err(std::io::Error::other)
}

/// Serves requests until the listener closes; `POST /plan` is closed.
#[allow(dead_code)]
pub fn run(server: &Server, token: &Option<String>, metrics: bool) {
    run_with(server, token, metrics, PlanSettings::default());
}

/// `run`, with the settings of `POST /plan`.
pub fn run_with(server: &Server, token: &Option<String>, metrics: bool, plan: PlanSettings) {
    METRICS.start();
    for request in server.incoming_requests() {
        if is_metrics_request(&request) {
            // closed unless the configuration opens it (then the route does not exist)
            if !metrics {
                write_json(request, 404, &json!({"error": "not found"}));
                continue;
            }
            // with a token the caller presents it; without one only a caller on this machine may read the numbers
            let allowed = if token.is_some() {
                authorized(&request, token)
            } else {
                request.remote_addr().is_some_and(|a| a.ip().is_loopback())
            };
            METRICS.respond(request, VERSION, allowed, &[]);
            continue;
        }
        let path = request.url().split('?').next().unwrap_or("").to_string();
        if request.method() == &Method::Post && path == "/plan" && plan.enabled {
            handle_plan(request, token, &plan);
            continue;
        }
        if request.method() == &Method::Get && path == "/health" {
            write_json(
                request,
                200,
                &json!({"status": "ok", "name": NAME, "version": VERSION}),
            );
        } else {
            write_json(request, 404, &json!({"error": "not found"}));
        }
    }
}

/// The `serve` command: returns the process exit code.
pub fn serve_command(args: &[String]) -> std::process::ExitCode {
    let options = match parse_options(args, std::env::var(TOKEN_ENV).ok()) {
        Ok(options) => options,
        Err(e) => {
            eprintln!("serve: {e}");
            eprintln!("Usage: serve [--addr HOST:PORT] [--token TOKEN] [--allow-anonymous] [--config FILE]");
            return std::process::ExitCode::FAILURE;
        }
    };
    if let Some(problem) = startup_problem(&options) {
        eprintln!("serve: {problem}");
        return std::process::ExitCode::FAILURE;
    }
    let (metrics, plan) = match &options.config {
        None => (false, PlanSettings::default()),
        Some(path) => match std::fs::read_to_string(path)
            .map_err(|e| format!("could not read {path}: {e}"))
            .and_then(|text| parse_config(&text))
        {
            Ok(settings) => settings,
            Err(e) => {
                eprintln!("serve: {e}");
                return std::process::ExitCode::FAILURE;
            }
        },
    };
    if let Some(problem) = plan_startup_problem(&options, &plan) {
        eprintln!("serve: {problem}");
        return std::process::ExitCode::FAILURE;
    }
    let server = match bind(&options.addr) {
        Ok(server) => server,
        Err(e) => {
            eprintln!("serve: could not listen on {}: {e}", options.addr);
            return std::process::ExitCode::FAILURE;
        }
    };
    let _ = writeln!(
        std::io::stderr(),
        "{NAME} listening on {} (GET /health{}{})",
        options.addr,
        if metrics { ", GET /metrics" } else { "" },
        if plan.enabled { ", POST /plan" } else { "" }
    );
    run_with(&server, &options.token, metrics, plan);
    std::process::ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::TcpStream;

    fn start_with(token: Option<&str>, metrics: bool) -> u16 {
        let server = bind("127.0.0.1:0").unwrap();
        let port = server.server_addr().to_ip().unwrap().port();
        let token = token.map(String::from);
        std::thread::spawn(move || run(&server, &token, metrics));
        port
    }

    /// A service whose configuration opens the metrics.
    fn start(token: Option<&str>) -> u16 {
        start_with(token, true)
    }

    fn get(port: u16, path: &str, bearer: Option<&str>) -> (u16, String) {
        let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
        let auth = bearer.map_or(String::new(), |t| format!("Authorization: Bearer {t}\r\n"));
        write!(
            stream,
            "GET {path} HTTP/1.1\r\nHost: localhost\r\n{auth}Connection: close\r\n\r\n"
        )
        .unwrap();
        let mut raw = String::new();
        stream.read_to_string(&mut raw).unwrap();
        let status = raw.split_whitespace().nth(1).unwrap().parse().unwrap();
        let body = raw
            .split_once("\r\n\r\n")
            .map_or("", |(_, b)| b)
            .to_string();
        (status, body)
    }

    fn args(list: &[&str]) -> Vec<String> {
        list.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn the_listener_is_loopback_by_default_and_refuses_the_network_without_a_token() {
        let defaults = parse_options(&[], None).unwrap();
        assert_eq!(defaults.addr, DEFAULT_ADDR);
        assert!(startup_problem(&defaults).is_none());
        assert_eq!(
            parse_options(&args(&["--config", "c.json"]), None)
                .unwrap()
                .config
                .as_deref(),
            Some("c.json")
        );
        let open = parse_options(&args(&["--addr", "0.0.0.0:8117"]), None).unwrap();
        assert!(startup_problem(&open).is_some());
        let with_token =
            parse_options(&args(&["--addr", "0.0.0.0:8117", "--token", "t"]), None).unwrap();
        assert!(startup_problem(&with_token).is_none());
        let from_env = parse_options(&args(&["--addr", "0.0.0.0:8117"]), Some("t".into())).unwrap();
        assert!(startup_problem(&from_env).is_none());
        let anonymous = parse_options(
            &args(&["--addr", "0.0.0.0:8117", "--allow-anonymous"]),
            None,
        )
        .unwrap();
        assert!(startup_problem(&anonymous).is_none());
        assert!(parse_options(&args(&["--nope"]), None).is_err());
        assert!(parse_options(&args(&["--addr"]), None).is_err());
    }

    #[test]
    fn health_and_metrics_answer_and_everything_else_is_404() {
        let port = start(None);
        let (status, body) = get(port, "/health", None);
        assert_eq!(status, 200, "{body}");
        assert!(body.contains(NAME));
        assert_eq!(get(port, "/plan", None).0, 404);
        let (status, text) = get(port, "/metrics", None);
        assert_eq!(status, 200, "{text}");
        assert!(text
            .contains("hydra_umc_service_info{service=\"hydra-umc-path-planner-3d\",version=\""));
        assert!(text.contains("hydra_umc_service_health{service=\"hydra-umc-path-planner-3d\"} 1"));
        assert!(text.contains("class=\"4xx\"}"));
        assert!(!text.contains("/plan"));
    }

    #[test]
    fn the_metrics_are_off_by_default_even_with_the_token() {
        let port = start_with(None, false);
        assert_eq!(get(port, "/health", None).0, 200);
        assert_eq!(get(port, "/metrics", None).0, 404);
        let guarded = start_with(Some("s3cret-token"), false);
        assert_eq!(get(guarded, "/metrics", Some("s3cret-token")).0, 404);
    }

    fn post(port: u16, path: &str, bearer: Option<&str>, body: &str) -> (u16, String) {
        let mut stream = TcpStream::connect(("127.0.0.1", port)).unwrap();
        let auth = bearer.map_or(String::new(), |t| format!("Authorization: Bearer {t}\r\n"));
        write!(
            stream,
            "POST {path} HTTP/1.1\r\nHost: localhost\r\n{auth}Content-Length: {}\r\nConnection: close\r\n\r\n{body}",
            body.len()
        )
        .unwrap();
        let mut raw = String::new();
        stream.read_to_string(&mut raw).unwrap();
        let status = raw.split_whitespace().nth(1).unwrap().parse().unwrap();
        let answer = raw
            .split_once("\r\n\r\n")
            .map_or("", |(_, b)| b)
            .to_string();
        (status, answer)
    }

    fn start_plan(token: Option<&str>, plan: PlanSettings) -> u16 {
        let server = bind("127.0.0.1:0").unwrap();
        let port = server.server_addr().to_ip().unwrap().port();
        let token = token.map(String::from);
        std::thread::spawn(move || run_with(&server, &token, false, plan));
        port
    }

    const OPEN: &str = r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
        "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}},"seed":7}"#;

    fn plan_on() -> PlanSettings {
        PlanSettings {
            enabled: true,
            ..PlanSettings::default()
        }
    }

    #[test]
    fn the_plan_route_is_closed_by_default_and_plans_when_the_configuration_opens_it() {
        let closed = start_plan(None, PlanSettings::default());
        assert_eq!(post(closed, "/plan", None, OPEN).0, 404);
        let port = start_plan(None, plan_on());
        let (status, answer) = post(port, "/plan", None, OPEN);
        assert_eq!(status, 200, "{answer}");
        let parsed: serde_json::Value = serde_json::from_str(&answer).unwrap();
        assert_eq!(parsed["status"], "ok");
        assert!(parsed["path"].as_array().unwrap().len() >= 2);
        // the same text has the same answer, from the cache
        assert_eq!(post(port, "/plan", None, OPEN), (status, answer));
        // a GET is not a plan
        assert_eq!(get(port, "/plan", None).0, 404);
    }

    #[test]
    fn the_plan_route_asks_for_the_token_when_one_is_set() {
        let port = start_plan(Some("s3cret-token"), plan_on());
        assert_eq!(post(port, "/plan", None, OPEN).0, 401);
        assert_eq!(post(port, "/plan", Some("wrong"), OPEN).0, 401);
        assert_eq!(post(port, "/plan", Some("s3cret-token"), OPEN).0, 200);
    }

    #[test]
    fn the_plan_route_refuses_what_it_cannot_plan_with_a_status_that_says_why() {
        let port = start_plan(None, plan_on());
        let (status, answer) = post(port, "/plan", None, "{not json");
        assert_eq!(status, 400, "{answer}");
        assert!(answer.contains("invalid_input"), "{answer}");
        let zones = r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},"zones_file":"/etc/passwd",
            "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#;
        let (status, answer) = post(port, "/plan", None, zones);
        assert_eq!(status, 400, "{answer}");
        assert!(answer.contains("zones_file"), "{answer}");
        let inside = r#"{"start":{"x":0,"y":0,"z":0},"goal":{"x":2,"y":0,"z":0},
            "obstacles":[{"center":{"x":2,"y":0,"z":0},"radius":0.5}],
            "workspace":{"min":{"x":-5,"y":-5,"z":-5},"max":{"x":5,"y":5,"z":5}}}"#;
        let (status, answer) = post(port, "/plan", None, inside);
        assert_eq!(status, 422, "{answer}");
        assert!(answer.contains("goal_inside_obstacle"), "{answer}");
    }

    #[test]
    fn the_plan_route_caps_the_size_of_the_body() {
        let port = start_plan(
            None,
            PlanSettings {
                enabled: true,
                max_body_bytes: 1024,
                ..PlanSettings::default()
            },
        );
        let big = format!("{{\"pad\":\"{}\"}}", "x".repeat(2000));
        assert_eq!(post(port, "/plan", None, &big).0, 413);
        assert_eq!(post(port, "/plan", None, OPEN).0, 200);
    }

    #[test]
    fn the_configuration_has_a_plan_section_that_is_checked_and_a_plan_on_the_network_needs_a_token(
    ) {
        assert_eq!(parse_config("").unwrap().1, PlanSettings::default());
        assert!(!parse_config("{}").unwrap().1.enabled);
        let (metrics, plan) = parse_config(
            r#"{"metrics":{"enabled":true},"plan":{"enabled":true,"max_body_bytes":4096,"max_duration_ms":500}}"#,
        )
        .unwrap();
        assert!(
            metrics && plan.enabled && plan.max_body_bytes == 4096 && plan.max_duration_ms == 500
        );
        for bad in [
            r#"{"plan":{"enabled":"yes"}}"#,
            r#"{"plan":{"max_body_bytes":10}}"#,
            r#"{"plan":{"max_duration_ms":0}}"#,
            r#"{"plan":{"x":1}}"#,
            r#"{"plan":5}"#,
        ] {
            assert!(parse_config(bad).is_err(), "{bad}");
        }
        let open = parse_options(
            &args(&["--addr", "0.0.0.0:8117", "--allow-anonymous"]),
            None,
        )
        .unwrap();
        assert!(plan_startup_problem(&open, &plan_on()).is_some());
        assert!(plan_startup_problem(&open, &PlanSettings::default()).is_none());
        let with_token =
            parse_options(&args(&["--addr", "0.0.0.0:8117", "--token", "t"]), None).unwrap();
        assert!(plan_startup_problem(&with_token, &plan_on()).is_none());
        let loopback = parse_options(&[], None).unwrap();
        assert!(plan_startup_problem(&loopback, &plan_on()).is_none());
    }

    #[test]
    fn the_configuration_opens_the_metrics_and_checks_its_keys() {
        assert!(!metrics_enabled("").unwrap());
        assert!(!metrics_enabled("{}").unwrap());
        assert!(!metrics_enabled(r#"{"metrics":{"enabled":false}}"#).unwrap());
        assert!(metrics_enabled(r#"{"metrics":{"enabled":true}}"#).unwrap());
        assert!(metrics_enabled(r#"{"metrics":{"enabled":"yes"}}"#).is_err());
        assert!(metrics_enabled(r#"{"metrics":{"enabled":true,"x":1}}"#).is_err());
        assert!(metrics_enabled(r#"{"other":1}"#).is_err());
        assert!(metrics_enabled("[1]").is_err());
    }

    #[test]
    fn the_metrics_ask_for_the_token_when_one_is_set_and_health_stays_open() {
        let port = start(Some("s3cret-token"));
        assert_eq!(get(port, "/health", None).0, 200);
        assert_eq!(get(port, "/metrics", None).0, 401);
        assert_eq!(get(port, "/metrics", Some("wrong-token")).0, 401);
        let (status, text) = get(port, "/metrics", Some("s3cret-token"));
        assert_eq!(status, 200);
        assert!(!text.contains("s3cret-token"));
    }
}
