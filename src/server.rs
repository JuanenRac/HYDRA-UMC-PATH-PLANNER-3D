// =============================================================================
// HYDRA-UMC-PATH-PLANNER-3D - src/server.rs
// Copyright (C) 2026 JuanenRac (Electro Hobby 3D) <electrohobby3d@gmail.com>
// GPL-3.0-or-later - see LICENSE
// =============================================================================
//! The only network surface of this program, and it is off unless asked for: `serve` starts a small HTTP listener (`tiny_http`, blocking, no async
//! runtime) that answers two read-only routes and nothing else - `GET /health` (liveness, open) and `GET /metrics` (the numbers of the service in the
//! text format of Prometheus, behind the token when one is set). It plans nothing and takes no scenario: planning stays a command of the command line.
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

/// The numbers of this service for `GET /metrics` (see `service_metrics.rs`).
static METRICS: ServiceMetrics = ServiceMetrics::new("hydra-umc-path-planner-3d");

/// What `serve` was asked for.
#[derive(Debug, PartialEq)]
pub struct Options {
    pub addr: String,
    pub token: Option<String>,
    pub allow_anonymous: bool,
}

/// Reads the options after `serve`; `env_token` is the value of `PATH_PLANNER_TOKEN`, used when `--token` is not given.
pub fn parse_options(args: &[String], env_token: Option<String>) -> Result<Options, String> {
    let mut options = Options {
        addr: DEFAULT_ADDR.to_string(),
        token: env_token.filter(|t| !t.is_empty()),
        allow_anonymous: false,
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

/// Serves requests until the listener closes.
pub fn run(server: &Server, token: &Option<String>) {
    METRICS.start();
    for request in server.incoming_requests() {
        if is_metrics_request(&request) {
            let allowed = authorized(&request, token);
            METRICS.respond(request, VERSION, allowed, &[]);
            continue;
        }
        let path = request.url().split('?').next().unwrap_or("").to_string();
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
            eprintln!("Usage: serve [--addr HOST:PORT] [--token TOKEN] [--allow-anonymous]");
            return std::process::ExitCode::FAILURE;
        }
    };
    if let Some(problem) = startup_problem(&options) {
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
        "{NAME} listening on {} (GET /health, GET /metrics)",
        options.addr
    );
    run(&server, &options.token);
    std::process::ExitCode::SUCCESS
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{Read, Write};
    use std::net::TcpStream;

    fn start(token: Option<&str>) -> u16 {
        let server = bind("127.0.0.1:0").unwrap();
        let port = server.server_addr().to_ip().unwrap().port();
        let token = token.map(String::from);
        std::thread::spawn(move || run(&server, &token));
        port
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
