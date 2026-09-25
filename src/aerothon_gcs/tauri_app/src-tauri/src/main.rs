// AeroTHON GCS — Tauri 2 backend.
// Holds the WebSocket connection to the ROS 2 aggregator (Q53): forwards
// inbound frames to the React UI as "ws" events, and relays outbound commands
// from the `send_command` Tauri command. Auto-reconnects.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use futures_util::{SinkExt, StreamExt};
use std::sync::{Arc, Mutex};
use tauri::{Emitter, Manager, State};
use tokio::sync::mpsc;
use tokio_tungstenite::tungstenite::Message;

const DEFAULT_AGG_URL: &str = "ws://127.0.0.1:8765";

struct Connection {
    commands: Mutex<Option<mpsc::UnboundedSender<String>>>,
    reconnect: Mutex<Option<mpsc::UnboundedSender<()>>>,
    endpoint: Arc<Mutex<String>>,
}

fn valid_endpoint(endpoint: &str) -> bool {
    (endpoint.starts_with("ws://") || endpoint.starts_with("wss://"))
        && endpoint
            .split("://")
            .nth(1)
            .is_some_and(|host| !host.is_empty())
}

#[tauri::command]
fn set_endpoint(state: State<Connection>, endpoint: String) -> Result<(), String> {
    let endpoint = endpoint.trim().trim_end_matches('/').to_owned();
    if !valid_endpoint(&endpoint) {
        return Err("Use a WebSocket URL such as ws://192.168.4.1:8765".into());
    }
    *state.endpoint.lock().unwrap() = endpoint;
    if let Some(tx) = state.reconnect.lock().unwrap().as_ref() {
        let _ = tx.send(());
    }
    Ok(())
}

#[tauri::command]
fn send_command(
    state: State<Connection>,
    cmd: String,
    args: serde_json::Value,
) -> Result<(), String> {
    let payload = serde_json::json!({
        "v": 1, "kind": "command", "t": 0.0,
        "data": {
            "cmd_id": uuid::Uuid::new_v4().to_string(),
            "cmd": cmd, "args": args, "confirm": true
        }
    })
    .to_string();
    let guard = state.commands.lock().unwrap();
    match guard.as_ref() {
        Some(tx) => tx.send(payload).map_err(|e| e.to_string()),
        None => Err("not connected".into()),
    }
}

fn main() {
    tauri::Builder::default()
        .manage(Connection {
            commands: Mutex::new(None),
            reconnect: Mutex::new(None),
            endpoint: Arc::new(Mutex::new(
                std::env::var("AEROTHON_GCS_WS_URL").unwrap_or_else(|_| DEFAULT_AGG_URL.into()),
            )),
        })
        .setup(|app| {
            let handle = app.handle().clone();
            let (command_tx, mut commands) = mpsc::unbounded_channel::<String>();
            let (reconnect_tx, mut reconnects) = mpsc::unbounded_channel::<()>();
            *app.state::<Connection>().commands.lock().unwrap() = Some(command_tx);
            *app.state::<Connection>().reconnect.lock().unwrap() = Some(reconnect_tx);
            let endpoint = app.state::<Connection>().endpoint.clone();

            tauri::async_runtime::spawn(async move {
                loop {
                    let url = endpoint.lock().unwrap().clone();
                    if let Ok((ws, _)) = tokio_tungstenite::connect_async(&url).await {
                        let _ = handle.emit(
                            "connection",
                            serde_json::json!({
                                "connected": true, "endpoint": url
                            }),
                        );
                        let (mut write, mut read) = ws.split();
                        loop {
                            tokio::select! {
                                out = commands.recv() => match out {
                                    Some(m) => { let _ = write.send(Message::Text(m)).await; }
                                    None => break,
                                },
                                _ = reconnects.recv() => break,
                                msg = read.next() => match msg {
                                    Some(Ok(Message::Text(t))) => { let _ = handle.emit("ws", t); }
                                    Some(Ok(_)) => {}
                                    _ => break,
                                },
                            }
                        }
                    }
                    let _ = handle.emit(
                        "connection",
                        serde_json::json!({
                            "connected": false, "endpoint": endpoint.lock().unwrap().clone()
                        }),
                    );
                    tokio::select! {
                        _ = tokio::time::sleep(std::time::Duration::from_secs(2)) => {},
                        _ = reconnects.recv() => {},
                    }
                }
            });
            Ok(())
        })
        .invoke_handler(tauri::generate_handler![set_endpoint, send_command])
        .run(tauri::generate_context!())
        .expect("error running AeroTHON GCS");
}
