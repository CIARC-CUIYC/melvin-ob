use serde_json::to_string_pretty;
use std::fs;
use std::path::Path;

#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {
        println!("\x1b[32m[INFO] [{}]\x1b[0m {}", chrono::Utc::now().format("%H:%M:%S"), format!($($arg)*))
    };
}

#[macro_export]
macro_rules! log {
    ($($arg:tt)*) => {
        println!("\x1b[33m[LOG]  [{}]\x1b[0m {}", chrono::Utc::now().format("%H:%M:%S"), format!($($arg)*))
    };
}

#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {
        println!("\x1b[35m[WARN] [{}]\x1b[0m {}", chrono::Utc::now().format("%H:%M:%S"), format!($($arg)*))
    };
}

#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {
        println!("\x1b[31m[ERROR][{}]\x1b[0m {}", chrono::Utc::now().format("%H:%M:%S"), format!($($arg)*))
    };
}

#[macro_export]
macro_rules! fatal {
    ($($arg:tt)*) => {
        panic!("\x1b[1;31m[FATAL][{}]\x1b[0m {}", chrono::Utc::now().format("%H:%M:%S"), format!($($arg)*))
    };
}

#[macro_export]
macro_rules! obj {
    ($($arg:tt)*) => {
        println!("\x1b[1;34m[OBJ]  [{}]\x1b[0m {}", chrono::Utc::now().format("%H:%M:%S"), format!($($arg)*))
    };
}

#[macro_export]
macro_rules! event {
    ($($arg:tt)*) => {
        if std::env::var("LOG_MELVIN_EVENTS").is_ok() {
            println!("\x1b[36m[EVENT][{}]\x1b[0m {}", chrono::Utc::now().format("%H:%M:%S"), format!($($arg)*))
        }
    };
}

#[macro_export]
macro_rules! log_burn {
    ($($arg:tt)*) => {
            println!("\x1b[36m[EVENT][{}]\x1b[0m {}", chrono::Utc::now().format("%H:%M:%S"), format!($($arg)*))
    };
}

pub trait JsonDump: serde::Serialize {
    fn file_name(&self) -> String;
    fn dir_name(&self) -> &'static str;
    fn dump_json(&self) {
        let path_str = format!("./dumps/{}/{}.json", self.dir_name(), self.file_name());
        let path = Path::new(&path_str);

        if let Ok(json_data) = to_string_pretty(&self) {
            if let Some(parent) = Path::new(&path).parent() {
                fs::create_dir_all(parent)
                    .is_err()
                    .then(|| warn!("Failed creating directory for JSON file: {parent:?}."));
            }
            fs::write(path, json_data)
                .is_err()
                .then(|| warn!("Failed writing JSON to file {path:?}."));
        };
    }
}
