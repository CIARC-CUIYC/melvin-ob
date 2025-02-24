use chrono::Utc;

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