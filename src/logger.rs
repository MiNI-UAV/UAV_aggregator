use std::fs::{File,remove_file,create_dir};
use std::io::Write;
use std::sync::Mutex;
use std::time::{SystemTime,UNIX_EPOCH};

static SESSION: Mutex<String> = Mutex::new(String::new());
pub static LOG_FILE: Mutex<Option<File>> = Mutex::new(Option::None);
const LOG_FOLDER: &str = "./logs/";

#[macro_export]
macro_rules! printLog
{
    () => {
        println("\n")
    };
    ($($arg:tt)*) => {{
        println!($($arg)*);
        let mut log_file = crate::logger::LOG_FILE.lock().unwrap();
        if let Some(file) = log_file.as_mut()
        {
            std::io::Write::write(file,format!("[{:-^15}] ", "Server").as_bytes()).unwrap();
            std::io::Write::write(file,format!($($arg)*).as_bytes()).expect("Unable to write log");
            std::io::Write::write(file,b"\n").unwrap();
        }
    }};
}

pub struct Logger
{}

impl Logger
{

    fn init()
    {
        let session = SESSION.lock().unwrap();
        if !session.is_empty()
        {
            return;
        }
        drop(session);
        let session = Self::determinateSessionName();
        let mut file = File::create("./logs/session").unwrap();
        file.write_all(session.as_bytes()).expect("Unable to write session name.");
        drop(file);
        create_dir(LOG_FOLDER.to_string() + session.as_str()).expect("Unable to create session log folder");
        let mut log_file = LOG_FILE.lock().unwrap();
        *log_file = Some(File::create("./logs/".to_string() + session.as_str() + "/server.log").expect("Unable to create log file"));
        drop(log_file);
        printLog!("UAV SERVER");
        printLog!("Session: {}",session);
    }

    fn determinateSessionName() -> String
    {
        let mut session = SESSION.lock().unwrap();
        *session = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs().to_string();
        session.to_string()
    }

    pub fn startSession()
    {
        Self::init();
    }

    pub fn endSession()
    {
        let session = SESSION.lock().unwrap();
        if !session.is_empty()
        {
            remove_file("./logs/session").expect("Unable to remove session file.");
        }
    }

    pub fn print(source: &str, msg: &str)
    {
        let mut log_file = crate::logger::LOG_FILE.lock().unwrap();
        if let Some(file) = log_file.as_mut()
        {
            file.write(format!("[{:-^15}] ", source).as_bytes()).unwrap();
            file.write(msg.as_bytes()).expect("Unable to write log");
            file.write(b"\n").unwrap();
            println!("[{:-^15}] {}", source, msg);
        }
    }
}