use std::fs::{File,remove_file,create_dir};
use std::io::Write;
use std::sync::Mutex;
use std::time::{SystemTime,UNIX_EPOCH};
use std::time::Instant;

/// session identificator
static SESSION: Mutex<String> = Mutex::new(String::new());
/// File descriptor to log file
pub static LOG_FILE: Mutex<Option<File>> = Mutex::new(Option::None);
/// Path to folder where logs are stored
const LOG_FOLDER: &str = "./logs/";
/// Start of application - timestamp
pub static START_TIME: Mutex<Option<Instant>> = Mutex::new(None);

/// Changes color of UNIX terminal to BLACK
pub const COLOR_NORMAL: &str =  "\x1B[0m";
/// Changes color of UNIX terminal to RED
pub const COLOR_RED: &str =  "\x1B[31m";
/// Changes color of UNIX terminal to GREEN
pub const COLOR_GREEN: &str =  "\x1B[32m";
/// Changes color of UNIX terminal to YELLOW
pub const COLOR_YELLOW: &str = "\x1B[33m";
/// Changes color of UNIX terminal to BLUE
pub const COLOR_BLUE: &str = "\x1B[34m";
/// Changes color of UNIX terminal to MAGENTA
pub const COLOR_MAGENTA: &str = "\x1B[35m";
/// Changes color of UNIX terminal to CYAN
pub const COLOR_CYAN: &str = "\x1B[36m";
/// Changes color of UNIX terminal to WHITE
pub const COLOR_WHITE: &str = "\x1B[37m";

/// Print wrapper that also log output to file
#[macro_export]
macro_rules! printLog
{
    () => {
        println("\n")
    };
    ($($arg:tt)*) => {{
        let time = crate::logger::START_TIME.lock().unwrap(); 
        let time_elapsed = time.unwrap().elapsed().as_secs_f32();
        drop(time);
        print!("{}{:9.3} {}[Server] ", crate::logger::COLOR_WHITE, time_elapsed, crate::logger::COLOR_YELLOW);
        println!($($arg)*);
        let mut log_file = crate::logger::LOG_FILE.lock().unwrap();
        if let Some(file) = log_file.as_mut()
        {
            std::io::Write::write(file,time_elapsed.to_string().as_bytes()).unwrap();
            std::io::Write::write(file,format!("{:9.3} [Server] ",time_elapsed).as_bytes()).unwrap();
            std::io::Write::write(file,format!($($arg)*).as_bytes()).expect("Unable to write log");
            std::io::Write::write(file,b"\n").unwrap();
        }
    }};
}
/// Logger. Used to log execution of program
pub struct Logger
{}

impl Logger
{
    /// Initialization
    fn init()
    {
        let mut time = START_TIME.lock().unwrap(); 
        *time = Some(Instant::now());
        drop(time);
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

    /// Get session identifier
    fn determinateSessionName() -> String
    {
        let mut session = SESSION.lock().unwrap();
        *session = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs().to_string();
        session.to_string()
    }

    /// Starts log session
    pub fn startSession()
    {
        Self::init();
    }

    /// End log session
    pub fn endSession()
    {
        let session = SESSION.lock().unwrap();
        if !session.is_empty()
        {
            remove_file("./logs/session").expect("Unable to remove session file.");
        }
    }

    /// Print wrapper that add extra prefix and color of message. Message is also logged to file
    pub fn print(name: &str, source: &str, color: &str, msg: &str)
    {
        let mut log_file = crate::logger::LOG_FILE.lock().unwrap();
        let time = crate::logger::START_TIME.lock().unwrap(); 
        let time_elapsed = time.unwrap().elapsed().as_secs_f32();
        drop(time);
        if let Some(file) = log_file.as_mut()
        {
            file.write(format!("{:9.3}[{}][{}] ", time_elapsed ,name, source).as_bytes()).unwrap();
            file.write(msg.as_bytes()).expect("Unable to write log");
            file.write(b"\n").unwrap();
            println!("{}{:9.3} {}[{}][{}] {}",COLOR_WHITE,time_elapsed,color, name, source, msg);
        }
    }
}