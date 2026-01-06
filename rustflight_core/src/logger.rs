// ******************************************************************************
// * File     : logger.rs
// * Date     : January 6, 2026
// ******************************************************************************
// *
// * Copyright (c) 2023, AeroVironment, Inc.
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions are met:
// *
// * 1.Redistributions of source code must retain the above copyright notice, this
// * list of conditions and the following disclaimer.
// *
// * 2.Redistributions in binary form must reproduce the above copyright notice,
// * this list of conditions and the following disclaimer in the documentation
// * and/or other materials provided with the distribution.
// *
// * 3.Neither the name of the copyright holder nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// ******************************************************************************

use core::fmt::Write;
use core::cell::RefCell;
use heapless::{String, Deque};

// WARNING: Critical Section forces interrupts to wait while processing, 
// never do complex processing from within a Critical Section!
use critical_section::Mutex;

use crate::comm_messages::enums::Severity;

// --- Configuration ---
// Adjust queue size based on RAM availability.
// 16 messages * ~54 bytes = ~864 bytes of static RAM.
const LOG_QUEUE_SIZE: usize = 16;
// Matches MAVLink STATUSTEXT length (50 chars)
const MAX_LOG_LEN: usize = 50; 

// --- Data Types ---

/// A single log entry holding severity and text
#[derive(Clone)]
pub struct LogEntry {
    pub severity: Severity,
    pub message: String<MAX_LOG_LEN>,
}

// --- Global Storage ---

static LOG_QUEUE: Mutex<RefCell<Deque<LogEntry, LOG_QUEUE_SIZE>>> = 
    Mutex::new(RefCell::new(Deque::new()));

// --- Public API ---

pub struct Logger;

impl Logger {
    /// The main logging function.
    /// Usage: Logger::log(Severity::Info, format_args!("Val: {}", 42));
    pub fn log(severity: Severity, args: core::fmt::Arguments) {
        critical_section::with(|cs| {
            // Borrow the queue mutably
            let mut queue = LOG_QUEUE.borrow_ref_mut(cs);

            // Create new entry
            let mut entry = LogEntry {
                severity,
                message: String::new(),
            };

            // Write text to buffer. 
            // write_fmt returns a Result. If the string is too long (>50 chars), 
            // it returns an error, but the buffer will contain as much as fits.
            // We ignore the error to ensure we still get the truncated log.
            let _ = entry.message.write_fmt(args);

            // Push to queue. 
            // If queue is full, we pop the oldest message to make room for the new one.
            if queue.is_full() {
                let _ = queue.pop_front();
            }
            let _ = queue.push_back(entry);
        });
    }

    // Convenience wrappers matching your existing comm_messages.rs enum names
    pub fn info(args: core::fmt::Arguments) { Self::log(Severity::Info, args); }
    pub fn warn(args: core::fmt::Arguments) { Self::log(Severity::Warning, args); }
    pub fn error(args: core::fmt::Arguments) { Self::log(Severity::Error, args); }
    pub fn debug(args: core::fmt::Arguments) { Self::log(Severity::Debug, args); }
    
    // Drain function for Main Loop
    pub fn pop() -> Option<LogEntry> {
        critical_section::with(|cs| {
            LOG_QUEUE.borrow_ref_mut(cs).pop_front()
        })
    }
}

// --- Macros ---
// These allow you to use log_info!("val: {}", x) anywhere in your code.
// The Macros can generally be used either by placing 
//  `use crate::log_<info, warn, error, or debug>` 
// at the top of the crate, then using `log_<info, warn, error, or debug>!("{}", var);`, 
// or by using `crate::log_<info, warn, error, or debug>!("{}", var);` directly.

#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => { $crate::logger::Logger::info(format_args!($($arg)*)) };
}

#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => { $crate::logger::Logger::warn(format_args!($($arg)*)) };
}

#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => { $crate::logger::Logger::error(format_args!($($arg)*)) };
}

#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => { $crate::logger::Logger::debug(format_args!($($arg)*)) };
}