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
use core::fmt;
use core::cell::RefCell;
use critical_section::Mutex;
use crate::comm_messages::enums::Severity; 

// --- Configuration ---
const LOG_QUEUE_SIZE: usize = 16;
const MAX_LOG_LEN: usize = 50; // Matches MAVLink STATUSTEXT length

// --- Data Structures ---

/// A minimal fixed-capacity string buffer (Replaces heapless::String)
#[derive(Clone, Copy)]
pub struct LogString {
    buffer: [u8; MAX_LOG_LEN],
    len: usize,
}

impl LogString {
    pub const fn new() -> Self {
        Self { buffer: [0; MAX_LOG_LEN], len: 0 }
    }

    pub fn as_str(&self) -> &str {
        // SAFETY: We only write valid UTF-8 via fmt::Write
        unsafe { core::str::from_utf8_unchecked(&self.buffer[..self.len]) }
    }
}

impl fmt::Write for LogString {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let bytes = s.as_bytes();
        let remaining = MAX_LOG_LEN - self.len;
        let copy_len = bytes.len().min(remaining);
        
        // Copy data
        self.buffer[self.len..self.len + copy_len].copy_from_slice(&bytes[..copy_len]);
        self.len += copy_len;
        
        if bytes.len() > remaining {
            return Err(fmt::Error); // Indicate truncation
        }
        Ok(())
    }
}

/// A combined entry holding severity and text
#[derive(Clone, Copy)]
pub struct LogEntry {
    pub severity: Severity,
    pub message: LogString,
}

impl LogEntry {
    pub const fn empty() -> Self {
        Self { 
            // FIXED: Updated to match your comm_messages.rs definition
            severity: Severity::Info, 
            message: LogString { buffer: [0; MAX_LOG_LEN], len: 0 } 
        }
    }
}

/// A minimal Ring Buffer (Replaces heapless::Deque)
struct LogQueue {
    storage: [LogEntry; LOG_QUEUE_SIZE],
    head: usize, // Write index
    tail: usize, // Read index
    full: bool,
}

impl LogQueue {
    const fn new() -> Self {
        Self {
            storage: [LogEntry::empty(); LOG_QUEUE_SIZE],
            head: 0,
            tail: 0,
            full: false,
        }
    }

    fn push(&mut self, entry: LogEntry) {
        self.storage[self.head] = entry;
        self.head = (self.head + 1) % LOG_QUEUE_SIZE;
        
        if self.full {
            // If full, head bumped into tail, so move tail (overwrite oldest)
            self.tail = (self.tail + 1) % LOG_QUEUE_SIZE;
        }
        
        self.full = self.head == self.tail;
    }

    fn pop(&mut self) -> Option<LogEntry> {
        if !self.full && self.head == self.tail {
            return None; // Empty
        }

        let entry = self.storage[self.tail];
        self.tail = (self.tail + 1) % LOG_QUEUE_SIZE;
        self.full = false;
        Some(entry)
    }
}

// --- Global State ---

static LOG_QUEUE: Mutex<RefCell<LogQueue>> = Mutex::new(RefCell::new(LogQueue::new()));

// --- Public API ---

pub struct Logger;

impl Logger {
    pub fn log(severity: Severity, args: fmt::Arguments) {
        critical_section::with(|cs| {
            let mut queue = LOG_QUEUE.borrow_ref_mut(cs);
            
            let mut entry = LogEntry::empty();
            entry.severity = severity;
            
            // Write the formatted string into our buffer
            // We ignore errors (truncation) to ensure we always log something
            let _ = fmt::Write::write_fmt(&mut entry.message, args);
            
            queue.push(entry);
        });
    }

    // FIXED: Updated variants to match comm_messages.rs
    pub fn info(args: fmt::Arguments) { Self::log(Severity::Info, args); }
    pub fn warn(args: fmt::Arguments) { Self::log(Severity::Warning, args); }
    pub fn error(args: fmt::Arguments) { Self::log(Severity::Error, args); }
    pub fn debug(args: fmt::Arguments) { Self::log(Severity::Debug, args); }

    /// Called by Main Loop to drain queue
    pub fn pop() -> Option<LogEntry> {
        critical_section::with(|cs| {
            LOG_QUEUE.borrow_ref_mut(cs).pop()
        })
    }
}

// Macro to make usage cleaner
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
// Add debug macro if you want it exposed
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => { $crate::logger::Logger::debug(format_args!($($arg)*))};
}