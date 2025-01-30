use core::slice;
use std::{
    ffi::c_void,
    ops::{Deref, DerefMut},
    os::fd::AsRawFd,
    path::Path,
    ptr::null_mut,
};

pub(crate) struct FileBackedBuffer {
    file: std::fs::File,
    ptr: *mut u8,
    length: usize,
}

impl FileBackedBuffer {
    #[allow(clippy::cast_possible_wrap)]
    pub(crate) fn open<T: AsRef<Path>>(path: T, length: usize) -> Result<Self, &'static str> {
        let file = std::fs::OpenOptions::new()
            .create(true)
            .write(true)
            .read(true)
            .truncate(false)
            .open(path)
            .unwrap();
        let res = unsafe { libc::ftruncate(file.as_raw_fd(), length as i64) };
        if res != 0 {
            return Err("ftruncate failed");
        }
        let ptr = unsafe {
            libc::mmap(
                null_mut(),
                length,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED | libc::MAP_FILE,
                file.as_raw_fd(),
                0,
            )
        };
        if ptr == libc::MAP_FAILED {
            return Err("mmap failed");
        }
        Ok(FileBackedBuffer {
            file,
            length,
            ptr: ptr.cast::<u8>(),
        })
    }
}

impl Drop for FileBackedBuffer {
    fn drop(&mut self) {
        unsafe {
            libc::munmap(self.ptr.cast::<c_void>(), self.length);
        }
    }
}

unsafe impl Send for FileBackedBuffer {}
unsafe impl Sync for FileBackedBuffer {}

impl Deref for FileBackedBuffer {
    type Target = [u8];

    fn deref(&self) -> &Self::Target { unsafe { slice::from_raw_parts(self.ptr, self.length) } }
}

impl DerefMut for FileBackedBuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { slice::from_raw_parts_mut(self.ptr, self.length) }
    }
}
