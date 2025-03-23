use core::slice;
use std::{
    ffi::c_void,
    ops::{Deref, DerefMut},
    os::fd::AsRawFd,
    path::Path,
    ptr::null_mut,
};

/// A buffer backed by a memory-mapped file.
///
/// This structure provides a way to read and write to a file-backed buffer as if it were in memory.
/// The file is memory-mapped using `mmap`, allowing efficient access to potentially large files
/// without loading them entirely into memory.
///
/// # Safety
///
/// This struct involves unsafe operations due to the use of `mmap` and raw pointers:
/// - The pointer (`ptr`) must not be dereferenced outside the bounds of the memory-mapped region.
/// - The file must remain open and valid for the lifetime of the memory-mapped buffer.
/// - Modifications to the underlying file outside of this struct can lead to undefined behavior.
///
/// Incorrect use of this struct could result in:
/// - Data races when the file or the mapped memory is accessed by multiple threads without proper synchronization.
/// - `mmap` being called on memory that is still in use, leading to segmentation faults.
///
/// # Advantages
///
/// - Provides an efficient way to work with large files, minimizing memory usage by paging data on demand.
/// - Offers memory-like access semantics for read and write operations, making it simpler to manipulate file data.
/// - Eliminates the need for manually reading/writing files in chunks.
pub(crate) struct FileBackedBuffer {
    /// The file backing this buffer. Must remain valid for the lifetime of the buffer.
    file: std::fs::File,
    /// Pointer to the memory-mapped region. Unsafe to dereference if the region is unmapped
    /// or access is performed outside the valid range.
    ptr: *mut u8,
    /// The length of the memory-mapped region in bytes. Used to ensure bounds are enforced.
    length: usize,
}

impl FileBackedBuffer {
    /// Opens or creates a memory-mapped file-backed buffer.
    ///
    /// # Arguments
    ///
    /// * `path` - The path to the file to be used for memory mapping.
    /// * `length` - The size of the memory-mapped buffer in bytes.
    ///
    /// # Returns
    ///
    /// A `Result` containing the created `FileBackedBuffer` on success,
    /// or an error message on failure.
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
        Ok(FileBackedBuffer { file, length, ptr: ptr.cast::<u8>() })
    }
}

impl Drop for FileBackedBuffer {
    /// Cleans up the memory-mapped region when the `FileBackedBuffer` is dropped.
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

    /// Provides immutable access to the contents of the memory-mapped region.
    fn deref(&self) -> &Self::Target { unsafe { slice::from_raw_parts(self.ptr, self.length) } }
}

impl DerefMut for FileBackedBuffer {
    /// Provides mutable access to the contents of the memory-mapped region.
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe { slice::from_raw_parts_mut(self.ptr, self.length) }
    }
}
