#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/stat.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/sysmacros.h>
#include <unistd.h>

static void copy_timestamp(struct statx_timestamp *dst, const struct timespec *src) {
  dst->tv_sec = src->tv_sec;
  dst->tv_nsec = (uint32_t)src->tv_nsec;
  dst->__reserved = 0;
}

int statx(int dirfd, const char *pathname, int flags, unsigned int mask,
          struct statx *buffer) {
  (void)mask;
  struct stat value;
  int supported_flags = flags & (AT_EMPTY_PATH | AT_NO_AUTOMOUNT | AT_SYMLINK_NOFOLLOW);
  if (fstatat(dirfd, pathname, &value, supported_flags) != 0)
    return -1;
  memset(buffer, 0, sizeof(*buffer));
  buffer->stx_mask = STATX_BASIC_STATS;
  buffer->stx_blksize = (uint32_t)value.st_blksize;
  buffer->stx_nlink = (uint32_t)value.st_nlink;
  buffer->stx_uid = value.st_uid;
  buffer->stx_gid = value.st_gid;
  buffer->stx_mode = (uint16_t)value.st_mode;
  buffer->stx_ino = value.st_ino;
  buffer->stx_size = value.st_size;
  buffer->stx_blocks = value.st_blocks;
  copy_timestamp(&buffer->stx_atime, &value.st_atim);
  copy_timestamp(&buffer->stx_btime, &value.st_ctim);
  copy_timestamp(&buffer->stx_ctime, &value.st_ctim);
  copy_timestamp(&buffer->stx_mtime, &value.st_mtim);
  buffer->stx_rdev_major = major(value.st_rdev);
  buffer->stx_rdev_minor = minor(value.st_rdev);
  buffer->stx_dev_major = major(value.st_dev);
  buffer->stx_dev_minor = minor(value.st_dev);
  return 0;
}

int faccessat2(int dirfd, const char *pathname, int mode, int flags) {
  if (flags == 0)
    return (int)syscall(SYS_faccessat, dirfd, pathname, mode);
  struct stat value;
  if (fstatat(dirfd, pathname, &value, flags & (AT_EMPTY_PATH | AT_SYMLINK_NOFOLLOW)) != 0)
    return -1;
  if (mode == F_OK)
    return 0;
  if (geteuid() == 0) {
    if ((mode & X_OK) && !(value.st_mode & (S_IXUSR | S_IXGRP | S_IXOTH))) {
      errno = EACCES;
      return -1;
    }
    return 0;
  }
  errno = ENOTSUP;
  return -1;
}

int faccessat(int dirfd, const char *pathname, int mode, int flags) {
  return faccessat2(dirfd, pathname, mode, flags);
}

int access(const char *pathname, int mode) {
  return faccessat2(AT_FDCWD, pathname, mode, 0);
}

int euidaccess(const char *pathname, int mode) {
  return faccessat2(AT_FDCWD, pathname, mode, AT_EACCESS);
}

int eaccess(const char *pathname, int mode) {
  return euidaccess(pathname, mode);
}

