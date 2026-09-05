# gz-transport13 13.5.0 EINTR vendor patch

This directory freezes the only upstream modification permitted for the final
Gazebo runtime. It is based on the official `gazebosim/gz-transport` tag
`gz-transport13_13.5.0` at commit
`7232d0b163b08f79dd390ad9913d6ea72efaf209`.

The upstream `NodeShared::Publish` sends one publication as four frames, or
five when topic statistics are enabled. The patch retries only the current
`zmq::message_t` when that frame's `send` throws `zmq::error_t` with
`num() == EINTR`. It makes at most three retries. A non-EINTR exception or a
fourth interrupted attempt is rethrown to the unchanged outer handler, which
logs the error and returns `false`. It never retries `NodeShared::Publish` or
replays frames already accepted by ZeroMQ.

Build it only in a fresh, user-controlled prefix after the Windows and Linux
memory start gates pass:

```bash
bash scripts/build_gz_transport13_eintr_vendor.sh \
  --work-root /home/zhexu/tzcup_gz_transport13_eintr_build \
  --install-prefix /home/zhexu/tzcup_gz_transport13_eintr_runtime \
  --parallel-workers 2
```

The builder clones the pinned official commit, verifies the Git tree, original
source hash and patch hash, applies the patch, checks the patched source hash,
builds with at most two workers, and uses the installed 13.5.0 library as the
ABI reference. Every exact mangled symbol that the reference exports in the
public `gz::transport::v13` namespace (including vtables, typeinfo and thunks)
must remain present. Compiler-internal weak definitions and additional inline
or template instantiations may differ; they are not public ABI. The
patch-private retry helper is required to have internal linkage and must not be
present in the dynamic symbol table. The validator also verifies the SONAME
and pinned system Protobuf linkage before emitting its JSON report. It never
writes `/opt` or another system prefix.

For final acceptance, pass the fresh merged runtime's `install` directory as
`--install-prefix` before the ROS packages are built. The builder replaces the
normal shared-library alias symlinks with byte-identical regular files so the
merged runtime stays non-symlink. Prepend that same `install/lib` to
`LD_LIBRARY_PATH` (and the prefix to `CMAKE_PREFIX_PATH`) while building and
running; the generated `activate_patched_runtime.sh` contains those two exact
exports. An independent test prefix is evidence of compilation only and must
not be described as the final merged runtime.
