# EGL loader for headless CI

This is the libglvnd 1.7.0 EGL dispatch loader, taken from the installed Debian amd64 package `libegl1 1.7.0-1+b2`. The library is 88,544 bytes, base64 encoded to keep the GitHub app transport textual. Its SHA-256 is `f4b60a4618fbf760f395f64dfd53320446ba1011151a80e411742b3232b194e6`.

Source: https://sources.debian.org/src/libglvnd/1.7.0-1/ and https://gitlab.freedesktop.org/glvnd/libglvnd. Copyright and license notices are retained in `COPYRIGHT.libglvnd`. The loader's applicable upstream license is MIT.

The loader requires Linux x86-64, glibc 2.38 or later and the runner's existing `libGLdispatch.so.0`. It is compatible with Ubuntu 24.04's glibc 2.39 and libglvnd 1.7.0. No rendering vendor, GPU driver or system configuration is bundled. The installer first uses existing runtime libraries; when EGL is missing it checks the payload hash, writes it only to the job temporary directory and sets that job's library search path. This avoids requiring Ubuntu package-server DNS for CPU point-cloud processing.
