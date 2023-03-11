# MAX14915 C Library
Basic library to control the MAX14915 SPI chip, a compact, octal high-side switch with diagnostics and a wide supply range.

It uses a basic I/O interface in the struct to access the SPI. See https://github.com/gfcwfzkm/library_template/blob/main/INTERFACE.md for more information about that interface.
The library has been written in a way to be "object-oriented-ish" - so the use of multiple of these high-side switch chips with this library is no problem.

As it is usual with my software libraries / projects, most of the documentation is written in the header files.