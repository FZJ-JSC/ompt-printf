OpenMP Tools Interface Example Tool: ompt-printf
====

## Description

This tool is a simple example of how to use the OpenMP Tools Interface (OMPT)
to collect information about the execution of an OpenMP program.

The OpenMP Tools Interface is part of the OpenMP standard since version 5.0
and allows programs like performance and correctness tools to collect
information such as parallel regions, tasking, worksharing, offloading and
more.
More information can be found in the [documentation](https://www.openmp.org/wp-content/uploads/OpenMP-API-Specification-5-2.pdf).

This tool implements the callback-based part of the OMPT interface. The tool
is registered through the standardised `ompt_start_tool` function:

```c++
extern "C" ompt_start_tool_result_t *
ompt_start_tool( unsigned int omp_version,
                 const char* runtime_version )
```

Here, we decide between three different modes of operation, which can
be controlled via the environment variable `OMPT_PRINTF_MODE`:

| Mode | Description                             |
|------|-----------------------------------------|
| 0    | Print no information                    |
| 1    | Print all events, but without arguments |
| 2    | Print all events with arguments         |

These modes are implemented via C++ templates. This should keep the overhead
of the tool as low as possible when choosing between the different modes.
By default, the tool is built with mode 2.

## Requirements

These are the requirements to build the tool with the available build system.
The library can also be built manually, but is not covered here.

- CMake 3.24 or newer
- A C++17 compliant compiler
- An OpenMP runtime supporting the OMPT interface (e.g. LLVM/Clang, oneAPI,
  NVHPC, ...)
- CMake will only check the presence of the `omp-tools.h` header file.
  Actual runtime support is not checked.
  
## Build the library

The library can easily be built with CMake. On a checked-out or downloaded
copy, simply run the following commands:

```bash
$ cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCOMPILER_TOOLCHAIN=[your-vendor]
$ cmake --build build
```

The `COMPILER_TOOLCHAIN` variable is used in favor of setting the compiler
manually via `CMAKE_C_COMPILER` and `CMAKE_CXX_COMPILER`, to ensure that
additional flags some compilers (like NVHPC) require are set correctly.
`find_package( OpenMP )` is not sufficient in this case.

The following vendors are supported and set these flags:

| Vendor   | C Compiler | C++ Compiler | CFLAGS      | CXXFLAGS    | LDFLAGS     |
|----------|------------|--------------|-------------|-------------|-------------|
| GNU      | `gcc`      | `g++`        | `-fopenmp`  | `-fopenmp`  | `-fopenmp`  |
| Clang    | `clang`    | `clang++`    | `-fopenmp`  | `-fopenmp`  | `-fopenmp`  |
| NVHPC    | `nvc`      | `nvc++`      | `-mp=ompt`  | `-mp=ompt`  | `-mp=ompt`  |
| AMDClang | `amdclang` | `amdclang++` | `-fopenmp`  | `-fopenmp`  | `-fopenmp`  |
| AOCC     | `clang`    | `clang++`    | `-fopenmp`  | `-fopenmp`  | `-fopenmp`  |
| Intel    | `icc`      | `icpc`       | `-fopenmp`  | `-fopenmp`  | `-fopenmp`  |
| oneAPI   | `icx`      | `icpx`       | `-fiopenmp` | `-fiopenmp` | `-fiopenmp` |
| Cray     | `cc`       | `CC`         | `-fopenmp`  | `-fopenmp`  | `-fopenmp`  |

After building the library, the tool can be used with the environment variable
`OMP_TOOL_LIBRARIES`. Please note that NVHPC requires adding `-mp=ompt` when building
a program since this affects code generation and the linked OpenMP library.

Here is an example on building a program with the tool enabled (Clang 17.0.6, Arch Linux):
```bash
$ cat my-openmp-example.c
int main( void )
{
    #pragma omp parallel
    {}
    return 0;
}
$ export OMP_TOOL_LIBRARIES=$(pwd)/build/src/libompt-printf.so
$ clang -fopenmp my-openmp-example.c -o my-openmp-example
$ OMP_NUM_THREADS=2 ./my-openmp-example
[ompt_start_tool] tid = -1 | omp_version = 201611 | runtime_version = LLVM OMP version: 5.0.20140926
[tool_initialize] tid = -1 | lookup = 0x71a593834e00 | initial_device_num = 0 | tool_data = 0x71a5938c55b8
[tool_initialize] tid = -1 |       thread_begin = always
[tool_initialize] tid = -1 |         thread_end = always
[tool_initialize] tid = -1 |     parallel_begin = always
[tool_initialize] tid = -1 |       parallel_end = always
[tool_initialize] tid = -1 |        task_create = always
[tool_initialize] tid = -1 |      task_schedule = always
[tool_initialize] tid = -1 |      implicit_task = always
[tool_initialize] tid = -1 |   sync_region_wait = always
[tool_initialize] tid = -1 |     mutex_released = always
[tool_initialize] tid = -1 |        dependences = always
[tool_initialize] tid = -1 |    task_dependence = always
[tool_initialize] tid = -1 |               work = always
[tool_initialize] tid = -1 |             masked = always
[tool_initialize] tid = -1 |        sync_region = always
[tool_initialize] tid = -1 |          lock_init = always
[tool_initialize] tid = -1 |       lock_destroy = always
[tool_initialize] tid = -1 |      mutex_acquire = always
[tool_initialize] tid = -1 |     mutex_acquired = always
[tool_initialize] tid = -1 |          nest_lock = always
[tool_initialize] tid = -1 |              flush = always
[tool_initialize] tid = -1 |             cancel = always
[tool_initialize] tid = -1 |          reduction = always
[tool_initialize] tid = -1 |           dispatch = always
[tool_initialize] tid = -1 |  device_initialize = always
[tool_initialize] tid = -1 |    device_finalize = always
[tool_initialize] tid = -1 |        device_load = always
[tool_initialize] tid = -1 |      device_unload = never
[tool_initialize] tid = -1 |         target_emi = always
[tool_initialize] tid = -1 |     target_map_emi = never
[tool_initialize] tid = -1 | target_data_op_emi = always
[tool_initialize] tid = -1 |  target_submit_emi = always
[callback_thread_begin] tid = 0 | thread_type = initial | thread_data = 0x5948ecc041c8
[callback_implicit_task] tid = 0 | endpoint = begin | parallel_data->value = 0 | task_data->value = 555000001 | actual_parallelism = 1 | index = 1 | flags = initial
[callback_parallel_begin] tid = 0 | encountering_task_data->value = 555000001 | encountering_task_frame = 0x5948ecbfc5a8 | parallel_data->value 666000001 | requested_parallelism = 2 | flags = runtime_team | codeptr_ra = 0x5948eb72215b
[callback_implicit_task] tid = 0 | endpoint = begin | parallel_data->value = 666000001 | task_data->value = 555000002 | actual_parallelism = 2 | index = 0 | flags = implicit
[callback_sync_region] tid = 0 | kind = barrier_implicit | endpoint = begin | parallel_data->value = 666000001 | task_data->value = 555000002 | codeptr_ra = 0x5948eb72215b
[callback_sync_region_wait] tid = 0 | kind = barrier_implicit | endpoint = begin | parallel_data->value = 666000001 | task_data->value = 555000002 | codeptr_ra = 0x5948eb72215b
[callback_thread_begin] tid = 1 | thread_type = worker | thread_data = 0x5948ecc078c8
[callback_implicit_task] tid = 1 | endpoint = begin | parallel_data->value = 666000001 | task_data->value = 555000003 | actual_parallelism = 2 | index = 1 | flags = implicit
[callback_sync_region] tid = 1 | kind = barrier_implicit | endpoint = begin | parallel_data->value = 666000001 | task_data->value = 555000003 | codeptr_ra = (nil)
[callback_sync_region_wait] tid = 1 | kind = barrier_implicit | endpoint = begin | parallel_data->value = 666000001 | task_data->value = 555000003 | codeptr_ra = (nil)
[callback_sync_region_wait] tid = 0 | kind = barrier_implicit | endpoint = end | parallel_data->value = 666666666 | task_data->value = 555000002 | codeptr_ra = 0x5948eb72215b
[callback_sync_region] tid = 0 | kind = barrier_implicit | endpoint = end | parallel_data->value = 666666666 | task_data->value = 555000002 | codeptr_ra = 0x5948eb72215b
[callback_implicit_task] tid = 0 | endpoint = end | parallel_data->value = 666666666 | task_data->value = 555000002 | actual_parallelism = 2 | index = 0 | flags = implicit
[callback_parallel_end] tid = 0 | parallel_data->value = 0x5948ecbfad60 | encountering_task_data->value = 555000001 | flags = runtime_team | codeptr_ra = 0x5948eb72215b
[callback_implicit_task] tid = 0 | endpoint = end | parallel_data->value = 0 | task_data->value = 555000001 | actual_parallelism = 0 | index = 1 | flags = initial
[callback_thread_end] tid = 0 | thread_data = 0x5948ecc041c8
[callback_sync_region_wait] tid = 1 | kind = barrier_implicit | endpoint = end | parallel_data->value = 666666666 | task_data->value = 555000003 | codeptr_ra = (nil)
[callback_sync_region] tid = 1 | kind = barrier_implicit | endpoint = end | parallel_data->value = 666666666 | task_data->value = 555000003 | codeptr_ra = (nil)
[callback_implicit_task] tid = 1 | endpoint = end | parallel_data->value = 666666666 | task_data->value = 555000003 | actual_parallelism = 0 | index = 1 | flags = implicit
[callback_thread_end] tid = 1 | thread_data = 0x5948ecc078c8
[tool_finalize] tid = 0 | tool_data = 0x71a5938c55b8
```
