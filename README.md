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
| 0    | Disable the tool entirely               |
| 1    | Enable tool, but print no information   |
| 2    | Print all events, but without arguments |
| 3    | Print all events with arguments         |

These modes are implemented via C++ templates. This should keep the overhead
of the tool as low as possible when choosing between the different modes.
By default, the tool is built with mode 2.

## Requirements

These are the requirements to build the tool with the available build system.
The library can also be built manually, but is not covered here.

- CMake 3.10 or newer
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
[-1][ompt_start_tool] omp_version = 201611 | runtime_version = LLVM OMP version: 5.0.20140926
[-1][tool_initialize] lookup = 0x77bdfa723820 | initial_device_num = 0 | tool_data = 0x77bdfa666428
[-1][tool_initialize]       thread_begin = always
[-1][tool_initialize]         thread_end = always
[-1][tool_initialize]     parallel_begin = always
[-1][tool_initialize]       parallel_end = always
[-1][tool_initialize]        task_create = always
[-1][tool_initialize]      task_schedule = always
[-1][tool_initialize]      implicit_task = always
[-1][tool_initialize]   sync_region_wait = always
[-1][tool_initialize]     mutex_released = always
[-1][tool_initialize]        dependences = always
[-1][tool_initialize]    task_dependence = always
[-1][tool_initialize]               work = always
[-1][tool_initialize]             masked = always
[-1][tool_initialize]        sync_region = always
[-1][tool_initialize]          lock_init = always
[-1][tool_initialize]       lock_destroy = always
[-1][tool_initialize]      mutex_acquire = always
[-1][tool_initialize]     mutex_acquired = always
[-1][tool_initialize]          nest_lock = always
[-1][tool_initialize]              flush = always
[-1][tool_initialize]             cancel = always
[-1][tool_initialize]          reduction = always
[-1][tool_initialize]           dispatch = always
[-1][tool_initialize]  device_initialize = always
[-1][tool_initialize]    device_finalize = always
[-1][tool_initialize]        device_load = always
[-1][tool_initialize]      device_unload = never
[-1][tool_initialize]         target_emi = always
[-1][tool_initialize]     target_map_emi = never
[-1][tool_initialize] target_data_op_emi = always
[-1][tool_initialize]  target_submit_emi = always
[0][callback_thread_begin] thread_type = initial | thread_data = 0x624479fb3808
[0][callback_implicit_task] endpoint = begin | parallel_data->value = 0 (0x624479faef20) | task_data->value = 555000001 (0x624479faf840) | actual_parallelism = 1 | index = 1 | flags = initial
[0][callback_parallel_begin] encountering_task_data->value = 555000001 (0x624479faf840) | encountering_task_frame = 0x624479faf828 | parallel_data->value = 666000001 (0x7ffe86f3cb00) | requested_parallelism = 2 | flags = runtime_team | codeptr_ra = 0x624479026166
[0][callback_implicit_task] endpoint = begin | parallel_data->value = 666000001 (0x624479fafb20) | task_data->value = 555000002 (0x624479fb0f00) | actual_parallelism = 2 | index = 0 | flags = implicit
[0][callback_sync_region] kind = barrier_implicit | endpoint = begin | parallel_data->value = 666000001 (0x624479fafb20) | task_data->value = 555000002 (0x624479fb0f00) | codeptr_ra = 0x624479026166
[0][callback_sync_region_wait] kind = barrier_implicit | endpoint = begin | parallel_data->value = 666000001 (0x624479fafb20) | task_data->value = 555000002 (0x624479fb0f00) | codeptr_ra = 0x624479026166
[1][callback_thread_begin] thread_type = worker | thread_data = 0x624479fbba88
[1][callback_implicit_task] endpoint = begin | parallel_data->value = 666000001 (0x624479fafb20) | task_data->value = 555000003 (0x624479fb1040) | actual_parallelism = 2 | index = 1 | flags = implicit
[1][callback_sync_region] kind = barrier_implicit | endpoint = begin | parallel_data->value = 666000001 (0x624479fafb20) | task_data->value = 555000003 (0x624479fb1040) | codeptr_ra = (nil)
[1][callback_sync_region_wait] kind = barrier_implicit | endpoint = begin | parallel_data->value = 666000001 (0x624479fafb20) | task_data->value = 555000003 (0x624479fb1040) | codeptr_ra = (nil)
[0][callback_sync_region_wait] kind = barrier_implicit | endpoint = end | parallel_data->value = 666666666 ((nil)) | task_data->value = 555000002 (0x624479fb0f00) | codeptr_ra = 0x624479026166
[0][callback_sync_region] kind = barrier_implicit | endpoint = end | parallel_data->value = 666666666 ((nil)) | task_data->value = 555000002 (0x624479fb0f00) | codeptr_ra = 0x624479026166
[0][callback_implicit_task] endpoint = end | parallel_data->value = 666666666 ((nil)) | task_data->value = 555000002 (0x624479fb0f00) | actual_parallelism = 2 | index = 0 | flags = implicit
[0][callback_parallel_end] parallel_data->value = 666000001 (0x624479fafb20) | encountering_task_data->value = 555000001 (0x624479faf840) | flags = runtime_team | codeptr_ra = 0x624479026166
[0][callback_implicit_task] endpoint = end | parallel_data->value = 0 (0x624479faef20) | task_data->value = 555000001 (0x624479faf840) | actual_parallelism = 0 | index = 1 | flags = initial
[0][callback_thread_end] thread_data = 0x624479fb3808
[1][callback_sync_region_wait] kind = barrier_implicit | endpoint = end | parallel_data->value = 666666666 ((nil)) | task_data->value = 555000003 (0x624479fbba90) | codeptr_ra = (nil)
[1][callback_sync_region] kind = barrier_implicit | endpoint = end | parallel_data->value = 666666666 ((nil)) | task_data->value = 555000003 (0x624479fbba90) | codeptr_ra = (nil)
[1][callback_implicit_task] endpoint = end | parallel_data->value = 666666666 ((nil)) | task_data->value = 555000003 (0x624479fbba90) | actual_parallelism = 0 | index = 1 | flags = implicit
[1][callback_thread_end] thread_data = 0x624479fbba88
[0][tool_finalize] tool_data = 0x77bdfa666428
```
