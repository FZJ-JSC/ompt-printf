#
# Copyright (c) 2024,
# Forschungszentrum Juelich GmbH, Germany
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#

include( CheckIncludeFile )
include( CheckTypeSize )

function( OMPT_CHECK_HEADER )

    check_include_file( "omp-tools.h" HAVE_OMP_TOOLS_HEADER )
    set( CMAKE_EXTRA_INCLUDE_FILES "omp-tools.h" )
    # Check for OpenMP 5.1+ symbols
    check_type_size( ompt_sync_region_barrier_implicit_workshare OMPT_SYNC_REGION_BARRIER_IMPLICIT_WORKSHARE_SIZE LANGUAGE C )
    check_type_size( ompt_scope_beginend OMPT_SCOPE_BEGINEND_SIZE LANGUAGE C )
    check_type_size( ompt_task_taskwait OMPT_TASK_TASKWAIT_SIZE LANGUAGE C )
    check_type_size( ompt_sync_region_barrier_implicit_parallel OMPT_SYNC_REGION_BARRIER_IMPLICIT_PARALLEL_SIZE LANGUAGE C )
    check_type_size( ompt_sync_region_barrier_teams OMPT_SYNC_REGION_BARRIER_TEAMS_SIZE LANGUAGE C )
    check_type_size( ompt_work_loop_static OMPT_WORK_LOOP_STATIC_SIZE LANGUAGE C )
    check_type_size( ompt_work_loop_dynamic OMPT_WORK_LOOP_DYNAMIC_SIZE LANGUAGE C )
    check_type_size( ompt_work_loop_guided OMPT_WORK_LOOP_GUIDED_SIZE LANGUAGE C )
    check_type_size( ompt_work_loop_other OMPT_WORK_LOOP_OTHER_SIZE LANGUAGE C )
    check_type_size( ompt_dispatch_ws_loop_chunk OMPT_DISPATCH_WS_LOOP_CHUNK_SIZE LANGUAGE C )
    check_type_size( ompt_dispatch_taskloop_chunk OMPT_DISPATCH_TASKLOOP_CHUNK_SIZE LANGUAGE C )
    check_type_size( ompt_dispatch_distribute_chunk OMPT_DISPATCH_DISTRIBUTE_CHUNK_SIZE LANGUAGE C )
    set( CMAKE_EXTRA_INCLUDE_FILES )

endfunction()

set( COMPILER_TOOLCHAIN GCC CACHE STRING "Set compiler toolchain. Will overwrite native CMake commands" )
function( OMPT_ADDITIONAL_CFLAGS )

    if( COMPILER_TOOLCHAIN STREQUAL "GCC" )
        set( OMPT_C_FLAGS   "-fopenmp" PARENT_SCOPE )
        set( OMPT_CXX_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "Clang" )
        set( OMPT_C_FLAGS   "-fopenmp" PARENT_SCOPE )
        set( OMPT_CXX_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "NVHPC" )
        set( OMPT_C_FLAGS   "-mp=ompt" PARENT_SCOPE )
        set( OMPT_CXX_FLAGS "-mp=ompt" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "AMDClang" )
        set( OMPT_C_FLAGS   "-fopenmp" PARENT_SCOPE )
        set( OMPT_CXX_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "AOCC" )
        set( OMPT_C_FLAGS   "-fopenmp" PARENT_SCOPE )
        set( OMPT_CXX_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "Intel" )
        set( OMPT_C_FLAGS   "-fopenmp" PARENT_SCOPE )
        set( OMPT_CXX_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "oneAPI" )
        set( OMPT_C_FLAGS   "-fiopenmp" PARENT_SCOPE )
        set( OMPT_CXX_FLAGS "-fiopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "Cray" )
        set( OMPT_C_FLAGS   "-fopenmp" PARENT_SCOPE )
        set( OMPT_CXX_FLAGS "-fopenmp" PARENT_SCOPE )
    else()
        message( FATAL_ERROR "Compiler toolchain not supported. Supported toolchains are: GCC, Clang, NVHPC, AMDClang, AOCC, Intel, oneAPI, Cray" )
    endif()

endfunction()


function( OMPT_ADDITIONAL_LDFLAGS )

    if( COMPILER_TOOLCHAIN STREQUAL "GCC" )
        set( OMPT_LINK_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "Clang" )
        set( OMPT_LINK_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "NVHPC" )
        set( OMPT_LINK_FLAGS "-mp=ompt" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "AMDClang" )
        set( OMPT_LINK_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "AOCC" )
        set( OMPT_LINK_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "Intel" )
        set( OMPT_LINK_FLAGS "-fopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "oneAPI" )
        set( OMPT_LINK_FLAGS "-fiopenmp" PARENT_SCOPE )
    elseif( COMPILER_TOOLCHAIN STREQUAL "Cray" )
        set( OMPT_LINK_FLAGS "-fopenmp" PARENT_SCOPE )
    else()
        message( FATAL_ERROR "Compiler toolchain not supported. Supported toolchains are: GCC, Clang, NVHPC, AMDClang, AOCC, Intel, oneAPI, Cray" )
    endif()

endfunction()
