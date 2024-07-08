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
include( CheckSymbolExists )
include( CheckTypeSize )

function( OMPT_HEADER_CHECK )

    check_include_file( "omp-tools.h" HAVE_OMP_TOOLS_HEADER )
    if( HAVE_OMP_TOOLS_HEADER )
    set( CMAKE_EXTRA_INCLUDE_FILES "omp-tools.h" )
        # OpenMP 5.1 checks
        # Check for added callback values
        check_type_size( ompt_callback_target_emi OMPT_CALLBACK_TARGET_EMI LANGUAGE C )
        check_type_size( ompt_callback_target_data_op_emi OMPT_CALLBACK_TARGET_DATA_OP_EMI LANGUAGE C )
        check_type_size( ompt_callback_target_submit_emi OMPT_CALLBACK_TARGET_SUBMIT_EMI LANGUAGE C )
        check_type_size( ompt_callback_target_map_emi OMPT_CALLBACK_TARGET_MAP_EMI LANGUAGE C )
        check_type_size( ompt_callback_masked OMPT_CALLBACK_MASKED LANGUAGE C )
        # Check for new enum values
        # New fields in ompt_scope_endpoint_t
        check_type_size( ompt_scope_beginend OMPT_SCOPE_BEGINEND LANGUAGE C )
        # New fields in ompt_sync_region_t
        check_type_size( ompt_sync_region_barrier_implicit_workshare OMPT_SYNC_REGION_BARRIER_IMPLICIT_WORKSHARE LANGUAGE C )
        check_type_size( ompt_sync_region_barrier_implicit_parallel OMPT_SYNC_REGION_BARRIER_IMPLICIT_PARALLEL LANGUAGE C )
        check_type_size( ompt_sync_region_barrier_teams OMPT_SYNC_REGION_BARRIER_TEAMS LANGUAGE C )
        # New fields in ompt_target_data_op_t
        check_type_size( ompt_target_data_alloc_async OMPT_TARGET_DATA_ALLOC_ASYNC LANGUAGE C )
        check_type_size( ompt_target_data_transfer_to_device_async OMPT_TARGET_DATA_TRANSFER_TO_DEVICE_ASYNC LANGUAGE C )
        check_type_size( ompt_target_data_transfer_from_device_async OMPT_TARGET_DATA_TRANSFER_FROM_DEVICE_ASYNC LANGUAGE C )
        check_type_size( ompt_target_data_delete_async OMPT_TARGET_DATA_DELETE_ASYNC LANGUAGE C )
        # New fields in ompt_work_t
        check_type_size( ompt_work_scope OMPT_WORK_SCOPE LANGUAGE C )
        # New fields in ompt_task_flag_t
        check_type_size( ompt_task_taskwait OMPT_TASK_TASKWAIT LANGUAGE C )
        # New fields in ompt_task_status_t
        check_type_size( ompt_taskwait_complete OMPT_TASKWAIT_COMPLETE LANGUAGE C )
        # New fields in ompt_target_t
        check_type_size( ompt_target_nowait OMPT_TARGET_NOWAIT LANGUAGE C )
        check_type_size( ompt_target_enter_data_nowait OMPT_TARGET_ENTER_DATA_NOWAIT LANGUAGE C )
        check_type_size( ompt_target_exit_data_nowait OMPT_TARGET_EXIT_DATA_NOWAIT LANGUAGE C )
        check_type_size( ompt_target_update_nowait OMPT_TARGET_UPDATE_NOWAIT LANGUAGE C )
        # New fields in ompt_dependence_type_t
        check_type_size( ompt_dependence_type_inoutset OMPT_DEPENDENCE_TYPE_INOUTSET LANGUAGE C )

        # OpenMP 5.2 checks
        # No new callbacks
        # Check for new enum values
        # New fields in ompt_dispatch_t
        check_type_size( ompt_dispatch_ws_loop_chunk OMPT_DISPATCH_WS_LOOP_CHUNK LANGUAGE C )
        check_type_size( ompt_dispatch_taskloop_chunk OMPT_DISPATCH_TASKLOOP_CHUNK LANGUAGE C )
        check_type_size( ompt_dispatch_distribute_chunk OMPT_DISPATCH_DISTRIBUTE_CHUNK LANGUAGE C )
        # New fields in ompt_work_t
        check_type_size( ompt_work_loop_static OMPT_WORK_LOOP_STATIC LANGUAGE C )
        check_type_size( ompt_work_loop_dynamic OMPT_WORK_LOOP_DYNAMIC LANGUAGE C )
        check_type_size( ompt_work_loop_guided OMPT_WORK_LOOP_GUIDED LANGUAGE C )
        check_type_size( ompt_work_loop_other OMPT_WORK_LOOP_OTHER LANGUAGE C )
        # New fields in ompt_target_map_flag_t
        check_type_size( ompt_target_map_flag_always OMPT_TARGET_MAP_FLAG_ALWAYS LANGUAGE C )
        check_type_size( ompt_target_map_flag_present OMPT_TARGET_MAP_FLAG_PRESENT LANGUAGE C )
        check_type_size( ompt_target_map_flag_close OMPT_TARGET_MAP_FLAG_CLOSE LANGUAGE C )
        check_type_size( ompt_target_map_flag_shared OMPT_TARGET_MAP_FLAG_SHARED LANGUAGE C )

        # OpenMP 6.0 checks
        # TR13 needs to be released first
        set( CMAKE_EXTRA_INCLUDE_FILES )
    endif()

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
