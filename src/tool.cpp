/*
 * Copyright (c) 2024,
 * Forschungszentrum Juelich GmbH, Germany
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <atomic>
#include <cassert>
#include <cstdarg>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <string>

#include <omp.h>
#include <omp-tools.h>
#include <unordered_map>

#include "tool-definitions.h"

/* Global variables */

constexpr int32_t             unknown_thread_id = -1;
constexpr ompt_id_t           unknown_task_id   = 555555555;
static std::atomic<ompt_id_t> next_task_id        { 555000000 };
constexpr ompt_id_t           unknown_parallel_id = 666666666;
static std::atomic<ompt_id_t> next_parallel_id    { 666000000 };
constexpr ompt_id_t           unknown_target_id = 777777777;
static std::atomic<ompt_id_t> next_target_id      { 777000000 };
constexpr ompt_id_t           unknown_host_op_id = 888888888;
static std::atomic<ompt_id_t> next_host_op_id     { 888000000 };

static __thread int32_t     thread_id = unknown_thread_id;
static std::atomic<int32_t> next_thread_id { 0 };

/* Helpers */

enum class printf_mode : int
{
    disable_tool          = 0,
    disable_output        = 1,
    callback              = 2,
    callback_include_args = 3,

    num_modes
};

typedef struct registration_data_t
{
    ompt_callbacks_t event;
    ompt_callback_t  callback;
    const char*      name;
} registration_data_t;

typedef struct device_functions_t
{
    /* Required functions */
    ompt_set_trace_ompt_t        set_trace_ompt;
    ompt_start_trace_t           start_trace;
    ompt_flush_trace_t           flush_trace;
    ompt_advance_buffer_cursor_t advance_buffer_cursor;
    ompt_get_record_ompt_t       get_record_ompt;
    /* Optional functions */
    ompt_stop_trace_t            stop_trace;
    ompt_pause_trace_t           pause_trace;
#if HAVE( OMPT_GET_BUFFER_LIMITS )
    ompt_get_buffer_limits_t     get_buffer_limits;  // ADDED_60
#endif
} device_functions_t;

typedef struct device_t
{
    ompt_device_t*     address = nullptr;
    std::string        name;
    device_functions_t device_functions = { nullptr };
#if HAVE( OMPT_GET_BUFFER_LIMITS )
    size_t             recommended_buffer_size   = 0;
    int                recommended_buffer_amount = 0;
    int                allocated_buffers         = 0;
#endif
} device_t;

static std::unordered_map<ompt_id_t, device_t*>&
get_devices()
{
    static std::unordered_map<ompt_id_t, device_t*>* d = NULL;
    if ( d == NULL )
    {
        d = new std::unordered_map<ompt_id_t, device_t*>;
    }
    return *d;
}

/* printf conversion output */

static inline int
atomic_printf( const char* format, ... )
{
    static std::mutex printf_mutex;
    int               ret;
    {
        std::lock_guard<std::mutex> lock( printf_mutex );
        va_list                     args;
        va_start( args, format );
        ret = printf( "[%d]", thread_id );
        if ( ret == EOF )
        {
            va_end( args );
            return ret;
        }
        ret = vprintf( format, args );
        va_end( args );
        fflush( stdout );
    }
    return ret;
}

static inline int
print_function_name( const char* function )
{
    return atomic_printf( "[%s]\n", function );
}

/* typedef enum ompt_set_result_t {
 *   ompt_set_error            = 0,
 *   ompt_set_never            = 1,
 *   ompt_set_impossible       = 2,
 *   ompt_set_sometimes        = 3,
 *   ompt_set_sometimes_paired = 4,
 *   ompt_set_always           = 5
 * } ompt_set_result_t; */
static inline std::string
set_result2string( const ompt_set_result_t t )
{
    switch ( t )
    {
        case ompt_set_error:
            return "error";
        case ompt_set_never:
            return "never";
        case ompt_set_impossible:
            return "impossible";
        case ompt_set_sometimes:
            return "sometimes";
        case ompt_set_sometimes_paired:
            return "sometimes_paired";
        case ompt_set_always:
            return "always";
        default:
            assert( false && "Unknown ompt_set_result_t" );
    }
    return "";
}

/* typedef enum ompt_thread_t {
 *   ompt_thread_initial  = 1,
 *   ompt_thread_worker   = 2,
 *   ompt_thread_other    = 3,
 *   ompt_thread_unknown  = 4
 * } ompt_thread_t; */
static inline std::string
thread2string( const ompt_thread_t t )
{
    switch ( t )
    {
        case ompt_thread_initial:
            return "initial";
        case ompt_thread_worker:
            return "worker";
        case ompt_thread_other:
            return "other";
        case ompt_thread_unknown:
            return "unknown";
        default:
            assert( false && "Unknown ompt_thread_t" );
    }
    return "";
}

/* typedef enum ompt_parallel_flag_t {
 *   ompt_parallel_invoker_program = 0x00000001,
 *   ompt_parallel_invoker_runtime = 0x00000002,
 *   ompt_parallel_league          = 0x40000000,
 *   ompt_parallel_team            = 0x80000000
 * } ompt_parallel_flag_t; */
static inline std::string
parallel_flag2string( uint32_t flags )
{
    std::stringstream result;
    if ( flags & ompt_parallel_invoker_program )
    {
        result << "program";
        flags -= ompt_parallel_invoker_program;
    }
    else if ( flags & ompt_parallel_invoker_runtime )
    {
        result << "runtime";
        flags -= ompt_parallel_invoker_runtime;
    }

    if ( flags & ompt_parallel_league )
    {
        result << "_league";
        flags -= ompt_parallel_league;
    }
    else if ( flags & ompt_parallel_team )
    {
        result << "_team";
        flags -= ompt_parallel_team;
    }
    assert( flags == 0 && "Unknown ompt_parallel_flag_t" );
    return result.str();
}

/* typedef enum ompt_task_flag_t {
 *   ompt_task_initial    = 0x00000001,
 *   ompt_task_implicit   = 0x00000002,
 *   ompt_task_explicit   = 0x00000004,
 *   ompt_task_target     = 0x00000008,
 *   ompt_task_taskwait   = 0x00000010, ADDED_51
 *   ompt_task_importing  = 0x02000000, ADDED_60
 *   ompt_task_exporting  = 0x04000000, ADDED_60
 *   ompt_task_undeferred = 0x08000000,
 *   ompt_task_untied     = 0x10000000,
 *   ompt_task_final      = 0x20000000,
 *   ompt_task_mergeable  = 0x40000000,
 *   ompt_task_merged     = 0x80000000
 * } ompt_task_flag_t; */
static inline std::string
task_flag2string( uint32_t flags )
{
    std::stringstream result;
    if ( flags & ompt_task_initial )
    {
        result << "initial";
        flags -= ompt_task_initial;
    }
    else if ( flags & ompt_task_implicit )
    {
        result << "implicit";
        flags -= ompt_task_implicit;
    }
    else if ( flags & ompt_task_explicit )
    {
        result << "explicit";
        flags -= ompt_task_explicit;
    }
    else if ( flags & ompt_task_target )
    {
        result << "_target";
        flags -= ompt_task_target;
    }
#if HAVE( OMPT_TASK_TASKWAIT )
    else if ( flags & ompt_task_taskwait )
    {
        result << "_taskwait";
        flags -= ompt_task_taskwait;
    }
#endif

    if ( flags & ompt_task_undeferred )
    {
        result << "_undeferred";
        flags -= ompt_task_undeferred;
    }
    else if ( flags & ompt_task_untied )
    {
        result << "_untied";
        flags -= ompt_task_untied;
    }
    else if ( flags & ompt_task_final )
    {
        result << "_final";
        flags -= ompt_task_final;
    }
    else if ( flags & ompt_task_mergeable )
    {
        result << "_mergeable";
        flags -= ompt_task_mergeable;
    }
    else if ( flags & ompt_task_merged )
    {
        result << "_merged";
        flags -= ompt_task_merged;
    }

#if HAVE( OMPT_TASK_IMPORTING ) && HAVE( OMPT_TASK_EXPORTING )
    if ( flags & ompt_task_importing )
    {
        result << "_importing";
        flags -= ompt_task_importing;
    }
    else if ( flags & ompt_task_exporting )
    {
        result << "_exporting";
        flags -= ompt_task_exporting;
    }
#endif

    assert( flags == 0 && "Unknown ompt_task_flag_t" );
    return result.str();
}

/* typedef enum ompt_scope_endpoint_t {
 *   ompt_scope_begin    = 1,
 *   ompt_scope_end      = 2,
 *   ompt_scope_beginend = 3, ADDED_51
 * } ompt_scope_endpoint_t; */
static inline std::string
endpoint2string( ompt_scope_endpoint_t t )
{
    switch ( t )
    {
        case ompt_scope_begin:
            return "begin";
        case ompt_scope_end:
            return "end";
#if HAVE( OMPT_SCOPE_BEGINEND )
        case ompt_scope_beginend:
            return "beginend";
#endif
        default:
            assert( false && "Unknown ompt_scope_endpoint_t" );
    }
    return "";
}

/* typedef enum ompt_task_status_t {
 *   ompt_task_complete      = 1,
 *   ompt_task_yield         = 2,
 *   ompt_task_cancel        = 3,
 *   ompt_task_detach        = 4,
 *   ompt_task_early_fulfill = 5,
 *   ompt_task_late_fulfill  = 6,
 *   ompt_task_switch        = 7,
 *   ompt_taskwait_complete  = 8, ADDED_51
 * } ompt_task_status_t; */
static inline std::string
task_status2string( ompt_task_status_t t )
{
    switch ( t )
    {
        case ompt_task_complete:
            return "complete";
        case ompt_task_yield:
            return "yield";
        case ompt_task_cancel:
            return "cancel";
        case ompt_task_detach:
            return "detach";
        case ompt_task_early_fulfill:
            return "early_fulfill";
        case ompt_task_late_fulfill:
            return "late_fulfill";
        case ompt_task_switch:
            return "switch";
#if HAVE( OMPT_TASKWAIT_COMPLETE )
        case ompt_taskwait_complete:
            return "taskwait_complete";
#endif
        default:
            assert( false && "Unknown ompt_task_status_t" );
    }
    return "";
}

/* typedef enum ompt_mutex_t {
 *     ompt_mutex_lock           = 1,
 *     ompt_mutex_test_lock      = 2,
 *     ompt_mutex_nest_lock      = 3,
 *     ompt_mutex_test_nest_lock = 4,
 *     ompt_mutex_critical       = 5,
 *     ompt_mutex_atomic         = 6,
 *     ompt_mutex_ordered        = 7
 * } ompt_mutex_t; */
static inline std::string
mutex2string( ompt_mutex_t t )
{
    switch ( t )
    {
        case ompt_mutex_lock:
            return "lock";
        case ompt_mutex_test_lock:
            return "test_lock";
        case ompt_mutex_nest_lock:
            return "nest_lock";
        case ompt_mutex_test_nest_lock:
            return "test_nest_lock";
        case ompt_mutex_critical:
            return "critical";
        case ompt_mutex_atomic:
            return "atomic";
        case ompt_mutex_ordered:
            return "ordered";
        default:
            assert( false && "Unknown ompt_mutex_t" );
    }
    return "";
}

/* typedef enum ompt_dispatch_t {
 *   ompt_dispatch_iteration        = 1,
 *   ompt_dispatch_section          = 2,
 *   ompt_dispatch_ws_loop_chunk    = 3, ADDED_52
 *   ompt_dispatch_taskloop_chunk   = 4, ADDED_52
 *   ompt_dispatch_distribute_chunk = 5, ADDED_52
 * } ompt_dispatch_t; */
static inline std::string
dispatch2string( ompt_dispatch_t t )
{
    switch ( t )
    {
        case ompt_dispatch_iteration:
            return "iteration";
        case ompt_dispatch_section:
            return "section";
#if HAVE( OMPT_DISPATCH_WS_LOOP_CHUNK )
        case ompt_dispatch_ws_loop_chunk:
            return "ws_loop_chunk";
#endif
#if HAVE( OMPT_DISPATCH_TASKLOOP_CHUNK )
        case ompt_dispatch_taskloop_chunk:
            return "taskloop_chunk";
#endif
#if HAVE( OMPT_DISPATCH_DISTRIBUTE_CHUNK )
        case ompt_dispatch_distribute_chunk:
            return "distribute_chunk";
#endif
        default:
            assert( false && "Unknown ompt_dispatch_t" );
    }
    return "";
}

/* typedef enum ompt_cancel_flag_t {
 *    ompt_cancel_parallel       = 0x01,
 *    ompt_cancel_sections       = 0x02,
 *    ompt_cancel_loop           = 0x04,
 *    ompt_cancel_taskgroup      = 0x08,
 *    ompt_cancel_activated      = 0x10,
 *    ompt_cancel_detected       = 0x20,
 *    ompt_cancel_discarded_task = 0x40
 *} ompt_cancel_flag_t; */
static inline std::string
cancel2string( int t )
{
    std::stringstream result;
    if ( t & ompt_cancel_activated )
    {
        result << "activated";
        t -= ompt_cancel_activated;
    }
    else if ( t & ompt_cancel_detected )
    {
        result << "detected";
        t -= ompt_cancel_detected;
    }

    if ( t & ompt_cancel_parallel )
    {
        result << "_parallel";
        t -= ompt_cancel_parallel;
    }
    else if ( t & ompt_cancel_sections )
    {
        result << "_sections";
        t -= ompt_cancel_sections;
    }
    else if ( t & ompt_cancel_loop )
    {
        result << "_loop";
        t -= ompt_cancel_loop;
    }
    else if ( t & ompt_cancel_taskgroup )
    {
        result << "_taskgroup";
        t -= ompt_cancel_taskgroup;
    }
    else if ( t & ompt_cancel_discarded_task )
    {
        result << "_discarded_task";
        t -= ompt_cancel_discarded_task;
    }
    assert( t == 0 && "Unknown ompt_cancel_flag_t" );
    return result.str();
}

/* typedef enum ompt_work_t {
 *     ompt_work_loop            = 1,
 *     ompt_work_sections        = 2,
 *     ompt_work_single_executor = 3,
 *     ompt_work_single_other    = 4,
 *     ompt_work_workshare       = 5,
 *     ompt_work_distribute      = 6,
 *     ompt_work_taskloop        = 7,
 *     ompt_work_scope           = 8, ADDED_51
 *     ompt_work_workdistribute  = 9, ADDED_60
 *     ompt_work_loop_static     = 10, ADDED_52
 *     ompt_work_loop_dynamic    = 11, ADDED_52
 *     ompt_work_loop_guided     = 12, ADDED_52
 *     ompt_work_loop_other      = 13, ADDED_52
 * } ompt_work_t; */
static inline std::string
work2string( ompt_work_t t )
{
    switch ( t )
    {
        case ompt_work_loop:
            return "loop";
        case ompt_work_sections:
            return "sections";
        case ompt_work_single_executor:
            return "single_executor";
        case ompt_work_single_other:
            return "single_other";
        case ompt_work_workshare:
            return "workshare";
        case ompt_work_distribute:
            return "distribute";
        case ompt_work_taskloop:
            return "taskloop";
#if HAVE( OMPT_WORK_SCOPE )
        case ompt_work_scope:
            return "scope";
#endif
#if HAVE( OMPT_WORK_WORKDISTRIBUTE )
        case ompt_work_workdistribute:
            return "workdistribute";
#endif
#if HAVE( OMPT_WORK_LOOP_STATIC )
        case ompt_work_loop_static:
            return "loop_static";
#endif
#if HAVE( OMPT_WORK_LOOP_DYNAMIC )
        case ompt_work_loop_dynamic:
            return "loop_dynamic";
#endif
#if HAVE( OMPT_WORK_LOOP_GUIDED )
        case ompt_work_loop_guided:
            return "loop_guided";
#endif
#if HAVE( OMPT_WORK_LOOP_OTHER )
        case ompt_work_loop_other:
            return "loop_other";
#endif
        default:
            assert( false && "Unknown ompt_work_t" );
    }
    return "";
}

/* typedef enum ompt_sync_region_t {
 *     ompt_sync_region_barrier                    = 1, DEPRECATED_51, REMOVED_60
 *     ompt_sync_region_barrier_implicit           = 2, DEPRECATED_51, REMOVED_60
 *     ompt_sync_region_barrier_explicit           = 3,
 *     ompt_sync_region_barrier_implementation     = 4,
 *     ompt_sync_region_taskwait                   = 5,
 *     ompt_sync_region_taskgroup                  = 6,
 *     ompt_sync_region_reduction                  = 7,
 *     ompt_sync_region_barrier_implicit_workshare = 8, ADDED_51
 *     ompt_sync_region_barrier_implicit_parallel  = 9, ADDED_51
 *     ompt_sync_region_barrier_teams              = 10 ADDED_51
 * } ompt_sync_region_t; */
static inline std::string
sync2string( ompt_sync_region_t t )
{
    switch ( t )
    {
#if HAVE( OMPT_SYNC_REGION_BARRIER )
        case ompt_sync_region_barrier:
            return "barrier";
#endif
#if HAVE( OMPT_SYNC_REGION_BARRIER_IMPLICIT )
        case ompt_sync_region_barrier_implicit:
            return "barrier_implicit";
#endif
        case ompt_sync_region_barrier_explicit:
            return "barrier_explicit";
        case ompt_sync_region_barrier_implementation:
            return "barrier_implementation";
        case ompt_sync_region_taskwait:
            return "taskwait";
        case ompt_sync_region_taskgroup:
            return "taskgroup";
        case ompt_sync_region_reduction:
            return "reduction";
#if HAVE( OMPT_SYNC_REGION_BARRIER_IMPLICIT_WORKSHARE )
        case ompt_sync_region_barrier_implicit_workshare:
            return "barrier_implicit_workshare";
#endif
#if HAVE( OMPT_SYNC_REGION_BARRIER_IMPLICIT_PARALLEL )
        case ompt_sync_region_barrier_implicit_parallel:
            return "barrier_implicit_parallel";
#endif
#if HAVE( OMPT_SYNC_REGION_BARRIER_TEAMS )
        case ompt_sync_region_barrier_teams:
            return "barrier_teams";
#endif
        default:
            assert( false && "Unknown ompt_sync_region_t" );
    }
    return "";
}

/* typedef enum ompt_target_data_op_t {
 *     ompt_target_data_alloc                      = 1,
 *     ompt_target_data_transfer_to_device         = 2, DEPRECATED_60
 *     ompt_target_data_transfer_from_device       = 3, DEPRECATED_60
 *     ompt_target_data_delete                     = 4,
 *     ompt_target_data_associate                  = 5,
 *     ompt_target_data_disassociate               = 6,
 *     ompt_target_data_transfer                   = 7, ADDED_60
 *     ompt_target_data_memset                     = 8, ADDED_60
 *     ompt_target_data_transfer_rect              = 9, ADDED_60
 *     ompt_target_data_alloc_async                = 17, ADDED_51
 *     ompt_target_data_transfer_to_device_async   = 18, ADDED_51, DEPRECATED_60
 *     ompt_target_data_transfer_from_device_async = 19, ADDED_51, DEPRECATED_60
 *     ompt_target_data_delete_async               = 20, ADDED_51
 *     ompt_target_data_transfer_async             = 23, ADDED_60
 *     ompt_target_data_memset_async               = 24, ADDED_60
 *     ompt_target_data_transfer_rect_async        = 25, ADDED_60
 * } ompt_target_data_op_t; */
static inline std::string
data_op2string( ompt_target_data_op_t t )
{
    switch ( t )
    {
        case ompt_target_data_alloc:
            return "alloc";
#if HAVE( OMPT_TARGET_DATA_TRANSFER_TO_DEVICE )
        case ompt_target_data_transfer_to_device:
            return "transfer_to_device";
#endif
#if HAVE( OMPT_TARGET_DATA_TRANSFER_FROM_DEVICE )
        case ompt_target_data_transfer_from_device:
            return "transfer_from_device";
#endif
        case ompt_target_data_delete:
            return "delete";
        case ompt_target_data_associate:
            return "associate";
        case ompt_target_data_disassociate:
            return "disassociate";
#if HAVE( OMPT_TARGET_DATA_TRANSFER )
        case ompt_target_data_transfer:
            return "transfer";
#endif
#if HAVE( OMPT_TARGET_DATA_MEMSET )
        case ompt_target_data_memset:
            return "memset";
#endif
#if HAVE( OMPT_TARGET_DATA_TRANSFER_RECT )
        case ompt_target_data_transfer_rect:
            return "transfer_rect";
#endif
#if HAVE( OMPT_TARGET_DATA_ALLOC_ASYNC )
        case ompt_target_data_alloc_async:
            return "alloc_async";
#endif
#if HAVE( OMPT_TARGET_DATA_TRANSFER_TO_DEVICE_ASYNC )
        case ompt_target_data_transfer_to_device_async:
            return "transfer_to_device_async";
#endif
#if HAVE( OMPT_TARGET_DATA_TRANSFER_FROM_DEVICE_ASYNC )
        case ompt_target_data_transfer_from_device_async:
            return "transfer_from_device_async";
#endif
#if HAVE( OMPT_TARGET_DATA_DELETE_ASYNC )
        case ompt_target_data_delete_async:
            return "delete_async";
#endif
#if HAVE( OMPT_TARGET_DATA_TRANSFER_ASYNC )
        case ompt_target_data_transfer_async:
            return "transfer_async";
#endif
#if HAVE( OMPT_TARGET_DATA_MEMSET_ASYNC )
        case ompt_target_data_memset_async:
            return "memset_async";
#endif
#if HAVE( OMPT_TARGET_DATA_TRANSFER_RECT_ASYNC )
        case ompt_target_data_transfer_rect_async:
            return "transfer_rect_async";
#endif
        default:
            assert( false && "Unknown ompt_target_data_op_t" );
    }
    return "";
}

/* typedef enum ompt_target_t {
 *   ompt_target                         = 1,
 *   ompt_target_enter_data              = 2,
 *   ompt_target_exit_data               = 3,
 *   ompt_target_update                  = 4,
 *   ompt_target_nowait                  = 9, ADDED_51
 *   ompt_target_enter_data_nowait       = 10, ADDED_51
 *   ompt_target_exit_data_nowait        = 11, ADDED_51
 *   ompt_target_update_nowait           = 12, ADDED_51
 * } ompt_target_t; */
static inline std::string
target2string( ompt_target_t t )
{
    switch ( t )
    {
        case ompt_target:
            return "target";
        case ompt_target_enter_data:
            return "target_enter_data";
        case ompt_target_exit_data:
            return "target_exit_data";
        case ompt_target_update:
            return "target_update";
#if HAVE( OMPT_TARGET_NOWAIT )
        case ompt_target_nowait:
            return "target_nowait";
#endif
#if HAVE( OMPT_TARGET_ENTER_DATA_NOWAIT )
        case ompt_target_enter_data_nowait:
            return "target_enter_data_nowait";
#endif
#if HAVE( OMPT_TARGET_EXIT_DATA_NOWAIT )
        case ompt_target_exit_data_nowait:
            return "target_exit_data_nowait";
#endif
#if HAVE( OMPT_TARGET_UPDATE_NOWAIT )
        case ompt_target_update_nowait:
            return "target_update_nowait";
#endif
        default:
            assert( false && "Unknown ompt_target_t" );
    }
    return "";
}

/* typedef enum ompt_target_map_flag_t {
 *   ompt_target_map_flag_to       = 0x01,
 *   ompt_target_map_flag_from     = 0x02,
 *   ompt_target_map_flag_alloc    = 0x04,
 *   ompt_target_map_flag_release  = 0x08,
 *   ompt_target_map_flag_delete   = 0x10,
 *   ompt_target_map_flag_implicit = 0x20,
 *   ompt_target_map_flag_always   = 0x40, ADDED_52
 *   ompt_target_map_flag_present  = 0x80, ADDED_52
 *   ompt_target_map_flag_close    = 0x100, ADDED_52
 *   ompt_target_map_flag_shared   = 0x200, ADDED_52
 * } ompt_target_map_flag_t; */
static inline std::string
map_flag2string( unsigned int t )
{
    std::stringstream result;
    if ( t & ompt_target_map_flag_to )
    {
        result << "to";
        t -= ompt_target_map_flag_to;
    }
    else if ( t & ompt_target_map_flag_from )
    {
        result << "from";
        t -= ompt_target_map_flag_from;
    }
    else if ( t & ompt_target_map_flag_alloc )
    {
        result << "alloc";
        t -= ompt_target_map_flag_alloc;
    }
    else if ( t & ompt_target_map_flag_release )
    {
        result << "release";
        t -= ompt_target_map_flag_release;
    }
    else if ( t & ompt_target_map_flag_delete )
    {
        result << "delete";
        t -= ompt_target_map_flag_delete;
    }
    else if ( t & ompt_target_map_flag_implicit )
    {
        result << "implicit";
        t -= ompt_target_map_flag_implicit;
    }
#if HAVE( OMPT_TARGET_MAP_FLAG_ALWAYS )
    else if ( t & ompt_target_map_flag_always )
    {
        result << "always";
        t -= ompt_target_map_flag_always;
    }
#endif
#if HAVE( OMPT_TARGET_MAP_FLAG_PRESENT )
    else if ( t & ompt_target_map_flag_present )
    {
        result << "present";
        t -= ompt_target_map_flag_present;
    }
#endif
#if HAVE( OMPT_TARGET_MAP_FLAG_CLOSE )
    else if ( t & ompt_target_map_flag_close )
    {
        result << "close";
        t -= ompt_target_map_flag_close;
    }
#endif
#if HAVE( OMPT_TARGET_MAP_FLAG_SHARED )
    else if ( t & ompt_target_map_flag_shared )
    {
        result << "shared";
        t -= ompt_target_map_flag_shared;
    }
#endif
    assert( t == 0 && "Unknown ompt_target_map_flag_t" );
    return result.str();
}

/* typedef enum ompt_dependence_type_t {
 *   ompt_dependence_type_in               = 1,
 *   ompt_dependence_type_out              = 2,
 *   ompt_dependence_type_inout            = 3,
 *   ompt_dependence_type_mutexinoutset    = 4,
 *   ompt_dependence_type_source           = 5,
 *   ompt_dependence_type_sink             = 6,
 *   ompt_dependence_type_inoutset         = 7, ADDED_51
 *   ompt_dependence_type_out_all_memory   = 34, ADDED_60
 *   ompt_dependence_type_inout_all_memory = 35, ADDED_60
 * } ompt_dependence_type_t; */
static inline std::string
dependence_type2string( ompt_dependence_type_t t )
{
    switch ( t )
    {
        case ompt_dependence_type_in:
            return "in";
        case ompt_dependence_type_out:
            return "out";
        case ompt_dependence_type_inout:
            return "inout";
        case ompt_dependence_type_mutexinoutset:
            return "mutexinoutset";
        case ompt_dependence_type_source:
            return "source";
        case ompt_dependence_type_sink:
            return "sink";
#if HAVE( OMPT_DEPENDENCE_TYPE_INOUTSET )
        case ompt_dependence_type_inoutset:
            return "inoutset";
#endif
#if HAVE( OMPT_DEPENDENCE_TYPE_OUT_ALL_MEMORY )
        case ompt_dependence_type_out_all_memory:
            return "out_all_memory";
#endif
#if HAVE( OMPT_DEPENDENCE_TYPE_INOUT_ALL_MEMORY )
        case ompt_dependence_type_inout_all_memory:
            return "inout_all_memory";
#endif
        default:
            assert( false && "Unknown ompt_dependence_type_t" );
    }
    return "";
}

/* Host side callbacks */

template<printf_mode mode>
void
callback_thread_begin( ompt_thread_t thread_type,
                       ompt_data_t*  thread_data )
{
    if constexpr ( mode > printf_mode::disable_output )
    {
        thread_id = next_thread_id++;
    }
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] thread_type = %s | thread_data = %p\n",
                       __FUNCTION__,
                       thread2string( thread_type ).c_str(),
                       thread_data );
    }
}

template<printf_mode mode>
void
callback_thread_end( ompt_data_t* thread_data )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] thread_data = %p\n",
                       __FUNCTION__,
                       thread_data );
    }
}

template<printf_mode mode>
void
callback_parallel_begin( ompt_data_t*        encountering_task_data,
                         const ompt_frame_t* encountering_task_frame,
                         ompt_data_t*        parallel_data,
                         unsigned int        requested_parallelism,
                         int                 flags,
                         const void*         codeptr_ra )
{
    if constexpr ( mode > printf_mode::disable_output )
    {
        if ( parallel_data )
        {
            parallel_data->value = ++next_parallel_id;
        }
    }

    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] encountering_task_data->value = %lu (%p) | encountering_task_frame = %p | "
                       "parallel_data->value = %lu (%p) | requested_parallelism = %u | flags = %s | codeptr_ra = %p\n",
                       __FUNCTION__,
                       encountering_task_data ? encountering_task_data->value : unknown_task_id,
                       encountering_task_data,
                       encountering_task_frame,
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       requested_parallelism,
                       parallel_flag2string( flags ).c_str(),
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_parallel_end( ompt_data_t* parallel_data,
                       ompt_data_t* encountering_task_data,
                       int          flags,
                       const void*  codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] parallel_data->value = %lu (%p) | encountering_task_data->value = %lu (%p) | flags = %s | "
                       "codeptr_ra = %p\n",
                       __FUNCTION__,
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       encountering_task_data ? encountering_task_data->value : unknown_task_id,
                       encountering_task_data,
                       parallel_flag2string( flags ).c_str(),
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_task_create( ompt_data_t*        encountering_task_data,
                      const ompt_frame_t* encountering_task_frame,
                      ompt_data_t*        new_task_data,
                      int                 flags,
                      int                 has_dependences,
                      const void*         codeptr_ra )
{
    if constexpr ( mode > printf_mode::disable_output )
    {
        if ( new_task_data )
        {
            new_task_data->value = ++next_task_id;
        }
    }

    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] encountering_task_data->value = %lu (%p) | encountering_task_frame = %p | "
                       "new_task_data->value = %lu (%p) | flags = %s | has_dependences = %d | codeptr_ra = %p\n",
                       __FUNCTION__,
                       encountering_task_data ? encountering_task_data->value : unknown_task_id,
                       encountering_task_data,
                       encountering_task_frame,
                       new_task_data ? new_task_data->value : unknown_task_id,
                       new_task_data,
                       task_flag2string( flags ).c_str(),
                       has_dependences,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_task_schedule( ompt_data_t*       prior_task_data,
                        ompt_task_status_t prior_task_status,
                        ompt_data_t*       next_task_data )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] prior_task_data->value = %lu (%p) | prior_task_status = %s | "
                       "next_task_data->value = %lu (%p)\n",
                       __FUNCTION__,
                       prior_task_data ? prior_task_data->value : unknown_task_id,
                       prior_task_data,
                       task_status2string( prior_task_status ).c_str(),
                       next_task_data ? next_task_data->value : unknown_task_id,
                       next_task_data );
    }
}

template<printf_mode mode>
void
callback_implicit_task( ompt_scope_endpoint_t endpoint,
                        ompt_data_t*          parallel_data,
                        ompt_data_t*          task_data,
                        unsigned int          actual_parallelism,
                        unsigned int          index,
                        int                   flags )
{
    if constexpr ( mode > printf_mode::disable_output )
    {
        if ( task_data && endpoint == ompt_scope_begin )
        {
            task_data->value = ++next_task_id;
        }
    }

    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] endpoint = %s | parallel_data->value = %lu (%p) | task_data->value = %lu (%p) | "
                       "actual_parallelism = %u | index = %u | flags = %s\n",
                       __FUNCTION__,
                       endpoint2string( endpoint ).c_str(),
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       actual_parallelism,
                       index,
                       task_flag2string( flags ).c_str() );
    }
}


template<printf_mode mode>
void
callback_sync_region_wait( ompt_sync_region_t    kind,
                           ompt_scope_endpoint_t endpoint,
                           ompt_data_t*          parallel_data,
                           ompt_data_t*          task_data,
                           const void*           codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | endpoint = %s | parallel_data->value = %lu (%p) | "
                       "task_data->value = %lu (%p) | codeptr_ra = %p\n",
                       __FUNCTION__,
                       sync2string( kind ).c_str(),
                       endpoint2string( endpoint ).c_str(),
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_mutex_released( ompt_mutex_t   kind,
                         ompt_wait_id_t wait_id,
                         const void*    codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | wait_id = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       mutex2string( kind ).c_str(),
                       wait_id,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_dependences( ompt_data_t*             task_data,
                      const ompt_dependence_t* deps,
                      int                      ndeps )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] task_data->value = %lu (%p) | deps = %p | ndeps = %d\n",
                       __FUNCTION__,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       deps,
                       ndeps );
        for ( int i = 0; i < ndeps; ++i )
        {
            atomic_printf( "[%s] deps[%d].variable_addr = %p | deps[%d].dependence_type = %s\n",
                           __FUNCTION__,
                           i,
                           deps[ i ].variable,
                           i,
                           dependence_type2string( deps[ i ].dependence_type ).c_str() );
        }
    }
}

template<printf_mode mode>
void
callback_task_dependence( ompt_data_t* src_task_data,
                          ompt_data_t* sink_task_data )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] src_task_data->value = %lu (%p) | sink_task_data->value = %lu (%p)\n",
                       __FUNCTION__,
                       src_task_data ? src_task_data->value : unknown_task_id,
                       src_task_data,
                       sink_task_data ? sink_task_data->value : unknown_task_id,
                       sink_task_data );
    }
}

template<printf_mode mode>
void
callback_work( ompt_work_t           work_type,
               ompt_scope_endpoint_t endpoint,
               ompt_data_t*          parallel_data,
               ompt_data_t*          task_data,
               uint64_t              count,
               const void*           codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] work_type = %s | endpoint = %s | parallel_data->value = %lu (%p) | "
                       "task_data->value = %lu (%p) | count = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       work2string( work_type ).c_str(),
                       endpoint2string( endpoint ).c_str(),
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       count,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_masked( ompt_scope_endpoint_t endpoint,
                 ompt_data_t*          parallel_data,
                 ompt_data_t*          task_data,
                 const void*           codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] endpoint = %s | parallel_data->value = %lu (%p) | "
                       "task_data->value = %lu (%p) | codeptr_ra = %p\n",
                       __FUNCTION__,
                       endpoint2string( endpoint ).c_str(),
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_sync_region( ompt_sync_region_t    kind,
                      ompt_scope_endpoint_t endpoint,
                      ompt_data_t*          parallel_data,
                      ompt_data_t*          task_data,
                      const void*           codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | endpoint = %s | parallel_data->value = %lu (%p) | "
                       "task_data->value = %lu (%p) | codeptr_ra = %p\n",
                       __FUNCTION__,
                       sync2string( kind ).c_str(),
                       endpoint2string( endpoint ).c_str(),
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_lock_init( ompt_mutex_t   kind,
                    unsigned int   hint,
                    unsigned int   impl,
                    ompt_wait_id_t wait_id,
                    const void*    codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | hint = %u | impl = %u | wait_id = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       mutex2string( kind ).c_str(),
                       hint,
                       impl,
                       wait_id,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_lock_destroy( ompt_mutex_t   kind,
                       ompt_wait_id_t wait_id,
                       const void*    codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | wait_id = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       mutex2string( kind ).c_str(),
                       wait_id,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_mutex_acquire( ompt_mutex_t   kind,
                        unsigned int   hint,
                        unsigned int   impl,
                        ompt_wait_id_t wait_id,
                        const void*    codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | hint = %u | impl = %u | wait_id = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       mutex2string( kind ).c_str(),
                       hint,
                       impl,
                       wait_id,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_mutex_acquired( ompt_mutex_t   kind,
                         ompt_wait_id_t wait_id,
                         const void*    codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | wait_id = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       mutex2string( kind ).c_str(),
                       wait_id,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_nest_lock( ompt_scope_endpoint_t endpoint,
                    ompt_wait_id_t        wait_id,
                    const void*           codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] endpoint = %s | wait_id = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       endpoint2string( endpoint ).c_str(),
                       wait_id,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_flush( ompt_data_t* thread_data,
                const void*  codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] thread_data = %p | codeptr_ra = %p\n",
                       __FUNCTION__,
                       thread_data,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_cancel( ompt_data_t* task_data,
                 int          flags,
                 const void*  codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] task_data->value = %lu (%p) | flags = %s | codeptr_ra = %p\n",
                       __FUNCTION__,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       cancel2string( flags ).c_str(),
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_reduction( ompt_sync_region_t    kind,
                    ompt_scope_endpoint_t endpoint,
                    ompt_data_t*          parallel_data,
                    ompt_data_t*          task_data,
                    const void*           codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | endpoint = %s | parallel_data->value = %lu (%p) | "
                       "task_data->value = %lu (%p) | codeptr_ra = %p\n",
                       __FUNCTION__,
                       sync2string( kind ).c_str(),
                       endpoint2string( endpoint ).c_str(),
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_dispatch( ompt_data_t*    parallel_data,
                   ompt_data_t*    task_data,
                   ompt_dispatch_t kind,
                   ompt_data_t     instance )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] parallel_data->value = %lu (%p) | task_data->value = %lu (%p) | kind = %s | "
                       "instance->value = %lu\n",
                       __FUNCTION__,
                       parallel_data ? parallel_data->value : unknown_parallel_id,
                       parallel_data,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       dispatch2string( kind ).c_str(),
                       instance.value );
    }
}

/* Host side accelerator callbacks */

/* // REMOVED_60
 * typedef struct ompt_record_ompt_t {
 *   ompt_callbacks_t type;
 *   ompt_device_time_t time;
 *   ompt_id_t thread_id;
 *   ompt_id_t target_id;
 *   union {
 *     ompt_record_thread_begin_t thread_begin;
 *     ompt_record_parallel_begin_t parallel_begin;
 *     ompt_record_parallel_end_t parallel_end;
 *     ompt_record_work_t work;
 *     ompt_record_dispatch_t dispatch;
 *     ompt_record_task_create_t task_create;
 *     ompt_record_dependences_t dependences;
 *     ompt_record_task_dependence_t task_dependence;
 *     ompt_record_task_schedule_t task_schedule;
 *     ompt_record_implicit_task_t implicit_task;
 *     ompt_record_masked_t masked;
 *     ompt_record_sync_region_t sync_region;
 *     ompt_record_mutex_acquire_t mutex_acquire;
 *     ompt_record_mutex_t mutex;
 *     ompt_record_nest_lock_t nest_lock;
 *     ompt_record_flush_t flush;
 *     ompt_record_cancel_t cancel;
 *     ompt_record_target_t target;
 *     ompt_record_target_data_op_t target_data_op;
 *     ompt_record_target_map_t target_map;
 *     ompt_record_target_kernel_t target_kernel;
 *     ompt_record_control_tool_t control_tool;
 *   } record;
 * } ompt_record_ompt_t;
 *
 * // ADDED_60
 * typedef union ompt_any_record_ompt_t {
 *     ompt_record_thread_begin_t thread_begin;
 *     ompt_record_parallel_begin_t parallel_begin;
 *     ompt_record_parallel_end_t parallel_end;
 *     ompt_record_work_t work;
 *     ompt_record_dispatch_t dispatch;
 *     ompt_record_task_create_t task_create;
 *     ompt_record_dependences_t dependences;
 *     ompt_record_task_dependence_t task_dependence;
 *     ompt_record_task_schedule_t task_schedule;
 *     ompt_record_implicit_task_t implicit_task;
 *     ompt_record_masked_t masked;
 *     ompt_record_sync_region_t sync_region;
 *     ompt_record_mutex_acquire_t mutex_acquire;
 *     ompt_record_mutex_t mutex;
 *     ompt_record_nest_lock_t nest_lock;
 *     ompt_record_flush_t flush;
 *     ompt_record_cancel_t cancel;
 *     ompt_record_target_emi_t target_emi;
 *     ompt_record_target_data_op_emi_t target_data_op_emi;
 *     ompt_record_target_map_emi_t target_map_emi;
 *     ompt_record_target_submit_emi_t target_submit_emi;
 *     ompt_record_control_tool_t control_tool;
 *     ompt_record_error_t error;
 * } ompt_any_record_ompt_t;
 *
 * typedef struct ompt_record_ompt_t {
 *     ompt_callbacks_t type;
 *     ompt_device_time_t time;
 *     ompt_id_t thread_id;
 *     ompt_id_t target_id;
 *     ompt_any_record_ompt_t record;
 * } ompt_record_ompt_t;
 * */

template <printf_mode mode>
void
callback_buffer_request( int device_num, ompt_buffer_t** buffer,
                         size_t* bytes )
{
#if HAVE( OMPT_GET_BUFFER_LIMITS )
    device_t* device = get_devices()[ device_num ];
    *bytes  = device->recommended_buffer_size;
    *buffer = ::operator new( *bytes );
    device->allocated_buffers++;
#else
    constexpr size_t buffer_size = 100;
    constexpr size_t record_size = sizeof( ompt_record_ompt_t );

    *buffer = new ompt_record_ompt_t[ buffer_size ];
    *bytes  = buffer_size * record_size;
#endif

    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
#if HAVE( OMPT_GET_BUFFER_LIMITS )
#if HAVE( OMPT_GET_BUFFER_LIMITS )
        atomic_printf( "[%s] device_num = %d | buffer = %p (%d/%d) | bytes = %lu\n",
                       __FUNCTION__,
                       device_num,
                       *buffer,
                       device->allocated_buffers,
                       device->recommended_buffer_amount,
                       *bytes );
#else
        atomic_printf( "[%s] device_num = %d | buffer = %p | bytes = %lu\n",
                       __FUNCTION__,
                       device_num,
                       buffer,
                       *bytes );
#endif
#endif
    }
}

template<printf_mode mode>
void
callback_buffer_complete( int                  device_num,
                          ompt_buffer_t*       buffer,
                          size_t               bytes,
                          ompt_buffer_cursor_t begin,
                          int                  buffer_owned )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] device_num = %d | buffer = %p | bytes = %lu | begin = %lu | buffer_owned = %d\n",
                       __FUNCTION__,
                       device_num,
                       buffer,
                       bytes,
                       begin,
                       buffer_owned );
    }

    /* AMD implementation might return buffer complete callback to indicate buffer
     * that can be freed, but does not set bytes (bytes == 0). This buffer
     * was previously already evaluated. Trying to evaluate it again will
     * cause a segmentation fault. */
    if ( bytes == 0 )
    {
        if ( buffer_owned )
        {
            delete[] ( ompt_record_ompt_t* )buffer;
        }
        return;
    }

    auto device = get_devices().find( device_num );
    assert( device != get_devices().end() && "Device not found" );
    auto current_cursor = begin;
    do
    {
        ompt_record_ompt_t* record = device->second->device_functions.get_record_ompt( buffer, current_cursor );
        if constexpr ( mode > printf_mode::disable_output )
        {
            /*   ompt_device_time_t time;
                 ompt_id_t thread_id;
                 ompt_id_t target_id; */
            switch ( record->type )
            {
#if HAVE( OMPT_CALLBACK_TARGET )
                case ompt_callback_target:
#endif
#if HAVE( OMPT_CALLBACK_TARGET_EMI )
                case ompt_callback_target_emi:
#endif
                {
#if HAVE( OMPT_RECORD_TARGET_EMI )
                    ompt_record_target_emi_t target = record->record.target_emi;
#elif HAVE( OMPT_RECORD_TARGET )
                    ompt_record_target_t target = record->record.target;
#endif
                    atomic_printf( "[%s] time = %lu | thread_id = %lu | target_id = %lu | kind = %s | "
                                   "endpoint = %s | device_num = %d | task_id = %lu | target_id = %lu | codeptr_ra = %p\n",
                                   __FUNCTION__,
                                   record->time,
                                   record->thread_id,
                                   record->target_id,
                                   target2string( target.kind ).c_str(),
                                   endpoint2string( target.endpoint ).c_str(),
                                   target.device_num,
                                   target.task_id,
                                   target.target_id,
                                   target.codeptr_ra );
                    break;
                }
#if HAVE( OMPT_CALLBACK_TARGET_DATA_OP )
                case ompt_callback_target_data_op:
#endif
#if HAVE( OMPT_CALLBACK_TARGET_DATA_OP_EMI )
                case ompt_callback_target_data_op_emi:
#endif
                {
#if HAVE( OMPT_RECORD_TARGET_DATA_OP_EMI )
                    ompt_record_target_data_op_emi_t data_op = record->record.target_data_op_emi;
#elif HAVE( OMPT_RECORD_TARGET_DATA_OP )
                    ompt_record_target_data_op_t data_op = record->record.target_data_op;
#endif
                    atomic_printf( "[%s] time = %lu | thread_id = %lu | target_id = %lu | host_op_id = %lu | "
                                   "optype = %s | src_addr = %p | src_device_num = %d | dest_addr = %p | "
                                   "dest_device_num = %d | bytes = %lu | end_time = %lu | codeptr_ra = %p\n",
                                   __FUNCTION__,
                                   record->time,
                                   record->thread_id,
                                   record->target_id,
                                   data_op.host_op_id,
                                   data_op2string( data_op.optype ).c_str(),
                                   data_op.src_addr,
                                   data_op.src_device_num,
                                   data_op.dest_addr,
                                   data_op.dest_device_num,
                                   data_op.bytes,
                                   data_op.end_time,
                                   data_op.codeptr_ra );
                    break;
                }
#if HAVE( OMPT_CALLBACK_TARGET_MAP )
                case ompt_callback_target_map:
#endif
#if HAVE( OMPT_CALLBACK_TARGET_MAP_EMI )
                case ompt_callback_target_map_emi:
#endif
                {
#if HAVE( OMPT_RECORD_TARGET_MAP_EMI )
                    ompt_record_target_map_emi_t map = record->record.target_map_emi;
#elif HAVE( OMPT_RECORD_TARGET_MAP )
                    ompt_record_target_map_t map = record->record.target_map;
#endif
                    atomic_printf( "[%s] time = %lu | thread_id = %lu | target_id = %lu | target_id = %lu | "
                                   "nitems = %u | codeptr_ra = %p\n",
                                   __FUNCTION__,
                                   record->time,
                                   record->thread_id,
                                   record->target_id,
                                   map.target_id,
                                   map.nitems,
                                   map.codeptr_ra );
                    for ( unsigned int i = 0; i < map.nitems; ++i )
                    {
                        atomic_printf(
                            "[%s] host_addr[%d] = %p | device_addr[%d] = %p | bytes[%d] = %lu | mapping_flags[%d] = %s\n",
                            __FUNCTION__,
                            i,
                            map.host_addr[ i ],
                            i,
                            map.device_addr[ i ],
                            i,
                            map.bytes[ i ],
                            i,
                            map_flag2string( record->record.target_map.mapping_flags[ i ] ).c_str() );
                    }
                    break;
                }
#if HAVE( OMPT_CALLBACK_TARGET_SUBMIT )
                case ompt_callback_target_submit:
#endif
#if HAVE( OMPT_CALLBACK_TARGET_SUBMIT_EMI )
                case ompt_callback_target_submit_emi:
#endif
                {
#if HAVE( OMPT_RECORD_TARGET_SUBMIT_EMI )
                    ompt_record_target_submit_emi_t kernel = record->record.target_submit_emi;
#elif HAVE( OMPT_RECORD_TARGET_KERNEL )
                    ompt_record_target_kernel_t kernel = record->record.target_kernel;
#endif
                    atomic_printf( "[%s] time = %lu | thread_id = %lu | target_id = %lu | host_op_id = %lu | "
                                   "requested_num_teams = %u | granted_num_teams = %u | end_time = %lu\n",
                                   __FUNCTION__,
                                   record->time,
                                   record->thread_id,
                                   record->target_id,
                                   kernel.host_op_id,
                                   kernel.requested_num_teams,
                                   kernel.granted_num_teams,
                                   kernel.end_time );
                    break;
                }
                default:
                    assert( false && "Unexpected ompt_record_ompt_t type" );
            }
        }
    }
    while ( device->second->device_functions.advance_buffer_cursor( device->second->address,
                                                                    buffer,
                                                                    bytes,
                                                                    current_cursor,
                                                                    &current_cursor ) );

    if ( buffer_owned )
    {
#if HAVE( OMPT_GET_BUFFER_LIMITS )
        get_devices()[ device_num ]->allocated_buffers--;
#endif
        delete[] ( ompt_record_ompt_t* )buffer;
    }
}

template<printf_mode mode>
void
callback_device_initialize( int                    device_num,
                            const char*            type,
                            ompt_device_t*         device,
                            ompt_function_lookup_t lookup,
                            const char*            documentation )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] device_num = %d | type = %s | device = %p | lookup = %p | documentation = %s\n",
                       __FUNCTION__,
                       device_num,
                       type,
                       device,
                       lookup,
                       documentation );
    }

    assert( get_devices().find( device_num ) == get_devices().end() &&
            "Device already initialized" );
    auto* new_device = new device_t { .address = device, .name = type };
    get_devices()[ device_num ] = new_device;
    if ( lookup )
    {
#define LOOKUP_DEVICE_FUNCTION( name ) \
    do { \
        new_device->device_functions.name = \
            ( ompt_##name##_t )lookup( "ompt_" #name ); \
        if ( !new_device->device_functions.name ) { \
            if constexpr ( mode > printf_mode::disable_output ) { \
                atomic_printf( "[%s] device_num = %d | %s not found\n", __FUNCTION__, \
                               device_num, #name ); \
            } \
            return; \
        } \
    } \
    while ( 0 )

        LOOKUP_DEVICE_FUNCTION( set_trace_ompt );
        LOOKUP_DEVICE_FUNCTION( start_trace );
        LOOKUP_DEVICE_FUNCTION( flush_trace );
        LOOKUP_DEVICE_FUNCTION( advance_buffer_cursor );
        LOOKUP_DEVICE_FUNCTION( get_record_ompt );

#undef LOOKUP_DEVICE_FUNCTION

        /* Optional functions */
        new_device->device_functions.stop_trace  = ( ompt_stop_trace_t )lookup( "ompt_stop_trace" );
        new_device->device_functions.pause_trace = ( ompt_pause_trace_t )lookup( "ompt_pause_trace" );
#if HAVE( OMPT_GET_BUFFER_LIMITS )
        new_device->device_functions.get_buffer_limits = ( ompt_get_buffer_limits_t )lookup( "ompt_get_buffer_limits" );
        new_device->device_functions.get_buffer_limits( new_device->address, &new_device->recommended_buffer_amount, &new_device->recommended_buffer_size );
#endif

        /* Register buffer events */
        ompt_set_result_t result;
        bool              registration_success = false;

        /* ompt_callback_target[_emi] */
#if HAVE( OMPT_CALLBACK_TARGET_EMI )
        result               = new_device->device_functions.set_trace_ompt( new_device->address, true, ompt_callback_target_emi );
        registration_success = result == ompt_set_always;
        if constexpr ( mode > printf_mode::disable_output )
        {
            atomic_printf( "[%s] device_num = %d | ompt_callback_target_emi = %s\n",
                           __FUNCTION__,
                           device_num,
                           set_result2string( result ).c_str() );
        }
#endif
#if HAVE( OMPT_CALLBACK_TARGET )
        if ( !registration_success )
        {
            result = new_device->device_functions.set_trace_ompt( new_device->address, true, ompt_callback_target );
            if constexpr ( mode > printf_mode::disable_output ) {
                atomic_printf( "[%s] device_num = %d | ompt_callback_target = %s\n",
                               __FUNCTION__,
                               device_num,
                               set_result2string( result ).c_str() );
            }
        }
#endif
        /* ompt_callback_target_data_op[_emi] */
        registration_success = false;
#if HAVE( OMPT_CALLBACK_TARGET_DATA_OP_EMI )
        result               = new_device->device_functions.set_trace_ompt( new_device->address, true, ompt_callback_target_data_op_emi );
        registration_success = result == ompt_set_always;
        if constexpr ( mode > printf_mode::disable_output )
        {
            atomic_printf( "[%s] device_num = %d | ompt_callback_target_data_op_emi = %s\n",
                           __FUNCTION__,
                           device_num,
                           set_result2string( result ).c_str() );
        }
#endif
#if HAVE( OMPT_CALLBACK_TARGET_DATA_OP )
        if ( !registration_success )
        {
            result = new_device->device_functions.set_trace_ompt( new_device->address, true, ompt_callback_target_data_op );
            if constexpr ( mode > printf_mode::disable_output ) {
                if ( result != ompt_set_error )
                {
                    atomic_printf( "[%s] device_num = %d | ompt_callback_target_data_op = %s\n",
                                   __FUNCTION__,
                                   device_num,
                                   set_result2string( result ).c_str() );
                }
            }
        }
#endif

        /* ompt_callback_target_map[_emi] */
        registration_success = false;
#if HAVE( OMPT_CALLBACK_TARGET_MAP_EMI )
        result               = new_device->device_functions.set_trace_ompt( new_device->address, true, ompt_callback_target_map_emi );
        registration_success = result == ompt_set_always;
        if constexpr ( mode > printf_mode::disable_output )
        {
            atomic_printf( "[%s] device_num = %d | ompt_callback_target_map_emi = %s\n",
                           __FUNCTION__,
                           device_num,
                           set_result2string( result ).c_str() );
        }
#endif
#if HAVE( OMPT_CALLBACK_TARGET_MAP )
        if ( !registration_success )
        {
            result = new_device->device_functions.set_trace_ompt( new_device->address, true, ompt_callback_target_map );
            if constexpr ( mode > printf_mode::disable_output ) {
                atomic_printf( "[%s] device_num = %d | ompt_callback_target_map = %s\n",
                               __FUNCTION__,
                               device_num,
                               set_result2string( result ).c_str() );
            }
        }
#endif

        /* ompt_callback_target_submit[_emi] */
        registration_success = false;
#if HAVE( OMPT_CALLBACK_TARGET_SUBMIT_EMI )
        result               = new_device->device_functions.set_trace_ompt( new_device->address, true, ompt_callback_target_submit_emi );
        registration_success = result == ompt_set_always;
        if constexpr ( mode > printf_mode::disable_output )
        {
            atomic_printf( "[%s] device_num = %d | ompt_callback_target_submit_emi = %s\n",
                           __FUNCTION__,
                           device_num,
                           set_result2string( result ).c_str() );
        }
#endif
#if HAVE( OMPT_CALLBACK_TARGET_SUBMIT )
        if ( !registration_success )
        {
            result = new_device->device_functions.set_trace_ompt( new_device->address, true,
                                                                  ompt_callback_target_submit );
            if constexpr ( mode > printf_mode::disable_output ) {
                atomic_printf( "[%s] device_num = %d | ompt_callback_target_submit = %s\n",
                               __FUNCTION__,
                               device_num,
                               set_result2string( result ).c_str() );
            }
        }
#endif

        if ( !new_device->device_functions.start_trace( new_device->address, &callback_buffer_request<mode>, &callback_buffer_complete<mode> ) )
        {
            return;
        }
    }
}

template<printf_mode mode>
void
callback_device_finalize( int device_num )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] device_num = %d\n",
                       __FUNCTION__,
                       device_num );
    }

    if ( get_devices().find( device_num ) == get_devices().end() )
    {
        if constexpr ( mode > printf_mode::disable_output )
        {
            atomic_printf( "[%s] Device %d not found. Number of devices: %d. "
                           "The runtime may have already cleaned up some state. Skip flushing buffers.\n",
                           __FUNCTION__,
                           get_devices().size(),
                           device_num );
        }
        return;
    }
    auto* finalized_device = get_devices()[ device_num ];
    if ( finalized_device->device_functions.flush_trace )
    {
        finalized_device->device_functions.flush_trace( finalized_device->address );
    }
    if ( finalized_device->device_functions.pause_trace )
    {
        finalized_device->device_functions.pause_trace( finalized_device->address, true );
    }
    if ( finalized_device->device_functions.stop_trace )
    {
        finalized_device->device_functions.stop_trace( finalized_device->address );
    }
    get_devices().erase( device_num );
    delete finalized_device;
}

template<printf_mode mode>
void
callback_device_load( int         device_num,
                      const char* filename,
                      int64_t     offset_in_file,
                      void*       vma_in_file,
                      size_t      bytes,
                      void*       host_addr,
                      void*       device_addr,
                      uint64_t    module_id )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] device_num = %d | filename = %s | offset_in_file = %ld | vma_in_file = %p | bytes = %lu "
                       "| host_addr = %p | device_addr = %p | module_id = %lu\n",
                       __FUNCTION__,
                       device_num,
                       filename,
                       offset_in_file,
                       vma_in_file,
                       bytes,
                       host_addr,
                       device_addr,
                       module_id );
    }
}

template<printf_mode mode>
void
callback_device_unload( int      device_num,
                        uint64_t module_id )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] device_num = %d | module_id = %lu\n",
                       __FUNCTION__,
                       device_num,
                       module_id );
    }
}

template<printf_mode mode>
void
callback_target_map( ompt_id_t     target_id,
                     unsigned int  nitems,
                     void**        host_addr,
                     void**        device_addr,
                     size_t*       bytes,
                     unsigned int* mapping_flags,
                     const void*   codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] target_id = %lu | nitems = %u | codeptr_ra = %p\n",
                       __FUNCTION__,
                       target_id,
                       nitems,
                       codeptr_ra );
        for ( unsigned int i = 0; i < nitems; ++i )
        {
            atomic_printf( "[%s] host_addr[%u] = %p | device_addr[%u] = %p | "
                           "bytes[%u] = %lu | mapping_flags[%u] = %u\n",
                           __FUNCTION__,
                           i,
                           host_addr[ i ],
                           i,
                           device_addr[ i ],
                           i,
                           bytes[ i ],
                           i,
                           map_flag2string( mapping_flags[ i ] ).c_str() );
        }
    }
}

template<printf_mode mode>
void
callback_target_map_emi( ompt_data_t*  target_data,
                         unsigned int  nitems,
                         void**        host_addr,
                         void**        device_addr,
                         size_t*       bytes,
                         unsigned int* mapping_flags,
                         const void*   codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] target_data->value = %lu (%p) | nitems = %u | codeptr_ra = %p\n",
                       __FUNCTION__,
                       target_data ? target_data->value : unknown_target_id,
                       target_data,
                       nitems,
                       codeptr_ra );
        for ( unsigned int i = 0; i < nitems; ++i )
        {
            atomic_printf( "[%s] host_addr[%u] = %p | device_addr[%u] = %p | "
                           "bytes[%u] = %lu | mapping_flags[%u] = %u\n",
                           __FUNCTION__,
                           i,
                           host_addr[ i ],
                           i,
                           device_addr[ i ],
                           i,
                           bytes[ i ],
                           i,
                           map_flag2string( mapping_flags[ i ] ).c_str() );
        }
    }
}

template<printf_mode mode>
void
callback_target( ompt_target_t         kind,
                 ompt_scope_endpoint_t endpoint,
                 int                   device_num,
                 ompt_data_t*          task_data,
                 ompt_id_t             target_id,
                 const void*           codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | endpoint = %s | device_num = %d | task_data->value = %lu (%p) | "
                       "target_id = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       target2string( kind ).c_str(),
                       endpoint2string( endpoint ).c_str(),
                       device_num,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       target_id,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_target_emi( ompt_target_t         kind,
                     ompt_scope_endpoint_t endpoint,
                     int                   device_num,
                     ompt_data_t*          task_data,
                     ompt_data_t*          target_task_data,
                     ompt_data_t*          target_data,
                     const void*           codeptr_ra )
{
    if constexpr ( mode > printf_mode::disable_output )
    {
        if ( task_data && endpoint == ompt_scope_begin )
        {
            target_data->value = ++next_target_id;
        }
    }

    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] kind = %s | endpoint = %s | device_num = %d | task_data->value = %lu (%p) | "
                       "target_task_data->value = %lu (%p) | target_data->value = %lu (%p) | codeptr_ra = %p\n",
                       __FUNCTION__,
                       target2string( kind ).c_str(),
                       endpoint2string( endpoint ).c_str(),
                       device_num,
                       task_data ? task_data->value : unknown_task_id,
                       task_data,
                       target_task_data ? target_task_data->value : unknown_task_id,
                       target_task_data,
                       target_data ? target_data->value : unknown_target_id,
                       target_data,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_target_data_op( ompt_id_t             target_id,
                         ompt_id_t             host_op_id,
                         ompt_target_data_op_t optype,
                         void*                 src_addr,
                         int                   src_device_num,
                         void*                 dest_addr,
                         int                   dest_device_num,
                         size_t                bytes,
                         const void*           codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] target_id = %lu | host_op_id = %lu | optype = %s | src_addr = %p | "
                       "src_device_num = %d | dest_addr = %p | dest_device_num = %d | bytes = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       target_id,
                       host_op_id,
                       data_op2string( optype ).c_str(),
                       src_addr,
                       src_device_num,
                       dest_addr,
                       dest_device_num,
                       bytes,
                       codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_target_data_op_emi( ompt_scope_endpoint_t endpoint,
                             ompt_data_t*          target_task_data,
                             ompt_data_t*          target_data,
                             ompt_id_t*            host_op_id,
                             ompt_target_data_op_t optype,
                             void*                 src_addr,         // CHANGED_60: dev1_addr
                             int                   src_device_num,   // CHANGED_60: dev1_device_num
                             void*                 dest_addr,        // CHANGED_60: dev2_addr
                             int                   dest_device_num,  // CHANGED_60: dev2_device_num
                             size_t                bytes,
                             const void*           codeptr_ra )
{
    if constexpr ( mode > printf_mode::disable_output )
    {
        if ( host_op_id && endpoint == ompt_scope_begin )
        {
            *host_op_id = ++next_host_op_id;
        }
    }

    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] endpoint = %s | target_task_data->value = %lu (%p) | target_data->value = %lu (%p) "
                       "| *host_op_id = %lu (%p) | optype = %s | src_addr = %p | src_device_num = %d | dest_addr = %p | "
                       "dest_device_num = %d | bytes = %lu | codeptr_ra = %p\n",
                       __FUNCTION__,
                       endpoint2string( endpoint ).c_str(),
                       target_task_data ? target_task_data->value : unknown_target_id,
                       target_task_data,
                       target_data ? target_data->value : unknown_target_id,
                       target_data,
                       host_op_id ? *host_op_id : unknown_host_op_id,
                       host_op_id,
                       data_op2string( optype ).c_str(),
                       src_addr,
                       src_device_num,
                       dest_addr,
                       dest_device_num,
                       bytes,
                       codeptr_ra );
        if (
            false
#if HAVE( OMPT_TARGET_DATA_TRANSFER_RECT )
            || optype == ompt_target_data_transfer_rect
#endif
#if HAVE( OMPT_TARGET_DATA_TRANSFER_RECT_ASYNC )
            || optype == ompt_target_data_transfer_rect_async
#endif
            )
        {
            /* typedef struct ompt_subvolume_t { // ADDED_60
             * const void *base;
             * uint64_t size;
             * uint64_t num_dims;
             * const uint64_t *volume;
             * const uint64_t *offsets;
             * const uint64_t *dimensions;
             * } ompt_subvolume_t; */
#if HAVE( OMPT_SUBVOLUME )
            ompt_subvolume_t* subvolume_src  = ( ompt_subvolume_t* )dev1_addr;
            ompt_subvolume_t* subvolume_dest = ( ompt_subvolume_t* )dev2_addr;

            atomic_printf( "[%s] src_base = %p | src_size = %lu | src_num_dims = %lu | src_volume = %p | "
                           "src_offsets = %p | src_dimensions = %p\n",
                           __FUNCTION__,
                           subvolume_src->base,
                           subvolume_src->size,
                           subvolume_src->num_dims,
                           subvolume_src->volume,
                           subvolume_src->offsets,
                           subvolume_src->dimensions );
            for ( int i = 0; i < subvolume_src->num_dims; i++ )
            {
                atomic_printf( "[%s] src_volume[%d] = %lu | src_offsets[%d] = %lu | src_dimensions[%d] = %lu\n",
                               __FUNCTION__,
                               i,
                               subvolume_src->volume[ i ],
                               i,
                               subvolume_src->offsets[ i ],
                               i,
                               subvolume_src->dimensions[ i ] );
            }

            atomic_printf( "[%s] dest_base = %p | dest_size = %lu | dest_num_dims = %lu | dest_volume = %p | "
                           "dest_offsets = %p | dest_dimensions = %p\n",
                           __FUNCTION__,
                           subvolume_dest->base,
                           subvolume_dest->size,
                           subvolume_dest->num_dims,
                           subvolume_dest->volume,
                           subvolume_dest->offsets,
                           subvolume_dest->dimensions );
            for ( int i = 0; i < subvolume_dest->num_dims; i++ )
            {
                atomic_printf( "[%s] dest_volume[%d] = %lu | dest_offsets[%d] = %lu | dest_dimensions[%d] = %lu\n",
                               __FUNCTION__,
                               i,
                               subvolume_dest->volume[ i ],
                               i,
                               subvolume_dest->offsets[ i ],
                               i,
                               subvolume_dest->dimensions[ i ] );
            }
#endif
        }
    }
}

template<printf_mode mode>
void
callback_target_submit( ompt_id_t    target_id,
                        ompt_id_t    host_op_id,
                        unsigned int requested_num_teams )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] target_id = %lu | host_op_id = %lu | requested_num_teams = %u\n",
                       __FUNCTION__,
                       target_id,
                       host_op_id,
                       requested_num_teams );
    }
}

template<printf_mode mode>
void
callback_target_submit_emi( ompt_scope_endpoint_t endpoint,
                            ompt_data_t*          target_data,
                            ompt_id_t*            host_op_id,
                            unsigned int          requested_num_teams )
{
    if constexpr ( mode > printf_mode::disable_output )
    {
        if ( host_op_id && endpoint == ompt_scope_begin )
        {
            *host_op_id = ++next_host_op_id;
        }
    }

    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] endpoint = %s | target_data->value = %lu (%p) | "
                       "*host_op_id = %lu (%p) | requested_num_teams = %u\n",
                       __FUNCTION__,
                       endpoint2string( endpoint ).c_str(),
                       target_data ? target_data->value : unknown_target_id,
                       target_data,
                       host_op_id ? *host_op_id : unknown_host_op_id,
                       host_op_id,
                       requested_num_teams );
    }
}

/* Tool control */

template<printf_mode mode>
int
callback_control_tool( uint64_t    command,
                       uint64_t    modifier,
                       void*       arg,
                       const void* codeptr_ra )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] command = %lu | modifier = %lu | arg = %p | codeptr_ra = %p\n",
                       __FUNCTION__,
                       command,
                       modifier,
                       arg,
                       codeptr_ra );
        return omp_control_tool_success;
    }
    switch ( command )
    {
        case omp_control_tool_start:
        {
            for ( auto device : get_devices() )
            {
                if ( device.second->device_functions.start_trace )
                {
                    device.second->device_functions.start_trace(
                        device.second->address, callback_buffer_request<mode>,
                        callback_buffer_complete<mode>);
                }
            }
            return omp_control_tool_success;
        }
        case omp_control_tool_flush:
        {
            for ( auto device : get_devices() )
            {
                if ( device.second->device_functions.flush_trace )
                {
                    device.second->device_functions.flush_trace( device.second->address );
                }
            }
            return omp_control_tool_success;
        }
        case omp_control_tool_pause:
        {
            for ( auto device : get_devices() )
            {
                if ( device.second->device_functions.pause_trace )
                {
                    device.second->device_functions.pause_trace(
                        device.second->address, static_cast<bool>( modifier ) );
                }
            }
        }
        case omp_control_tool_end:
        {
            for ( auto device : get_devices() )
            {
                if ( device.second->device_functions.stop_trace )
                {
                    device.second->device_functions.stop_trace( device.second->address );
                }
            }
            return omp_control_tool_success;
        }
        default:
            return omp_control_tool_ignored;
    }
}

/* Initialization / Finalization sequence */

template<printf_mode mode>
int
tool_initialize( ompt_function_lookup_t lookup,
                 int                    initial_device_num,
                 ompt_data_t*           tool_data )
{
    constexpr auto function_name = __FUNCTION__;
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] lookup = %p | initial_device_num = %d | tool_data = %p\n",
                       __FUNCTION__,
                       lookup,
                       initial_device_num,
                       tool_data );
    }

    /* ompt_function_lookup_t allows us to look-up runtime functions and register
     * callbacks which will get invoked on OpenMP events by the runtime. */
    auto ompt_set_callback = ( ompt_set_callback_t )lookup( "ompt_set_callback" );
    assert( ompt_set_callback && "Could not find ompt_set_callback" );

    /* typedef enum ompt_callbacks_t {
     *   ompt_callback_thread_begin             = 1,
     *   ompt_callback_thread_end               = 2,
     *   ompt_callback_parallel_begin           = 3,
     *   ompt_callback_parallel_end             = 4,
     *   ompt_callback_task_create              = 5,
     *   ompt_callback_task_schedule            = 6,
     *   ompt_callback_implicit_task            = 7,
     *   ompt_callback_target                   = 8, DEPRECATED_60
     *   ompt_callback_target_data_op           = 9, DEPRECATED_60
     *   ompt_callback_target_submit            = 10, DEPRECATED_60
     *   ompt_callback_control_tool             = 11,
     *   ompt_callback_device_initialize        = 12,
     *   ompt_callback_device_finalize          = 13,
     *   ompt_callback_device_load              = 14,
     *   ompt_callback_device_unload            = 15,
     *   ompt_callback_sync_region_wait         = 16,
     *   ompt_callback_mutex_released           = 17,
     *   ompt_callback_dependences              = 18,
     *   ompt_callback_task_dependence          = 19,
     *   ompt_callback_work                     = 20,
     *   ompt_callback_master                   = 21, DEPRECATED_51, REMOVED_60
     *   ompt_callback_masked                   = 21, ADDED_51
     *   ompt_callback_target_map               = 22, DEPRECATED_60
     *   ompt_callback_sync_region              = 23,
     *   ompt_callback_lock_init                = 24,
     *   ompt_callback_lock_destroy             = 25,
     *   ompt_callback_mutex_acquire            = 26,
     *   ompt_callback_mutex_acquired           = 27,
     *   ompt_callback_nest_lock                = 28,
     *   ompt_callback_flush                    = 29,
     *   ompt_callback_cancel                   = 30,
     *   ompt_callback_reduction                = 31,
     *   ompt_callback_dispatch                 = 32,
     *   ompt_callback_target_emi               = 33, ADDED_51
     *   ompt_callback_target_data_op_emi       = 34, ADDED_51
     *   ompt_callback_target_submit_emi        = 35, ADDED_51
     *   ompt_callback_target_map_emi           = 36, ADDED_51
     *   ompt_callback_error                    = 37
     * } ompt_callbacks_t; */

    /* Register callbacks for the host side */
#define CALLBACK( name ) \
    { \
        ompt_callback_##name, ( ompt_callback_t )&callback_##name<mode>, #name \
    }

    /* Registers all callbacks in initializer list */
    const auto register_all_callbacks =
        [ &ompt_set_callback ]( const std::initializer_list<registration_data_t>& callbacks )
        {
            for ( const auto& cb: callbacks )
            {
                ompt_set_result_t result = ompt_set_callback( cb.event, cb.callback );
                if constexpr ( mode >= printf_mode::callback )
                {
                    atomic_printf( "[%s] %18s = %s\n",
                                   function_name,
                                   cb.name,
                                   set_result2string( result ).c_str() );
                }
            }
        };

    /* Registers all callbacks in initializer list, but breaks once a callback was successfully registered.
     * This is required for the target callbacks, where only one of the two options can be registered. */
    const auto register_callbacks_break_on_success =
        [ &ompt_set_callback ]( const std::initializer_list<registration_data_t>& callbacks )
        {
            for ( const auto& cb: callbacks )
            {
                ompt_set_result_t result = ompt_set_callback( cb.event, cb.callback );
                if constexpr ( mode >= printf_mode::callback )
                {
                    atomic_printf( "[%s] %18s = %s\n",
                                   function_name,
                                   cb.name,
                                   set_result2string( result ).c_str() );
                }
                if ( result == ompt_set_always )
                {
                    break;
                }
            }
        };

    const std::initializer_list<registration_data_t> host_callbacks = {
        CALLBACK( thread_begin ),
        CALLBACK( thread_end ),
        CALLBACK( parallel_begin ),
        CALLBACK( parallel_end ),
        CALLBACK( task_create ),
        CALLBACK( task_schedule ),
        CALLBACK( implicit_task ),
        CALLBACK( sync_region_wait ),
        CALLBACK( mutex_released ),
        CALLBACK( dependences ),
        CALLBACK( task_dependence ),
        CALLBACK( work ),
#if HAVE( OMPT_CALLBACK_MASKED )
        CALLBACK( masked ),
#endif
        CALLBACK( sync_region ),
        CALLBACK( lock_init ),
        CALLBACK( lock_destroy ),
        CALLBACK( mutex_acquire ),
        CALLBACK( mutex_acquired ),
        CALLBACK( nest_lock ),
        CALLBACK( flush ),
        CALLBACK( cancel ),
        CALLBACK( reduction ),
        CALLBACK( dispatch ),
        CALLBACK( control_tool ),
    };
    register_all_callbacks( host_callbacks );

    /* Register callbacks for the host side accelerator */
    const std::initializer_list<registration_data_t> host_accel_callbacks = {
        CALLBACK( device_initialize ),
        CALLBACK( device_finalize ),
        CALLBACK( device_load ),
        CALLBACK( device_unload ),
    };
    register_all_callbacks( host_accel_callbacks );

    const std::initializer_list<registration_data_t> host_accel_target = {
#if HAVE( OMPT_CALLBACK_TARGET_EMI )
        CALLBACK( target_emi ),
#endif
#if HAVE( OMPT_CALLBACK_TARGET )
        CALLBACK( target ),
#endif
    };
    register_callbacks_break_on_success( host_accel_target );

    const std::initializer_list<registration_data_t> host_accel_target_map = {
#if HAVE( OMPT_CALLBACK_TARGET_MAP_EMI )
        CALLBACK( target_map_emi ),
#endif
#if HAVE( OMPT_CALLBACK_TARGET_MAP )
        CALLBACK( target_map ),
#endif
    };
    register_callbacks_break_on_success( host_accel_target_map );

    const std::initializer_list<registration_data_t> host_accel_target_data_op = {
#if HAVE( OMPT_CALLBACK_TARGET_DATA_OP_EMI )
        CALLBACK( target_data_op_emi ),
#endif
#if HAVE( OMPT_CALLBACK_TARGET_DATA_OP )
        CALLBACK( target_data_op ),
#endif
    };
    register_callbacks_break_on_success( host_accel_target_data_op );

    const std::initializer_list<registration_data_t> host_accel_target_submit = {
#if HAVE( OMPT_CALLBACK_TARGET_SUBMIT_EMI )
        CALLBACK( target_submit_emi ),
#endif
#if HAVE( OMPT_CALLBACK_TARGET_SUBMIT )
        CALLBACK( target_submit )
#endif
    };
    register_callbacks_break_on_success( host_accel_target_submit );

#undef CALLBACK

    return true;  /* non-zero indicates success */
}

template<printf_mode mode>
void
tool_finalize( ompt_data_t* tool_data )
{
    if constexpr ( mode == printf_mode::callback )
    {
        print_function_name( __FUNCTION__ );
    }
    else if constexpr ( mode == printf_mode::callback_include_args )
    {
        atomic_printf( "[%s] tool_data = %p\n",
                       __FUNCTION__,
                       tool_data );
    }
}

extern "C" ompt_start_tool_result_t *
ompt_start_tool( unsigned int omp_version,
                 const char*  runtime_version )
{
    printf_mode chosen_printf_mode = printf_mode::callback_include_args;

    /* Check if the OMPT_mode == printf_mode::callback_include_args environment variable is set.
     * Depending on the chosen integer value, the printing mode is chosen. */
    const char* env_printf_mode     = std::getenv( "OMPT_PRINTF_MODE" );
    const int   env_printf_mode_int = env_printf_mode ? std::stoi( env_printf_mode ) : -1;
    assert( env_printf_mode_int >= -1 && env_printf_mode_int < static_cast<int> ( printf_mode::num_modes ) && "Invalid OMPT_PRINTF_MODE" );
    if ( env_printf_mode_int >= 0 && env_printf_mode_int < static_cast<int>( printf_mode::num_modes ) )
    {
        chosen_printf_mode = static_cast<printf_mode>( env_printf_mode_int );
    }
    atomic_printf( "[%s] Chosen printf mode: %d\n",
                   __FUNCTION__,
                   chosen_printf_mode );

    static ompt_start_tool_result_t tool;
    switch ( chosen_printf_mode )
    {
        case printf_mode::disable_tool:
            /* According to OpenMP 5.2 spec.: A tool may return NULL from
             * ompt_start_tool to indicate that it will not use the OMPT
             * interface in a particular execution. */
            return nullptr;
        case printf_mode::disable_output:
            tool.initialize = &tool_initialize<printf_mode::disable_output>;
            tool.finalize   = &tool_finalize<printf_mode::disable_output>;
            break;
        case printf_mode::callback:
            print_function_name( __FUNCTION__ );
            tool.initialize = &tool_initialize<printf_mode::callback>;
            tool.finalize   = &tool_finalize<printf_mode::callback>;
            break;
        case printf_mode::callback_include_args:
            atomic_printf( "[%s] omp_version = %u | runtime_version = %s\n",
                           __FUNCTION__,
                           omp_version,
                           runtime_version );
            tool.initialize = &tool_initialize<printf_mode::callback_include_args>;
            tool.finalize   = &tool_finalize<printf_mode::callback_include_args>;
            break;
        default:
            assert( false && "Unknown printf mode" );
    }
    tool.tool_data = ompt_data_none;

    return &tool;
}
