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

#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdlib>
#include <sstream>
#include <string>

#include <omp-tools.h>

/* Global variables */

constexpr int32_t             unknown_thread_id   = -1;
constexpr ompt_id_t           unknown_task_id     = 555555555;
static std::atomic<ompt_id_t> next_task_id        = 555000000;
constexpr ompt_id_t           unknown_parallel_id = 666666666;
static std::atomic<ompt_id_t> next_parallel_id    = 666000000;
constexpr ompt_id_t           unknown_target_id   = 777777777;
static std::atomic<ompt_id_t> next_target_id      = 777000000;
constexpr ompt_id_t           unknown_host_op_id  = 888888888;
static std::atomic<ompt_id_t> next_host_op_id     = 888000000;

static ompt_get_unique_id_t get_unique_id = nullptr;
static __thread int32_t     thread_id     = unknown_thread_id;
static std::atomic<int32_t> next_thread_id { 0 };

/* Helpers */

enum class printf_mode : int
{
    disable               = 0,
    callback              = 1,
    callback_include_args = 2,

    num_modes
};

typedef struct registration_data_t
{
    ompt_callbacks_t event;
    ompt_callback_t  callback;
    const char*      name;
} registration_data_t;

/* printf conversion output */

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
 *   ompt_task_taskwait   = 0x00000010,
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
    else if ( flags & ompt_task_taskwait )
    {
        result << "_taskwait";
        flags -= ompt_task_taskwait;
    }

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
    assert( flags == 0 && "Unknown ompt_task_flag_t" );
    return result.str();
}

/* typedef enum ompt_scope_endpoint_t {
 *   ompt_scope_begin    = 1,
 *   ompt_scope_end      = 2,
 *   ompt_scope_beginend = 3
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
        case ompt_scope_beginend:
            return "beginend";
        default:
            assert( false && "Unknown ompt_scope_endpoint_t" );
    }
}

/* typedef enum ompt_task_status_t {
 *   ompt_task_complete      = 1,
 *   ompt_task_yield         = 2,
 *   ompt_task_cancel        = 3,
 *   ompt_task_detach        = 4,
 *   ompt_task_early_fulfill = 5,
 *   ompt_task_late_fulfill  = 6,
 *   ompt_task_switch        = 7,
 *   ompt_taskwait_complete  = 8
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
        case ompt_taskwait_complete:
            return "taskwait_complete";
        default:
            assert( false && "Unknown ompt_task_status_t" );
    }
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
}

/* typedef enum ompt_dispatch_t {
 *   ompt_dispatch_iteration        = 1,
 *   ompt_dispatch_section          = 2,
 *   ompt_dispatch_ws_loop_chunk    = 3,
 *   ompt_dispatch_taskloop_chunk   = 4,
 *   ompt_dispatch_distribute_chunk = 5
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
        case ompt_dispatch_ws_loop_chunk:
            return "ws_loop_chunk";
        case ompt_dispatch_taskloop_chunk:
            return "taskloop_chunk";
        case ompt_dispatch_distribute_chunk:
            return "distribute_chunk";
        default:
            assert( false && "Unknown ompt_dispatch_t" );
    }
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
 *     ompt_work_scope           = 8,
 *     ompt_work_loop_static     = 10,
 *     ompt_work_loop_dynamic    = 11,
 *     ompt_work_loop_guided     = 12,
 *     ompt_work_loop_other      = 13
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
        case ompt_work_scope:
            return "scope";
        case ompt_work_loop_static:
            return "loop_static";
        case ompt_work_loop_dynamic:
            return "loop_dynamic";
        case ompt_work_loop_guided:
            return "loop_guided";
        case ompt_work_loop_other:
            return "loop_other";
        default:
            assert( false && "Unknown ompt_work_t" );
    }
}

/* typedef enum ompt_sync_region_t {
 *     ompt_sync_region_barrier                    DEPRECATED_51 = 1,
 *     ompt_sync_region_barrier_implicit           DEPRECATED_51 = 2,
 *     ompt_sync_region_barrier_explicit           = 3,
 *     ompt_sync_region_barrier_implementation     = 4,
 *     ompt_sync_region_taskwait                   = 5,
 *     ompt_sync_region_taskgroup                  = 6,
 *     ompt_sync_region_reduction                  = 7,
 *     ompt_sync_region_barrier_implicit_workshare = 8,
 *     ompt_sync_region_barrier_implicit_parallel  = 9,
 *     ompt_sync_region_barrier_teams              = 10
 * } ompt_sync_region_t; */
static inline std::string
sync2string( ompt_sync_region_t t )
{
    switch ( t )
    {
        case ompt_sync_region_barrier:
            return "barrier";
        case ompt_sync_region_barrier_implicit:
            return "barrier_implicit";
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
        case ompt_sync_region_barrier_implicit_workshare:
            return "barrier_implicit_workshare";
        case ompt_sync_region_barrier_implicit_parallel:
            return "barrier_implicit_parallel";
        case ompt_sync_region_barrier_teams:
            return "barrier_teams";
        default:
            assert( false && "Unknown ompt_sync_region_t" );
    }
}

/* typedef enum ompt_target_data_op_t {
 *     ompt_target_data_alloc                      = 1,
 *     ompt_target_data_transfer_to_device         = 2,
 *     ompt_target_data_transfer_from_device       = 3,
 *     ompt_target_data_delete                     = 4,
 *     ompt_target_data_associate                  = 5,
 *     ompt_target_data_disassociate               = 6,
 *     ompt_target_data_alloc_async                = 17,
 *     ompt_target_data_transfer_to_device_async   = 18,
 *     ompt_target_data_transfer_from_device_async = 19,
 *     ompt_target_data_delete_async               = 20
 * } ompt_target_data_op_t; */
static inline std::string
data_op2string( ompt_target_data_op_t t )
{
    switch ( t )
    {
        case ompt_target_data_alloc:
            return "data_alloc";
        case ompt_target_data_transfer_to_device:
            return "transfer_to_device";
        case ompt_target_data_transfer_from_device:
            return "transfer_from_device";
        case ompt_target_data_delete:
            return "data_delete";
        case ompt_target_data_associate:
            return "data_associate";
        case ompt_target_data_disassociate:
            return "data_disassociate";
        case ompt_target_data_alloc_async:
            return "data_alloc_async";
        case ompt_target_data_transfer_to_device_async:
            return "transfer_to_device_async";
        case ompt_target_data_transfer_from_device_async:
            return "transfer_from_device_async";
        case ompt_target_data_delete_async:
            return "data_delete_async";
        default:
            assert( false && "Unknown ompt_target_data_op_t" );
    }
}

/* typedef enum ompt_target_t {
 *   ompt_target                         = 1,
 *   ompt_target_enter_data              = 2,
 *   ompt_target_exit_data               = 3,
 *   ompt_target_update                  = 4,
 *   ompt_target_nowait                  = 9,
 *   ompt_target_enter_data_nowait       = 10,
 *   ompt_target_exit_data_nowait        = 11,
 *   ompt_target_update_nowait           = 12
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
        case ompt_target_nowait:
            return "target_nowait";
        case ompt_target_enter_data_nowait:
            return "target_enter_data_nowait";
        case ompt_target_exit_data_nowait:
            return "target_exit_data_nowait";
        case ompt_target_update_nowait:
            return "target_update_nowait";
        default:
            assert( false && "Unknown ompt_target_t" );
    }
}

/* Host side callbacks */

template<printf_mode mode>
void
callback_thread_begin( ompt_thread_t thread_type,
                       ompt_data_t*  thread_data )
{
    if ( mode > printf_mode::disable )
    {
        thread_id = next_thread_id++;
    }
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | thread_type = %s | thread_data = %p\n",
                __FUNCTION__,
                thread_id,
                thread2string( thread_type ).c_str(),
                thread_data );
    }
}

template<printf_mode mode>
void
callback_thread_end( ompt_data_t* thread_data )
{
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | thread_data = %p\n",
                __FUNCTION__,
                thread_id,
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
    if ( mode > printf_mode::disable )
    {
        if ( parallel_data )
        {
            parallel_data->value = ++next_parallel_id;
        }
    }

    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | encountering_task_data->value = %lu | encountering_task_frame = %p | "
                "parallel_data->value %lu | requested_parallelism = %u | flags = %s | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                encountering_task_data ? encountering_task_data->value : unknown_task_id,
                encountering_task_frame,
                parallel_data ? parallel_data->value : unknown_parallel_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | parallel_data->value = %p | encountering_task_data->value = %lu | flags = %s | "
                "codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                parallel_data,
                encountering_task_data ? encountering_task_data->value : unknown_task_id,
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
    if ( mode > printf_mode::disable )
    {
        if ( new_task_data )
        {
            new_task_data->value = ++next_task_id;
        }
    }

    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | encountering_task_data->value = %lu | encountering_task_frame = %p | "
                "new_task_data->value = %lu | flags = %s | has_dependences = %d | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                encountering_task_data ? encountering_task_data->value : unknown_task_id,
                encountering_task_frame,
                new_task_data ? new_task_data->value : unknown_task_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | prior_task_data->value = %lu | prior_task_status = %s | "
                "next_task_data->value = %lu\n",
                __FUNCTION__,
                thread_id,
                prior_task_data ? prior_task_data->value : unknown_task_id,
                task_status2string( prior_task_status ).c_str(),
                next_task_data ? next_task_data->value : unknown_task_id );
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
    if ( mode > printf_mode::disable )
    {
        if ( task_data && endpoint == ompt_scope_begin )
        {
            task_data->value = ++next_task_id;
        }
    }

    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | endpoint = %s | parallel_data->value = %lu | task_data->value = %lu | "
                "actual_parallelism = %u | index = %u | flags = %s\n",
                __FUNCTION__,
                thread_id,
                endpoint2string( endpoint ).c_str(),
                parallel_data ? parallel_data->value : unknown_parallel_id,
                task_data ? task_data->value : unknown_task_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | endpoint = %s | parallel_data->value = %lu | "
                "task_data->value = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                sync2string( kind ).c_str(),
                endpoint2string( endpoint ).c_str(),
                parallel_data ? parallel_data->value : unknown_parallel_id,
                task_data ? task_data->value : unknown_task_id,
                codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_mutex_released( ompt_mutex_t   kind,
                         ompt_wait_id_t wait_id,
                         const void*    codeptr_ra )
{
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | wait_id = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__, thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | task_data->value = %lu | deps = %p | ndeps = %d\n",
                __FUNCTION__,
                thread_id,
                task_data ? task_data->value : unknown_task_id,
                deps,
                ndeps );
    }
}

template<printf_mode mode>
void
callback_task_dependence( ompt_data_t* src_task_data,
                          ompt_data_t* sink_task_data )
{
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | src_task_data->value = %lu | sink_task_data->value = %lu\n",
                __FUNCTION__,
                thread_id,
                src_task_data ? src_task_data->value : unknown_task_id,
                sink_task_data ? sink_task_data->value : unknown_task_id );
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | work_type = %s | endpoint = %s | parallel_data->value = %lu | "
                "task_data->value = %lu | count = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                work2string( work_type ).c_str(),
                endpoint2string( endpoint ).c_str(),
                parallel_data ? parallel_data->value : unknown_parallel_id,
                task_data ? task_data->value : unknown_task_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | endpoint = %s | parallel_data->value = %lu | "
                "task_data->value = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                endpoint2string( endpoint ).c_str(),
                parallel_data ? parallel_data->value : unknown_parallel_id,
                task_data ? task_data->value : unknown_task_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | endpoint = %s | parallel_data->value = %lu | "
                "task_data->value = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                sync2string( kind ).c_str(),
                endpoint2string( endpoint ).c_str(),
                parallel_data ? parallel_data->value : unknown_parallel_id,
                task_data ? task_data->value : unknown_task_id,
                codeptr_ra );
    }
}

template<printf_mode mode>
void
callback_lock_init( ompt_mutex_t   kind,
                    ompt_wait_id_t wait_id,
                    const void*    codeptr_ra )
{
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | wait_id = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                mutex2string( kind ).c_str(),
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | wait_id = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | hint = %u | impl = %u | wait_id = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | wait_id = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | endpoint = %s | wait_id = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n", __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | thread_data = %p | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | task_data->value = %lu | flags = %s | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                task_data ? task_data->value : unknown_task_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | endpoint = %s | parallel_data->value = %lu | "
                "task_data->value = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                sync2string( kind ).c_str(),
                endpoint2string( endpoint ).c_str(),
                parallel_data ? parallel_data->value : unknown_parallel_id,
                task_data ? task_data->value : unknown_task_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | parallel_data->value = %lu | task_data->value = %lu | kind = %s | "
                "instance->value = %lu\n",
                __FUNCTION__,
                thread_id,
                parallel_data ? parallel_data->value : unknown_parallel_id,
                task_data ? task_data->value : unknown_task_id,
                dispatch2string( kind ).c_str(),
                instance.value );
    }
}

/* Host side accelerator callbacks */

template<printf_mode mode>
void
callback_device_initialize( int                    device_num,
                            const char*            type,
                            ompt_device_t*         device,
                            ompt_function_lookup_t lookup,
                            const char*            documentation )
{
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | device_num = %d | type = %s | device = %p | lookup = %p | documentation = %s\n",
                __FUNCTION__,
                thread_id,
                device_num,
                type,
                device,
                lookup,
                documentation );
    }
}

template<printf_mode mode>
void
callback_device_finalize( int device_num )
{
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | device_num = %d\n",
                __FUNCTION__,
                thread_id,
                device_num );
    }
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | device_num = %d | filename = %s | offset_in_file = %ld | vma_in_file = %p | bytes = %lu "
                "| host_addr = %p | device_addr = %p | module_id = %lu\n",
                __FUNCTION__,
                thread_id,
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | device_num = %d | module_id = %lu\n",
                __FUNCTION__,
                thread_id,
                device_num,
                module_id );
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
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | target_data->value = %lu | nitems = %u | host_addr = %p | device_addr = %p | "
                "bytes = %p | mapping_flags = %p | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                target_data ? target_data->value : unknown_target_id,
                nitems,
                host_addr,
                device_addr,
                bytes,
                mapping_flags,
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
    if ( mode > printf_mode::disable )
    {
        if ( task_data && endpoint == ompt_scope_begin )
        {
            target_data->value = ++next_target_id;
        }
    }

    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | kind = %s | endpoint = %s | device_num = %d | task_data->value = %lu | "
                "target_task_data->value = %lu | target_data->value = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                target2string( kind ).c_str(),
                endpoint2string( endpoint ).c_str(),
                device_num,
                task_data ? task_data->value : unknown_task_id,
                target_task_data ? target_task_data->value : unknown_task_id,
                target_data ? target_data->value : unknown_target_id,
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
                             void*                 src_addr,
                             int                   src_device_num,
                             void*                 dest_addr,
                             int                   dest_device_num,
                             size_t                bytes,
                             const void*           codeptr_ra )
{
    if ( mode > printf_mode::disable )
    {
        if ( host_op_id && endpoint == ompt_scope_begin )
        {
            *host_op_id = ++next_host_op_id;
        }
    }

    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | endpoint = %s | target_task_data->value = %lu | target_data->value = %lu "
                "| *host_op_id = %lu | optype = %s | src_addr = %p | src_device_num = %d | dest_addr = %p | "
                "dest_device_num = %d | bytes = %lu | codeptr_ra = %p\n",
                __FUNCTION__,
                thread_id,
                endpoint2string( endpoint ).c_str(),
                target_task_data ? target_task_data->value : unknown_target_id,
                target_data ? target_data->value : unknown_target_id,
                host_op_id ? *host_op_id : unknown_host_op_id,
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
callback_target_submit_emi( ompt_scope_endpoint_t endpoint,
                            ompt_data_t*          target_data,
                            ompt_id_t*            host_op_id,
                            unsigned int          requested_num_teams )
{
    if ( mode > printf_mode::disable )
    {
        if ( host_op_id && endpoint == ompt_scope_begin )
        {
            *host_op_id = ++next_host_op_id;
        }
    }

    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | endpoint = %s | target_data->value = %lu | "
                "*host_op_id = %lu | requested_num_teams = %u\n",
                __FUNCTION__,
                thread_id,
                endpoint2string( endpoint ).c_str(),
                target_data ? target_data->value : unknown_target_id,
                host_op_id ? *host_op_id : unknown_host_op_id,
                requested_num_teams );
    }
}

/* Initialization / Finalization sequence */

template<printf_mode mode>
int
tool_initialize( ompt_function_lookup_t lookup,
                 int                    initial_device_num,
                 ompt_data_t*           tool_data )
{
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | lookup = %p | initial_device_num = %d | tool_data = %p\n",
                __FUNCTION__,
                thread_id,
                lookup,
                initial_device_num,
                tool_data );
    }

    /* ompt_function_lookup_t allows us to look-up runtime functions and register
     * callbacks which will get invoked on OpenMP events by the runtime. */
    auto ompt_set_callback = ( ompt_set_callback_t )lookup( "ompt_set_callback" );
    assert( ompt_set_callback && "Could not find ompt_set_callback" );
    if ( mode > printf_mode::disable )
    {
        get_unique_id = ( ompt_get_unique_id_t )lookup( "ompt_get_unique_id" );
        assert( get_unique_id && "Could not find ompt_get_unique_id" );
    }

    /* Register callbacks for the host side */
#define CALLBACK( name )                                                       \
    {                                                                          \
        ompt_callback_##name, ( ompt_callback_t )&callback_##name<mode>, #name \
    }
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
        CALLBACK( masked ),
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
    };

    for ( const auto& cb: host_callbacks )
    {
        ompt_set_result_t result = ompt_set_callback( cb.event, cb.callback );
        if ( mode >= printf_mode::callback )
        {
            printf( "[%s] tid = %d | %18s = %s\n",
                    __FUNCTION__, thread_id,
                    cb.name,
                    set_result2string( result ).c_str() );
        }
    }

    /* Register callbacks for the host side accelerator */
    const std::initializer_list<registration_data_t> host_accel_callbacks = {
        CALLBACK( device_initialize ),
        CALLBACK( device_finalize ),
        CALLBACK( device_load ),
        CALLBACK( device_unload ),
        CALLBACK( target_emi ),
        CALLBACK( target_map_emi ),
        CALLBACK( target_data_op_emi ),
        CALLBACK( target_submit_emi )
    };

    for ( const auto& cb: host_accel_callbacks )
    {
        ompt_set_result_t result = ompt_set_callback( cb.event, cb.callback );
        if ( mode >= printf_mode::callback )
        {
            printf( "[%s] tid = %d | %18s = %s\n",
                    __FUNCTION__,
                    thread_id,
                    cb.name,
                    set_result2string( result ).c_str() );
        }
    }
#undef CALLBACK

    return true; /* non-zero indicates success */
}

template<printf_mode mode>
void
tool_finalize( ompt_data_t* tool_data )
{
    if ( mode == printf_mode::callback )
    {
        printf( "[%s] tid = %d\n",
                __FUNCTION__,
                thread_id );
    }
    else if ( mode == printf_mode::callback_include_args )
    {
        printf( "[%s] tid = %d | tool_data = %p\n",
                __FUNCTION__,
                thread_id,
                tool_data );
    }
}

extern "C" ompt_start_tool_result_t *
ompt_start_tool( unsigned int omp_version,
                 const char* runtime_version )
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

    static ompt_start_tool_result_t tool;
    switch ( chosen_printf_mode )
    {
        case printf_mode::disable:
            tool.initialize = &tool_initialize<printf_mode::disable>;
            tool.finalize   = &tool_finalize<printf_mode::disable>;
            break;
        case printf_mode::callback:
            printf( "[%s] tid = %d\n",
                    __FUNCTION__,
                    thread_id );
            tool.initialize = &tool_initialize<printf_mode::callback>;
            tool.finalize   = &tool_finalize<printf_mode::callback>;
            break;
        case printf_mode::callback_include_args:
            printf( "[%s] tid = %d | omp_version = %u | runtime_version = %s\n",
                    __FUNCTION__,
                    thread_id,
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
