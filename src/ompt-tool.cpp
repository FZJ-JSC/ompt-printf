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
#include <cassert>
#include <cstdlib>
#include <string>

#include <omp-tools.h>

/* Helper struct */
typedef struct registration_data_t {
    ompt_callbacks_t event;
    ompt_callback_t callback;
    const char *name;
} registration_data_t;

/* printf conversion output */

static const char *
set_result2string(ompt_set_result_t t) {
    switch (t) {
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
    }
    assert(false && "Unknown ompt_set_result_t");
}

static const char *
thread2string(ompt_thread_t t) {
    switch (t) {
        case ompt_thread_initial:
            return "initial";
        case ompt_thread_worker:
            return "worker";
        case ompt_thread_other:
            return "other";
        case ompt_thread_unknown:
            return "unknown";
    }
    assert(false && "Unknown ompt_thread_t");
}

/* Host side callbacks */

template<bool enable_printf>
void
callback_thread_begin(ompt_thread_t thread_type,
                      ompt_data_t *thread_data) {
    if (enable_printf) {
        printf("[%25s] thread_type = %s | thread_data = %p\n", __FUNCTION__, thread2string(thread_type), thread_data);
    }
}

template<bool enable_printf>
void
callback_thread_end(ompt_data_t *thread_data) {
    if (enable_printf) {
        printf("[%25s] thread_data = %p\n", __FUNCTION__, thread_data);
    }
}

template<bool enable_printf>
void
callback_parallel_begin(ompt_data_t *encountering_task_data,
                        const ompt_frame_t *encountering_task_frame,
                        ompt_data_t *parallel_data,
                        unsigned int requested_parallelism,
                        int flags,
                        const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] encountering_task_data = %p | encountering_task_frame = %p | parallel_data = %p | requested_parallelism = %u | flags = %d | codeptr_ra = %p\n",
               __FUNCTION__, encountering_task_data, encountering_task_frame, parallel_data, requested_parallelism,
               flags, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_parallel_end(ompt_data_t *parallel_data,
                      ompt_data_t *encountering_task_data,
                      int flags,
                      const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] parallel_data = %p | encountering_task_data = %p | flags = %d | codeptr_ra = %p\n",
               __FUNCTION__, parallel_data, encountering_task_data, flags, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_task_create(ompt_data_t *encountering_task_data,
                     const ompt_frame_t *encountering_task_frame,
                     ompt_data_t *new_task_data,
                     int flags,
                     int has_dependences,
                     const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] encountering_task_data = %p | encountering_task_frame = %p | new_task_data = %p | flags = %d | has_dependences = %d | codeptr_ra = %p\n",
               __FUNCTION__, encountering_task_data, encountering_task_frame, new_task_data, flags, has_dependences,
               codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_task_schedule(ompt_data_t *prior_task_data,
                       ompt_task_status_t prior_task_status,
                       ompt_data_t *next_task_data) {
    if (enable_printf) {
        printf("[%25s] prior_task_data = %p | prior_task_status = %d | next_task_data = %p\n",
               __FUNCTION__, prior_task_data, prior_task_status, next_task_data);
    }
}

template<bool enable_printf>
void
callback_implicit_task(ompt_scope_endpoint_t endpoint,
                       ompt_data_t *parallel_data,
                       ompt_data_t *task_data,
                       unsigned int actual_parallelism,
                       unsigned int index,
                       int flags) {
    if (enable_printf) {
        printf("[%25s] endpoint = %d | parallel_data = %p | task_data = %p | actual_parallelism = %u | index = %u | flags = %d\n",
               __FUNCTION__, endpoint, parallel_data, task_data, actual_parallelism, index, flags);
    }
}

template<bool enable_printf>
void
callback_control_tool(uint64_t command,
                      uint64_t modifier,
                      void *arg,
                      const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] command = %lu | modifier = %lu | arg = %p | codeptr_ra = %p\n",
               __FUNCTION__, command, modifier, arg, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_sync_region_wait(ompt_sync_region_t kind,
                          ompt_scope_endpoint_t endpoint,
                          ompt_data_t *parallel_data,
                          ompt_data_t *task_data,
                          const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | endpoint = %d | parallel_data = %p | task_data = %p | codeptr_ra = %p\n",
               __FUNCTION__, kind, endpoint, parallel_data, task_data, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_mutex_released(ompt_mutex_t kind,
                        ompt_wait_id_t wait_id,
                        const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | wait_id = %lu | codeptr_ra = %p\n",
               __FUNCTION__, kind, wait_id, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_dependences(ompt_data_t *task_data,
                     const ompt_dependence_t *deps,
                     int ndeps) {
    if (enable_printf) {
        printf("[%25s] task_data = %p | deps = %p | ndeps = %d\n",
               __FUNCTION__, task_data, deps, ndeps);
    }
}

template<bool enable_printf>
void
callback_task_dependence(ompt_data_t *src_task_data,
                         ompt_data_t *sink_task_data) {
    if (enable_printf) {
        printf("[%25s] src_task_data = %p | sink_task_data = %p\n",
               __FUNCTION__, src_task_data, sink_task_data);
    }
}

template<bool enable_printf>
void
callback_work(ompt_work_t work_type,
              ompt_scope_endpoint_t endpoint,
              ompt_data_t *parallel_data,
              ompt_data_t *task_data,
              uint64_t count,
              const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] work_type = %d | endpoint = %d | parallel_data = %p | task_data = %p | count = %lu | codeptr_ra = %p\n",
               __FUNCTION__, work_type, endpoint, parallel_data, task_data, count, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_masked(ompt_scope_endpoint_t endpoint,
                ompt_data_t *parallel_data,
                ompt_data_t *task_data,
                const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] endpoint = %d | parallel_data = %p | task_data = %p | codeptr_ra = %p\n",
               __FUNCTION__, endpoint, parallel_data, task_data, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_sync_region(ompt_sync_region_t kind,
                     ompt_scope_endpoint_t endpoint,
                     ompt_data_t *parallel_data,
                     ompt_data_t *task_data,
                     const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | endpoint = %d | parallel_data = %p | task_data = %p | codeptr_ra = %p\n",
               __FUNCTION__, kind, endpoint, parallel_data, task_data, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_lock_init(ompt_mutex_t kind,
                   ompt_wait_id_t wait_id,
                   const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | wait_id = %lu | codeptr_ra = %p\n",
               __FUNCTION__, kind, wait_id, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_lock_destroy(ompt_mutex_t kind,
                      ompt_wait_id_t wait_id,
                      const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | wait_id = %lu | codeptr_ra = %p\n",
               __FUNCTION__, kind, wait_id, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_mutex_acquire(ompt_mutex_t kind,
                       unsigned int hint,
                       unsigned int impl,
                       ompt_wait_id_t wait_id,
                       const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | hint = %u | impl = %u | wait_id = %lu | codeptr_ra = %p\n",
               __FUNCTION__, kind, hint, impl, wait_id, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_mutex_acquired(ompt_mutex_t kind,
                        ompt_wait_id_t wait_id,
                        const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | wait_id = %lu | codeptr_ra = %p\n",
               __FUNCTION__, kind, wait_id, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_nest_lock(ompt_scope_endpoint_t endpoint,
                   ompt_wait_id_t wait_id,
                   const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] endpoint = %d | wait_id = %lu | codeptr_ra = %p\n",
               __FUNCTION__, endpoint, wait_id, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_flush(ompt_data_t *thread_data,
               const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] thread_data = %p | codeptr_ra = %p\n",
               __FUNCTION__, thread_data, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_cancel(ompt_data_t *task_data,
                int flags,
                const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] task_data = %p | flags = %d | codeptr_ra = %p\n",
               __FUNCTION__, task_data, flags, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_reduction(ompt_sync_region_t kind,
                   ompt_scope_endpoint_t endpoint,
                   ompt_data_t *parallel_data,
                   ompt_data_t *task_data,
                   const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | endpoint = %d | parallel_data = %p | task_data = %p | codeptr_ra = %p\n",
               __FUNCTION__, kind, endpoint, parallel_data, task_data, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_dispatch(ompt_data_t *parallel_data,
                  ompt_data_t *task_data,
                  ompt_dispatch_t kind,
                  ompt_data_t instance) {
    if (enable_printf) {
        printf("[%25s] parallel_data = %p | task_data = %p | kind = %d | instance->ptr = %p\n",
               __FUNCTION__, parallel_data, task_data, kind, instance.ptr);
    }
}

template<bool enable_printf>
void
callback_error(ompt_severity_t severity,
               const char *message, size_t length,
               const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] severity = %d | message = %s | length = %lu | codeptr_ra = %p\n",
               __FUNCTION__, severity, message, length, codeptr_ra);
    }
}

/* Host side accelerator callbacks */

template<bool enable_printf>
void
callback_device_initialize(int device_num,
                           const char *type,
                           ompt_device_t *device,
                           ompt_function_lookup_t lookup,
                           const char *documentation) {
    if (enable_printf) {
        printf("[%25s] device_num = %d | type = %s | device = %p | lookup = %p | documentation = %s\n",
               __FUNCTION__, device_num, type, device, lookup, documentation);
    }
}

template<bool enable_printf>
void
callback_device_finalize(int device_num) {
    if (enable_printf) {
        printf("[%25s] device_num = %d\n", __FUNCTION__, device_num);
    }
}

template<bool enable_printf>
void
callback_device_load(int device_num,
                     const char *filename,
                     int64_t offset_in_file,
                     void *vma_in_file,
                     size_t bytes,
                     void *host_addr,
                     void *device_addr,
                     uint64_t module_id) {
    if (enable_printf) {
        printf("[%25s] device_num = %d | filename = %s | offset_in_file = %ld | vma_in_file = %p | bytes = %lu | host_addr = %p | device_addr = %p | module_id = %lu\n",
               __FUNCTION__, device_num, filename, offset_in_file, vma_in_file, bytes, host_addr, device_addr,
               module_id);
    }
}

template<bool enable_printf>
void
callback_device_unload(int device_num,
                       uint64_t module_id) {
    if (enable_printf) {
        printf("[%25s] device_num = %d | module_id = %lu\n", __FUNCTION__, device_num, module_id);
    }
}

template<bool enable_printf>
void
callback_target_map_emi(ompt_data_t *target_data,
                        unsigned int nitems,
                        void **host_addr,
                        void **device_addr,
                        size_t *bytes,
                        unsigned int *mapping_flags,
                        const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] target_data = %p | nitems = %u | host_addr = %p | device_addr = %p | bytes = %p | mapping_flags = %p | codeptr_ra = %p\n",
               __FUNCTION__, target_data, nitems, host_addr, device_addr, bytes, mapping_flags, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_target_emi(ompt_target_t kind,
                    ompt_scope_endpoint_t endpoint,
                    int device_num,
                    ompt_data_t *task_data,
                    ompt_data_t *target_task_data,
                    ompt_data_t *target_data,
                    const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] kind = %d | endpoint = %d | device_num = %d | task_data = %p | target_task_data = %p | target_data = %p | codeptr_ra = %p\n",
               __FUNCTION__, kind, endpoint, device_num, task_data, target_task_data, target_data, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_target_data_op_emi(ompt_scope_endpoint_t endpoint,
                            ompt_data_t *target_task_data,
                            ompt_data_t *target_data,
                            ompt_id_t *host_op_id,
                            ompt_target_data_op_t optype,
                            void *src_addr,
                            int src_device_num,
                            void *dest_addr,
                            int dest_device_num,
                            size_t bytes,
                            const void *codeptr_ra) {
    if (enable_printf) {
        printf("[%25s] endpoint = %d | target_task_data = %p | target_data = %p | host_op_id = %p | optype = %d | src_addr = %p | src_device_num = %d | dest_addr = %p | dest_device_num = %d | bytes = %lu | codeptr_ra = %p\n",
               __FUNCTION__, endpoint, target_task_data, target_data, host_op_id, optype, src_addr, src_device_num,
               dest_addr, dest_device_num, bytes, codeptr_ra);
    }
}

template<bool enable_printf>
void
callback_target_submit_emi(ompt_scope_endpoint_t endpoint,
                           ompt_data_t *target_data,
                           ompt_id_t *host_op_id,
                           unsigned int requested_num_teams) {
    if (enable_printf) {
        printf("[%25s] endpoint = %d | target_data = %p | host_op_id = %p | requested_num_teams = %u\n",
               __FUNCTION__, endpoint, target_data, host_op_id, requested_num_teams);
    }
}

/* Initialization / Finalization sequence */

template<bool enable_printf>
int
tool_initialize(ompt_function_lookup_t lookup,
                int initial_device_num,
                ompt_data_t *tool_data) {
    if (enable_printf) {
        printf("[%25s] lookup = %p | initial_device_num = %d | tool_data = %p\n",
               __FUNCTION__, lookup, initial_device_num, tool_data);
    }
    
    /* ompt_function_lookup_t allows us to look-up runtime functions and register
     * callbacks which will get invoked on OpenMP events by the runtime. */
    auto ompt_set_callback = (ompt_set_callback_t) lookup("ompt_set_callback");
    assert(ompt_set_callback && "Could not find ompt_set_callback");

    /* Register callbacks for the host side */
#define CALLBACK(name) { ompt_callback_ ## name, ( ompt_callback_t )&callback_ ## name<enable_printf>, #name }
    const std::initializer_list<registration_data_t> host_callbacks = {
            CALLBACK(thread_begin),
            CALLBACK(thread_end),
            CALLBACK(parallel_begin),
            CALLBACK(parallel_end),
            CALLBACK(task_create),
            CALLBACK(task_schedule),
            CALLBACK(implicit_task),
            CALLBACK(control_tool),
            CALLBACK(sync_region_wait),
            CALLBACK(mutex_released),
            CALLBACK(dependences),
            CALLBACK(task_dependence),
            CALLBACK(work),
            CALLBACK(masked),
            CALLBACK(sync_region),
            CALLBACK(lock_init),
            CALLBACK(lock_destroy),
            CALLBACK(mutex_acquire),
            CALLBACK(mutex_acquired),
            CALLBACK(nest_lock),
            CALLBACK(flush),
            CALLBACK(cancel),
            CALLBACK(reduction),
            CALLBACK(dispatch),
            CALLBACK(error)
    };

    for (const auto &cb: host_callbacks) {
        ompt_set_result_t result = ompt_set_callback(cb.event, cb.callback);
        if (enable_printf) {
            printf("[%25s] %18s = %s\n", __FUNCTION__, cb.name, set_result2string(result));
        }
    }

    /* Register callbacks for the host side accelerator */
    const std::initializer_list<registration_data_t> host_accel_callbacks = {
            CALLBACK(device_initialize),
            CALLBACK(device_finalize),
            CALLBACK(device_load),
            CALLBACK(device_unload),
            CALLBACK(target_map_emi),
            CALLBACK(target_emi),
            CALLBACK(target_data_op_emi),
            CALLBACK(target_submit_emi)
    };

    for (const auto &cb: host_accel_callbacks) {
        ompt_set_result_t result = ompt_set_callback(cb.event, cb.callback);
        if (enable_printf) {
            printf("[%25s] %18s = %s\n", __FUNCTION__, cb.name, set_result2string(result));
        }
    }
#undef CALLBACK

    return true; /* non-zero indicates success */
}

template<bool enable_printf>
void
tool_finalize(ompt_data_t *tool_data) {
    if (enable_printf) {
        printf("[%25s] tool_data = %p\n", __FUNCTION__, tool_data);
    }
}

extern "C" ompt_start_tool_result_t *
ompt_start_tool(unsigned int omp_version,
                const char *runtime_version) {
    bool enable_printf = false;
    /* Check state of OMPT_ENABLE_PRINTF environment variable.
     * If true / yes / 1, enable the tool. Otherwise, disable it. */
    const char *env_enable_printf_c = std::getenv("OMPT_ENABLE_PRINTF");
    std::string env_enable_printf = env_enable_printf_c ? env_enable_printf_c : "false";
    std::transform(env_enable_printf.begin(), env_enable_printf.end(), env_enable_printf.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (env_enable_printf == "yes" || env_enable_printf == "1" || env_enable_printf == "true") {
        enable_printf = true;
    }

    static ompt_start_tool_result_t tool;
    if (enable_printf) {
        printf("[%25s] omp_version = %d | runtime_version = %s\n", __FUNCTION__, omp_version, runtime_version);
        tool = {tool_initialize<true>,
                tool_finalize<true>,
                ompt_data_none};
    } else {
        tool = {tool_initialize<false>,
                tool_finalize<false>,
                ompt_data_none};
    }

    return &tool;
}
