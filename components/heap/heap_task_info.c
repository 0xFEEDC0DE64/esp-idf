/*
 * SPDX-FileCopyrightText: 2018-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <multi_heap.h>
#include "multi_heap_internal.h"
#include "heap_private.h"
#include "esp_heap_task_info.h"
#include "esp_heap_task_info_internal.h"
#include "heap_memory_layout.h"
#include "esp_log.h"

#ifdef CONFIG_HEAP_TASK_TRACKING

const static char *TAG = "heap_task_tracking";

static SemaphoreHandle_t s_task_tracking_mutex = NULL;
static StaticSemaphore_t s_task_tracking_mutex_buf;

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
typedef heap_caps_block_owner_t alloc_stats_t;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
/**
 * @brief Internal singly linked list used to gather information of the heap used
 * by a given task.
 */
typedef struct heap_stats {
    multi_heap_handle_t heap;
    const char *name;
    size_t size;
    uint32_t caps;
    size_t current_usage;
    size_t peak_usage;
    size_t alloc_count;
    SLIST_HEAD(alloc_stats_ll, heap_caps_block_owner) allocs_stats;
    SLIST_ENTRY(heap_stats) next_heap_stat;
} heap_stats_t;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
/**
 * @brief Internal singly linked list used to gather information of the heap used
 * by a given subtask.
 */
typedef struct subtask_stats {
    const char * name;
    size_t peak_usage;
    size_t current_usage;
    SLIST_ENTRY(subtask_stats) next_subtask_stat;
} subtask_stats_t;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

/** @brief Internal singly linked list used to gather information on all created
 * tasks since startup.
 */
typedef struct task_stats {
    TaskHandle_t handle;
#ifdef CONFIG_HEAP_TRACK_DELETED_TASKS
    char name[configMAX_TASK_NAME_LEN];
#endif // CONFIG_HEAP_TRACK_DELETED_TASKS
    bool is_alive;
    size_t overall_peak_usage;
    size_t overall_current_usage;
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    size_t subtask_count;
    SLIST_HEAD(subtask_stats_ll, subtask_stats) subtasks_stats;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    size_t heap_count;
    SLIST_HEAD(heap_stats_ll, heap_stats) heaps_stats;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    SLIST_ENTRY(task_stats) next_task_info;
} task_info_t;

static SLIST_HEAD(task_stats_ll, task_stats) task_stats = SLIST_HEAD_INITIALIZER(task_stats);

FORCE_INLINE_ATTR heap_t* find_biggest_heap(void)
{
    heap_t *heap = NULL;
    heap_t *biggest_heap = NULL;
    SLIST_FOREACH(heap, &registered_heaps, next) {
        /* In the case where we are currently looking for the biggest heap during startup,
         * before the scheduler stated, all the memory regions marked as startup stacks will
         * be NULL here. As such, they must be ignored. After boot up, this statement will
         * never be true. */
        if (heap->heap == NULL) {
            /* Continue the loop */
        } else if (biggest_heap == NULL) {
            biggest_heap = heap;
        } else if ((biggest_heap->end - biggest_heap->start) < (heap->end - heap->start)) {
            biggest_heap = heap;
        }
    }
    return biggest_heap;
}

static heap_caps_block_owner_t get_initial_block_owner(void) {
    return (heap_caps_block_owner_t) {
        .task_info = NULL, // initially this will always be NULL, updated later
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
        .subtask = heap_caps_get_user_subtask(),
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    };
}

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
static bool global_subtask_initialized = false;
static __thread const char * current_thread_subtask = NULL;
extern const char * heap_caps_set_user_subtask(const char * name) {
    const char * old_subtask = current_thread_subtask;
    current_thread_subtask = name;
    global_subtask_initialized = true;
    return old_subtask;
}

extern const char * heap_caps_get_user_subtask(void) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && global_subtask_initialized) {
        return current_thread_subtask;
    } else {
        return NULL;
    }
}

extern void heap_caps_cleanup_user_subtask(const char ** p_subtask) {
    const char * subtask = *p_subtask;
    if (subtask) {
        *p_subtask = NULL;
        heap_caps_set_user_subtask(subtask);
    }
}
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

static void copy_task_name(char * buf, size_t buf_size, TaskHandle_t task_handle) {
    if (task_handle == NULL) {
        strncpy(buf, "Pre-scheduler", buf_size);
    } else {
        strncpy(buf, pcTaskGetName(task_handle), buf_size);
    }
}

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
/**
 * @brief Add allocation to linked list
 *
 * @param heap_stats The heap statistics of the heap used for the allocation
 * @param block_owner_ptr The address of the block owner ptr of the allocation
 */
static HEAP_IRAM_ATTR void link_alloc_entry(heap_stats_t *heap_stats, alloc_stats_t *block_owner_ptr)
{
    SLIST_INSERT_HEAD(&heap_stats->allocs_stats, block_owner_ptr, next_alloc);
}

/**
 * @brief Remove allocation from linked list
 *
 * @param heap_stats The heap statistics of the heap used for the allocation
 * @param block_owner_ptr The address of the block owner ptr of the allocation
 */
static HEAP_IRAM_ATTR void unlink_alloc_entry(heap_stats_t *heap_stats, alloc_stats_t *block_owner_ptr)
{
    SLIST_REMOVE(&heap_stats->allocs_stats, block_owner_ptr, heap_caps_block_owner, next_alloc);
}
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
/**
 * @brief Create a new heap stats entry object
 *
 * @param used_heap Information about the heap used for the allocation
 * @param task_stats The task statistics of the task that triggered the allocation
 * @param caps The caps of the heap used for the allocation
 */
static HEAP_IRAM_ATTR void create_new_heap_stats_entry(heap_t *used_heap, task_info_t *task_stats, alloc_stats_t *block_owner_ptr, size_t size, uint32_t caps)
{
    // find the heap with the most available free memory to store the statistics
    heap_t *heap_used_for_alloc = find_biggest_heap();

    // init the list of heap with a new entry in task_stats->heaps_stats. No need
    // to memset the memory since all field will be set later in the function.
    heap_stats_t *heap_stats = multi_heap_malloc(heap_used_for_alloc->heap, sizeof(heap_stats_t));
    if (!heap_stats) {
        ESP_LOGE(TAG, "Could not allocate memory to add new task statistics");
        return;
    }

    // create the alloc stats for the new heap entry
    SLIST_INIT(&heap_stats->allocs_stats);

    heap_stats->heap = used_heap->heap;
    heap_stats->name = used_heap->name;
    heap_stats->size = used_heap->end - used_heap->start;
    heap_stats->caps = caps;
    heap_stats->current_usage = size;
    heap_stats->peak_usage = size;
    heap_stats->alloc_count = 1;

    task_stats->heap_count += 1;
    SLIST_INSERT_HEAD(&task_stats->heaps_stats, heap_stats, next_heap_stat);

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
    link_alloc_entry(heap_stats, block_owner_ptr);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
}
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
/**
 * @brief Create a new subtask stats entry object
 *
 * @param subtask_name The subtask name
 * @param task_stats The task statistics of the task that triggered the allocation
 */
static HEAP_IRAM_ATTR void create_new_subtask_stats_entry(const char * subtask_name, task_info_t *task_stats, size_t size)
{
    // find the heap with the most available free memory to store the statistics
    heap_t *heap_used_for_alloc = find_biggest_heap();

    // init the list of subtasks with a new entry in task_stats->subtasks_stats. No need
    // to memset the memory since all fields will be set later in the function.
    subtask_stats_t *subtask_stats = multi_heap_malloc(heap_used_for_alloc->heap, sizeof(subtask_stats_t));
    if (!subtask_stats) {
        ESP_LOGE(TAG, "Could not allocate memory to add new task statistics");
        return;
    }

    subtask_stats->name = subtask_name;
    subtask_stats->current_usage = size;
    subtask_stats->peak_usage = size;

    task_stats->subtask_count += 1;
    SLIST_INSERT_HEAD(&task_stats->subtasks_stats, subtask_stats, next_subtask_stat);
}
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

/**
 * @brief Create a new task info entry in task_stats.
 *
 * @param task_handle The task handle of the task allocating memory
 */
static HEAP_IRAM_ATTR task_info_t * create_new_task_stats_entry(TaskHandle_t task_handle)
{
    // find the heap with the most available free memory to store the statistics
    heap_t *heap_used_for_alloc = find_biggest_heap();
    // create the task_stats entry. No need to memset since all fields are set later
    task_info_t * task_info = multi_heap_malloc(heap_used_for_alloc->heap, sizeof(task_info_t));
    if (!task_info) {
        ESP_LOGE(TAG, "Could not allocate memory to add new task statistics");
        return NULL;
    }

    task_info->handle = task_handle;
#ifdef CONFIG_HEAP_TRACK_DELETED_TASKS
    copy_task_name(task_info->name, sizeof task_info->name, task_handle);
#endif // CONFIG_HEAP_TRACK_DELETED_TASKS
    task_info->is_alive = true;
    task_info->overall_peak_usage = 0;
    task_info->overall_current_usage = 0;
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    task_info->subtask_count = 0;
    // create the subtask stats tailq for the new task entry
    SLIST_INIT(&task_info->subtasks_stats);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    task_info->heap_count = 0;
    // create the heap stats for the new task entry
    SLIST_INIT(&task_info->heaps_stats);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

    SLIST_INSERT_HEAD(&task_stats, task_info, next_task_info);

    return task_info;
}

/**
 * @brief Find task info entry for a running task by handle.
 *
 * @param task_handle The task handle of the searched for task
 */
static HEAP_IRAM_ATTR task_info_t * find_task_stats_entry_for_handle(TaskHandle_t task_handle)
{
    // try to find the current block's task stats entry by handle
    // the task must be alive
    task_info_t * task_info = NULL;
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        if (task_info->handle == task_handle && task_info->is_alive) {
            return task_info;
        }
    }

    return NULL;
}

/**
 * @brief Find or create a new task info entry in task_stats.
 *
 * @param block_owner_ptr The block owner ptr for the currently allocated block
 * @param task_handle The task handle of the task allocating memory
 */
static HEAP_IRAM_ATTR task_info_t * find_or_create_new_task_stats_entry(void * block_owner_ptr, TaskHandle_t task_handle)
{
    task_info_t * task_info = find_task_stats_entry_for_handle(task_handle);

    // if not found, create a new task stats entry
    if (task_info == NULL) {
        task_info = create_new_task_stats_entry(task_handle);
    }

    // finally, update the block owner info
    // if we failed to allocate a task stats entry, it's okay to initialize the task_info with NULL
    heap_caps_block_owner_t block_owner = get_initial_block_owner();
    block_owner.task_info = (void *)task_info;
    MULTI_HEAP_GET_BLOCK_OWNER(block_owner_ptr) = block_owner;

    return task_info;
}

/**
 * @brief Find task info entry for an already allocated block.
 *
 * @param block_owner The heap block owner information for the current block
 */
static HEAP_IRAM_ATTR task_info_t * find_task_stats_entry_for_block(heap_caps_block_owner_t block_owner)
{
#if CONFIG_HEAP_TRACK_DELETED_TASKS && CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
    // if we track deleted tasks, then the task_info pointer of the block
    // is guaranteed to be still valid
    return (task_info_t *)block_owner.task_info;
#else // !(CONFIG_HEAP_TRACK_DELETED_TASKS && CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS)
    // if we don't track deleted_tasks, then we could have deleted this task's
    // task_info pointer already
    //
    // try to find the current block's task stats entry directly by pointer
    // via the task_info pointer stored in the block owner
    task_info_t * task_info = NULL;
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        if ((void *)task_info == block_owner.task_info) {
            return task_info;
        }
    }

    return NULL;
#endif // !(CONFIG_HEAP_TRACK_DELETED_TASKS && CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS)
}

#if !CONFIG_HEAP_TRACK_DELETED_TASKS || !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
/**
 * @brief Delete an entry from the list of task statistics
 *
 * @param task_info The task statistics to delete from the list of task statistics
 */
static HEAP_IRAM_ATTR void delete_task_info_entry(task_info_t *task_info)
{
    if (task_info == NULL) {
        return;
    }

    // pointer used to free the memory of the statistics
    heap_t *containing_heap = NULL;

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    heap_stats_t *current_heap_stat = SLIST_FIRST(&task_info->heaps_stats);
    heap_stats_t *prev_heap_stat = NULL;

    // remove all entries from task_info->heaps_stats and free the memory
    while(current_heap_stat != NULL) {
        prev_heap_stat = current_heap_stat;
        current_heap_stat = SLIST_NEXT(current_heap_stat, next_heap_stat);

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
        /* remove all entries from heap_stats->allocs_stats */
        alloc_stats_t *alloc_stat = NULL;
        while ((alloc_stat = SLIST_FIRST( &prev_heap_stat->allocs_stats)) != NULL) {
            current_heap_stat->alloc_count--;
            unlink_alloc_entry(prev_heap_stat, alloc_stat);
            alloc_stat->task_info = NULL; // avoid task linking to a dangling pointer
        }
        if (!SLIST_EMPTY(&prev_heap_stat->allocs_stats)) continue;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

        task_info->heap_count--;
        SLIST_REMOVE(&task_info->heaps_stats, prev_heap_stat, heap_stats, next_heap_stat);
        containing_heap = find_containing_heap(prev_heap_stat);
        // prev_heap_stat must be allocated somewhere
        if (containing_heap != NULL) {
            multi_heap_free(containing_heap->heap, prev_heap_stat);
        }
    }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    subtask_stats_t *current_subtask_stat = SLIST_FIRST(&task_info->subtasks_stats);
    subtask_stats_t *prev_subtask_stat = NULL;

    // remove all entries from task_info->subtasks_stats and free the memory
    while(current_subtask_stat != NULL) {
        prev_subtask_stat = current_subtask_stat;
        current_subtask_stat = SLIST_NEXT(current_subtask_stat, next_subtask_stat);

        task_info->subtask_count--;
        SLIST_REMOVE(&task_info->subtasks_stats, prev_subtask_stat, subtask_stats, next_subtask_stat);
        containing_heap = find_containing_heap(prev_subtask_stat);
        // prev_subtask_stat must be allocated somewhere
        if (containing_heap != NULL) {
            multi_heap_free(containing_heap->heap, prev_subtask_stat);
        }
    }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    if (!SLIST_EMPTY(&task_info->heaps_stats)) return;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    if (!SLIST_EMPTY(&task_info->subtasks_stats)) return;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

    // remove task_info from task_stats (and free the memory)
    SLIST_REMOVE(&task_stats, task_info, task_stats, next_task_info);
    containing_heap = find_containing_heap(task_info);
    if (containing_heap != NULL) {
        multi_heap_free(containing_heap->heap, task_info);
    }
}
#endif // !CONFIG_HEAP_TRACK_DELETED_TASKS || !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS

HEAP_IRAM_ATTR void heap_caps_update_per_task_info_alloc(heap_t *heap, void *ptr, size_t size, uint32_t caps)
{
    if (s_task_tracking_mutex == NULL) {
        s_task_tracking_mutex = xSemaphoreCreateMutexStatic(&s_task_tracking_mutex_buf);
        assert(s_task_tracking_mutex);
    }

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);

    void *block_owner_ptr = MULTI_HEAP_REMOVE_BLOCK_OWNER_OFFSET(ptr);
    task_info_t * task_info = find_or_create_new_task_stats_entry(block_owner_ptr, xTaskGetCurrentTaskHandle());
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    heap_caps_block_owner_t block_owner = MULTI_HEAP_GET_BLOCK_OWNER(block_owner_ptr);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

    if (task_info != NULL) {
        task_info->overall_current_usage += size;
        if (task_info->overall_current_usage > task_info->overall_peak_usage) {
            task_info->overall_peak_usage = task_info->overall_current_usage;
        }

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
        bool heap_found = false;
        heap_stats_t *heap_stats = NULL;
        /* find the heap in the list and update the overall stats */
        SLIST_FOREACH(heap_stats, &task_info->heaps_stats, next_heap_stat) {
            if (heap_stats->heap == heap->heap) {
                heap_found = true;

                heap_stats->current_usage += size;
                heap_stats->alloc_count++;
                if (heap_stats->current_usage > heap_stats->peak_usage) {
                    heap_stats->peak_usage = heap_stats->current_usage;
                }

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
                /* add the alloc info to the list */
                link_alloc_entry(heap_stats, block_owner_ptr);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

                break;
            }
        }

        if (!heap_found) {
            create_new_heap_stats_entry(heap, task_info, block_owner_ptr, size, caps);
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
        bool subtask_found = false;
        subtask_stats_t *subtask_stats = NULL;
        /* find the subtask in the list and update the overall stats */
        SLIST_FOREACH(subtask_stats, &task_info->subtasks_stats, next_subtask_stat) {
            if (subtask_stats->name == block_owner.subtask) {
                subtask_stats->current_usage += size;
                if (subtask_stats->current_usage > subtask_stats->peak_usage) {
                    subtask_stats->peak_usage = subtask_stats->current_usage;
                }

                subtask_found = true;
                break;
            }
        }

        if (!subtask_found) {
            create_new_subtask_stats_entry(block_owner.subtask, task_info, size);
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    }

    xSemaphoreGive(s_task_tracking_mutex);
}

HEAP_IRAM_ATTR void heap_caps_update_per_task_info_realloc(heap_t *heap, void *old_ptr, void *new_ptr,
                                                           size_t old_size, heap_caps_block_owner_t old_block_owner,
                                                           size_t new_size, uint32_t caps)
{
    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
    void *old_block_owner_ptr = MULTI_HEAP_REMOVE_BLOCK_OWNER_OFFSET(old_ptr);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
    task_info_t * old_task_info = find_task_stats_entry_for_block(old_block_owner);
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
    alloc_stats_t *alloc_stat = NULL;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

    if (old_task_info != NULL) {
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
        bool allocation_found = false;
#else // !defined(CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION)
        bool allocation_found = true;
#endif // !defined(CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION)

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
        heap_stats_t * heap_stats = NULL;
        SLIST_FOREACH(heap_stats, &old_task_info->heaps_stats, next_heap_stat) {
            if (heap_stats->heap == heap->heap) {
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
                /* remove the alloc from the list. The updated alloc stats are added later
                 * in the function */
                SLIST_FOREACH(alloc_stat, &heap_stats->allocs_stats, next_alloc) {
                    if (alloc_stat == old_block_owner_ptr) {
                        unlink_alloc_entry(heap_stats, alloc_stat);
                        allocation_found = true;
                        break;
                    }
                }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

                if (allocation_found) {
                    heap_stats->current_usage -= old_size;
                    heap_stats->alloc_count--;
                }

                break;
            }
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
        subtask_stats_t *subtask_stats = NULL;
        SLIST_FOREACH(subtask_stats, &old_task_info->subtasks_stats, next_subtask_stat) {
            if (subtask_stats->name == old_block_owner.subtask) {
                if (allocation_found) {
                    subtask_stats->current_usage -= old_size;
                }

                break;
            }
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

        if (allocation_found) {
            old_task_info->overall_current_usage -= old_size;
        }

#if CONFIG_HEAP_TRACK_DELETED_TASKS && !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
        if (!old_task_info->is_alive && old_task_info->overall_current_usage == 0) {
            delete_task_info_entry(old_task_info);
        }
#endif // CONFIG_HEAP_TRACK_DELETED_TASKS && !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
    }

    void *new_block_owner_ptr = MULTI_HEAP_REMOVE_BLOCK_OWNER_OFFSET(new_ptr);
    task_info_t * new_task_info = find_or_create_new_task_stats_entry(new_block_owner_ptr, xTaskGetCurrentTaskHandle());
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    heap_caps_block_owner_t new_block_owner = MULTI_HEAP_GET_BLOCK_OWNER(new_block_owner_ptr);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

    if (new_task_info != NULL) {
        new_task_info->overall_current_usage += new_size;
        if (new_task_info->overall_current_usage > new_task_info->overall_peak_usage) {
            new_task_info->overall_peak_usage = new_task_info->overall_current_usage;
        }

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
        bool heap_found = false;
        heap_stats_t * heap_stats = NULL;
        SLIST_FOREACH(heap_stats, &new_task_info->heaps_stats, next_heap_stat) {
            if (heap_stats->heap == heap->heap) {
                heap_found = true;

                heap_stats->current_usage  += new_size;
                heap_stats->alloc_count++;
                if (heap_stats->current_usage > heap_stats->peak_usage) {
                    heap_stats->peak_usage = heap_stats->current_usage;
                }

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
                link_alloc_entry(heap_stats, alloc_stat);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

                break;
            }
        }

        if (!heap_found) {
            create_new_heap_stats_entry(heap, new_task_info, new_block_owner_ptr, new_size, caps);
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
        bool subtask_found = false;
        subtask_stats_t *subtask_stats = NULL;
        /* find the subtask in the list and update the overall stats */
        SLIST_FOREACH(subtask_stats, &new_task_info->subtasks_stats, next_subtask_stat) {
            if (subtask_stats->name == new_block_owner.subtask) {
                subtask_stats->current_usage += new_size;
                if (subtask_stats->current_usage > subtask_stats->peak_usage) {
                    subtask_stats->peak_usage = subtask_stats->current_usage;
                }

                subtask_found = true;
                break;
            }
        }

        if (!subtask_found) {
            create_new_subtask_stats_entry(new_block_owner.subtask, new_task_info, new_size);
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

#if CONFIG_HEAP_TRACK_DELETED_TASKS && !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
        if (!old_task_info->is_alive && old_task_info->overall_current_usage == 0) {
            delete_task_info_entry(old_task_info);
        }
#endif // CONFIG_HEAP_TRACK_DELETED_TASKS && !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
    }

    xSemaphoreGive(s_task_tracking_mutex);
}

HEAP_IRAM_ATTR void heap_caps_update_per_task_info_free(heap_t *heap, void *ptr)
{
    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);

    // when a task is deleted, esp_caps_free is called to delete the TCB of the task from vTaskDelete.
    // Try to make a TaskHandle out of ptr and compare it to the list of tasks in task_stats.
    // If one task_info contains the newly made TaskHandle from ptr it means that esp_caps_free
    // was indeed called from vTaskDelete. We can then update the task_stats by marking the corresponding
    // task as deleted.
    task_info_t * dying_task_info = find_task_stats_entry_for_handle((TaskHandle_t)ptr);
    if (dying_task_info != NULL) {
        dying_task_info->is_alive = false;
#if !CONFIG_HEAP_TRACK_DELETED_TASKS
        delete_task_info_entry(dying_task_info);
#elif CONFIG_HEAP_TRACK_DELETED_TASKS && !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
        if (dying_task_info->overall_current_usage == 0) {
            delete_task_info_entry(dying_task_info);
        }
#endif // CONFIG_HEAP_TRACK_DELETED_TASKS && !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
    }

    void *block_owner_ptr = MULTI_HEAP_REMOVE_BLOCK_OWNER_OFFSET(ptr);
    heap_caps_block_owner_t block_owner = MULTI_HEAP_GET_BLOCK_OWNER(block_owner_ptr);
    size_t size = multi_heap_get_full_block_size(heap->heap, block_owner_ptr);
    task_info_t * task_info = find_task_stats_entry_for_block(block_owner);

    if (task_info != NULL) {
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
        bool allocation_found = false;
#else // !defined(CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION)
        bool allocation_found = true;
#endif // !defined(CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION)

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
        heap_stats_t * heap_stats = NULL;
        SLIST_FOREACH(heap_stats, &task_info->heaps_stats, next_heap_stat) {
            if (heap_stats->heap == heap->heap) {
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
                /* remove the alloc from the list */
                alloc_stats_t *alloc_stat = NULL;
                SLIST_FOREACH(alloc_stat, &heap_stats->allocs_stats, next_alloc) {
                    if (alloc_stat == block_owner_ptr) {
                        unlink_alloc_entry(heap_stats, alloc_stat);
                        allocation_found = true;
                        break;
                    }
                }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

                if (allocation_found) {
                    heap_stats->current_usage -= size;
                    heap_stats->alloc_count--;
                }

                break;
            }
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
        subtask_stats_t *subtask_stats = NULL;
        SLIST_FOREACH(subtask_stats, &task_info->subtasks_stats, next_subtask_stat) {
            if (subtask_stats->name == block_owner.subtask) {
                if (allocation_found) {
                    subtask_stats->current_usage -= size;
                }

                break;
            }
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

        if (allocation_found) {
            task_info->overall_current_usage -= size;
        }

#if CONFIG_HEAP_TRACK_DELETED_TASKS && !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
        if (!task_info->is_alive && task_info->overall_current_usage == 0) {
            delete_task_info_entry(task_info);
        }
#endif // CONFIG_HEAP_TRACK_DELETED_TASKS && !CONFIG_HEAP_TRACK_DELETED_TASKS_WITHOUT_ALLOCATIONS
    }

    xSemaphoreGive(s_task_tracking_mutex);
}

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
static heap_task_block_t capture_alloc_stats(task_info_t * task_info, heap_stats_t * heap_info, alloc_stats_t * alloc_stats) {
    return (heap_task_block_t) {
        .task = task_info->handle,
        .address = MULTI_HEAP_ADD_BLOCK_OWNER_OFFSET(alloc_stats),
        .size = multi_heap_get_full_block_size(heap_info->heap, alloc_stats),
    };
}
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
static heap_stat_t capture_heap_stats(heap_stats_t * heap_info) {
    return (heap_stat_t) {
        .name = heap_info->name,
        .caps = heap_info->caps,
        .size = heap_info->size,
        .current_usage = heap_info->current_usage,
        .peak_usage = heap_info->peak_usage,
        .alloc_count = heap_info->alloc_count,
        .alloc_stat = NULL,
    };
}
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
static subtask_stat_t capture_subtask_stats(subtask_stats_t * subtask_info) {
    return (subtask_stat_t) {
        .name = subtask_info->name,
        .peak_usage = subtask_info->peak_usage,
        .current_usage = subtask_info->current_usage,
    };
}
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

static task_stat_t capture_task_stats(task_info_t * task_info) {
    task_stat_t stat = {
        .handle = task_info->handle,
        .is_alive = task_info->is_alive,
        .overall_peak_usage = task_info->overall_peak_usage,
        .overall_current_usage = task_info->overall_current_usage,
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
        .subtask_count = task_info->subtask_count,
        .subtask_stat = NULL,
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
        .heap_count = task_info->heap_count,
        .heap_stat = NULL,
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    };

#ifdef CONFIG_HEAP_TRACK_DELETED_TASKS
    memcpy(stat.name, task_info->name, sizeof stat.name);
    _Static_assert(sizeof stat.name == sizeof task_info->name, "task name buffer sizes must match");
#else // !CONFIG_HEAP_TRACK_DELETED_TASKS
    copy_task_name(task_info->name, sizeof task_info->name, task_handle);
#endif // !CONFIG_HEAP_TRACK_DELETED_TASKS

    return stat;
}

esp_err_t heap_caps_get_all_task_stat(heap_all_tasks_stat_t *tasks_stat)
{
    if (tasks_stat == NULL ||
        (tasks_stat->stat_arr == NULL && tasks_stat->task_count != 0) ||
        (tasks_stat->heap_stat_start == NULL && tasks_stat->heap_count != 0) ||
        (tasks_stat->alloc_stat_start == NULL && tasks_stat->alloc_count != 0)) {
        return ESP_ERR_INVALID_ARG;
    }

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    if (tasks_stat->subtask_stat_start == NULL && tasks_stat->subtask_count != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t subtask_index = 0;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

    size_t task_index = 0;
    size_t heap_index = 0;
    size_t alloc_index = 0;
    task_info_t *task_info = NULL;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        // If there is no more task stat entries available in tasks_stat->stat_arr
        // break the loop and return the function.
        if (task_index >= tasks_stat->task_count) {
            break;
        }

        task_stat_t *current_task_stat = &tasks_stat->stat_arr[task_index++];
        *current_task_stat = capture_task_stats(task_info);

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
        // If no more heap stat entries in the array are available, just proceed
        // with filling task stats but skip filling info on heap stat and alloc stat.
        if (heap_index + task_info->heap_count > tasks_stat->heap_count) {
            current_task_stat->heap_stat = NULL;
        } else {
            // set the pointer where the heap info for the given task will
            // be in the user array
            current_task_stat->heap_stat = tasks_stat->heap_stat_start + heap_index;
            heap_index += task_info->heap_count;

            // copy the stats of the different heaps the task has used and the different allocs
            // allocated in those heaps. If the number of entries remaining for alloc stats is
            // inferior to the number of allocs allocated on the current heap no alloc stat will
            // be copied at all.
            size_t h_index = 0;
            heap_stats_t *heap_info = SLIST_FIRST(&task_info->heaps_stats);
            while(h_index < task_info->heap_count && heap_info != NULL) {

                heap_stat_t *current_heap_stat = &current_task_stat->heap_stat[h_index++];
                *current_heap_stat = capture_heap_stats(heap_info);

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
                // increase alloc_index before filling the alloc info of the given heap
                // to avoid running out of alloc stat entry while doing it.
                if (alloc_index + heap_info->alloc_count > tasks_stat->alloc_count) {
                    current_heap_stat->alloc_stat = NULL;
                } else {
                    // set the pointer where the alloc info for the given heap will
                    // be in the user array
                    current_heap_stat->alloc_stat = tasks_stat->alloc_stat_start + alloc_index;
                    // fill the alloc array in heap_info by running through all blocks of a given heap
                    // and storing info about the blocks allocated by the given task
                    alloc_stats_t *alloc_stats = NULL;
                    size_t a_index = 0;
                    SLIST_FOREACH(alloc_stats, &heap_info->allocs_stats, next_alloc) {
                        current_heap_stat->alloc_stat[a_index++] = capture_alloc_stats(task_info, heap_info, alloc_stats);
                    }

                    alloc_index += heap_info->alloc_count;
                }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

                heap_info = SLIST_NEXT(heap_info, next_heap_stat);
            }
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
        // If no more subtask stat entries in the array are available, just proceed
        // with filling task stats but skip filling info on subtask stat and alloc stat.
        if (subtask_index + task_info->subtask_count > tasks_stat->subtask_count) {
            current_task_stat->subtask_stat = NULL;
        } else if (task_info->subtask_count == 1 && SLIST_FIRST(&task_info->subtasks_stats)->name == NULL) {
            // skip if only the trivial (null) subtask exists to avoid wasting subtask_stat entries
            current_task_stat->subtask_count = 0;
            current_task_stat->subtask_stat = NULL;
        } else {
            // set the pointer where the subtask info for the given task will
            // be in the user array
            current_task_stat->subtask_stat = tasks_stat->subtask_stat_start + subtask_index;
            subtask_index += task_info->subtask_count;

            // copy the stats of the different subtasks the task has used.
            size_t s_index = 0;
            subtask_stats_t *subtask_info = SLIST_FIRST(&task_info->subtasks_stats);
            while(s_index < task_info->subtask_count && subtask_info != NULL) {
                current_task_stat->subtask_stat[s_index++] = capture_subtask_stats(subtask_info);
                subtask_info = SLIST_NEXT(subtask_info, next_subtask_stat);
            }
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    }

    xSemaphoreGive(s_task_tracking_mutex);

    tasks_stat->task_count = task_index;
    tasks_stat->heap_count = heap_index;
    tasks_stat->alloc_count = alloc_index;

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    tasks_stat->subtask_count = subtask_index;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

    return ESP_OK;
}

esp_err_t heap_caps_get_single_task_stat(heap_single_task_stat_t *task_stat, TaskHandle_t task_handle)
{
    if (task_stat == NULL ||
        (task_stat->heap_stat_start == NULL && task_stat->heap_count != 0) ||
        (task_stat->alloc_stat_start == NULL && task_stat->alloc_count != 0)) {
        return ESP_ERR_INVALID_ARG;
    }

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    if (task_stat->subtask_stat_start == NULL && task_stat->subtask_count != 0) {
        return ESP_ERR_INVALID_ARG;
    }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

    if (task_handle == NULL) {
        task_handle = xTaskGetCurrentTaskHandle();
    }

    task_info_t *task_info = NULL;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        if(task_info->handle == task_handle) {
            // copy the task_stat of the task itself
            task_stat->stat = capture_task_stats(task_info);
            break;
        }
    }
    xSemaphoreGive(s_task_tracking_mutex);

    if (task_info == NULL) {
        return ESP_FAIL;
    }

    size_t heap_index = 0;
    size_t alloc_index = 0;

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    task_stat->stat.heap_stat = task_stat->heap_stat_start;

    // copy the stats of the different heaps the task has used and the different blocks
    // allocated in those heaps. If the number of entries remaining for block stats is
    // inferior to the number of blocks allocated on the current heap no block stat will
    // be copied at all.

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    heap_stats_t *heap_info = SLIST_FIRST(&task_info->heaps_stats);
    while(heap_index < task_info->heap_count && heap_info != NULL) {
        // check that there is enough heap_stat entry left to add another one to the user defined
        // array of heap_stat
        if (heap_index >= task_stat->heap_count) {
            break;
        }

        heap_stat_t *current_heap_stat = &task_stat->stat.heap_stat[heap_index++];
        *current_heap_stat = capture_heap_stats(heap_info);

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
        // increase alloc_index before filling the block info of the given heap
        // to avoid running out of block stat entry while doing it.
        if (alloc_index + heap_info->alloc_count > task_stat->alloc_count) {
            current_heap_stat->alloc_stat = NULL;
        } else {
            // set the pointer where the block info for the given heap will
            // be in the user array
            current_heap_stat->alloc_stat = task_stat->alloc_stat_start + alloc_index;

            // fill the alloc array in heap_info by running through all blocks of a given heap
            // and storing info about the blocks allocated by the given task
            alloc_stats_t *alloc_stats = NULL;
            size_t a_index = 0;
            SLIST_FOREACH(alloc_stats, &heap_info->allocs_stats, next_alloc) {
                current_heap_stat->alloc_stat[a_index++] = capture_alloc_stats(task_info, heap_info, alloc_stats);
            }

            alloc_index += heap_info->alloc_count;
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION

        heap_info = SLIST_NEXT(heap_info, next_heap_stat);
    }
    xSemaphoreGive(s_task_tracking_mutex);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    task_stat->stat.subtask_stat = task_stat->subtask_stat_start;

    // copy the stats of the different heaps the task has used and the different blocks
    // allocated in those heaps. If the number of entries remaining for block stats is
    // inferior to the number of blocks allocated on the current heap no block stat will
    // be copied at all.

    size_t subtask_index = 0;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    subtask_stats_t *subtask_info = SLIST_FIRST(&task_info->subtasks_stats);
    if (task_info->subtask_count == 1 && subtask_info->name == NULL) {
        // skip if only the trivial (null) subtask exists to avoid wasting subtask_stat entries
        task_stat->stat.subtask_count = 0;
        task_stat->stat.subtask_stat = NULL;
    } else {
        // otherwise copy subtask stats
        while (subtask_index < task_info->subtask_count && subtask_info != NULL) {
            // check that there is enough subtask_stat entry left to add another one to the user defined
            // array of subtask_stat
            if (subtask_index >= task_stat->subtask_count) {
                break;
            }

            task_stat->stat.subtask_stat[subtask_index++] = capture_subtask_stats(subtask_info);
            subtask_info = SLIST_NEXT(subtask_info, next_subtask_stat);
        }
    }
    xSemaphoreGive(s_task_tracking_mutex);

    task_stat->subtask_count = subtask_index;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK

    task_stat->heap_count = heap_index;
    task_stat->alloc_count = alloc_index;

    return ESP_OK;
}

static void heap_caps_print_task_info(FILE *stream, task_info_t *task_info, bool is_last_task_info)
{
    if (stream == NULL) {
        stream = stdout;
    }

    const char *task_info_visual = is_last_task_info ? " " : "│";
    const char *task_info_visual_start = is_last_task_info ? "└" : "├";
    task_stat_t task_stat = capture_task_stats(task_info);
    fprintf(stream, "%s %s: %s, CURRENT MEMORY USAGE %d, PEAK MEMORY USAGE %d, TOTAL HEAP USED %d:\n", task_info_visual_start,
                                                                                                      task_stat.is_alive ? "ALIVE" : "DELETED",
                                                                                                      task_stat.name,
                                                                                                      task_stat.overall_current_usage,
                                                                                                      task_stat.overall_peak_usage,
                                                                                                      task_stat.heap_count);

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    heap_stats_t *heap_info = NULL;
    SLIST_FOREACH(heap_info, &task_info->heaps_stats, next_heap_stat) {
        const char *next_heap_visual = !SLIST_NEXT(heap_info, next_heap_stat) ? " " : "│";
        const char *next_heap_visual_start = !SLIST_NEXT(heap_info, next_heap_stat) ? "└" : "├";
        heap_stat_t heap_stat = capture_heap_stats(heap_info);
        fprintf(stream, "%s    %s HEAP: %s, CAPS: 0x%08lx, SIZE: %d, USAGE: CURRENT %d (%d%%), PEAK %d (%d%%), ALLOC COUNT: %d\n",
                task_info_visual,
                next_heap_visual_start,
                heap_stat.name,
                heap_stat.caps,
                heap_stat.size,
                heap_stat.current_usage,
                (heap_stat.current_usage * 100) / heap_stat.size,
                heap_stat.peak_usage,
                (heap_stat.peak_usage * 100) / heap_stat.size,
                heap_stat.alloc_count);

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
        alloc_stats_t *alloc_stats = NULL;
        SLIST_FOREACH(alloc_stats, &heap_info->allocs_stats, next_alloc) {
            heap_task_block_t alloc = capture_alloc_stats(task_info, heap_info, alloc_stats);
            fprintf(stream, "%s    %s    ├ ALLOC %p, SIZE %" PRIu32 "\n", task_info_visual,
                                                                next_heap_visual,
                                                                alloc.address,
                                                                alloc.size);
        }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
    }
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP
}

static void heap_caps_print_task_overview(FILE *stream, task_info_t *task_info, bool is_first_task_info, bool is_last_task_info)
{
    if (stream == NULL) {
        stream = stdout;
    }

    if (is_first_task_info) {
        fprintf(stream, "┌────────────────────┬─────────┬──────────────────────┬───────────────────┬─────────────────┐\n");
        fprintf(stream, "│ TASK               │ STATUS  │ CURRENT MEMORY USAGE │ PEAK MEMORY USAGE │ TOTAL HEAP USED │\n");
        fprintf(stream, "├────────────────────┼─────────┼──────────────────────┼───────────────────┼─────────────────┤\n");
    }

    task_stat_t task_stat = capture_task_stats(task_info);
    fprintf(stream, "│ %18s │ %7s │ %20d │ %17d │ %15d │\n",
                    task_stat.name,
                    task_stat.is_alive ? "ALIVE  " : "DELETED",
                    task_stat.overall_current_usage,
                    task_stat.overall_peak_usage,
                    task_stat.heap_count);

    if (is_last_task_info) {
        fprintf(stream, "└────────────────────┴─────────┴──────────────────────┴───────────────────┴─────────────────┘\n");
    }
}

void heap_caps_print_single_task_stat(FILE *stream, TaskHandle_t task_handle)
{
    if (task_handle == NULL) {
        task_handle = xTaskGetCurrentTaskHandle();
    }

    task_info_t *task_info = NULL;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        if (task_info->handle == task_handle) {
            heap_caps_print_task_info(stream, task_info, true);

            xSemaphoreGive(s_task_tracking_mutex);
            return;
        }
    }
    xSemaphoreGive(s_task_tracking_mutex);
}

void heap_caps_print_all_task_stat(FILE *stream)
{
    task_info_t *task_info = NULL;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        const bool last_task_info = (SLIST_NEXT(task_info, next_task_info) == NULL);
        heap_caps_print_task_info(stream, task_info, last_task_info);
    }
    xSemaphoreGive(s_task_tracking_mutex);
}

void heap_caps_print_single_task_stat_overview(FILE *stream, TaskHandle_t task_handle)
{
    if (task_handle == NULL) {
        task_handle = xTaskGetCurrentTaskHandle();
    }

    task_info_t *task_info = NULL;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        if (task_info->handle == task_handle) {
            heap_caps_print_task_overview(stream, task_info, true, true);

            xSemaphoreGive(s_task_tracking_mutex);
            return;
        }
    }
    xSemaphoreGive(s_task_tracking_mutex);
}

void heap_caps_print_all_task_stat_overview(FILE *stream)
{
    task_info_t *task_info = NULL;
    bool is_first_task_info = true;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        const bool last_task_info = (SLIST_NEXT(task_info, next_task_info) == NULL);
        heap_caps_print_task_overview(stream, task_info, is_first_task_info, last_task_info);
        is_first_task_info = false;
    }
    xSemaphoreGive(s_task_tracking_mutex);
}

esp_err_t heap_caps_alloc_single_task_stat_arrays(heap_single_task_stat_t *task_stat, TaskHandle_t task_handle)
{
    if (task_handle == NULL) {
        task_handle = xTaskGetCurrentTaskHandle();
    }

    task_stat->heap_stat_start = NULL;
    task_stat->alloc_stat_start = NULL;
    task_stat->heap_count = 0;
    task_stat->alloc_count = 0;

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    task_info_t *task_info = NULL;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        if(task_info->handle == task_handle && task_info->is_alive) {
            task_stat->heap_count = task_info->heap_count;
            heap_stats_t *heap_info = NULL;
            SLIST_FOREACH(heap_info, &task_info->heaps_stats, next_heap_stat) {
                task_stat->alloc_count += heap_info->alloc_count;
            }
            break;
        }
    }
    xSemaphoreGive(s_task_tracking_mutex);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

    // allocate the memory used to store the statistics of allocs, heaps
    if (task_stat->heap_count != 0) {
        heap_t *heap_used_for_alloc = find_biggest_heap();
        task_stat->heap_stat_start = multi_heap_malloc(heap_used_for_alloc->heap, task_stat->heap_count * sizeof(heap_stat_t));
        if (task_stat->heap_stat_start == NULL) {
            return ESP_FAIL;
        }
    }
    if (task_stat->alloc_count != 0) {
        heap_t *heap_used_for_alloc = find_biggest_heap();
        task_stat->alloc_stat_start = multi_heap_malloc(heap_used_for_alloc->heap, task_stat->alloc_count * sizeof(heap_task_block_t));
        if (task_stat->alloc_stat_start == NULL) {
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

void heap_caps_free_single_task_stat_arrays(heap_single_task_stat_t *task_stat)
{
    if (task_stat->heap_stat_start != NULL) {
        heap_t *heap_used_for_alloc = find_containing_heap(task_stat->heap_stat_start);
        assert(heap_used_for_alloc != NULL);
        multi_heap_free(heap_used_for_alloc->heap, task_stat->heap_stat_start);
        task_stat->heap_stat_start = NULL;
        task_stat->heap_count = 0;
    }
    if (task_stat->alloc_stat_start != NULL) {
        heap_t *heap_used_for_alloc = find_containing_heap(task_stat->alloc_stat_start);
        assert(heap_used_for_alloc != NULL);
        multi_heap_free(heap_used_for_alloc->heap, task_stat->alloc_stat_start);
        task_stat->alloc_stat_start = NULL;
        task_stat->alloc_count = 0;
    }
}

esp_err_t heap_caps_alloc_all_task_stat_arrays(heap_all_tasks_stat_t *tasks_stat)
{
    tasks_stat->stat_arr = NULL;
    tasks_stat->heap_stat_start = NULL;
    tasks_stat->alloc_stat_start = NULL;
    tasks_stat->task_count = 0;
    tasks_stat->heap_count = 0;
    tasks_stat->alloc_count = 0;

#ifdef CONFIG_HEAP_TASK_TRACKING_PER_HEAP
    task_info_t *task_info = NULL;

    xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
    SLIST_FOREACH(task_info, &task_stats, next_task_info) {
        tasks_stat->task_count += 1;

        tasks_stat->heap_count += task_info->heap_count;
        heap_stats_t *heap_info = NULL;
        SLIST_FOREACH(heap_info, &task_info->heaps_stats, next_heap_stat) {
            tasks_stat->alloc_count += heap_info->alloc_count;
        }
    }
    xSemaphoreGive(s_task_tracking_mutex);
#endif // CONFIG_HEAP_TASK_TRACKING_PER_HEAP

    // allocate the memory used to store the statistics of allocs, heaps and tasks
    if (tasks_stat->task_count != 0) {
        heap_t *heap_used_for_alloc = find_biggest_heap();
        tasks_stat->stat_arr = multi_heap_malloc(heap_used_for_alloc->heap, tasks_stat->task_count * sizeof(task_stat_t));
        if (tasks_stat->stat_arr == NULL) {
            return ESP_FAIL;
        }
    }
    if (tasks_stat->heap_count != 0) {
        heap_t *heap_used_for_alloc = find_biggest_heap();
        tasks_stat->heap_stat_start = multi_heap_malloc(heap_used_for_alloc->heap, tasks_stat->heap_count * sizeof(heap_stat_t));
        if (tasks_stat->heap_stat_start == NULL) {
            return ESP_FAIL;
        }
    }
    if (tasks_stat->alloc_count != 0) {
        heap_t *heap_used_for_alloc = find_biggest_heap();
        tasks_stat->alloc_stat_start = multi_heap_malloc(heap_used_for_alloc->heap, tasks_stat->alloc_count * sizeof(heap_task_block_t));
        if (tasks_stat->alloc_stat_start == NULL) {
            return ESP_FAIL;
        }
    }
    return ESP_OK;
}

void heap_caps_free_all_task_stat_arrays(heap_all_tasks_stat_t *tasks_stat)
{
    if (tasks_stat->stat_arr != NULL) {
        heap_t *heap_used_for_alloc = find_containing_heap(tasks_stat->stat_arr);
        assert(heap_used_for_alloc != NULL);
        multi_heap_free(heap_used_for_alloc->heap, tasks_stat->stat_arr);
        tasks_stat->stat_arr = NULL;
        tasks_stat->task_count = 0;
    }
    if (tasks_stat->heap_stat_start != NULL) {
        heap_t *heap_used_for_alloc = find_containing_heap(tasks_stat->heap_stat_start);
        assert(heap_used_for_alloc != NULL);
        multi_heap_free(heap_used_for_alloc->heap, tasks_stat->heap_stat_start);
        tasks_stat->heap_stat_start = NULL;
        tasks_stat->heap_count = 0;
    }
    if (tasks_stat->alloc_stat_start != NULL) {
        heap_t *heap_used_for_alloc = find_containing_heap(tasks_stat->alloc_stat_start);
        assert(heap_used_for_alloc != NULL);
        multi_heap_free(heap_used_for_alloc->heap, tasks_stat->alloc_stat_start);
        tasks_stat->alloc_stat_start = NULL;
        tasks_stat->alloc_count = 0;
    }
}

/*
 * Return per-task heap allocation totals and lists of blocks.
 *
 * For each task that has allocated memory from the heap, return totals for
 * allocations within regions matching one or more sets of capabilities.
 *
 * Optionally also return an array of structs providing details about each
 * block allocated by one or more requested tasks, or by all tasks.
 *
 * Returns the number of block detail structs returned.
 */
size_t heap_caps_get_per_task_info(heap_task_info_params_t *params)
{
    heap_t *reg;
    heap_task_block_t *blocks = params->blocks;
    size_t count = *params->num_totals;
    size_t remaining = params->max_blocks;

    // Clear out totals for any prepopulated tasks.
    if (params->totals) {
        for (size_t i = 0; i < count; ++i) {
            for (size_t type = 0; type < NUM_HEAP_TASK_CAPS; ++type) {
                params->totals[i].size[type] = 0;
                params->totals[i].count[type] = 0;
            }
        }
    }

    SLIST_FOREACH(reg, &registered_heaps, next) {
        multi_heap_handle_t heap = reg->heap;
        if (heap == NULL) {
            continue;
        }

        // Find if the capabilities of this heap region match on of the desired
        // sets of capabilities.
        uint32_t caps = get_all_caps(reg);
        uint32_t type;
        for (type = 0; type < NUM_HEAP_TASK_CAPS; ++type) {
            if ((caps & params->mask[type]) == params->caps[type]) {
                break;
            }
        }
        if (type == NUM_HEAP_TASK_CAPS) {
            continue;
        }

        multi_heap_block_handle_t b = multi_heap_get_first_block(heap);
        multi_heap_internal_lock(heap);
        for ( ; b ; b = multi_heap_get_next_block(heap, b)) {
            if (multi_heap_is_free(b)) {
                continue;
            }
            void *p = multi_heap_get_block_address(b);  // Safe, only arithmetic
            size_t bsize = multi_heap_get_allocated_size(heap, p); // Validates

            heap_caps_block_owner_t bowner = MULTI_HEAP_GET_BLOCK_OWNER(p);
            xSemaphoreTake(s_task_tracking_mutex, portMAX_DELAY);
            task_info_t * task_info = find_task_stats_entry_for_block(bowner);
            xSemaphoreGive(s_task_tracking_mutex);
            TaskHandle_t btask = task_info != NULL && task_info->is_alive ? task_info->handle : NULL;

            // Accumulate per-task allocation totals.
            if (params->totals) {
                size_t i;
                for (i = 0; i < count; ++i) {
                    if (params->totals[i].task == btask) {
                        break;
                    }
                }
                if (i < count) {
                    params->totals[i].size[type] += bsize;
                    params->totals[i].count[type] += 1;
                } else {
                    if (count < params->max_totals) {
                        params->totals[count].task = btask;
                        params->totals[count].size[type] = bsize;
                        params->totals[i].count[type] = 1;
                        ++count;
                    }
                }
            }

            // Return details about allocated blocks for selected tasks.
            if (blocks && remaining > 0) {
                if (params->tasks) {
                    size_t i;
                    for (i = 0; i < params->num_tasks; ++i) {
                        if (btask == params->tasks[i]) {
                            break;
                        }
                    }
                    if (i == params->num_tasks) {
                        continue;
                    }
                }
                blocks->task = btask;
                blocks->address = p;
                blocks->size = bsize;
                ++blocks;
                --remaining;
            }
        }
        multi_heap_internal_unlock(heap);
    }
    *params->num_totals = count;
    return params->max_blocks - remaining;
}

#endif // CONFIG_HEAP_TASK_TRACKING
