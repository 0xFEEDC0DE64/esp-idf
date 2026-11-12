/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef CONFIG_HEAP_TASK_TRACKING

#include "sys/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

// forward-declaration
typedef struct heap_t_ heap_t;

typedef struct heap_caps_block_owner {
    void * task_info;
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
    const char * subtask;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_USER_SUBTASK
#ifdef CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
    SLIST_ENTRY(heap_caps_block_owner) next_alloc;
#endif // CONFIG_HEAP_TASK_TRACKING_PER_ALLOCATION
} heap_caps_block_owner_t;

void heap_caps_update_per_task_info_alloc(heap_t *heap, void *ptr, size_t size, uint32_t caps);
void heap_caps_update_per_task_info_free(heap_t *heap, void *ptr);
void heap_caps_update_per_task_info_realloc(heap_t *heap, void *old_ptr, void *new_ptr, size_t old_size, heap_caps_block_owner_t old_block_owner, size_t new_size, uint32_t caps);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_HEAP_TASK_TRACKING
