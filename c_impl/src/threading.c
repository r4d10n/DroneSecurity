/**
 * @file threading.c
 * @brief Multi-threading support for dual-core Zynq 7000
 */

#include "threading.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef __linux__
#include <sched.h>
#endif

/* Work queue implementation */
int work_queue_init(work_queue_t* queue) {
    if (!queue) return DRONEID_ERROR_INVALID_ARG;

    memset(queue, 0, sizeof(work_queue_t));

    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;

    pthread_mutex_init(&queue->mutex, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);

    return DRONEID_SUCCESS;
}

void work_queue_destroy(work_queue_t* queue) {
    if (!queue) return;

    pthread_mutex_destroy(&queue->mutex);
    pthread_cond_destroy(&queue->not_empty);
    pthread_cond_destroy(&queue->not_full);
}

int work_queue_push(work_queue_t* queue, const work_item_t* item) {
    if (!queue || !item) return DRONEID_ERROR_INVALID_ARG;

    pthread_mutex_lock(&queue->mutex);

    /* Wait if queue is full */
    while (queue->count >= WORK_QUEUE_SIZE) {
        pthread_cond_wait(&queue->not_full, &queue->mutex);
    }

    /* Add item */
    queue->items[queue->tail] = *item;
    queue->tail = (queue->tail + 1) % WORK_QUEUE_SIZE;
    queue->count++;

    /* Signal that queue is not empty */
    pthread_cond_signal(&queue->not_empty);

    pthread_mutex_unlock(&queue->mutex);

    return DRONEID_SUCCESS;
}

int work_queue_pop(work_queue_t* queue, work_item_t* item) {
    if (!queue || !item) return DRONEID_ERROR_INVALID_ARG;

    pthread_mutex_lock(&queue->mutex);

    /* Wait if queue is empty */
    while (queue->count == 0) {
        pthread_cond_wait(&queue->not_empty, &queue->mutex);
    }

    /* Get item */
    *item = queue->items[queue->head];
    queue->head = (queue->head + 1) % WORK_QUEUE_SIZE;
    queue->count--;

    /* Signal that queue is not full */
    pthread_cond_signal(&queue->not_full);

    pthread_mutex_unlock(&queue->mutex);

    return DRONEID_SUCCESS;
}

/* Worker thread function */
static void* worker_thread(void* arg) {
    thread_pool_t* pool = (thread_pool_t*)arg;

    while (1) {
        work_item_t item;

        /* Get work from queue */
        int ret = work_queue_pop(&pool->queue, &item);
        if (ret != 0) continue;

        /* Check for shutdown */
        if (item.type == WORK_SHUTDOWN) {
            break;
        }

        /* Process work item */
        /* This is where specific work functions would be called */
        /* For now, just invoke callback if provided */
        if (item.callback) {
            item.callback(item.result, item.user_data);
        }

        /* Update stats */
        pthread_mutex_lock(&pool->stats_mutex);
        pool->stats.frames_decoded++;
        pthread_mutex_unlock(&pool->stats_mutex);
    }

    return NULL;
}

int thread_pool_init(thread_pool_t* pool, int num_threads) {
    if (!pool) return DRONEID_ERROR_INVALID_ARG;

    memset(pool, 0, sizeof(thread_pool_t));

    /* Auto-detect number of CPUs if not specified */
    if (num_threads <= 0) {
        num_threads = sysconf(_SC_NPROCESSORS_ONLN);
        if (num_threads <= 0) num_threads = 2;  /* Default to 2 for Zynq */
    }

    if (num_threads > MAX_WORKER_THREADS) {
        num_threads = MAX_WORKER_THREADS;
    }

    pool->num_threads = num_threads;
    pool->shutdown = false;

    /* Initialize work queue */
    int ret = work_queue_init(&pool->queue);
    if (ret != 0) return ret;

    /* Initialize stats mutex */
    pthread_mutex_init(&pool->stats_mutex, NULL);

    /* Create worker threads */
    for (int i = 0; i < num_threads; i++) {
        ret = pthread_create(&pool->threads[i], NULL, worker_thread, pool);
        if (ret != 0) {
            LOG_ERROR("Failed to create thread %d: %d\n", i, ret);
            pool->num_threads = i;
            thread_pool_shutdown(pool);
            return DRONEID_ERROR_IO;
        }

        /* Set CPU affinity for dual-core Zynq */
        if (num_threads == 2) {
            thread_set_affinity(pool->threads[i], i);
        }
    }

    LOG_INFO("Thread pool initialized with %d workers\n", num_threads);

    return DRONEID_SUCCESS;
}

int thread_pool_submit(thread_pool_t* pool,
                       work_type_t type,
                       void* data,
                       void (*callback)(void* result, void* user_data),
                       void* user_data) {
    if (!pool || pool->shutdown) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    work_item_t item;
    item.type = type;
    item.data = data;
    item.result = NULL;
    item.callback = callback;
    item.user_data = user_data;

    return work_queue_push(&pool->queue, &item);
}

void thread_pool_wait(thread_pool_t* pool) {
    if (!pool) return;

    /* Wait for queue to be empty */
    pthread_mutex_lock(&pool->queue.mutex);
    while (pool->queue.count > 0) {
        pthread_mutex_unlock(&pool->queue.mutex);
        usleep(1000);  /* Sleep 1ms */
        pthread_mutex_lock(&pool->queue.mutex);
    }
    pthread_mutex_unlock(&pool->queue.mutex);
}

void thread_pool_shutdown(thread_pool_t* pool) {
    if (!pool || pool->shutdown) return;

    pool->shutdown = true;

    /* Send shutdown messages to all workers */
    work_item_t shutdown_item;
    shutdown_item.type = WORK_SHUTDOWN;
    shutdown_item.data = NULL;
    shutdown_item.result = NULL;
    shutdown_item.callback = NULL;
    shutdown_item.user_data = NULL;

    for (int i = 0; i < pool->num_threads; i++) {
        work_queue_push(&pool->queue, &shutdown_item);
    }

    /* Wait for all threads to finish */
    for (int i = 0; i < pool->num_threads; i++) {
        pthread_join(pool->threads[i], NULL);
    }

    LOG_INFO("Thread pool shutdown complete\n");
}

void thread_pool_destroy(thread_pool_t* pool) {
    if (!pool) return;

    if (!pool->shutdown) {
        thread_pool_shutdown(pool);
    }

    work_queue_destroy(&pool->queue);
    pthread_mutex_destroy(&pool->stats_mutex);

    memset(pool, 0, sizeof(thread_pool_t));
}

void thread_pool_get_stats(thread_pool_t* pool, droneid_stats_t* stats) {
    if (!pool || !stats) return;

    pthread_mutex_lock(&pool->stats_mutex);
    *stats = pool->stats;
    pthread_mutex_unlock(&pool->stats_mutex);
}

int thread_set_affinity(pthread_t thread_id, int cpu_core) {
#ifdef __linux__
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_core, &cpuset);

    int ret = pthread_setaffinity_np(thread_id, sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        LOG_WARN("Failed to set CPU affinity to core %d: %d\n", cpu_core, ret);
        return DRONEID_ERROR_IO;
    }

    LOG_DEBUG("Thread pinned to CPU core %d\n", cpu_core);
    return DRONEID_SUCCESS;
#else
    LOG_WARN("CPU affinity not supported on this platform\n");
    return DRONEID_ERROR_IO;
#endif
}
