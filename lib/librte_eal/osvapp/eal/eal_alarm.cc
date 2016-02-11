#include <algorithm>
#include <atomic>
#include <functional>
#include <list>

#include <rte_alarm.h>
#include <rte_errno.h>
#include <rte_malloc.h>
#include <rte_spinlock.h>
#include <eal_private.h>

#include <osv/async.hh>
#include <osv/mutex.h>

struct alarm_item {
	rte_eal_alarm_callback cb_fn;	 /**< callback fn; used as key in cancel */
	void *cb_arg;			 /**< callback arg; used as key in cancel */
	async::serial_timer_task *task;	 /**< pointer to async callback task */
	mutex task_mutex;		 /**< mutex for async task */
	std::atomic_bool in_progress;	 /**< in progress flag; needed to prevent
					  * cancels from callback succeeding;
					  * can't use the mutex as it's recursive */
};

static std::list<alarm_item *> alarm_list = {};	 /* list of all alarms */
static mutex list_mutex;			 /* prevent concurrent list access */

/*
 * The alarm unit test checks for invalid input based on the linuxapp
 * implementation internals.  Sigh.
 */
static const uint64_t linuxapp_impl_nonsense = UINT64_MAX - 1e6;

int
rte_eal_alarm_init(void)
{
	return 0;
}

void eal_alarm_callback(async::serial_timer_task &task, struct alarm_item *item)
{
	WITH_LOCK(item->task_mutex) {
		if (task.try_fire()) {
			item->in_progress.store(true);
			item->cb_fn(item->cb_arg);
			item->in_progress.store(false);
		}
	}

	async::run_later([item] {
		WITH_LOCK(list_mutex) {
                        alarm_list.remove_if(
                                [item](struct alarm_item *curr) -> bool {
                                        if (item == curr) {
                                                WITH_LOCK(curr->task_mutex) {
                                                        delete curr->task;
                                                }
                                                rte_free(curr);
                                                return true;
                                        }
                                        return false;
                                });
                }
	});
}

int
rte_eal_alarm_set(uint64_t us, rte_eal_alarm_callback cb_fn, void *cb_arg)
{
	using namespace std::placeholders;

	/* Validate input */
	if (us < 1 || us > linuxapp_impl_nonsense || cb_fn == NULL) {
		return -EINVAL;
	}

	struct alarm_item *item = static_cast<struct alarm_item *>(
		rte_zmalloc(NULL, sizeof(*item), 0));
	if (item == NULL) {
		return -ENOMEM;
	}

	item->cb_fn = cb_fn;
	item->cb_arg = cb_arg;
	item->in_progress = false;
	mutex_init(&item->task_mutex);

	WITH_LOCK(item->task_mutex) {
		item->task = new async::serial_timer_task(
			item->task_mutex,
			std::bind(eal_alarm_callback, _1, item));

		if (item->task == NULL) {
			rte_free(item);
			return -ENOMEM;
		}

		item->task->reschedule(std::chrono::microseconds(us));
	}

	WITH_LOCK(list_mutex) {
		alarm_list.push_front(item);
	}

	return 0;
}

int
rte_eal_alarm_cancel(rte_eal_alarm_callback cb_fn, void *cb_arg)
{
	if (cb_fn == NULL) {
		rte_errno = EINVAL;
		return -1;
	}

	size_t old_size = 0, new_size = 0;
	int error = 0;

	WITH_LOCK(list_mutex) {
		old_size = alarm_list.size();
		alarm_list.remove_if(
			[cb_fn, cb_arg](struct alarm_item *item) -> bool {
				if (cb_fn == item->cb_fn
					&& (cb_arg == reinterpret_cast<void *>(-1)
						|| cb_arg == item->cb_arg)
					&& !item->in_progress.load()) {
					WITH_LOCK(item->task_mutex) {
						item->task->cancel_sync();
						rte_free(item);
						return true;
					}
				}

				return false;
			});
		new_size = alarm_list.size();
	}

	if (old_size == new_size) {
		rte_errno = ENOENT;
	}

	return (old_size - new_size);
}
