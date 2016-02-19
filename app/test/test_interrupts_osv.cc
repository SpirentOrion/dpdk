#include <rte_log.h>

#include "test_interrupts_osv.h"
#include "eal_interrupts_osv.h"

#include <osv/types.h>
#include <osv/pci.hh>

class test_device : public pci::device {
public:
	test_device(u8 bus, u8 device, u8 func)
		: pci::device(bus, device, func) {};
	~test_device() {};

	bool parse_pci_config() { return true; };
	void dump_config() {};
};

extern "C" {

/**
 * predefined interrupt handle types
 * This is lifted directly out of test_interrupts.c and needs to match
 * _exactly_.
 */
	enum test_interrupt_handle_type {
		TEST_INTERRUPT_HANDLE_INVALID,
		TEST_INTERRUPT_HANDLE_VALID,
		TEST_INTERRUPT_HANDLE_CASE1,
		TEST_INTERRUPT_HANDLE_MAX
	};

	static struct rte_pollable *test_pollables[TEST_INTERRUPT_HANDLE_MAX];

	int test_interrupt_osv_init(struct rte_intr_handle *intr_handle)
	{
		/* Setup bogus test handles */
		struct rte_intr_handle *test_handle =
			&intr_handle[TEST_INTERRUPT_HANDLE_INVALID];
		test_handle->pollable = test_pollables[TEST_INTERRUPT_HANDLE_INVALID] = nullptr;
		test_handle->device = nullptr;
		test_handle->type = RTE_INTR_HANDLE_UNKNOWN;

		/* this seems misnamed to me, but whatever... */
		test_handle = &intr_handle[TEST_INTERRUPT_HANDLE_VALID];
		test_handle->pollable = test_pollables[TEST_INTERRUPT_HANDLE_VALID] =
			rte_pollable_allocate();
		test_handle->device = new test_device(0, 48, 14);
		test_handle->type = RTE_INTR_HANDLE_UNKNOWN;

		test_handle = &intr_handle[TEST_INTERRUPT_HANDLE_CASE1];
		test_handle->pollable = test_pollables[TEST_INTERRUPT_HANDLE_CASE1] =
			rte_pollable_allocate();
		test_handle->device = new test_device(0, 48, 14);
		test_handle->type = RTE_INTR_HANDLE_MSI;

		return 0;
	}

	int test_interrupt_osv_deinit(struct rte_intr_handle *intr_handle)
	{
		/* Destroy bogus test handles */
		for (int i = 0; i < TEST_INTERRUPT_HANDLE_MAX; i++) {
			struct rte_intr_handle *test_handle = &intr_handle[i];

			if (test_handle->pollable != nullptr) {
				rte_pollable_free(&test_handle->pollable);
			}

			test_pollables[i] = nullptr;

			if (test_handle->device != nullptr) {
				test_device *dev =
					static_cast<test_device *>(test_handle->device);
				delete dev;
			}
		}

		return 0;
	}

	int test_interrupt_osv_trigger_interrupt(int idx)
	{
		if (test_pollables[idx] == nullptr) {
			return -1;
		}

		rte_pollable_wake(test_pollables[idx]);
		return 0;
	}

	int test_interrupt_handle_sanity_check(struct rte_intr_handle *intr_handle)
	{
		if (intr_handle == nullptr
			|| intr_handle->pollable == nullptr
			|| intr_handle->device == nullptr) {
			return -1;
		}

		return 0;
	}

	int test_interrupt_handle_compare(struct rte_intr_handle *intr_handle_l,
					struct rte_intr_handle *intr_handle_r)
	{
		if (intr_handle_l == nullptr
			|| intr_handle_r == nullptr
			|| intr_handle_l->type != intr_handle_r->type
			|| intr_handle_l->pollable != intr_handle_r->pollable
			|| intr_handle_l->device != intr_handle_r->device) {
			return -1;
		}

		return 0;
	}

}
