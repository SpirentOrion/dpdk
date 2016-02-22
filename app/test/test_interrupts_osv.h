#ifndef TEST_INTERRUPTS_OSV_H_
#define TEST_INTERRUPTS_OSV_H_

#ifdef __cplusplus
extern "C" {
#endif

struct rte_intr_handle;

int test_interrupt_osv_init(struct rte_intr_handle *intr_handle);
int test_interrupt_osv_deinit(struct rte_intr_handle *intr_handle);
int test_interrupt_osv_trigger_interrupt(int idx);
int test_interrupt_handle_sanity_check(struct rte_intr_handle *intr_handle);
int test_interrupt_handle_compare(struct rte_intr_handle *intr_handle_l,
				  struct rte_intr_handle *intr_handle_r);

#ifdef __cplusplus
}
#endif

#endif /* TEST_INTERRUPTS_OSV_H_ */
