/*
 * Copyright (c) 2013-2014, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/*******************************************************************************
 * This is the Secure Payload Dispatcher (SPD). The dispatcher is meant to be a
 * plug-in component to the Secure Monitor, registered as a runtime service. The
 * SPD is expected to be a functional extension of the Secure Payload (SP) that
 * executes in Secure EL1. The Secure Monitor will delegate all SMCs targeting
 * the Trusted OS/Applications range to the dispatcher. The SPD will either
 * handle the request locally or delegate it to the Secure Payload. It is also
 * responsible for initialising and maintaining communication with the SP.
 ******************************************************************************/
#include <arch_helpers.h>
#include <assert.h>
#include <bl_common.h>
#include <bl31.h>
#include <context_mgmt.h>
#include <debug.h>
#include <errno.h>
#include <platform.h>
#include <runtime_svc.h>
#include <stddef.h>
#include <string.h>
#include <uuid.h>
#include "opteed_private.h"
#include "teesmc_opteed_macros.h"
#include "teesmc_opteed.h"

#define OPTEE_MAGIC		0x4554504f
#define OPTEE_VERSION		1
#define OPTEE_ARCH_ARM32	0
#define OPTEE_ARCH_ARM64	1

struct optee_header {
	uint32_t magic;
	uint8_t version;
	uint8_t arch;
	uint16_t flags;
	uint32_t init_size;
	uint32_t init_load_addr_hi;
	uint32_t init_load_addr_lo;
	uint32_t init_mem_usage;
	uint32_t paged_size;
};

/*******************************************************************************
 * Address of the entrypoint vector table in OPTEE. It is
 * initialised once on the primary core after a cold boot.
 ******************************************************************************/
optee_vectors_t *optee_vectors;

/*******************************************************************************
 * Array to keep track of per-cpu OPTEE state
 ******************************************************************************/
optee_context_t opteed_sp_context[OPTEED_CORE_COUNT];
uint32_t opteed_rw;



static int32_t opteed_init(void);

/*******************************************************************************
 * This function is the handler registered for S-EL1 interrupts by the
 * OPTEED. It validates the interrupt and upon success arranges entry into
 * the OPTEE at 'optee_fiq_entry()' for handling the interrupt.
 ******************************************************************************/
static uint64_t opteed_sel1_interrupt_handler(uint32_t id,
					    uint32_t flags,
					    void *handle,
					    void *cookie)
{
	uint32_t linear_id;
	uint64_t mpidr;
	optee_context_t *optee_ctx;

	/* Check the security state when the exception was generated */
	assert(get_interrupt_src_ss(flags) == NON_SECURE);

#if IMF_READ_INTERRUPT_ID
	/* Check the security status of the interrupt */
	assert(plat_ic_get_interrupt_type(id) == INTR_TYPE_S_EL1);
#endif

	/* Sanity check the pointer to this cpu's context */
	mpidr = read_mpidr();
	assert(handle == cm_get_context(NON_SECURE));

	/* Save the non-secure context before entering the OPTEE */
	cm_el1_sysregs_context_save(NON_SECURE);

	/* Get a reference to this cpu's OPTEE context */
	linear_id = platform_get_core_pos(mpidr);
	optee_ctx = &opteed_sp_context[linear_id];
	assert(&optee_ctx->cpu_ctx == cm_get_context(SECURE));

	cm_set_elr_el3(SECURE, (uint64_t)&optee_vectors->fiq_entry);
	cm_el1_sysregs_context_restore(SECURE);
	cm_set_next_eret_context(SECURE);

	/*
	 * Tell the OPTEE that it has to handle an FIQ (synchronously).
	 * Also the instruction in normal world where the interrupt was
	 * generated is passed for debugging purposes. It is safe to
	 * retrieve this address from ELR_EL3 as the secure context will
	 * not take effect until el3_exit().
	 */
	SMC_RET1(&optee_ctx->cpu_ctx, read_elr_el3());
}


static int is_mem_free(uint64_t free_base, size_t free_size,
		       uint64_t addr, size_t size)
{
	return (addr >= free_base) && (addr + size <= free_base + free_size);
}

/*******************************************************************************
 * OPTEE Dispatcher setup. The OPTEED finds out the OPTEE entrypoint and type
 * (aarch32/aarch64) if not already known and initialises the context for entry
 * into OPTEE for its initialization.
 ******************************************************************************/
int32_t opteed_setup(void)
{
	entry_point_info_t *ep_info;
	struct optee_header *header;
	uint64_t mpidr = read_mpidr();
	uint32_t linear_id;
	uintptr_t init_load_addr;
	size_t init_size;
	size_t init_mem_usage;
	uintptr_t payload_addr;
	uintptr_t mem_limit;
	uintptr_t paged_part;
	uintptr_t paged_size;

	linear_id = platform_get_core_pos(mpidr);

	/*
	 * Get information about the Secure Payload (BL32) image. Its
	 * absence is a critical failure.  TODO: Add support to
	 * conditionally include the SPD service
	 */
	ep_info = bl31_plat_get_next_image_ep_info(SECURE);
	if (!ep_info) {
		WARN("No OPTEE provided by BL2 boot loader.\n");
		goto err;
	}

	header = (struct optee_header *)ep_info->pc;

	if (header->magic != OPTEE_MAGIC || header->version != OPTEE_VERSION) {
		WARN("Invalid OPTEE header.\n");
		goto err;
	}

	if (header->arch == OPTEE_ARCH_ARM32)
		opteed_rw = OPTEE_AARCH32;
	else if (header->arch == OPTEE_ARCH_ARM64)
		opteed_rw = OPTEE_AARCH64;
	else {
		WARN("Invalid OPTEE architecture (%d)\n", header->arch);
		goto err;
	}

	init_load_addr = ((uint64_t)header->init_load_addr_hi << 32) |
				header->init_load_addr_lo;
	init_size = header->init_size;
	init_mem_usage = header->init_mem_usage;
	payload_addr = (uintptr_t)(header + 1);
	paged_size = header->paged_size;

	/*
	 * Move OPTEE binary to the required location in memory.
	 *
	 * There's two ways OPTEE can be running in memory:
	 * 1. A memory large enough to keep the entire OPTEE binary
	 *    (DRAM currently)
	 * 2. A part of OPTEE in a smaller (and more secure) memory
	 *    (SRAM currently). This is achieved with demand paging
	 *    of read-only data/code against a backing store in some
	 *    larger memory (DRAM currently).
	 *
	 * In either case dictates init_load_addr in the OPTEE
	 * header the address where what's after the header
	 * (payload) should be residing when started. init_size in
	 * the header tells how much of the payload that need to be
	 * copied. init_mem_usage tells how much runtime memory in
	 * total is needed by OPTEE.
	 *
	 * In alternative 2 there's additional data after
	 * init_size, this is the rest of OPTEE which is demand
	 * paged into memory.  A pointer to that data is supplied
	 * to OPTEE when initializing.
	 *
	 * Alternative 1 only uses DRAM when executing OPTEE while
	 * alternative 2 uses both SRAM and DRAM to execute.
	 *
	 * All data written which is later read by OPTEE must be flushed
	 * out to memory since OPTEE starts with MMU turned off and caches
	 * disabled.
	 */
	if (is_mem_free(BL32_SRAM_BASE,
			 BL32_SRAM_LIMIT - BL32_SRAM_BASE,
			 init_load_addr, init_mem_usage)) {
		/* Running in SRAM, paging some code against DRAM */
		memcpy((void *)init_load_addr, (void *)payload_addr,
		       init_size);
		flush_dcache_range(init_load_addr, init_size);
		paged_part = payload_addr + init_size;
		mem_limit = BL32_SRAM_LIMIT;
	} else if (is_mem_free(BL32_DRAM_BASE,
			       BL32_DRAM_LIMIT - BL32_DRAM_BASE,
			       init_load_addr, init_mem_usage)) {
		/*
		 * Running in DRAM.
		 *
		 * The paged part normally empty, but if it isn't,
		 * move it to the end of DRAM before moving the
		 * init part in place.
		 */
		paged_part = BL32_DRAM_LIMIT - paged_size;
		if (paged_size) {
			if (!is_mem_free(BL32_DRAM_BASE,
					 BL32_DRAM_LIMIT - BL32_DRAM_BASE,
					 init_load_addr,
					 init_mem_usage + paged_size)) {
				WARN("Failed to reserve memory 0x%lx - 0x%lx\n",
				      init_load_addr,
				      init_load_addr + init_mem_usage +
					paged_size);
				goto err;
			}

			memcpy((void *)paged_part,
				(void *)(payload_addr + init_size),
				paged_size);
			flush_dcache_range(paged_part, paged_size);
		}

		memmove((void *)init_load_addr, (void *)payload_addr,
			init_size);
		flush_dcache_range(init_load_addr, init_size);
		mem_limit = BL32_DRAM_LIMIT;
	} else {
		WARN("Failed to reserve memory 0x%lx - 0x%lx\n",
			init_load_addr, init_load_addr + init_mem_usage);
		goto err;
	}


	opteed_init_optee_ep_state(ep_info, opteed_rw, init_load_addr,
				   paged_part, mem_limit,
				   &opteed_sp_context[linear_id]);

	/*
	 * All OPTEED initialization done. Now register our init function with
	 * BL31 for deferred invocation
	 */
	bl31_register_bl32_init(&opteed_init);

	return 0;

err:
	WARN("Booting device without OPTEE initialization.\n");
	WARN("SMC`s destined for OPTEE will return SMC_UNK\n");
	return 1;
}

/*******************************************************************************
 * This function passes control to the OPTEE image (BL32) for the first time
 * on the primary cpu after a cold boot. It assumes that a valid secure
 * context has already been created by opteed_setup() which can be directly
 * used.  It also assumes that a valid non-secure context has been
 * initialised by PSCI so it does not need to save and restore any
 * non-secure state. This function performs a synchronous entry into
 * OPTEE. OPTEE passes control back to this routine through a SMC.
 ******************************************************************************/
static int32_t opteed_init(void)
{
	uint64_t mpidr = read_mpidr();
	uint32_t linear_id = platform_get_core_pos(mpidr);
	optee_context_t *optee_ctx = &opteed_sp_context[linear_id];
	entry_point_info_t *optee_entry_point;
	uint64_t rc;

	/*
	 * Get information about the OPTEE (BL32) image. Its
	 * absence is a critical failure.
	 */
	optee_entry_point = bl31_plat_get_next_image_ep_info(SECURE);
	assert(optee_entry_point);

	cm_init_context(mpidr, optee_entry_point);

	/*
	 * Arrange for an entry into OPTEE. It will be returned via
	 * OPTEE_ENTRY_DONE case
	 */
	rc = opteed_synchronous_sp_entry(optee_ctx);
	assert(rc != 0);

	return rc;
}


/*******************************************************************************
 * This function is responsible for handling all SMCs in the Trusted OS/App
 * range from the non-secure state as defined in the SMC Calling Convention
 * Document. It is also responsible for communicating with the Secure
 * payload to delegate work and return results back to the non-secure
 * state. Lastly it will also return any information that OPTEE needs to do
 * the work assigned to it.
 ******************************************************************************/
uint64_t opteed_smc_handler(uint32_t smc_fid,
			 uint64_t x1,
			 uint64_t x2,
			 uint64_t x3,
			 uint64_t x4,
			 void *cookie,
			 void *handle,
			 uint64_t flags)
{
	cpu_context_t *ns_cpu_context;
	unsigned long mpidr = read_mpidr();
	uint32_t linear_id = platform_get_core_pos(mpidr);
	optee_context_t *optee_ctx = &opteed_sp_context[linear_id];
	uint64_t rc;

	/*
	 * Determine which security state this SMC originated from
	 */

	if (is_caller_non_secure(flags)) {
		/*
		 * This is a fresh request from the non-secure client.
		 * The parameters are in x1 and x2. Figure out which
		 * registers need to be preserved, save the non-secure
		 * state and send the request to the secure payload.
		 */
		assert(handle == cm_get_context(NON_SECURE));

		cm_el1_sysregs_context_save(NON_SECURE);

		/*
		 * We are done stashing the non-secure context. Ask the
		 * OPTEE to do the work now.
		 */

		/*
		 * Verify if there is a valid context to use, copy the
		 * operation type and parameters to the secure context
		 * and jump to the fast smc entry point in the secure
		 * payload. Entry into S-EL1 will take place upon exit
		 * from this function.
		 */
		assert(&optee_ctx->cpu_ctx == cm_get_context(SECURE));

		/* Set appropriate entry for SMC.
		 * We expect OPTEE to manage the PSTATE.I and PSTATE.F
		 * flags as appropriate.
		 */
		if (GET_SMC_TYPE(smc_fid) == SMC_TYPE_FAST) {
			cm_set_elr_el3(SECURE, (uint64_t)
					&optee_vectors->fast_smc_entry);
		} else {
			cm_set_elr_el3(SECURE, (uint64_t)
					&optee_vectors->std_smc_entry);
		}

		cm_el1_sysregs_context_restore(SECURE);
		cm_set_next_eret_context(SECURE);

		/* Propagate hypervisor client ID */
		write_ctx_reg(get_gpregs_ctx(&optee_ctx->cpu_ctx),
			      CTX_GPREG_X7,
			      read_ctx_reg(get_gpregs_ctx(handle),
					   CTX_GPREG_X7));

		SMC_RET4(&optee_ctx->cpu_ctx, smc_fid, x1, x2, x3);
	}

	/*
	 * Returning from OPTEE
	 */

	switch (smc_fid) {
	/*
	 * OPTEE has finished initialising itself after a cold boot
	 */
	case TEESMC_OPTEED_RETURN_ENTRY_DONE:
		/*
		 * Stash the OPTEE entry points information. This is done
		 * only once on the primary cpu
		 */
		assert(optee_vectors == NULL);
		optee_vectors = (optee_vectors_t *) x1;

		if (optee_vectors) {
			set_optee_pstate(optee_ctx->state, OPTEE_PSTATE_ON);

			/*
			 * OPTEE has been successfully initialized.
			 * Register power management hooks with PSCI
			 */
			psci_register_spd_pm_hook(&opteed_pm);

			/*
			 * Register an interrupt handler for S-EL1 interrupts
			 * when generated during code executing in the
			 * non-secure state.
			 */
			flags = 0;
			set_interrupt_rm_flag(flags, NON_SECURE);
			rc = register_interrupt_type_handler(INTR_TYPE_S_EL1,
						opteed_sel1_interrupt_handler,
						flags);
			if (rc)
				panic();
		}

		/*
		 * OPTEE reports completion. The OPTEED must have initiated
		 * the original request through a synchronous entry into
		 * OPTEE. Jump back to the original C runtime context.
		 */
		opteed_synchronous_sp_exit(optee_ctx, x1);


	/*
	 * These function IDs is used only by OP-TEE to indicate it has
	 * finished:
	 * 1. turning itself on in response to an earlier psci
	 *    cpu_on request
	 * 2. resuming itself after an earlier psci cpu_suspend
	 *    request.
	 */
	case TEESMC_OPTEED_RETURN_ON_DONE:
	case TEESMC_OPTEED_RETURN_RESUME_DONE:


	/*
	 * These function IDs is used only by the SP to indicate it has
	 * finished:
	 * 1. suspending itself after an earlier psci cpu_suspend
	 *    request.
	 * 2. turning itself off in response to an earlier psci
	 *    cpu_off request.
	 */
	case TEESMC_OPTEED_RETURN_OFF_DONE:
	case TEESMC_OPTEED_RETURN_SUSPEND_DONE:
	case TEESMC_OPTEED_RETURN_SYSTEM_OFF_DONE:
	case TEESMC_OPTEED_RETURN_SYSTEM_RESET_DONE:

		/*
		 * OPTEE reports completion. The OPTEED must have initiated the
		 * original request through a synchronous entry into OPTEE.
		 * Jump back to the original C runtime context, and pass x1 as
		 * return value to the caller
		 */
		opteed_synchronous_sp_exit(optee_ctx, x1);

	/*
	 * OPTEE is returning from a call or being preempted from a call, in
	 * either case execution should resume in the normal world.
	 */
	case TEESMC_OPTEED_RETURN_CALL_DONE:
		/*
		 * This is the result from the secure client of an
		 * earlier request. The results are in x0-x3. Copy it
		 * into the non-secure context, save the secure state
		 * and return to the non-secure state.
		 */
		assert(handle == cm_get_context(SECURE));
		cm_el1_sysregs_context_save(SECURE);

		/* Get a reference to the non-secure context */
		ns_cpu_context = cm_get_context(NON_SECURE);
		assert(ns_cpu_context);

		/* Restore non-secure state */
		cm_el1_sysregs_context_restore(NON_SECURE);
		cm_set_next_eret_context(NON_SECURE);

		SMC_RET4(ns_cpu_context, x1, x2, x3, x4);

	/*
	 * OPTEE has finished handling a S-EL1 FIQ interrupt. Execution
	 * should resume in the normal world.
	 */
	case TEESMC_OPTEED_RETURN_FIQ_DONE:
		/* Get a reference to the non-secure context */
		ns_cpu_context = cm_get_context(NON_SECURE);
		assert(ns_cpu_context);

		/*
		 * Restore non-secure state. There is no need to save the
		 * secure system register context since OPTEE was supposed
		 * to preserve it during S-EL1 interrupt handling.
		 */
		cm_el1_sysregs_context_restore(NON_SECURE);
		cm_set_next_eret_context(NON_SECURE);

		SMC_RET0((uint64_t) ns_cpu_context);

	default:
		panic();
	}
}

/* Define an OPTEED runtime service descriptor for fast SMC calls */
DECLARE_RT_SVC(
	opteed_fast,

	OEN_TOS_START,
	OEN_TOS_END,
	SMC_TYPE_FAST,
	opteed_setup,
	opteed_smc_handler
);

/* Define an OPTEED runtime service descriptor for standard SMC calls */
DECLARE_RT_SVC(
	opteed_std,

	OEN_TOS_START,
	OEN_TOS_END,
	SMC_TYPE_STD,
	NULL,
	opteed_smc_handler
);
