#pragma once

#include <tlm_utils/simple_target_socket.h>
#include <systemc>

#include "core/common/irq_if.h"
#include "util/tlm_map.h"
#include "util/memory_map.h"

template <unsigned NumberCores, unsigned NumberInterrupts, unsigned NumberInterruptEntries, uint32_t MaxPriority>
struct PLIC : public sc_core::sc_module, public interrupt_gateway {
	static_assert(NumberInterrupts <= 4096, "out of bound");
	static_assert(NumberCores <= 15360, "out of bound");

	tlm_utils::simple_target_socket<PLIC> tsock;

	std::array<external_interrupt_target*, NumberCores> target_harts{};

	// shared for all harts priority 1 is the lowest. Zero means do not interrupt
	// NOTE: addressing starts at 0x4 because interrupt 0 is reserved, however some example SW still writes to address 0x0, hence we added it to the address map
	RegisterRange regs_interrupt_priorities{0x0, 4*(NumberInterrupts+1)};
	ArrayView<uint32_t> interrupt_priorities{regs_interrupt_priorities};

	RegisterRange regs_pending_interrupts{0x1000, 4*NumberInterruptEntries};
	ArrayView<uint32_t> pending_interrupts{regs_pending_interrupts};

	struct HartConfig {
		uint32_t priority_threshold;
		uint32_t claim_response;
	};

	RegisterRange regs_hart_enabled_interrupts{0x2000, 4*NumberInterruptEntries*NumberCores};
	ArrayView<uint32_t> hart_enabled_interrupts{regs_hart_enabled_interrupts};

	RegisterRange regs_hart_config{0x200000, sizeof(HartConfig)*NumberCores};
	ArrayView<HartConfig> hart_config{regs_hart_config};

	std::vector<RegisterRange *> register_ranges{
			&regs_interrupt_priorities,
			&regs_pending_interrupts,
			&regs_hart_enabled_interrupts,
			&regs_hart_config
	};

	PrivilegeLevel irq_level;
	std::array<bool, NumberCores> hart_eip{};

	sc_core::sc_event e_run;
	sc_core::sc_time clock_cycle;

	SC_HAS_PROCESS(PLIC);

	PLIC(sc_core::sc_module_name, PrivilegeLevel level = MachineMode) {
		clock_cycle = sc_core::sc_time(10, sc_core::SC_NS);
		tsock.register_b_transport(this, &PLIC::transport);

		regs_pending_interrupts.readonly = true;
		regs_hart_config.alignment = 4;

		regs_interrupt_priorities.post_write_callback = std::bind(&PLIC::post_write_interrupt_priorities, this, std::placeholders::_1);
		regs_hart_config.post_write_callback = std::bind(&PLIC::post_write_hart_config, this, std::placeholders::_1);
		regs_hart_config.pre_read_callback = std::bind(&PLIC::pre_read_hart_config, this, std::placeholders::_1);

		for (unsigned i=0; i<NumberInterrupts; ++i) {
			interrupt_priorities[i] = 1;
		}

		for (unsigned n=0; n<NumberCores; ++n) {
			hart_eip[n] = false;
			for (unsigned i=0; i<NumberInterruptEntries; ++i) {
				hart_enabled_interrupts(n,i) = 0x0;	// all interrupts disabled by default
			}
		}

		irq_level = level;
		SC_THREAD(run);
	}


	void gateway_trigger_interrupt(uint32_t irq_id) {
		// NOTE: can use different techniques for each gateway, in this case a
		// simple non queued edge trigger
		assert(irq_id > 0 && irq_id < NumberInterrupts);
		//std::cout << "[vp::plic] incoming interrupt " << irq_id << std::endl;

		unsigned idx = irq_id / 32;
		unsigned off = irq_id % 32;

		pending_interrupts[idx] |= 1 << off;

		e_run.notify(clock_cycle);
	}


	void clear_pending_interrupt(unsigned irq_id) {
		assert(irq_id >= 0 && irq_id < NumberInterrupts);  //NOTE: ignore clear of zero interrupt (zero is not available)
		//std::cout << "[vp::plic] clear pending interrupt " << irq_id << std::endl;

		unsigned idx = irq_id / 32;
		unsigned off = irq_id % 32;

		pending_interrupts[idx] &= ~(1 << off);
	}


	unsigned hart_get_next_pending_interrupt(unsigned hart_id, bool consider_threshold) {
		unsigned min_id = 0;
		unsigned max_priority = 0;

		for (unsigned i=1; i<NumberInterrupts; ++i) {
			unsigned idx = i / 32;
			unsigned off = i % 32;

			if (hart_enabled_interrupts(hart_id,idx) & (1 << off)) {
				if (pending_interrupts[idx] & (1 << off)) {
					auto prio = interrupt_priorities[i];
					if (prio > 0 && (!consider_threshold || (prio > hart_config[hart_id].priority_threshold))) {
						if (prio > max_priority) {
							max_priority = prio;
							min_id = i;
						}
					}
				}
			}
		}

		return min_id;
	}


	void transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
		delay += 4*clock_cycle;

        vp::mm::route("PLIC", register_ranges, trans, delay);
	}

	void post_write_interrupt_priorities(RegisterRange::WriteInfo t) {
        (void) t;

		for (auto &x : interrupt_priorities)
			x = std::min(x, MaxPriority);
	}

	bool pre_read_hart_config(RegisterRange::ReadInfo t) {
		assert (t.addr % 4 == 0);
		unsigned idx = t.addr / 4;

		if ((idx % 2) == 1) {
			// access is directed to claim response register
			assert (t.size == 4);
			--idx;

			unsigned min_id = hart_get_next_pending_interrupt(0, false);
			hart_config[idx].claim_response = min_id;
			clear_pending_interrupt(min_id);
		}

		return true;
	}

	void post_write_hart_config(RegisterRange::WriteInfo t) {
		assert (t.addr % 4 == 0);
		unsigned idx = t.addr / 4;

		if ((idx % 2) == 1) {
			// access is directed to claim response register
			assert (t.size == 4);
			--idx;

			if (hart_has_pending_enabled_interrupts(idx)) {
				assert(hart_eip[idx]);
				// trigger again to make this work even if the SW clears the harts interrupt pending bit
				target_harts[idx]->trigger_external_interrupt(irq_level);
			} else {
				hart_eip[idx] = false;
				target_harts[idx]->clear_external_interrupt(irq_level);
				//std::cout << "[vp::plic] clear eip" << std::endl;
			}
		}
	}


	bool hart_has_pending_enabled_interrupts(unsigned hart_id) {
		return hart_get_next_pending_interrupt(hart_id, true) > 0;
	}


	void run() {
		while (true) {
			sc_core::wait(e_run);

			for (unsigned i=0; i<NumberCores; ++i) {
				if (!hart_eip[i]) {
					if (hart_has_pending_enabled_interrupts(i)) {
						//std::cout << "[vp::plic] trigger interrupt" << std::endl;
						hart_eip[i] = true;
						target_harts[i]->trigger_external_interrupt(irq_level);
					}
				}
			}
		}
	}
};
