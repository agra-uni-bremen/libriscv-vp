#ifndef RISCV_ISA_BUS_H
#define RISCV_ISA_BUS_H

#include <map>
#include <stdexcept>

#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <systemc>

struct PortMapping {
	uint64_t start;
	uint64_t end;

	PortMapping(uint64_t start, uint64_t end) : start(start), end(end) {
		assert(end > start);
	}

	bool contains(uint64_t addr) {
		return addr >= start && addr < end;
	}

	uint64_t global_to_local(uint64_t addr) {
		return addr - start;
	}
};

template <unsigned int NR_OF_INITIATORS, unsigned int NR_OF_TARGETS>
struct SimpleBus : sc_core::sc_module {
	std::array<tlm_utils::simple_target_socket<SimpleBus>, NR_OF_INITIATORS> tsocks;

	std::array<tlm_utils::simple_initiator_socket<SimpleBus>, NR_OF_TARGETS> isocks;
	std::array<PortMapping *, NR_OF_TARGETS> ports;

	SimpleBus(sc_core::sc_module_name) {
		for (auto &s : tsocks)
			s.register_b_transport(this, &SimpleBus::transport);
	}

	unsigned decode(uint64_t addr) {
		for (unsigned i = 0; i < NR_OF_TARGETS; ++i) {
			if (ports[i]->contains(addr))
				return i;
		}
		throw std::runtime_error("unable to find target port for address " + std::to_string(addr));
	}

	void transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
		auto addr = trans.get_address();
		auto id = decode(addr);

		trans.set_address(ports[id]->global_to_local(addr));
		isocks[id]->b_transport(trans, delay);
	}
};


#include "core/common/bus_lock_if.h"

/*
 * Use this adapter to attach peripherals with write access (e.g. DMA) to the bus.
 * This ensures that those peripherals do not violate the RISC-V LR/SC atomic semantic.
 */
struct PeripheralConnector : sc_core::sc_module {
	tlm_utils::simple_target_socket<PeripheralConnector> tsock;
	tlm_utils::simple_initiator_socket<PeripheralConnector> isock;
	std::shared_ptr<bus_lock_if> bus_lock;

	PeripheralConnector() {
		tsock.register_b_transport(this, &PeripheralConnector::transport);
	}

	void transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay) {
		bus_lock->wait_until_unlocked();

		isock->b_transport(trans, delay);
	}
};


class BusLock : public bus_lock_if {
	bool locked = false;
	unsigned owner = 0;
	sc_core::sc_event lock_event;

public:
	virtual void lock(unsigned hart_id) override {
		if (locked && (hart_id != owner)) {
			wait_until_unlocked();
		}

		assert (!locked);
		locked = true;
		owner = hart_id;
	}

	virtual void unlock(unsigned hart_id) override {
		if (locked && (owner == hart_id)) {
		    locked = false;
            lock_event.notify(sc_core::SC_ZERO_TIME);
        }
	}

	virtual bool is_locked() override {
		return locked;
	}

	virtual bool is_locked(unsigned hart_id) override {
		return locked && (owner == hart_id);
	}

	virtual void wait_until_unlocked() override {
		while (locked)
			sc_core::wait(lock_event);
	}
};

#endif  // RISCV_ISA_BUS_H
