#ifndef MQNIC_IO_H
#define MQNIC_IO_H
#include <stdint.h>
#include <unistd.h>
#include "common/log.h"

// TODO: check endianness

// getters/setters for PCIe memory mapped registers
// this code looks like it's in need of some memory barrier intrinsics, but that's apparently not needed on x86
// dpdk has release/acquire memory order calls before/after the memory accesses, but they are defined as
// simple compiler barriers (i.e., the same empty asm with dependency on memory as here) on x86
// dpdk also defines an additional relaxed load/store for the registers that only uses a volatile access,  we skip that for simplicity

static inline void set_reg32(uint8_t* addr, int reg, uint32_t value) {
    __asm__ volatile ("" : : : "memory");
    *((volatile uint32_t*) (addr + reg)) = value;
}

static inline uint32_t get_reg32(const uint8_t* addr, int reg) {
    __asm__ volatile ("" : : : "memory");
    return *((volatile uint32_t*) (addr + reg));
}

static inline void set_flags32(uint8_t* addr, int reg, uint32_t flags) {
    set_reg32(addr, reg, get_reg32(addr, reg) | flags);
}

static inline void clear_flags32(uint8_t* addr, int reg, uint32_t flags) {
    set_reg32(addr, reg, get_reg32(addr, reg) & ~flags);
}

static inline void wait_clear_reg32(const uint8_t* addr, int reg, uint32_t mask) {
    __asm__ volatile ("" : : : "memory");
    uint32_t cur = 0;
    while (cur = *((volatile uint32_t*) (addr + reg)), (cur & mask) != 0) {
        debug("waiting for flags 0x%08X in register 0x%05X to clear, current value 0x%08X", mask, reg, cur);
        usleep(10000);
        __asm__ volatile ("" : : : "memory");
    }
}

static inline void wait_set_reg32(const uint8_t* addr, int reg, uint32_t mask) {
    __asm__ volatile ("" : : : "memory");
    uint32_t cur = 0;
    while (cur = *((volatile uint32_t*) (addr + reg)), (cur & mask) != mask) {
        debug("waiting for flags 0x%08X in register 0x%05X, current value 0x%08X", mask, reg, cur);
        usleep(10000);
        __asm__ volatile ("" : : : "memory");
    }
}

static inline uint32_t read_io32(int fd, size_t offset) {
    __asm__ volatile("" : : : "memory");
    uint32_t temp;
    if (pread(fd, &temp, sizeof(temp), offset) != sizeof(temp))
        log_error("pread io resource");
    return temp;
}


// Workaround to port kernel driver
static inline void iowrite32(uint32_t value, uint8_t* addr) {
    set_reg32(addr, 0, value);
}

static inline uint32_t ioread32(const uint8_t* addr) {
    return get_reg32(addr, 0);
}

#endif // MQNIC_IO_H
