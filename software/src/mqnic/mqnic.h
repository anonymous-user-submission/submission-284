#ifndef MQNIC_H
#define MQNIC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <endian.h>
#include <assert.h>
#include <vector>
#include <ctime>

#include "common/log.h"

#include "pci.h"
#include "mqnic_hw.h"
#include "memory.h"

// DPDK-related headers
#include <rte_eal.h>
#include <rte_errno.h>
#include <rte_lcore.h>
#include <rte_launch.h>
#include <rte_memzone.h>
#include <rte_mempool.h>
#include <rte_memory.h>
#include <rte_mbuf.h>
#include <rte_byteorder.h>
#include <rte_config.h>
#include <rte_malloc.h>
#include <rte_mbuf_core.h>

// #define min(x, y) ((x) < (y) ? (x) : (y))
#define min_t(type, x, y) ({            \
    type __min1 = (x);          \
    type __min2 = (y);          \
    __min1 < __min2 ? __min1: __min2; })
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
inline static int ilog2(unsigned long v)
{
    int l = 0;
    for (; v > 1; v >>= 1) l++;
    return l;
}
inline static unsigned long roundup_pow_of_two(unsigned long v) {
    unsigned long ret = 1 << ilog2(v);
    if (ret < v) ret <<= 1;
    assert(ret >= v);
    return ret;
}
inline static unsigned long rounddown_pow_of_two(unsigned long v) {
    unsigned long ret = 1 << ilog2(v);
    assert(ret <= v);
    return ret;
}
#define cpu_to_le64(x) htole64(x)
#define cpu_to_le32(x) htole32(x)
#define cpu_to_le16(x) htole16(x)
#define le16_to_cpu(x) le16toh(x)
#define le32_to_cpu(x) le32toh(x)
// TODO: replace with appropriate barriers
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, y) (x = y)

// DPDK mempool
#define MEMZONE_NAME "QINGNIAO_MEMZONE"

extern unsigned int mqnic_num_ev_queue_entries;
extern unsigned int mqnic_num_tx_queue_entries;
extern unsigned int mqnic_num_rx_queue_entries;

static const int NUM_RX_QUEUE_ENTRIES = 512;
static const int NUM_TX_QUEUE_ENTRIES = 512;
static const int PKT_BUF_ENTRY_SIZE = 4096; // > 4*MTU
static const int MIN_MEMPOOL_ENTRIES = 256;
static const int MTU_SIZE = 2048 - 14;

// TODO: Revisit typedefs later
typedef long long unsigned int resource_size_t;
typedef uintptr_t dma_addr_t; 

struct mqnic_reg_block {
    uint32_t type;
    uint32_t version;
    uint8_t *regs;
    uint8_t *base;
};

struct mqnic_sched {
    // struct device *dev;
    struct mqnic_if *interface;
    struct mqnic_sched_block *sched_block;

    struct mqnic_reg_block *rb;

    int index;

    uint32_t type;
    uint32_t offset;
    uint32_t channel_count;
    uint32_t channel_stride;

    uint8_t *hw_addr;
};

struct mqnic_sched_block {
    // struct device *dev;
    struct mqnic_if *interface;

    struct mqnic_reg_block *block_rb;
    struct mqnic_reg_block *rb_list;

    int index;

    uint32_t tx_queue_count;

    uint32_t sched_count;
    struct mqnic_sched *sched[MQNIC_MAX_PORTS];
};

struct mqnic_if {
    struct mqnic_dev *mdev;

    struct mqnic_reg_block *rb_list;
    struct mqnic_reg_block *if_ctrl_rb;
    struct mqnic_reg_block *event_queue_rb;
    struct mqnic_reg_block *tx_queue_rb;
    struct mqnic_reg_block *tx_cpl_queue_rb;
    struct mqnic_reg_block *rx_queue_rb;
    struct mqnic_reg_block *rx_cpl_queue_rb;
    struct mqnic_reg_block *rx_queue_map_rb;

    int index;

    int dev_port_base;
    int dev_port_max;
    int dev_port_limit;

    uint32_t if_features;

    uint32_t max_tx_mtu;
    uint32_t max_rx_mtu;

    uint32_t event_queue_offset;
    uint32_t event_queue_count;
    uint32_t event_queue_stride;

    uint32_t tx_queue_offset;
    uint32_t tx_queue_count;
    uint32_t tx_queue_stride;

    uint32_t tx_cpl_queue_offset;
    uint32_t tx_cpl_queue_count;
    uint32_t tx_cpl_queue_stride;

    uint32_t rx_queue_offset;
    uint32_t rx_queue_count;
    uint32_t rx_queue_stride;

    uint32_t rx_cpl_queue_offset;
    uint32_t rx_cpl_queue_count;
    uint32_t rx_cpl_queue_stride;

    uint32_t port_count;
    struct mqnic_port *port[MQNIC_MAX_PORTS];

    uint32_t sched_block_count;
    struct mqnic_sched_block *sched_block[MQNIC_MAX_PORTS];

    uint32_t max_desc_block_size;

    resource_size_t hw_regs_size;
    uint8_t *hw_addr;
    uint8_t *csr_hw_addr;

    //
    struct mqnic_ring *tx_ring;
    struct mqnic_ring *rx_ring;
};

struct mqnic_dev {
    const char* pci_addr;
    resource_size_t hw_regs_size;
    uint8_t *hw_addr;

    struct mqnic_reg_block *rb_list;
    struct mqnic_reg_block *fw_id_rb;
    struct mqnic_reg_block *if_rb;
    struct mqnic_reg_block *phc_rb;

    // stats related 
    struct mqnic_reg_block *stats_rb;
    uint32_t stats_offset;
    uint32_t stats_count;
    uint32_t stats_stride;
    uint32_t stats_flags;

    int dev_port_max;
    int dev_port_limit;

    uint32_t fpga_id;
    uint32_t fw_id;
    uint32_t fw_ver;
    uint32_t board_id;
    uint32_t board_ver;
    uint32_t build_date;
    uint32_t git_hash;
    uint32_t rel_info;

    uint32_t if_offset;
    uint32_t if_count;
    uint32_t if_stride;
    uint32_t if_csr_offset;

    struct mqnic_if *interface[MQNIC_MAX_IF];
};

struct mqnic_ring {
    uint8_t index;
    uint8_t active;
    uint32_t q_head_ptr;
    uint32_t q_tail_ptr;
    uint32_t q_clean_tail_ptr;

    uint32_t cpl_head_ptr;
    uint32_t cpl_tail_ptr;

    // mostly constant
    uint32_t size;
    uint32_t full_size;
    uint32_t size_mask;
    uint32_t stride;

    uint32_t desc_block_size;
    uint32_t log_desc_block_size;

    // desc 
    dma_addr_t *buf_dma_addr;
    volatile struct mqnic_desc *q_descs;

    // cpl desc
    volatile struct mqnic_cpl *cpl_descs;

    union {
        struct rte_mbuf **tx_info;
        struct rte_mbuf **rx_info;
    };

    uint32_t hw_ptr_mask;
    uint8_t *q_hw_addr;
    uint8_t *q_hw_head_ptr;
    uint8_t *q_hw_tail_ptr;

    uint8_t *cpl_hw_addr;
    uint8_t *cpl_hw_head_ptr;
    uint8_t *cpl_hw_tail_ptr;

    // for timestamp sync
    uint64_t ts_s;
    uint8_t  ts_valid;

    struct rte_mempool* mempool;
} ;

struct mqnic_port {
    // struct device *dev;
    struct mqnic_if *interface;

    struct mqnic_reg_block *port_rb;
    struct mqnic_reg_block *rb_list;
    struct mqnic_reg_block *port_ctrl_rb;

    int index;

    uint32_t port_features;
};

//
typedef int (*dev_init_mmap_t)(struct mqnic_dev *dev,
                                    const char *pci_addr);
typedef int (*dev_init_t)(struct mqnic_dev *dev);
typedef int (*dev_alloc_rings_t)(struct mqnic_dev *dev,
                                    const uint32_t qcnt);
typedef int (*dev_start_t)(struct mqnic_dev *dev,
                                    const uint32_t qcnt,
                                    const uint32_t rss_mask);


typedef uint32_t (*rx_burst_t)(struct mqnic_if *interface,
                                uint16_t queue_id,
                                struct rte_mbuf **rx_pkts,
                                uint32_t nb_pkts);
typedef uint32_t (*tx_burst_t)(struct mqnic_if *interface,
                                uint16_t queue_id,
                                struct rte_mbuf **tx_pkts,
                                uint32_t nb_pkts);

// generic device data structure
struct mqnic_eth_dev_ops {
    dev_init_mmap_t             dev_init_mmap;
    dev_init_t                  dev_init;
    dev_alloc_rings_t           dev_alloc_rings;
    dev_start_t                 dev_start;
};

struct mqnic_eth_dev {
    // driver ops for initialization
    const mqnic_eth_dev_ops*    dev_ops;
    // tx/rx on a specific queue
    rx_burst_t                  rx_pkts_burst;
    tx_burst_t                  tx_pkts_burst;

    uint32_t                    qcnt;
    struct mqnic_dev            *dev;
};

int mqnic_eth_dev_create(struct mqnic_eth_dev **dev, 
                            const char* pci_addr, 
                            const uint32_t qcnt, 
                            const uint32_t rss_mask);

int mqnic_create_interface(struct mqnic_dev *mdev, struct mqnic_if **interface_ptr,
        int index, uint8_t *hw_addr);

// mqnic_reg_block
struct mqnic_reg_block *mqnic_find_reg_block(struct mqnic_reg_block *list, uint32_t type, uint32_t version, int index);
struct mqnic_reg_block *mqnic_enumerate_reg_block_list(uint8_t /*__iomem*/ *base, size_t offset, size_t size);
void mqnic_free_reg_block_list(struct mqnic_reg_block *list);

// mqnic_port.c
int mqnic_create_port(struct mqnic_if *interface, struct mqnic_port **port_ptr,
        int index, struct mqnic_reg_block *port_rb);
int mqnic_start_port(struct mqnic_if *interface, const uint32_t qcnt, const uint32_t rss_mask);
void mqnic_destroy_port(struct mqnic_port **port_ptr);
uint32_t mqnic_port_get_tx_status(struct mqnic_port *port);
uint32_t mqnic_port_get_rx_status(struct mqnic_port *port);

// mqnic_sched_block.c
int mqnic_create_sched_block(struct mqnic_if *interface, struct mqnic_sched_block **block_ptr,
        int index, struct mqnic_reg_block *rb);
void mqnic_destroy_sched_block(struct mqnic_sched_block **block_ptr);
int mqnic_activate_sched_block(struct mqnic_sched_block *block);
void mqnic_deactivate_sched_block(struct mqnic_sched_block *block);

// mqnic_scheduler.c
int mqnic_create_scheduler(struct mqnic_sched_block *block, struct mqnic_sched **sched_ptr,
        int index, struct mqnic_reg_block *rb);
void mqnic_destroy_scheduler(struct mqnic_sched **sched_ptr);
int mqnic_scheduler_enable(struct mqnic_sched *sched);
void mqnic_scheduler_disable(struct mqnic_sched *sched);

// mqnic_tx.c
uint32_t mqnic_tx_batch(struct mqnic_if *interface, uint16_t queue_id, struct rte_mbuf* mbufs[], uint32_t num_bufs);
int mqnic_create_txqs (struct mqnic_if *interface, uint8_t *q_hw_addr, uint8_t *cpl_hw_addr);
int mqnic_alloc_txqs(struct mqnic_if *interface, const uint32_t qcnt, const int n_descs);
int mqnic_activate_txqs(struct mqnic_if *interface, const uint32_t qcnt);

// mqnic_rx.c
uint32_t mqnic_rx_batch(struct mqnic_if* interface, uint16_t queue_id, struct rte_mbuf* mbufs[], uint32_t num_bufs);
int mqnic_create_rxqs (struct mqnic_if *interface, uint8_t *q_hw_addr, uint8_t *cpl_hw_addr);
int mqnic_alloc_rxqs(struct mqnic_if *interface, const uint32_t qcnt, const int n_descs);
int mqnic_activate_rxqs(struct mqnic_if *interface, const uint32_t qcnt);

// mqnic_ptp.cpp
void start_sync_fpga_time(struct mqnic_dev *mqnic);
struct timespec mqnic_read_cpl_ts(struct mqnic_dev *mqnic, struct mqnic_ring *ring,
                const struct mqnic_cpl *cpl);
// mqnic_stats.cpp
uint64_t mqnic_stats_read(struct mqnic_dev *mqnic, int index);

#endif //MQNIC_H
