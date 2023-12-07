#include <rte_eal.h>
#include <rte_errno.h>
#include <rte_lcore.h>
#include <rte_launch.h>
#include <rte_memory.h>
#include <rte_memzone.h>
#include <rte_mempool.h>
#include <rte_ether.h>
#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_mbuf.h>

#include "common/app_thrd_info.h"
#include "mqnic.h"
#include "memory.h"
#include "io.h"
#include "mqnic_hw.h"

int mqnic_create_txqs (struct mqnic_if *interface, uint8_t *q_hw_addr, uint8_t *cpl_hw_addr) {

    assert (interface->tx_queue_count == interface->tx_cpl_queue_count);
    assert (interface->tx_queue_stride == interface->tx_cpl_queue_stride);

    interface->tx_ring = (struct mqnic_ring *)rte_zmalloc(
                                            NULL,
                                            sizeof(struct mqnic_ring) * interface->tx_queue_count, 0);
    info("%d", interface->rx_cpl_queue_count);
    if (NULL == interface->tx_ring) {
        log_error("rte_malloc error");
        return -ENOMEM;
    }

    for (uint16_t i=0; i<interface->tx_queue_count; i++) {
        struct mqnic_ring *txq = interface->tx_ring + i;

        txq->index = i;
        txq->active = 0;
        txq->hw_ptr_mask = 0xffff;

        // queue part
        txq->q_hw_addr = q_hw_addr + i*interface->tx_queue_stride;
        txq->q_hw_head_ptr = txq->q_hw_addr + MQNIC_QUEUE_HEAD_PTR_REG;
        txq->q_hw_tail_ptr = txq->q_hw_addr + MQNIC_QUEUE_TAIL_PTR_REG;

        txq->q_head_ptr = txq->q_tail_ptr = txq->q_clean_tail_ptr = 0;

        // completion part
        txq->cpl_hw_addr = cpl_hw_addr + i*interface->tx_cpl_queue_stride;
        txq->cpl_hw_head_ptr = txq->cpl_hw_addr + MQNIC_QUEUE_HEAD_PTR_REG;
        txq->cpl_hw_tail_ptr = txq->cpl_hw_addr + MQNIC_QUEUE_TAIL_PTR_REG;

        txq->cpl_head_ptr = txq->cpl_tail_ptr = 0;

        // deactivate queue
        set_reg32(txq->q_hw_addr, MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, 0);
        set_reg32(txq->cpl_hw_addr, MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, 0);
    }

    return 0;
}

int mqnic_alloc_txqs(struct mqnic_if *interface, const uint32_t qcnt, const int n_descs) {

    // 1 desc per block
    for (int i=0; i<qcnt; i++) {
        auto txq = interface->tx_ring + i;

        // queue part
        txq->size = roundup_pow_of_two(n_descs);
        txq->full_size = txq->size >> 1;
        txq->size_mask = txq->size - 1;
        txq->stride = roundup_pow_of_two(MQNIC_DESC_SIZE);

        txq->tx_info = (struct rte_mbuf **)rte_zmalloc(NULL, sizeof(struct rte_mbuf *) * txq->size, 0);
        if (NULL == txq->tx_info) {
            log_error("rte_malloc error");
            return -ENOMEM;
        }

        char mz_name[RTE_MEMZONE_NAMESIZE];
        snprintf(mz_name, sizeof(mz_name), "tx_desc_ring_%d", i);
        const struct rte_memzone *mz = NULL;
        mz = rte_memzone_reserve(mz_name, n_descs * MQNIC_DESC_SIZE, rte_socket_id(), RTE_MEMZONE_IOVA_CONTIG);

        if (NULL == mz) {
            log_error("rte_memzone_reserve error");
            return -ENOMEM;
        }

        txq->q_descs = (struct mqnic_desc *)(mz->addr);
        dma_addr_t dma_addr = mz->iova;

        // deactivate queue
        set_reg32(txq->q_hw_addr, MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, 0);
        // set base address
        set_reg32(txq->q_hw_addr, MQNIC_QUEUE_BASE_ADDR_REG + 0, (uint32_t) (dma_addr & 0xFFFFFFFFull)); // a change here: added the masking " & 0xFFFFFFFFull"
        set_reg32(txq->q_hw_addr, MQNIC_QUEUE_BASE_ADDR_REG + 4, (uint32_t) (dma_addr >> 32));
        // set completion queue index
        set_reg32(txq->q_hw_addr, MQNIC_QUEUE_CPL_QUEUE_INDEX_REG, 0);
        // set pointers
        set_reg32(txq->q_hw_addr, MQNIC_QUEUE_HEAD_PTR_REG, txq->q_head_ptr & txq->hw_ptr_mask);
        set_reg32(txq->q_hw_addr, MQNIC_QUEUE_TAIL_PTR_REG, txq->q_tail_ptr & txq->hw_ptr_mask);
        // set size
        set_reg32(txq->q_hw_addr, MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, ilog2(txq->size) | (0 << 8));


        // completion part
        snprintf(mz_name, sizeof(mz_name), "tx_cp_desc_ring_%d", i);
        const struct rte_memzone *cpl_mz = NULL;
        cpl_mz = rte_memzone_reserve(mz_name, n_descs * MQNIC_CPL_SIZE , rte_socket_id(), RTE_MEMZONE_IOVA_CONTIG);
        if (NULL == cpl_mz) {
            log_error("rte_memzone_reserve errror");
            return -1;
        }
        txq->cpl_descs = (struct mqnic_cpl *)(cpl_mz->addr);
        dma_addr = cpl_mz->iova;

        // deactivate queue
        set_reg32(txq->cpl_hw_addr, MQNIC_CPL_QUEUE_ACTIVE_LOG_SIZE_REG, 0);
        // set base address
        set_reg32(txq->cpl_hw_addr, MQNIC_CPL_QUEUE_BASE_ADDR_REG + 0, (uint32_t) (dma_addr & 0xFFFFFFFFull));
        set_reg32(txq->cpl_hw_addr, MQNIC_CPL_QUEUE_BASE_ADDR_REG + 4, (uint32_t) (dma_addr >> 32));
        // set interrupt index
        set_reg32(txq->cpl_hw_addr, MQNIC_CPL_QUEUE_INTERRUPT_INDEX_REG, 0);
        // set pointers
        set_reg32(txq->cpl_hw_addr, MQNIC_CPL_QUEUE_HEAD_PTR_REG, txq->cpl_head_ptr & txq->hw_ptr_mask);
        set_reg32(txq->cpl_hw_addr, MQNIC_CPL_QUEUE_TAIL_PTR_REG, txq->cpl_tail_ptr & txq->hw_ptr_mask);
        // set size
        set_reg32(txq->cpl_hw_addr, MQNIC_CPL_QUEUE_ACTIVE_LOG_SIZE_REG, ilog2(txq->size));
    }

    return 0;
}

int mqnic_activate_txqs(struct mqnic_if *interface, const uint32_t qcnt) {
    assert(qcnt <= interface->tx_queue_count);
    for (int i=0; i<qcnt; i++) {
        auto txq = interface->tx_ring + i;
        // deactivate
        iowrite32(ilog2(txq->size) | (0 << 8), txq->q_hw_addr + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG);
        txq->active = 0;

        //
        iowrite32(0, txq->q_hw_addr + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG);
        iowrite32(i, txq->q_hw_addr + MQNIC_QUEUE_CPL_QUEUE_INDEX_REG);

        iowrite32(ilog2(txq->size) | (0 << 8) | MQNIC_QUEUE_ACTIVE_MASK,
            txq->q_hw_addr + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG);
        iowrite32(ilog2(txq->size) | MQNIC_CPL_QUEUE_ACTIVE_MASK,
            txq->cpl_hw_addr + MQNIC_CPL_QUEUE_ACTIVE_LOG_SIZE_REG);

        txq->active = 1;

        // arm
        set_reg32(txq->cpl_hw_addr, MQNIC_CPL_QUEUE_INTERRUPT_INDEX_REG, i | MQNIC_CPL_QUEUE_ARM_MASK);
    }
    return 0;
}


bool mqnic_is_tx_ring_empty(const struct mqnic_ring *ring)
{
    return ring->q_head_ptr == ring->q_clean_tail_ptr;
}

bool mqnic_is_tx_ring_full(const struct mqnic_ring *ring)
{
    return ring->q_head_ptr - ring->q_clean_tail_ptr >= ring->full_size;
}

void mqnic_tx_read_tail_ptr(struct mqnic_ring *ring)
{
    ring->q_tail_ptr += (ioread32(ring->q_hw_tail_ptr) - ring->q_tail_ptr) & ring->hw_ptr_mask;
}

void mqnic_tx_write_head_ptr(struct mqnic_ring *ring)
{
    iowrite32(ring->q_head_ptr & ring->hw_ptr_mask, ring->q_hw_head_ptr);
}

static void mqnic_cq_write_tail_ptr(struct mqnic_ring *ring)
{
    set_reg32(ring->cpl_hw_tail_ptr, 0, ring->cpl_tail_ptr & ring->hw_ptr_mask);
}

static inline int mqnic_process_tx_cq(struct mqnic_ring *tx_ring, uint32_t budget)
{
    volatile struct mqnic_cpl *cpl = NULL;

    // process completion queue
    uint32_t cq_tail_ptr = tx_ring->cpl_tail_ptr;
    uint32_t cq_index = cq_tail_ptr & tx_ring->size_mask;
    uint32_t done = 0;

    cpl = (struct mqnic_cpl *)(tx_ring->cpl_descs + cq_index);

    // while (cq_ring->head_ptr != cq_tail_ptr/* && done < budget*/) {
    while (cpl->len && done<budget) {
        // NOTE: we do not need this rsvd0 for QingNiao anymore
        // uint32_t index = le16_to_cpu(cpl->rsvd0) & tx_ring->size_mask;
        // uint32_t index = le16_to_cpu(cpl->index) & tx_ring->size_mask;
        uint32_t index = cpl->index & tx_ring->size_mask;
        struct rte_mbuf **tx_mbuf = &tx_ring->tx_info[index];

        rte_pktmbuf_free_seg(*tx_mbuf);
        *tx_mbuf = NULL;
        cpl->len = 0;

        done++;
        cq_tail_ptr++;
        cq_index = cq_tail_ptr & tx_ring->size_mask;

        //
        cpl = (struct mqnic_cpl *)(tx_ring->cpl_descs + cq_index);
    }

    // update CQ tail
    if (done > 0) {
        tx_ring->cpl_tail_ptr = cq_tail_ptr;
        mqnic_cq_write_tail_ptr(tx_ring);
    }

    tx_ring->q_tail_ptr = tx_ring->q_tail_ptr + done;

    uint32_t ring_clean_tail_ptr = READ_ONCE(tx_ring->q_clean_tail_ptr);
    uint32_t index = ring_clean_tail_ptr & tx_ring->size_mask;

    while (ring_clean_tail_ptr != tx_ring->q_tail_ptr) {
        if (tx_ring->tx_info[index])
            break;

        ring_clean_tail_ptr++;
        index = ring_clean_tail_ptr & tx_ring->size_mask;
    }

    // update ring tail
    WRITE_ONCE(tx_ring->q_clean_tail_ptr, ring_clean_tail_ptr);

    return done;
}

#define TXCQ_BUDGET 32

uint32_t mqnic_tx_batch(struct mqnic_if *interface, uint16_t queue_id, struct rte_mbuf* mbufs[], uint32_t num_bufs)
{
    uint32_t sent, index;
    struct mqnic_ring* ring = interface->tx_ring + queue_id;
    volatile struct mqnic_desc* tx_desc;

    // reclaim tx mbufs
    mqnic_process_tx_cq(ring, TXCQ_BUDGET);

    // send as many packets as possible
    for (sent = 0; sent < num_bufs; sent++) {
        if (mqnic_is_tx_ring_full(ring)) {
            break;
        }
        struct rte_mbuf* mbuf = mbufs[sent];
        index = ring->q_head_ptr & ring->size_mask;
        tx_desc = (struct mqnic_desc *)(ring->q_descs + index);
        // TODO: currently no checksum offload
        tx_desc[0].tx_csum_cmd = 0;
        // tx_desc[0].addr = cpu_to_le64(rte_mbuf_data_iova(mbuf));
        // tx_desc[0].len = cpu_to_le32(mbuf->data_len);
        tx_desc[0].addr = rte_mbuf_data_iova(mbuf);
        tx_desc[0].len = mbuf->data_len;
        // NOTE: we do not need this rsvd0 for QingNiao anymore
        // tx_desc[0].rsvd0 = cpu_to_le16(index);
        ring->tx_info[index] = mbuf;
        ring->q_head_ptr++;
    }
    if (sent > 0)
        mqnic_tx_write_head_ptr(ring);
    return sent;
}
