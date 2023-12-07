#include <assert.h>
#include <cstdint>
#include <vector>
#include <ctime>

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
#include "io.h"
#include "memory.h"



int mqnic_create_rxqs (struct mqnic_if *interface, uint8_t *q_hw_addr, uint8_t *cpl_hw_addr) {

    assert (interface->rx_queue_count == interface->rx_cpl_queue_count);
    assert (interface->rx_queue_stride == interface->rx_cpl_queue_stride);

    interface->rx_ring = (struct mqnic_ring *)rte_zmalloc(
                                            NULL,
                                            sizeof(struct mqnic_ring) * interface->rx_queue_count, 0);
    if (NULL == interface->rx_ring) {
        log_error("rte_malloc error");
        return -ENOMEM;
    }

    for (int i=0; i<interface->rx_queue_count; i++) {
        auto rxq = interface->rx_ring + i;

        rxq->index = i;
        rxq->active = 0;
        rxq->hw_ptr_mask = 0xffff;

        // queue part
        rxq->q_hw_addr = q_hw_addr + i*interface->rx_queue_stride;
        rxq->q_hw_head_ptr = rxq->q_hw_addr + MQNIC_QUEUE_HEAD_PTR_REG;
        rxq->q_hw_tail_ptr = rxq->q_hw_addr + MQNIC_QUEUE_TAIL_PTR_REG;

        rxq->q_head_ptr = rxq->q_tail_ptr = rxq->q_clean_tail_ptr = 0;

        // completion part
        rxq->cpl_hw_addr = cpl_hw_addr + i*interface->rx_cpl_queue_stride;
        rxq->cpl_hw_head_ptr = rxq->cpl_hw_addr + MQNIC_QUEUE_HEAD_PTR_REG;
        rxq->cpl_hw_tail_ptr = rxq->cpl_hw_addr + MQNIC_QUEUE_TAIL_PTR_REG;

        rxq->cpl_head_ptr = rxq->cpl_tail_ptr = 0;

        // deactivate queue
        set_reg32(rxq->q_hw_addr, MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, 0);
        set_reg32(rxq->cpl_hw_addr, MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, 0);
    }

    return 0;
}

int mqnic_alloc_rxqs(struct mqnic_if *interface, const uint32_t qcnt, const int n_descs) {

    // 1 desc per block
    for (int i=0; i<qcnt; i++) {
        auto rxq = interface->rx_ring + i;

        // queue part
        rxq->size = roundup_pow_of_two(n_descs);
        rxq->full_size = rxq->size >> 1;
        rxq->size_mask = rxq->size - 1;
        rxq->stride = roundup_pow_of_two(MQNIC_DESC_SIZE);

        rxq->rx_info = (struct rte_mbuf **)rte_zmalloc(NULL, sizeof(struct rte_mbuf *) * rxq->size, RTE_CACHE_LINE_SIZE);
        if (NULL == rxq->rx_info) {
            log_error("rte_malloc error");
            return -ENOMEM;
        }

        char mz_name[RTE_MEMZONE_NAMESIZE];
        snprintf(mz_name, sizeof(mz_name), "rx_desc_ring_%d", i);
        const struct rte_memzone *mz = NULL;
        mz = rte_memzone_reserve(mz_name, n_descs * MQNIC_DESC_SIZE, rte_socket_id(), RTE_MEMZONE_IOVA_CONTIG);

        if (NULL == mz) {
            log_error("rte_memzone_reserve error");
            return -ENOMEM;
        }

        rxq->q_descs = (struct mqnic_desc *)(mz->addr);
        dma_addr_t dma_addr = mz->iova;

        // deactivate queue
        set_reg32(rxq->q_hw_addr, MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, 0);
        // set base address
        set_reg32(rxq->q_hw_addr, MQNIC_QUEUE_BASE_ADDR_REG + 0, (uint32_t) (dma_addr & 0xFFFFFFFFull)); // a change here: added the masking " & 0xFFFFFFFFull"
        set_reg32(rxq->q_hw_addr, MQNIC_QUEUE_BASE_ADDR_REG + 4, (uint32_t) (dma_addr >> 32));
        // set completion queue index
        set_reg32(rxq->q_hw_addr, MQNIC_QUEUE_CPL_QUEUE_INDEX_REG, 0);
        // set pointers
        set_reg32(rxq->q_hw_addr, MQNIC_QUEUE_HEAD_PTR_REG, rxq->q_head_ptr & rxq->hw_ptr_mask);
        set_reg32(rxq->q_hw_addr, MQNIC_QUEUE_TAIL_PTR_REG, rxq->q_tail_ptr & rxq->hw_ptr_mask);
        // set size
        set_reg32(rxq->q_hw_addr, MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, ilog2(rxq->size) | (0 << 8));

        // RX mempool
        char mp_name[RTE_MEMZONE_NAMESIZE];
        snprintf(mp_name, sizeof(mp_name), "ring_mempool_%d", i);
        // TODO: we should have large enough space
        int mempool_size = NUM_RX_QUEUE_ENTRIES * 8 + NUM_TX_QUEUE_ENTRIES * 8;
        rxq->mempool = rte_pktmbuf_pool_create(mp_name,
                                            mempool_size < MIN_MEMPOOL_ENTRIES ? MIN_MEMPOOL_ENTRIES : mempool_size,
                                            256,
                                            0,
                                            PKT_BUF_ENTRY_SIZE + RTE_PKTMBUF_HEADROOM,
                                            rte_socket_id());
        if (NULL==rxq->mempool) {
            log_error("rte_pktmbuf_pool_create error");
            return -1;
        }

        info("rx memp %p", rxq->mempool);

        // completion part
        snprintf(mz_name, sizeof(mz_name), "rx_cp_desc_ring_%d", i);
        const struct rte_memzone *cpl_mz = NULL;
        cpl_mz = rte_memzone_reserve(mz_name, n_descs * MQNIC_CPL_SIZE , rte_socket_id(), RTE_MEMZONE_IOVA_CONTIG);
        if (NULL == cpl_mz) {
            log_error("rte_memzone_reserve errror");
            return -1;
        }
        rxq->cpl_descs = (struct mqnic_cpl *)(cpl_mz->addr);
        dma_addr = cpl_mz->iova;

        // deactivate queue
        set_reg32(rxq->cpl_hw_addr, MQNIC_CPL_QUEUE_ACTIVE_LOG_SIZE_REG, 0);
        // set base address
        set_reg32(rxq->cpl_hw_addr, MQNIC_CPL_QUEUE_BASE_ADDR_REG + 0, (uint32_t) (dma_addr & 0xFFFFFFFFull));
        set_reg32(rxq->cpl_hw_addr, MQNIC_CPL_QUEUE_BASE_ADDR_REG + 4, (uint32_t) (dma_addr >> 32));
        // set interrupt index
        set_reg32(rxq->cpl_hw_addr, MQNIC_CPL_QUEUE_INTERRUPT_INDEX_REG, 0);
        // set pointers
        set_reg32(rxq->cpl_hw_addr, MQNIC_CPL_QUEUE_HEAD_PTR_REG, rxq->cpl_head_ptr & rxq->hw_ptr_mask);
        set_reg32(rxq->cpl_hw_addr, MQNIC_CPL_QUEUE_TAIL_PTR_REG, rxq->cpl_tail_ptr & rxq->hw_ptr_mask);
        // set size
        set_reg32(rxq->cpl_hw_addr, MQNIC_CPL_QUEUE_ACTIVE_LOG_SIZE_REG, ilog2(rxq->size));
    }

    return 0;
}

void mqnic_refill_rx_buffers(struct mqnic_ring *ring);

int mqnic_activate_rxqs(struct mqnic_if *interface, const uint32_t qcnt) {
    assert(qcnt <= interface->rx_queue_count);
    for (int i=0; i<qcnt; i++) {
        auto rxq = interface->rx_ring + i;
        // deactivate
        iowrite32(ilog2(rxq->size) | (0 << 8), rxq->q_hw_addr + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG);
        rxq->active = 0;

        //
        iowrite32(0, rxq->q_hw_addr + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG);
        iowrite32(i, rxq->q_hw_addr + MQNIC_QUEUE_CPL_QUEUE_INDEX_REG);

        iowrite32(ilog2(rxq->size) | (0 << 8) | MQNIC_QUEUE_ACTIVE_MASK,
            rxq->q_hw_addr + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG);
        iowrite32(ilog2(rxq->size) | MQNIC_CPL_QUEUE_ACTIVE_MASK,
            rxq->cpl_hw_addr + MQNIC_CPL_QUEUE_ACTIVE_LOG_SIZE_REG);

        rxq->active = 1;

        mqnic_refill_rx_buffers(rxq);
        // arm
        set_reg32(rxq->cpl_hw_addr, MQNIC_CPL_QUEUE_INTERRUPT_INDEX_REG, i | MQNIC_CPL_QUEUE_ARM_MASK);

    }
    return 0;
}


bool mqnic_is_rx_ring_empty(const struct mqnic_ring *ring)
{
    return ring->q_head_ptr == ring->q_clean_tail_ptr;
}

bool mqnic_is_rx_ring_full(const struct mqnic_ring *ring)
{
    return ring->q_head_ptr - ring->q_clean_tail_ptr >= ring->size;
}

void mqnic_rx_read_tail_ptr(struct mqnic_ring *ring)
{
    ring->q_tail_ptr += (ioread32(ring->q_hw_tail_ptr) - ring->q_tail_ptr) & ring->hw_ptr_mask;
}

void mqnic_rx_write_head_ptr(struct mqnic_ring *ring)
{
    iowrite32(ring->q_head_ptr & ring->hw_ptr_mask, ring->q_hw_head_ptr);
}

static void mqnic_cq_write_tail_ptr(struct mqnic_ring *ring)
{
    set_reg32(ring->cpl_hw_tail_ptr, 0, ring->cpl_tail_ptr & ring->hw_ptr_mask);
}

int mqnic_prepare_rx_desc(struct mqnic_ring *ring, int index, struct rte_mbuf *mbuf)
{
    struct rte_mbuf **rx_mbuf = &ring->rx_info[index];
    volatile struct mqnic_desc *rx_desc = (struct mqnic_desc *)(ring->q_descs + index);
    // rx_desc->addr = cpu_to_le64(rte_mbuf_data_iova_default(mbuf)); // dma_addr
    // rx_desc->len = cpu_to_le32(ring->mempool->elt_size - RTE_PKTMBUF_HEADROOM);
    rx_desc->addr = rte_mbuf_data_iova_default(mbuf);
    rx_desc->len = ring->mempool->elt_size - RTE_PKTMBUF_HEADROOM;
    *rx_mbuf = mbuf;
    return 0;
}

void mqnic_refill_rx_buffers(struct mqnic_ring *ring)
{
    uint32_t missing = ring->size - (ring->q_head_ptr - ring->q_clean_tail_ptr);

    if (missing < 8)
        return;

    for (; missing-- > 0;) {
        auto nmb = rte_mbuf_raw_alloc(ring->mempool);
        if (NULL != nmb) {
            nmb->data_off = RTE_PKTMBUF_HEADROOM;
            mqnic_prepare_rx_desc(ring, ring->q_head_ptr & ring->size_mask, nmb);
            ring->q_head_ptr++;
        }
        else {
            break;
        }
    }

    // update NIC header ptr
    mqnic_rx_write_head_ptr(ring);
}

#define RXCQ_BATCH_SIZE 8
#define INVALID_CPL 0xffffffff

uint32_t mqnic_rx_batch(struct mqnic_if* interface, 
                            uint16_t queue_id, 
                            struct rte_mbuf* mbufs[], 
                            uint32_t num_bufs)
{
    struct mqnic_ring *ring = interface->rx_ring + queue_id;

    uint32_t cq_tail_ptr = ring->cpl_tail_ptr;
    uint32_t cq_index = cq_tail_ptr & ring->size_mask;
    uint32_t cq_next_index = 0;
    uint32_t done = 0;
    uint32_t next_batch = 0;
    bool if_next = false;

    volatile struct mqnic_cpl *cpl = NULL;
    volatile struct mqnic_cpl *next_cpl = NULL;

    cq_next_index = (cq_index + RXCQ_BATCH_SIZE) & ring->size_mask;
    next_cpl = (struct mqnic_cpl *)(ring->cpl_descs + cq_next_index);
    if_next = next_cpl->len != 0;
    next_batch = RXCQ_BATCH_SIZE;
    while (if_next && done < num_bufs) {

        cpl = (struct mqnic_cpl *)(ring->cpl_descs + cq_index);
        // uint32_t index = le16_to_cpu(cpl->index) & ring->size_mask;
        uint32_t index = cpl->index & ring->size_mask;

        // if (le32_to_cpu(cpl->rsvd5) != INVALID_CPL) {
        if (cpl->rsvd5 != INVALID_CPL) {
            //
            auto mbuf = ring->rx_info[index];
            mbufs[done] = mbuf;
            // TODO: size;
            mbufs[done]->data_off = RTE_PKTMBUF_HEADROOM;
            // mbufs[done]->pkt_len = le16_to_cpu(cpl->len); //, bufs[done]->size);
            // mbufs[done]->data_len = le16_to_cpu(cpl->len); //, bufs[done]->size);
            mbufs[done]->pkt_len = cpl->len; //, bufs[done]->size);
            mbufs[done]->data_len = cpl->len; //, bufs[done]->size);
            mbufs[done]->nb_segs = 1;
            mbufs[done]->next = NULL;
            done++;
        }
        else {
            info("received timeout cpl");
        }

        ring->rx_info[index] = NULL;
        cpl->len = 0;
        cq_tail_ptr++;
        cq_index = cq_tail_ptr & ring->size_mask;

        if (next_batch != 0) {
            if_next = true;
            next_batch--;
        }
        else {
            cq_next_index = (cq_index + RXCQ_BATCH_SIZE) & ring->size_mask;
            next_cpl = (struct mqnic_cpl *)(ring->cpl_descs + cq_next_index);
            if_next = next_cpl->len != 0;
            next_batch = RXCQ_BATCH_SIZE;
        }
    }
    // update CQ tail
    if (done > 0) {
        ring->cpl_tail_ptr = cq_tail_ptr;
        mqnic_cq_write_tail_ptr(ring);
    }

    ring->q_tail_ptr = (done + ring->q_tail_ptr) & ring->hw_ptr_mask;

    uint32_t ring_clean_tail_ptr = READ_ONCE(ring->q_clean_tail_ptr);
    uint32_t index = ring_clean_tail_ptr & ring->size_mask;

    while (ring_clean_tail_ptr != ring->q_tail_ptr) {
        if (ring->rx_info[index])
            break;

        ring_clean_tail_ptr++;
        index = ring_clean_tail_ptr & ring->size_mask;
    }

    // update ring tail
    WRITE_ONCE(ring->q_clean_tail_ptr, ring_clean_tail_ptr);

    // replenish buffers
    mqnic_refill_rx_buffers(ring);

    return done;
}
