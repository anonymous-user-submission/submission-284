#include "mqnic.h"
#include "io.h"
#include "mqnic_hw.h"
#include <linux/if_ether.h>
#define MAX_DESC_BLOCK_SIZE 1

uint32_t mqnic_interface_get_tx_mtu(struct mqnic_if *interface)
{
    return ioread32(interface->if_ctrl_rb->regs + MQNIC_RB_IF_CTRL_REG_TX_MTU);
}

void mqnic_interface_set_tx_mtu(struct mqnic_if *interface, uint32_t mtu)
{
    iowrite32(mtu, interface->if_ctrl_rb->regs + MQNIC_RB_IF_CTRL_REG_TX_MTU);
}

uint32_t mqnic_interface_get_rx_mtu(struct mqnic_if *interface)
{
    return ioread32(interface->if_ctrl_rb->regs + MQNIC_RB_IF_CTRL_REG_RX_MTU);
}

void mqnic_interface_set_rx_mtu(struct mqnic_if *interface, uint32_t mtu)
{
    iowrite32(mtu, interface->if_ctrl_rb->regs + MQNIC_RB_IF_CTRL_REG_RX_MTU);
}

uint32_t mqnic_interface_get_rx_queue_map_offset(struct mqnic_if *interface, int port)
{
    return ioread32(interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_OFFSET);
}

void mqnic_interface_set_rx_queue_map_offset(struct mqnic_if *interface, int port, uint32_t val)
{
    iowrite32(val, interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_OFFSET);
}

uint32_t mqnic_interface_get_rx_queue_map_rss_mask(struct mqnic_if *interface, int port)
{
    return ioread32(interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_RSS_MASK);
}

void mqnic_interface_set_rx_queue_map_rss_mask(struct mqnic_if *interface, int port, uint32_t val)
{
    iowrite32(val, interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_RSS_MASK);
}

uint32_t mqnic_interface_get_rx_queue_map_app_mask(struct mqnic_if *interface, int port)
{
    return ioread32(interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_APP_MASK);
}

void mqnic_interface_set_rx_queue_map_app_mask(struct mqnic_if *interface, int port, uint32_t val)
{
    iowrite32(val, interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_APP_MASK);
}

uint32_t mqnic_interface_get_rx_queue_map_ing_enable(struct mqnic_if *interface, int port)
{
    return ioread32(interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_ING_ENABLE);
}

void mqnic_interface_set_rx_queue_map_ing_enable(struct mqnic_if *interface, int port, int32_t val)
{
    iowrite32(val, interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_ING_ENABLE);
}

uint32_t mqnic_interface_get_rx_queue_map_rxcpl_timeout(struct mqnic_if *interface, int port)
{
    return ioread32(interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_RXCPL_HARD_TIMEOUT_IN_US);
}

void mqnic_interface_set_rx_queue_map_rxcpl_timeout(struct mqnic_if *interface, int port, int32_t val)
{
    iowrite32(val, interface->rx_queue_map_rb->regs + MQNIC_RB_RX_QUEUE_MAP_CH_OFFSET +
            MQNIC_RB_RX_QUEUE_MAP_CH_STRIDE*port + MQNIC_RB_RX_QUEUE_MAP_CH_REG_RXCPL_HARD_TIMEOUT_IN_US);
}

int mqnic_create_interface(struct mqnic_dev *mdev, struct mqnic_if **interface_ptr,
        int index, uint8_t *hw_addr)
{
    struct mqnic_if *interface;
    struct mqnic_reg_block *rb;
    int ret = 0;
    int k;

    interface = (struct mqnic_if *)rte_malloc(NULL, sizeof(*interface), 0);
    if (!interface) {
        log_error("malloc failed");
    }

    *interface_ptr = interface;

    interface->mdev = mdev;

    interface->index = index;

    interface->hw_regs_size = mdev->if_stride;
    interface->hw_addr = hw_addr;
    interface->csr_hw_addr = hw_addr + mdev->if_csr_offset;

    // Enumerate registers
    interface->rb_list = mqnic_enumerate_reg_block_list(interface->hw_addr, mdev->if_csr_offset, interface->hw_regs_size);
    if (!interface->rb_list) {
        log_error("Failed to enumerate blocks");
    }

    info("Interface-level register blocks:");
    for (rb = interface->rb_list; rb->regs; rb++)
        info(" type 0x%08x (v %d.%d.%d.%d)", rb->type, rb->version >> 24,
                (rb->version >> 16) & 0xff, (rb->version >> 8) & 0xff, rb->version & 0xff);

    interface->if_ctrl_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_IF_CTRL_TYPE, MQNIC_RB_IF_CTRL_VER, 0);

    if (!interface->if_ctrl_rb) {
        log_error("Interface control block not found");
    }

    interface->if_features = get_reg32(interface->if_ctrl_rb->regs, MQNIC_RB_IF_CTRL_REG_FEATURES);
    interface->port_count = get_reg32(interface->if_ctrl_rb->regs, MQNIC_RB_IF_CTRL_REG_PORT_COUNT);
    interface->sched_block_count = get_reg32(interface->if_ctrl_rb->regs, MQNIC_RB_IF_CTRL_REG_SCHED_COUNT);
    interface->max_tx_mtu = get_reg32(interface->if_ctrl_rb->regs, MQNIC_RB_IF_CTRL_REG_MAX_TX_MTU);
    interface->max_rx_mtu = get_reg32(interface->if_ctrl_rb->regs, MQNIC_RB_IF_CTRL_REG_MAX_RX_MTU);

    info("IF features: 0x%08x", interface->if_features);
    info("Port count: %d", interface->port_count);
    info("Scheduler block count: %d", interface->sched_block_count);
    info("Max TX MTU: %d", interface->max_tx_mtu);
    info("Max RX MTU: %d", interface->max_rx_mtu);

    // TODO: consolidate steps 1-5 into one shared code block
    /************ 1. event_queue *******************************/
    interface->event_queue_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_EVENT_QM_TYPE, MQNIC_RB_EVENT_QM_VER, 0);

    if (!interface->event_queue_rb) {
        log_error("Event queue block not found");
    }

    interface->event_queue_offset = get_reg32(interface->event_queue_rb->regs, MQNIC_RB_EVENT_QM_REG_OFFSET);
    interface->event_queue_count = get_reg32(interface->event_queue_rb->regs, MQNIC_RB_EVENT_QM_REG_COUNT);
    interface->event_queue_stride = get_reg32(interface->event_queue_rb->regs, MQNIC_RB_EVENT_QM_REG_STRIDE);

    info("Event queue offset: 0x%08x", interface->event_queue_offset);
    info("Event queue count: %d", interface->event_queue_count);
    info("Event queue stride: 0x%08x", interface->event_queue_stride);

    interface->event_queue_count = min_t(uint32_t, interface->event_queue_count, MQNIC_MAX_EVENT_RINGS);

    /************ 2. tx_queue *******************************/
    interface->tx_queue_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_TX_QM_TYPE, MQNIC_RB_TX_QM_VER, 0);

    if (!interface->tx_queue_rb) {
        log_error("TX queue block not found");
    }

    interface->tx_queue_offset = get_reg32(interface->tx_queue_rb->regs, MQNIC_RB_TX_QM_REG_OFFSET);
    interface->tx_queue_count = get_reg32(interface->tx_queue_rb->regs, MQNIC_RB_TX_QM_REG_COUNT);
    interface->tx_queue_stride = get_reg32(interface->tx_queue_rb->regs, MQNIC_RB_TX_QM_REG_STRIDE);

    info("TX queue offset: 0x%08x", interface->tx_queue_offset);
    info("TX queue count: %d", interface->tx_queue_count);
    info("TX queue stride: 0x%08x", interface->tx_queue_stride);

    interface->tx_queue_count = min_t(uint32_t, interface->tx_queue_count, MQNIC_MAX_TX_RINGS);

    /************ 3. tx_cpl_queue *******************************/
    interface->tx_cpl_queue_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_TX_CQM_TYPE, MQNIC_RB_TX_CQM_VER, 0);

    if (!interface->tx_cpl_queue_rb) {
        log_error("TX completion queue block not found");
    }

    interface->tx_cpl_queue_offset = get_reg32(interface->tx_cpl_queue_rb->regs, MQNIC_RB_TX_CQM_REG_OFFSET);
    interface->tx_cpl_queue_count = get_reg32(interface->tx_cpl_queue_rb->regs, MQNIC_RB_TX_CQM_REG_COUNT);
    interface->tx_cpl_queue_stride = get_reg32(interface->tx_cpl_queue_rb->regs, MQNIC_RB_TX_CQM_REG_STRIDE);

    info("TX completion queue offset: 0x%08x", interface->tx_cpl_queue_offset);
    info("TX completion queue count: %d", interface->tx_cpl_queue_count);
    info("TX completion queue stride: 0x%08x", interface->tx_cpl_queue_stride);

    interface->tx_cpl_queue_count = min_t(uint32_t, interface->tx_cpl_queue_count, MQNIC_MAX_TX_CPL_RINGS);

    /************ 4. rx_queue *******************************/
    interface->rx_queue_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_RX_QM_TYPE, MQNIC_RB_RX_QM_VER, 0);

    if (!interface->rx_queue_rb) {
        log_error("RX queue block not found");
    }

    interface->rx_queue_offset = get_reg32(interface->rx_queue_rb->regs, MQNIC_RB_RX_QM_REG_OFFSET);
    interface->rx_queue_count = get_reg32(interface->rx_queue_rb->regs, MQNIC_RB_RX_QM_REG_COUNT);
    interface->rx_queue_stride = get_reg32(interface->rx_queue_rb->regs, MQNIC_RB_RX_QM_REG_STRIDE);


    interface->rx_queue_count = min_t(uint32_t, interface->rx_queue_count, MQNIC_MAX_RX_RINGS);
    info("RX queue offset: 0x%08x", interface->rx_queue_offset);
    info("RX queue count: %d", interface->rx_queue_count);
    info("RX queue stride: 0x%08x", interface->rx_queue_stride);

    /************ 5. rx_cpl_queue *******************************/
    interface->rx_cpl_queue_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_RX_CQM_TYPE, MQNIC_RB_RX_CQM_VER, 0);

    if (!interface->rx_cpl_queue_rb) {
        log_error("RX completion queue block not found");
    }

    interface->rx_cpl_queue_offset = get_reg32(interface->rx_cpl_queue_rb->regs, MQNIC_RB_RX_CQM_REG_OFFSET);
    interface->rx_cpl_queue_count = get_reg32(interface->rx_cpl_queue_rb->regs, MQNIC_RB_RX_CQM_REG_COUNT);
    interface->rx_cpl_queue_stride = get_reg32(interface->rx_cpl_queue_rb->regs, MQNIC_RB_RX_CQM_REG_STRIDE);


    interface->rx_cpl_queue_count = min_t(uint32_t, interface->rx_cpl_queue_count, MQNIC_MAX_RX_CPL_RINGS);
    info("RX completion queue offset: 0x%08x", interface->rx_cpl_queue_offset);
    info("RX completion queue count: %d", interface->rx_cpl_queue_count);
    info("RX completion queue stride: 0x%08x", interface->rx_cpl_queue_stride);
    /************ queues end *******************************/

    interface->rx_queue_map_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_RX_QUEUE_MAP_TYPE, MQNIC_RB_RX_QUEUE_MAP_VER, 0);

    if (!interface->rx_queue_map_rb) {
        log_error("RX queue map block not found");
    }

    for (k = 0; k < interface->port_count; k++) {
        mqnic_interface_set_rx_queue_map_offset(interface, k, 0);
        mqnic_interface_set_rx_queue_map_rss_mask(interface, k, 0);
        mqnic_interface_set_rx_queue_map_app_mask(interface, k, 0);
#ifdef QINGNIAO
        mqnic_interface_set_rx_queue_map_ing_enable(interface, k, 0);
#endif
    }

    // determine desc block size
    set_reg32(hw_addr, interface->tx_queue_offset + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, 0xf << 8);
    interface->max_desc_block_size = 1 << ((get_reg32(hw_addr, interface->tx_queue_offset + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG) >> 8) & 0xf);
    set_reg32(hw_addr, interface->tx_queue_offset + MQNIC_QUEUE_ACTIVE_LOG_SIZE_REG, 0);

    info("Max desc block size: %d", interface->max_desc_block_size);

    interface->max_desc_block_size = min_t(uint32_t, interface->max_desc_block_size, MQNIC_MAX_FRAGS);

    // create tx/rx rings
    ret = mqnic_create_txqs(interface, hw_addr + interface->tx_queue_offset, hw_addr + interface->tx_cpl_queue_offset);
    if (ret < 0) {
        log_error("create txqs error");
        goto fail;
    }

    ret = mqnic_create_rxqs(interface, hw_addr + interface->rx_queue_offset, hw_addr + interface->rx_cpl_queue_offset);
    if (ret < 0) {
        log_error("create rxqs error");
        goto fail;
    }

    // create ports
    for (k = 0; k < interface->port_count; k++) {
        struct mqnic_reg_block *port_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_PORT_TYPE, MQNIC_RB_PORT_VER, k);

        if (!port_rb) {
            ret = -EIO;
            info("Port index %d not found", k);
            goto fail;
        }

        ret = mqnic_create_port(interface, &interface->port[k], k, port_rb);
        if (ret)
            goto fail;
    }

    // create schedulers
    for (k = 0; k < interface->sched_block_count; k++) {
        struct mqnic_reg_block *sched_block_rb = mqnic_find_reg_block(interface->rb_list, MQNIC_RB_SCHED_BLOCK_TYPE, MQNIC_RB_SCHED_BLOCK_VER, k);

        if (!sched_block_rb) {
            ret = -EIO;
            info("Scheduler block index %d not found", k);
            goto fail;
        }

        ret = mqnic_create_sched_block(interface, &interface->sched_block[k],
                k, sched_block_rb);
        if (ret)
            goto fail;
    }

    return 0;

fail:
    return ret;
}

int mqnic_start_port(struct mqnic_if *interface, const uint32_t qcnt, const uint32_t rss_mask) {
    // set up RX queues
    mqnic_activate_rxqs(interface, qcnt);
    // set up TX queues
    mqnic_activate_txqs(interface, qcnt);

    // set MTU
    mqnic_interface_set_tx_mtu(interface, MTU_SIZE + ETH_HLEN);
    mqnic_interface_set_rx_mtu(interface, MTU_SIZE + ETH_HLEN);

    // configure RSS
    mqnic_interface_set_rx_queue_map_rss_mask(interface, 0, rss_mask); //rounddown_pow_of_two(interface->rx_queue_count)-1);
#ifdef QINGNIAO
    // enable ingress
    mqnic_interface_set_rx_queue_map_ing_enable(interface, 0, 1);
    mqnic_interface_set_rx_queue_map_rxcpl_timeout(interface, 0, 10000000); // 10s
#endif

    // enable first scheduler
    mqnic_activate_sched_block(interface->sched_block[0]);
    info("mqnic_start_port succeeded!");
    info("rxcpl timeout in us: %d", mqnic_interface_get_rx_queue_map_rxcpl_timeout(interface, 0));

    return 0;
}
