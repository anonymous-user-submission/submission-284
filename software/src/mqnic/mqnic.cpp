#include "mqnic.h"

#include <linux/limits.h>
#include <linux/version.h>
#include <linux/rtc.h>
#include <linux/kernel.h>
#include <net/if_arp.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <unistd.h>

#include "common/log.h"
#include "mqnic_hw.h"
#include "io.h"

// for smaller #q, use 1024 descs
unsigned int mqnic_num_ev_queue_entries = 4096;
unsigned int mqnic_num_tx_queue_entries = 4096;
unsigned int mqnic_num_rx_queue_entries = 4096;

struct mqnic_reg_block *mqnic_enumerate_reg_block_list(uint8_t *base, size_t offset, size_t size) {

    int max_count = 8;
    struct mqnic_reg_block *reg_block_list =
        (struct mqnic_reg_block *)rte_malloc(NULL, max_count * sizeof(struct mqnic_reg_block), 0);
    int count = 0;
    int k;

    uint8_t *ptr;

    uint32_t rb_type;
    uint32_t rb_version;

    if (!reg_block_list) return NULL;

    while (1) {
        reg_block_list[count].type = 0;
        reg_block_list[count].version = 0;
        reg_block_list[count].base = 0;
        reg_block_list[count].regs = 0;

        if ((offset == 0 && count != 0) || offset >= size) 
            break;

        ptr = base + offset;

        for (k = 0; k < count; k++) {
            if (ptr == reg_block_list[k].regs) {
                log_error("Register blocks form a loop");
            }
        }

        rb_type = get_reg32(ptr, MQNIC_RB_REG_TYPE);
        rb_version = get_reg32(ptr, MQNIC_RB_REG_VER);
        offset = get_reg32(ptr, MQNIC_RB_REG_NEXT_PTR);

        reg_block_list[count].type = rb_type;
        reg_block_list[count].version = rb_version;
        reg_block_list[count].base = base;
        reg_block_list[count].regs = ptr;

        count++;

        if (count >= max_count) {
            struct mqnic_reg_block *tmp;
            max_count += 4;
            tmp = (struct mqnic_reg_block*)rte_realloc(reg_block_list, max_count * sizeof(struct mqnic_reg_block), 0);
            if (!tmp) {
                log_error("realloc failed with size %d*%lu=%lu",
                                            max_count,
                                            sizeof(struct mqnic_reg_block),
                                            max_count * sizeof(struct mqnic_reg_block));
            }
            reg_block_list = tmp;
        }
    }
    info("%d reg blocks found", count);
    return reg_block_list;
}

struct mqnic_reg_block *mqnic_find_reg_block(struct mqnic_reg_block *list, uint32_t type, uint32_t version, int index)
{
    struct mqnic_reg_block *rb = list;

    while (rb->regs) {
        if (rb->type == type && (!version || rb->version == version)) {
            if (index > 0)
                index--;
            else
                return rb;
        }

        rb++;
    }

    return NULL;
}
void mqnic_free_reg_block_list(struct mqnic_reg_block *list)
{
    free(list);
}

static int mqnic_reset_and_init(struct mqnic_dev *mqnic) {  // mimicing common_probe
    info("Resetting device %s", mqnic->pci_addr);
    struct mqnic_reg_block *rb;
    // struct rtc_time tm;
    int k = 0, ret = 0;
    // Enumerate registers
    mqnic->rb_list =
        mqnic_enumerate_reg_block_list(mqnic->hw_addr, 0, mqnic->hw_regs_size);
    if (!mqnic->rb_list) {
        log_error("Failed to enumerate blocks");
        return -1;
    }
    info("Device-level register blocks:");
    for (rb = mqnic->rb_list; rb->regs; rb++) {
        info(" type 0x%08x (v %d.%d.%d.%d)", rb->type, rb->version >> 24,
                (rb->version >> 16) & 0xff, (rb->version >> 8) & 0xff, rb->version & 0xff);
    }
    // Read ID registers
    mqnic->fw_id_rb = mqnic_find_reg_block(mqnic->rb_list, MQNIC_RB_FW_ID_TYPE, MQNIC_RB_FW_ID_VER, 0);

    if (!mqnic->fw_id_rb) {
        log_error("Error: FW ID block not found");
        return -1;
    }

    mqnic->fpga_id      = get_reg32(mqnic->fw_id_rb->regs, MQNIC_RB_FW_ID_REG_FPGA_ID);
    mqnic->fw_id        = get_reg32(mqnic->fw_id_rb->regs, MQNIC_RB_FW_ID_REG_FW_ID);
    mqnic->fw_ver       = get_reg32(mqnic->fw_id_rb->regs, MQNIC_RB_FW_ID_REG_FW_VER);
    mqnic->board_id     = get_reg32(mqnic->fw_id_rb->regs, MQNIC_RB_FW_ID_REG_BOARD_ID);
    mqnic->board_ver    = get_reg32(mqnic->fw_id_rb->regs, MQNIC_RB_FW_ID_REG_BOARD_VER);
    mqnic->build_date   = get_reg32(mqnic->fw_id_rb->regs, MQNIC_RB_FW_ID_REG_BUILD_DATE);
    mqnic->git_hash     = get_reg32(mqnic->fw_id_rb->regs, MQNIC_RB_FW_ID_REG_GIT_HASH);
    mqnic->rel_info     = get_reg32(mqnic->fw_id_rb->regs, MQNIC_RB_FW_ID_REG_REL_INFO);

    info("FPGA ID: 0x%08x", mqnic->fpga_id);
    info("FW ID: 0x%08x", mqnic->fw_id);
    info("FW version: %d.%d.%d.%d", mqnic->fw_ver >> 24,
                                    (mqnic->fw_ver >> 16) & 0xff,
                                    (mqnic->fw_ver >> 8) & 0xff,
                                    mqnic->fw_ver & 0xff);
    info("Board ID: 0x%08x", mqnic->board_id);
    info("Board version: %d.%d.%d.%d", mqnic->board_ver >> 24,
                                        (mqnic->board_ver >> 16) & 0xff,
                                        (mqnic->board_ver >> 8) & 0xff,
                                        mqnic->board_ver & 0xff);
    info("Git hash: %08x", mqnic->git_hash);
    info("Release info: %08x", mqnic->rel_info);

    mqnic->phc_rb = mqnic_find_reg_block(mqnic->rb_list, MQNIC_RB_PHC_TYPE, MQNIC_RB_PHC_VER, 0);


    /*
    mqnic->stats_rb = mqnic_find_reg_block(mqnic->rb_list, MQNIC_RB_STATS_TYPE, MQNIC_RB_STATS_VER, 0);

    if (!mqnic->stats_rb) {
        log_error("Error: stats block not found");
    }

    mqnic->stats_offset = ioread32(mqnic->stats_rb->regs + MQNIC_RB_STATS_REG_OFFSET);
    mqnic->stats_count = ioread32(mqnic->stats_rb->regs + MQNIC_RB_STATS_REG_COUNT);
    mqnic->stats_stride = ioread32(mqnic->stats_rb->regs + MQNIC_RB_STATS_REG_STRIDE);
    mqnic->stats_flags = ioread32(mqnic->stats_rb->regs + MQNIC_RB_STATS_REG_FLAGS);

    info("stats count: %u", mqnic->stats_count);
    */

    // Enumerate interfaces
    mqnic->if_rb = mqnic_find_reg_block(mqnic->rb_list, MQNIC_RB_IF_TYPE, MQNIC_RB_IF_VER, 0);

    if (!mqnic->if_rb) {
        log_error("Error: interface block not found");
        return -1;
    }
    mqnic->if_offset        = get_reg32(mqnic->if_rb->regs, MQNIC_RB_IF_REG_OFFSET);
    mqnic->if_count         = get_reg32(mqnic->if_rb->regs, MQNIC_RB_IF_REG_COUNT);
    mqnic->if_stride        = get_reg32(mqnic->if_rb->regs, MQNIC_RB_IF_REG_STRIDE);
    mqnic->if_csr_offset    = get_reg32(mqnic->if_rb->regs, MQNIC_RB_IF_REG_CSR_OFFSET);

    info("IF offset: 0x%08x", mqnic->if_offset);
    info("IF count: %d", mqnic->if_count);
    info("IF stride: 0x%08x", mqnic->if_stride);
    info("IF CSR offset: 0x%08x", mqnic->if_csr_offset);

    // check BAR size
    if (mqnic->if_count * mqnic->if_stride > mqnic->hw_regs_size) {
        log_error("Invalid BAR configuration (%d IF * 0x%x > 0x%llx)",
                mqnic->if_count, mqnic->if_stride, mqnic->hw_regs_size);
    }

    // Set up interfaces
    mqnic->dev_port_max     = 0;
    mqnic->dev_port_limit   = MQNIC_MAX_IF;

    mqnic->if_count = min_t(uint32_t, mqnic->if_count, MQNIC_MAX_IF);

    for (k = 0; (uint32_t)k < mqnic->if_count; k++) {
        info("Creating interface %d", k);
        ret = mqnic_create_interface(mqnic, &mqnic->interface[k], k, mqnic->hw_addr + k * mqnic->if_stride);
        if (ret) {
            log_error("Failed to create interface: %d", ret);
        }
        mqnic->dev_port_max = mqnic->interface[k]->dev_port_max;
    }

    return 0;
}

static int mqnic_init_mmap(struct mqnic_dev *dev, const char *pci_addr) {
    if (getuid()) {
        warn("Not running as root, this will probably fail");
    }

    // Read PCI configuration space
    // every config file should be world-readable, and here we
    // only read the vendor and device id.
    int config = pci_open_resource(pci_addr, "config", O_RDONLY);
    uint32_t class_id;
    assert(pread(config, &class_id, sizeof(class_id), 8) != -1);
    class_id >>= 24;
    close(config);
    if (class_id != 2) {
        log_error("Device %s is not a NIC", pci_addr);
        return -1;
    }

    dev->pci_addr = strdup(pci_addr);
    // Map BAR0 region
    debug("mapping BAR0 region via pci file...");
    dev->hw_addr = pci_map_resource(pci_addr, &dev->hw_regs_size);

    return 0;
}

static int mqnic_alloc_rings(struct mqnic_dev *dev, const uint32_t qcnt) {

    const uint32_t max_desc_block_size = 1;

    int ret;
    struct mqnic_if *interface = dev->interface[0];
    assert(interface != NULL);
    assert(qcnt<=interface->tx_queue_count &&
            qcnt<=interface->rx_queue_count &&
            qcnt<=interface->tx_cpl_queue_count &&
            qcnt<=interface->rx_cpl_queue_count);

    uint32_t desc_block_size = min_t(uint32_t, interface->max_desc_block_size, max_desc_block_size);

    ret = mqnic_alloc_txqs(interface, qcnt, mqnic_num_tx_queue_entries);
    if (ret < 0) {
        log_error("alloc tx error");
        return -1;
    }

    ret = mqnic_alloc_rxqs(interface, qcnt, mqnic_num_rx_queue_entries);
    if (ret < 0) {
        log_error("alloc rx error");
        return -1;
    }

    return 0;
}

static int mqnic_start(struct mqnic_dev *dev, const uint32_t qcnt, const uint32_t rss_mask) {

    // TODO: RSS mask
    struct mqnic_if *interface = dev->interface[0];
    mqnic_start_port(interface, qcnt, rss_mask);
    return 0;
}

static const struct mqnic_eth_dev_ops mqnic_dev_ops = {
    .dev_init_mmap = &mqnic_init_mmap,
    .dev_init = &mqnic_reset_and_init,
    .dev_alloc_rings = &mqnic_alloc_rings,
    .dev_start = &mqnic_start,
};

int mqnic_eth_dev_create(struct mqnic_eth_dev **dev, 
                            const char* pci_addr, 
                            const uint32_t qcnt,
                            const uint32_t rss_mask) {
    struct mqnic_dev *pdev = NULL;

    *dev = (struct mqnic_eth_dev *)rte_malloc(NULL, sizeof(struct mqnic_eth_dev), 0);
    if (NULL == dev) {
        log_error("rte_malloc error");
        return -1;
    }

    (*dev)->dev_ops = &mqnic_dev_ops;
    (*dev)->rx_pkts_burst = &mqnic_rx_batch;
    (*dev)->tx_pkts_burst = &mqnic_tx_batch;
    (*dev)->qcnt = qcnt;

    (*dev)->dev = (struct mqnic_dev *)rte_malloc(NULL, sizeof(struct mqnic_dev), 0);
    pdev = (*dev)->dev;
    if (NULL == pdev) {
        log_error("rte_malloc error");
        return -1;
    }

    // initialize mqnic_dev
    check_err((*dev)->dev_ops->dev_init_mmap(pdev, pci_addr), "init mmap error");
    check_err((*dev)->dev_ops->dev_init(pdev), "init error");
    check_err((*dev)->dev_ops->dev_alloc_rings(pdev, qcnt), "alloc rings error");
    check_err((*dev)->dev_ops->dev_start(pdev, qcnt, rss_mask), "dev start error");

    info("dev initialized");

    return 0;
}

