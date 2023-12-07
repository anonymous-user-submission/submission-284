#include "mqnic.h"
#include "io.h"

int mqnic_create_sched_block(struct mqnic_if *interface, struct mqnic_sched_block **block_ptr,
        int index, struct mqnic_reg_block *block_rb)
{
    // struct device *dev = interface->dev;
    struct mqnic_sched_block *block;
    struct mqnic_reg_block *rb;
    uint32_t offset;
    int ret = 0;

    block = (struct mqnic_sched_block *)rte_malloc(NULL, sizeof(*block), 0);
    if (!block) {
        info("malloc sched failed.");
        return -ENOMEM;
    }

    *block_ptr = block;

    // block->dev = dev;
    block->interface = interface;

    block->index = index;

    block->tx_queue_count = interface->tx_queue_count;

    block->block_rb = block_rb;

    offset = get_reg32(block_rb->regs, MQNIC_RB_SCHED_BLOCK_REG_OFFSET);

    block->rb_list = mqnic_enumerate_reg_block_list(interface->hw_addr, offset, interface->hw_regs_size - offset);

    if (!block->rb_list) {
        ret = -EIO;
        info("Failed to enumerate blocks");
        goto fail;
    }

    info("Scheduler block-level register blocks:");
    for (rb = block->rb_list; rb->regs; rb++)
        info(" type 0x%08x (v %d.%d.%d.%d)", rb->type, rb->version >> 24,
                (rb->version >> 16) & 0xff, (rb->version >> 8) & 0xff, rb->version & 0xff);

    block->sched_count = 0;
    for (rb = block->rb_list; rb->regs; rb++) {
        if (rb->type == MQNIC_RB_SCHED_RR_TYPE && rb->version == MQNIC_RB_SCHED_RR_VER) {
            ret = mqnic_create_scheduler(block, &block->sched[block->sched_count],
                    block->sched_count, rb);

            if (ret)
                goto fail;

            block->sched_count++;
        }
    }

    info("Scheduler count: %d", block->sched_count);

    mqnic_deactivate_sched_block(block);

    return 0;

fail:
    mqnic_destroy_sched_block(block_ptr);
    return ret;
}

void mqnic_destroy_sched_block(struct mqnic_sched_block **block_ptr)
{
    struct mqnic_sched_block *block = *block_ptr;
    int k;

    mqnic_deactivate_sched_block(block);

    for (k = 0; k < ARRAY_SIZE(block->sched); k++)
        if (block->sched[k])
            mqnic_destroy_scheduler(&block->sched[k]);

    if (block->rb_list)
        mqnic_free_reg_block_list(block->rb_list);

    *block_ptr = NULL;
    free(block);
}

int mqnic_activate_sched_block(struct mqnic_sched_block *block)
{
    int k;

    // enable schedulers
    for (k = 0; k < ARRAY_SIZE(block->sched); k++)
        if (block->sched[k])
            mqnic_scheduler_enable(block->sched[k]);

    return 0;
}

void mqnic_deactivate_sched_block(struct mqnic_sched_block *block)
{
    int k;

    // disable schedulers
    // for (k = 0; k < ARRAY_SIZE(block->sched); k++)
    for (k = 0; k < block->sched_count; k++)
        if (block->sched[k])
            mqnic_scheduler_disable(block->sched[k]);
}
