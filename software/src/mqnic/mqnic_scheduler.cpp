#include "mqnic.h"
#include "io.h"

int mqnic_create_scheduler(struct mqnic_sched_block *block, struct mqnic_sched **sched_ptr,
        int index, struct mqnic_reg_block *rb)
{
    // struct device *dev = block->dev;
    struct mqnic_sched *sched;

    sched = (struct mqnic_sched *)rte_malloc(NULL, sizeof(*sched), 0);
    if (!sched)
        return -ENOMEM;

    *sched_ptr = sched;

    // sched->dev = dev;
    sched->interface = block->interface;
    sched->sched_block = block;

    sched->index = index;

    sched->rb = rb;

    sched->type = rb->type;
    sched->offset = get_reg32(rb->regs, MQNIC_RB_SCHED_RR_REG_OFFSET);
    sched->channel_count = get_reg32(rb->regs, MQNIC_RB_SCHED_RR_REG_CH_COUNT);
    sched->channel_stride = get_reg32(rb->regs, MQNIC_RB_SCHED_RR_REG_CH_STRIDE);

    sched->hw_addr = block->interface->hw_addr + sched->offset;

    info("Scheduler type: 0x%08x", sched->type);
    info("Scheduler offset: 0x%08x", sched->offset);
    info("Scheduler channel count: %d", sched->channel_count);
    info("Scheduler channel stride: 0x%08x", sched->channel_stride);

    mqnic_scheduler_disable(sched);

    return 0;
}

void mqnic_destroy_scheduler(struct mqnic_sched **sched_ptr)
{
    struct mqnic_sched *sched = *sched_ptr;
    *sched_ptr = NULL;

    mqnic_scheduler_disable(sched);

    free(sched);
}

int mqnic_scheduler_enable(struct mqnic_sched *sched)
{
    int k;

    // enable scheduler
    set_reg32(sched->rb->regs, MQNIC_RB_SCHED_RR_REG_CTRL, 1);

    // enable queues
    for (k = 0; k < sched->channel_count; k++)
        set_reg32(sched->hw_addr, k * sched->channel_stride, 3);

    return 0;
}

void mqnic_scheduler_disable(struct mqnic_sched *sched)
{
    // disable scheduler
    set_reg32(sched->rb->regs, MQNIC_RB_SCHED_RR_REG_CTRL, 0);
}
