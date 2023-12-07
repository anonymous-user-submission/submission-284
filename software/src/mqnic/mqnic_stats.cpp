#include <cstdint>

#include "mqnic.h"
#include "io.h"


uint64_t mqnic_stats_read(struct mqnic_dev *mqnic, int index) {
    uint64_t ret = 0;

    if (!mqnic->stats_rb || index<0 || index>=mqnic->stats_count)
        return 0;

    ret = (uint64_t)ioread32(mqnic->hw_addr + mqnic->stats_offset + index*8 + 0);
    ret |= (uint64_t)ioread32(mqnic->hw_addr + mqnic->stats_offset + index*8 + 4) << 32;

    return ret;
}
