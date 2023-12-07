#include "mqnic.h"
#include "io.h"

int mqnic_create_port(struct mqnic_if *interface, struct mqnic_port **port_ptr,
        int index, struct mqnic_reg_block *port_rb)
{
    // struct device *dev = interface->dev;
    struct mqnic_port *port;
    struct mqnic_reg_block *rb;
    uint32_t offset;
    int ret = 0;

    port = (struct mqnic_port *)rte_malloc(NULL, sizeof(*port), 0);
    if (!port) {
    info("malloc port failed.");
        return -ENOMEM;
  }

    *port_ptr = port;

    // port->dev = dev;
    port->interface = interface;

    port->index = index;

    port->port_rb = port_rb;

    offset = get_reg32(port_rb->regs, MQNIC_RB_SCHED_BLOCK_REG_OFFSET);

    port->rb_list = mqnic_enumerate_reg_block_list(interface->hw_addr, offset, interface->hw_regs_size - offset);

    if (!port->rb_list) {
        ret = -EIO;
        info("Failed to enumerate blocks");
        goto fail;
    }

    info("Port-level register blocks:");
    for (rb = port->rb_list; rb->regs; rb++)
        info(" type 0x%08x (v %d.%d.%d.%d)", rb->type, rb->version >> 24,
                (rb->version >> 16) & 0xff, (rb->version >> 8) & 0xff, rb->version & 0xff);

    port->port_ctrl_rb = mqnic_find_reg_block(port->rb_list, MQNIC_RB_PORT_CTRL_TYPE, MQNIC_RB_PORT_CTRL_VER, 0);

    if (!port->port_ctrl_rb) {
        ret = -EIO;
        info("Port control register block not found");
        goto fail;
    }

    port->port_features = get_reg32(port->port_ctrl_rb->regs, MQNIC_RB_PORT_CTRL_REG_FEATURES);

    info("Port features: 0x%08x", port->port_features);

    info("Port TX status: 0x%08x", mqnic_port_get_tx_status(port));
    info("Port RX status: 0x%08x", mqnic_port_get_rx_status(port));

    return 0;

fail:
    mqnic_destroy_port(port_ptr);
    return ret;
}

void mqnic_destroy_port(struct mqnic_port **port_ptr)
{
    struct mqnic_port *port = *port_ptr;

    if (port->rb_list)
        mqnic_free_reg_block_list(port->rb_list);

    *port_ptr = NULL;
    free(port);
}

uint32_t mqnic_port_get_tx_status(struct mqnic_port *port)
{
    return get_reg32(port->port_ctrl_rb->regs, MQNIC_RB_PORT_CTRL_REG_TX_STATUS);
}

uint32_t mqnic_port_get_rx_status(struct mqnic_port *port)
{
    return get_reg32(port->port_ctrl_rb->regs, MQNIC_RB_PORT_CTRL_REG_RX_STATUS);
}
