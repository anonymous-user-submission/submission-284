#ifndef MQNIC_PCI_H
#define MQNIC_PCI_H

#include <stdint.h>

void remove_driver(const char* pci_addr);
void enable_dma(const char* pci_addr);
uint8_t* pci_map_resource(const char* bus_id, long long unsigned int* size); // FIXME: fix type
int pci_open_resource(const char* pci_addr, const char* resource, int flags);

#endif // MQNIC_PCI_H
