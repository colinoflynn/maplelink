#ifndef NET_CONFIG_H
#define NET_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint32_t device_ip;   // host-order IPv4, e.g. 0xC0A80701 for 192.168.7.1
  uint32_t dhcp_start;  // host-order IPv4 start
  uint32_t dhcp_end;    // host-order IPv4 end (inclusive)
} net_config_t;

void net_config_init(void);
void net_config_get(net_config_t *out);
bool net_config_set_and_save(const net_config_t *cfg, char *err, size_t err_sz);
bool net_parse_ipv4(const char *s, uint32_t *out_host_order);
void net_format_ipv4(uint32_t ip_host_order, char *out, size_t out_sz);

#endif
