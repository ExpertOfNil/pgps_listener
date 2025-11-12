#ifndef CIMPL_NETWORK_H
#define CIMPL_NETWORK_H

#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "cimpl_core.h"
#include "cimpl_string.h"

typedef struct IpV4Addr {
    u32 addr;
    u16 port;
} IpV4Addr;

void IpV4Addr_print(IpV4Addr*);
i32 IpV4Addr_from_str(IpV4Addr*, char*);
i32 parse_ip_str(IpV4Addr*, const char*);
i32 udp_listener_setup(isize*, IpV4Addr);

#ifdef CIMPL_IMPLEMENTATION
void IpV4Addr_print(IpV4Addr* ip) {
    u8 parts[4] = {0};
    parts[0] = (u8)(ip->addr >> 24);
    parts[1] = (u8)(ip->addr >> 16);
    parts[2] = (u8)(ip->addr >> 8);
    parts[3] = (u8)(ip->addr);
    printf(
        "IP: %d.%d.%d.%d:%d\n", parts[0], parts[1], parts[2], parts[3], ip->port
    );
}

i32 IpV4Addr_from_str(IpV4Addr* ip_addr, char* ip_str) {
    u32 size = strlen(ip_str);
    if (size < 12 || size > 20) {
        fprintf(stderr, "Invalid IP address: %s\n", ip_str);
        return -1;
    }
    u32 addr_part = 3;
    StringView ip_str_view = {
        .items = ip_str,
    };
    for (u32 i = 0; i < size; ++i) {
        char c = ip_str[i];
        // Reached the end of an address component
        if (c == '.') {
            ip_addr->addr |=
                (u32)(StringView_to_u8(&ip_str_view) << (8 * addr_part--));
            // Advance to the char after the period
            ip_str_view.items = &ip_str[i + 1];
            ip_str_view.count = 0;
            continue;
        }
        // Reached the end of an address component.  Rest is port.
        if (c == ':') {
            // Ensure we are ready for the last part of the address and the port
            if (addr_part != 0) {
                fprintf(stderr, "Invalid IP address: %s\n", ip_str);
                return -1;
            }
            // Ensure there are 4 chars left to be parsed
            u32 chars_left = size - (i + 1);
            if (chars_left != 4) {
                fprintf(stderr, "Invalid IP address: %s\n", ip_str);
                return -1;
            }

            // Extract the last part of the address
            ip_addr->addr |= (u32)StringView_to_u8(&ip_str_view);
            ip_str_view.items = &ip_str[i + 1];
            ip_str_view.count = chars_left;
            addr_part--;

            // Extract the port
            ip_addr->port = StringView_to_u16(&ip_str_view);
            break;
        }
        if (addr_part < 0) {
            fprintf(stderr, "Invalid IP address: %s\n", ip_str);
            return -1;
        }
        ip_str_view.count++;
    }
    return 0;
}

i32 parse_ip_str(IpV4Addr* ip_addr, const char* ip_str) {
    u32 size = strlen(ip_str);
    if (size < 12 || size > 20) {
        fprintf(stderr, "Invalid IP address: %s\n", ip_str);
        return -1;
    }
    u32 addr_part = 3;
    StringView ip_str_view = {
        .items = (char*)ip_str,
    };
    for (u32 i = 0; i < size; ++i) {
        char c = ip_str[i];
        // Reached the end of an address component
        if (c == '.') {
            ip_addr->addr |=
                (u32)(StringView_to_u8(&ip_str_view) << (8 * addr_part--));
            // Advance to the char after the period
            ip_str_view.items = (char*)&ip_str[i + 1];
            ip_str_view.count = 0;
            continue;
        }
        // Reached the end of an address component.  Rest is port.
        if (c == ':') {
            // Ensure we are ready for the last part of the address and the port
            if (addr_part != 0) {
                fprintf(stderr, "Invalid IP address: %s\n", ip_str);
                return -1;
            }
            // Ensure there are 4 chars left to be parsed
            u32 chars_left = size - (i + 1);
            if (chars_left != 4) {
                fprintf(stderr, "Invalid IP address: %s\n", ip_str);
                return -1;
            }

            // Extract the last part of the address
            ip_addr->addr |= (u32)StringView_to_u8(&ip_str_view);
            ip_str_view.items = (char*)&ip_str[i + 1];
            ip_str_view.count = chars_left;
            addr_part--;

            // Extract the port
            ip_addr->port = StringView_to_u16(&ip_str_view);
            break;
        }
        if (addr_part < 0) {
            fprintf(stderr, "Invalid IP address: %s\n", ip_str);
            return -1;
        }
        ip_str_view.count++;
    }
    return 0;
}

i32 udp_listener_setup(isize* socket_fd, IpV4Addr ip) {
    struct sockaddr_in listen_addr = {0};
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(ip.addr);
    listen_addr.sin_port = htons(ip.port);

    char ip_addr_str[INET_ADDRSTRLEN] = {0};
    if (inet_ntop(
            listen_addr.sin_family,
            &(listen_addr.sin_addr),
            ip_addr_str,
            INET_ADDRSTRLEN
        ) == NULL) {
        log_error("Malformed IP address.\n");
        return EXIT_FAILURE;
    }

    *socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (*socket_fd < 0) {
        log_error("Failed to create socket for %s:%d.", ip_addr_str, ip.port);
        return EXIT_FAILURE;
    }
    int reuse = 1;
    if (setsockopt(
            *socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)
        ) < 0) {
        log_error("Failed to set SO_REUSEADDR");
    }
    int bind_err = bind(
        *socket_fd, (const struct sockaddr*)&listen_addr, sizeof(listen_addr)
    );
    if (bind_err < 0) {
        log_error(
            "[%d] Failed to bind to %s:%d.", bind_err, ip_addr_str, ip.port
        );
        return EXIT_FAILURE;
    }
    log_info("Listening at %s:%d\n", ip_addr_str, ip.port);
    return EXIT_SUCCESS;
}
#endif /* CIMPL_IMPLEMENTATION */

#endif /* CIMPL_NETWORK_H */
