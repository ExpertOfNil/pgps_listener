#include <errno.h>
#include <stdbool.h>
#include <sys/time.h>

#define CIMPL_IMPLEMENTATION
#include "cimpl_core.h"
#include "cimpl_glm.h"
#include "cimpl_network.h"

typedef struct Pose {
    char id[32];
    char timestamp[28];
    Vec3 position;
    Quat rotation;
    uint8_t confidence;
    bool trigger_activated;
} Pose;

i32 udp_listener_setup_with_timeout(
    isize* socket_fd, IpV4Addr ip, uint32_t timeout_sec
) {
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

    struct timeval timeout_tv = {
        .tv_sec = timeout_sec,
    };
    if (setsockopt(
            *socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout_tv, sizeof(timeout_tv)
        ) < 0) {
        log_error("Failed to set SO_RCVTIMEO");
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

void help(char** argv) {
    printf("Usage: %s [IP_ADDR] [OUTPUT_PATH]", argv[0]);
    return;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        help(argv);
        return -1;
    }
    IpV4Addr server_addr = {0};
    IpV4Addr_from_str(&server_addr, argv[1]);
    isize socket_fd = -1;

    udp_listener_setup_with_timeout(&socket_fd, server_addr, 5);

    struct sockaddr_in client_info = {0};
    socklen_t client_len = sizeof(client_info);

    FILE* fp = fopen(argv[2], "w");
    if (fp == NULL) {
        perror("fopen");
        return 1;
    }
    fprintf(fp, "id,px,py,pz,qx,qy,qz,qw\n");
    Pose recv_pose = {0};
    u32 pose_count = 0;
    while (pose_count < 165) {
        // Don't use timeout until the first message has been received
        isize recv_bytes;
        if (pose_count == 0) {
            recv_bytes = recvfrom(
                socket_fd,
                &recv_pose,
                sizeof(Pose),
                0,
                (struct sockaddr*)&client_info,
                &client_len
            );
        } else {
            recv_bytes = recvfrom(
                socket_fd,
                &recv_pose,
                sizeof(Pose),
                0,
                (struct sockaddr*)&client_info,
                &client_len
            );
        }

        if (recv_bytes == -1) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (pose_count > 0) {
                    fprintf(
                        stderr, "Timout reached trying to receive packet.\n"
                    );
                    break;
                }
                continue;
            } else {
                fprintf(stderr, "Failed to receive packet\n");
                perror("recvfrom");
                break;
            }
        }

        printf("Bytes received: %ld\n", recv_bytes);
        fprintf(
            fp,
            "%s"
            ",%d"
            ",%12.6f,%12.6f,%12.6f"
            ",%12.6f,%12.6f,%12.6f,%12.6f\n",
            recv_pose.id,
            pose_count,
            recv_pose.position.x,
            recv_pose.position.y,
            recv_pose.position.z,
            recv_pose.rotation.x,
            recv_pose.rotation.y,
            recv_pose.rotation.z,
            recv_pose.rotation.w
        );
        pose_count++;
    }

    close(socket_fd);
    fclose(fp);
    return 0;
}
