#include <stdbool.h>

#define CIMPL_IMPLEMENTATION
#include "cimpl_core.h"
#include "cimpl_glm.h"
#include "cimpl_network.h"

#define SERVER_IP_ADDR "127.0.0.1:6969"
#define OUTPUT_FILE "UCGAC2413007_poses_60_64.csv"

typedef struct Pose {
    char id[32];
    char timestamp[28];
    Vec3 position;
    Quat rotation;
    uint8_t confidence;
    bool trigger_activated;
} Pose;

int main(void) {
    IpV4Addr server_addr = {0};
    IpV4Addr_from_str(&server_addr, SERVER_IP_ADDR);
    isize socket_fd = -1;
    udp_listener_setup(&socket_fd, server_addr);

    struct sockaddr_in client_info = {0};
    socklen_t client_len = sizeof(client_info);

    FILE* fp = fopen(OUTPUT_FILE, "w");
    if (fp == NULL) {
        perror("fopen");
        return 1;
    }
    fprintf(fp, "id,px,py,pz,qx,qy,qz,qw\n");
    Pose recv_pose = {0};
    while (1) {
        isize recv_bytes = recvfrom(
            socket_fd,
            &recv_pose,
            sizeof(Pose),
            0,
            (struct sockaddr*)&client_info,
            &client_len
        );

        if (recv_bytes == -1) {
            fprintf(stderr, "Failed to receive packet\n");
            perror("recvfrom");
            close(socket_fd);
            fclose(fp);
            return 1;
        }

        printf("Bytes received: %ld\n", recv_bytes);
        fprintf(
            fp,
            "%s"
            ",%12.6f,%12.6f,%12.6f"
            ",%12.6f,%12.6f,%12.6f,%12.6f\n",
            recv_pose.id,
            recv_pose.position.x,
            recv_pose.position.y,
            recv_pose.position.z,
            recv_pose.rotation.x,
            recv_pose.rotation.y,
            recv_pose.rotation.z,
            recv_pose.rotation.w
        );
    }

    close(socket_fd);
    fclose(fp);
}
