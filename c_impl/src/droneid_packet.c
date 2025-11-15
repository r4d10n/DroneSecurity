/**
 * @file droneid_packet.c
 * @brief Drone-ID packet parsing implementation
 */

#include "droneid_packet.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* DJI device type mapping */
static const struct {
    uint8_t id;
    const char* name;
} device_types[] = {
    {1, "Inspire 1"},
    {2, "Phantom 3 Series"},
    {3, "Phantom 3 Std"},
    {4, "Phantom 3 4K"},
    {5, "Mavic Pro"},
    {6, "Inspire 2"},
    {7, "Phantom 4"},
    {10, "Mavic Air"},
    {11, "Mavic 2"},
    {14, "Mavic 2 Enterprise"},
    {15, "Mavic Mini"},
    {16, "Mavic Air 2"},
    {17, "Phantom 4 Pro V2"},
    {18, "Phantom 4 Multispectral"},
    {20, "Mavic Air 2S"},
    {21, "DJI FPV"},
    {23, "Mini 2"},
    {24, "Mini SE"},
    {26, "Air 2S"},
    {27, "DJI Mini 3 Pro"},
    {0, "Unknown"}
};

void droneid_get_device_name(uint8_t device_id, char* name, int max_len) {
    if (!name || max_len == 0) return;

    for (int i = 0; device_types[i].id != 0; i++) {
        if (device_types[i].id == device_id) {
            strncpy(name, device_types[i].name, max_len - 1);
            name[max_len - 1] = '\0';
            return;
        }
    }

    strncpy(name, "Unknown", max_len - 1);
    name[max_len - 1] = '\0';
}

/* CRC-16 for Drone-ID (X.25 / CRC-CCITT) */
uint16_t droneid_compute_crc(const uint8_t* bytes, int len) {
    uint16_t crc = 0xFFFF;

    for (int i = 0; i < len; i++) {
        crc ^= bytes[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0x8408;  /* Reversed polynomial */
            } else {
                crc >>= 1;
            }
        }
    }

    return crc ^ 0xFFFF;
}

int droneid_parse_packet(const uint8_t* bytes, int len, droneid_payload_t* payload) {
    if (!bytes || !payload || len < 88) {
        return DRONEID_ERROR_INVALID_ARG;
    }

    memset(payload, 0, sizeof(droneid_payload_t));

    /* Parse header */
    int offset = 0;

    payload->pkt_len = bytes[offset] | (bytes[offset + 1] << 8);
    offset += 2;

    payload->unk = bytes[offset++];
    payload->version = bytes[offset++];

    payload->sequence_number = bytes[offset] | (bytes[offset + 1] << 8);
    offset += 2;

    payload->state_info = bytes[offset] | (bytes[offset + 1] << 8);
    offset += 2;

    /* Serial number (16 bytes) */
    memcpy(payload->serial_number, &bytes[offset], 16);
    payload->serial_number[15] = '\0';  /* Ensure null-terminated */
    offset += 16;

    /* GPS coordinates (drone) */
    int32_t lon_raw = bytes[offset] | (bytes[offset + 1] << 8) |
                      (bytes[offset + 2] << 16) | (bytes[offset + 3] << 24);
    payload->longitude = (double)lon_raw / 1e7;
    offset += 4;

    int32_t lat_raw = bytes[offset] | (bytes[offset + 1] << 8) |
                      (bytes[offset + 2] << 16) | (bytes[offset + 3] << 24);
    payload->latitude = (double)lat_raw / 1e7;
    offset += 4;

    /* Altitude and height (16-bit) */
    int16_t alt_raw = bytes[offset] | (bytes[offset + 1] << 8);
    payload->altitude = (float)alt_raw / 10.0f;
    offset += 2;

    int16_t hgt_raw = bytes[offset] | (bytes[offset + 1] << 8);
    payload->height = (float)hgt_raw / 10.0f;
    offset += 2;

    /* Velocity */
    payload->v_north = (int16_t)(bytes[offset] | (bytes[offset + 1] << 8));
    offset += 2;

    payload->v_east = (int16_t)(bytes[offset] | (bytes[offset + 1] << 8));
    offset += 2;

    payload->v_up = (int16_t)(bytes[offset] | (bytes[offset + 1] << 8));
    offset += 2;

    /* Additional telemetry */
    payload->d_1_angle = bytes[offset] | (bytes[offset + 1] << 8);
    offset += 2;

    /* GPS time (64-bit) */
    payload->gps_time = 0;
    for (int i = 0; i < 8; i++) {
        payload->gps_time |= ((uint64_t)bytes[offset + i] << (i * 8));
    }
    offset += 8;

    /* App (smartphone) location */
    int32_t app_lat_raw = bytes[offset] | (bytes[offset + 1] << 8) |
                          (bytes[offset + 2] << 16) | (bytes[offset + 3] << 24);
    payload->app_lat = (double)app_lat_raw / 1e7;
    offset += 4;

    int32_t app_lon_raw = bytes[offset] | (bytes[offset + 1] << 8) |
                          (bytes[offset + 2] << 16) | (bytes[offset + 3] << 24);
    payload->app_lon = (double)app_lon_raw / 1e7;
    offset += 4;

    /* Home point */
    int32_t home_lon_raw = bytes[offset] | (bytes[offset + 1] << 8) |
                           (bytes[offset + 2] << 16) | (bytes[offset + 3] << 24);
    payload->longitude_home = (double)home_lon_raw / 1e7;
    offset += 4;

    int32_t home_lat_raw = bytes[offset] | (bytes[offset + 1] << 8) |
                           (bytes[offset + 2] << 16) | (bytes[offset + 3] << 24);
    payload->latitude_home = (double)home_lat_raw / 1e7;
    offset += 4;

    /* Device type */
    uint8_t device_id = bytes[offset++];
    droneid_get_device_name(device_id, payload->device_type, MAX_DEVICE_NAME);

    /* UUID (optional) */
    payload->uuid_len = bytes[offset++];
    if (payload->uuid_len > 0 && offset + payload->uuid_len < len) {
        int copy_len = (payload->uuid_len < MAX_UUID - 1) ? payload->uuid_len : MAX_UUID - 1;
        memcpy(payload->uuid, &bytes[offset], copy_len);
        payload->uuid[copy_len] = '\0';
        offset += payload->uuid_len;
    }

    /* CRC (last 2 bytes) */
    if (offset + 2 <= len) {
        payload->crc_packet = bytes[len - 2] | (bytes[len - 1] << 8);

        /* Compute CRC over everything except last 2 bytes */
        payload->crc_calculated = droneid_compute_crc(bytes, len - 2);

        payload->crc_valid = (payload->crc_packet == payload->crc_calculated);
    } else {
        payload->crc_valid = false;
    }

    return DRONEID_SUCCESS;
}

bool droneid_check_crc(const droneid_payload_t* payload) {
    return payload && payload->crc_valid;
}

void droneid_print_payload(const droneid_payload_t* payload) {
    if (!payload) return;

    printf("\n## Drone-ID Payload ##\n");
    printf("{\n");
    printf("    \"pkt_len\": %u,\n", payload->pkt_len);
    printf("    \"unk\": %u,\n", payload->unk);
    printf("    \"version\": %u,\n", payload->version);
    printf("    \"sequence_number\": %u,\n", payload->sequence_number);
    printf("    \"state_info\": %u,\n", payload->state_info);
    printf("    \"serial_number\": \"%s\",\n", payload->serial_number);
    printf("    \"longitude\": %.15f,\n", payload->longitude);
    printf("    \"latitude\": %.15f,\n", payload->latitude);
    printf("    \"altitude\": %.2f,\n", payload->altitude);
    printf("    \"height\": %.2f,\n", payload->height);
    printf("    \"v_north\": %d,\n", payload->v_north);
    printf("    \"v_east\": %d,\n", payload->v_east);
    printf("    \"v_up\": %d,\n", payload->v_up);
    printf("    \"d_1_angle\": %u,\n", payload->d_1_angle);
    printf("    \"gps_time\": %lu,\n", (unsigned long)payload->gps_time);
    printf("    \"app_lat\": %.15f,\n", payload->app_lat);
    printf("    \"app_lon\": %.15f,\n", payload->app_lon);
    printf("    \"longitude_home\": %.15f,\n", payload->longitude_home);
    printf("    \"latitude_home\": %.15f,\n", payload->latitude_home);
    printf("    \"device_type\": \"%s\",\n", payload->device_type);
    printf("    \"uuid_len\": %u,\n", payload->uuid_len);
    printf("    \"uuid\": \"%s\",\n", payload->uuid);
    printf("    \"crc-packet\": \"%04x\",\n", payload->crc_packet);
    printf("    \"crc-calculated\": \"%04x\"\n", payload->crc_calculated);
    printf("}\n");

    if (!payload->crc_valid) {
        printf("CRC Check FAILED!\n");
    }
}
