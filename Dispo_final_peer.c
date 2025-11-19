#pragma GCC optimize ("O0")
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include <string.h>

#define debug 1

//wifi - P2P configuration
#define WIFI_SSID "Labyrinth"
#define WIFI_PASS "alpha_november_tango_oscar"
#define PEER_IP "192.168.0.102"  // IP address of the sensor Pico
#define PEER_PORT 5678           // UDP port on sensor Pico
#define LOCAL_PORT 5678          // Local UDP port to listen on

#define BUF_SIZE 6000
#define MAX_CHUNKS 7  // 5 data chunks + 1 final + 1 CRC


// Receive buffers
char received_data[BUF_SIZE] = {0};
int data_offset = 0;
uint16_t expected_crc = 0;
bool data_complete = false;
uint8_t chunks_received = 0;

static struct udp_pcb *send_pcb = NULL;   // For sending ACK
static struct udp_pcb *recv_pcb = NULL;   // For receiving data
static ip_addr_t peer_addr;

// CRC16 calculation function
uint16_t crc16_x25(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408;
            else
                crc >>= 1;
        }
    }
    return ~crc;
}

// Send ACK back to sender
err_t send_ack(uint8_t code) {
    if (send_pcb) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, 1, PBUF_RAM);
        if (p) {
            uint8_t ack_byte = code;
            pbuf_take(p, &ack_byte, 1);
            err_t err = udp_sendto(send_pcb, p, &peer_addr, PEER_PORT);
            pbuf_free(p);
            printf("ACK sent: 0x%02X\n", code);
            return err;
        }
        return ERR_MEM;
    }
    return ERR_CONN;
}

// UDP receive callback
static void recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (!p) return;

    // Update peer address from received packet
    ip_addr_copy(peer_addr, *addr);

    uint16_t len = p->len;
    
    // Check if this is the CRC packet (2 bytes)
    if (len == 2 && chunks_received == 6) {
        uint8_t crc_bytes[2];
        pbuf_copy_partial(p, crc_bytes, 2, 0);
        expected_crc = (crc_bytes[1] << 8) | crc_bytes[0];
        
        // Calculate and verify CRC
        uint16_t calculated_crc = crc16_x25((uint8_t*)received_data, data_offset);
        printf("Received CRC: 0x%04X, Calculated CRC: 0x%04X\n", expected_crc, calculated_crc);
        
        if (expected_crc == calculated_crc) {
            printf("✓ CRC Valid - Data transfer complete!\n");
            data_complete = true;
            send_ack(0xAA);  // Send ACK
            chunks_received++;
        } else {
            printf("✗ CRC Mismatch - Data corrupted!\n");
            send_ack(0x55);  // Send NACK
        }
    } else {
        // Regular data chunk
        printf("Received chunk %d: %d bytes\n", chunks_received, len);
        
        // Append data to buffer
        if (data_offset + len <= BUF_SIZE) {
            pbuf_copy_partial(p, (uint8_t*)&received_data[data_offset], len, 0);
            data_offset += len;
            chunks_received++;
            
            // Send ACK for each chunk
            send_ack(0xAA);
        } else {
            printf("ERROR: Receive buffer overflow!\n");
            send_ack(0x55);  // Send NACK
        }
    }
    
    pbuf_free(p);
}

// Parse and display received sensor data
void parse_sensor_data() {
    if (!data_complete || data_offset < 11) {
        printf("Data incomplete or invalid\n");
        return;
    }
    
    printf("\n========== SENSOR DATA RECEIVED ==========\n");
    
    // Parse header format: !ooooooo-bbbbbbb-p-c-eeeeeeee(x1280)?
    // Position 0: '!'
    printf("Start marker: %c\n", received_data[0]);
    
    // Position 1-2: SpO2 (hex)
    uint8_t spo2_hex[2];
    spo2_hex[0] = received_data[1];
    spo2_hex[1] = received_data[2];
    printf("SpO2: 0x%02X%02X\n", spo2_hex[0], spo2_hex[1]);
    
    // Position 3: separator
    // Position 4-5: Battery (hex)
    uint8_t battery = 0;
    // Convert hex ASCII to value
    for (int i = 0; i < 2; i++) {
        if (received_data[4+i] >= '0' && received_data[4+i] <= '9')
            battery = battery * 16 + (received_data[4+i] - '0');
        else if (received_data[4+i] >= 'A' && received_data[4+i] <= 'F')
            battery = battery * 16 + (received_data[4+i] - 'A' + 10);
    }
    printf("Battery: %u%%\n", battery);
    
    // Position 6: separator
    // Position 7: Panic + ID (hex)
    uint8_t panic_id;
    if (received_data[7] >= '0' && received_data[7] <= '9')
        panic_id = received_data[7] - '0';
    else if (received_data[7] >= 'A' && received_data[7] <= 'F')
        panic_id = received_data[7] - 'A' + 10;
    else
        panic_id = 0;
    printf("Device ID: 0x%X, Panic: %s\n", panic_id & 0x0F, (panic_id & 0x80) ? "YES" : "NO");
    
    // Position 8: separator
    // Position 9: Fall detection (hex)
    uint8_t fall = 0;
    if (received_data[9] >= '0' && received_data[9] <= '9')
        fall = received_data[9] - '0';
    else if (received_data[9] >= 'A' && received_data[9] <= 'F')
        fall = received_data[9] - 'A' + 10;
    printf("Fall Detected: %s\n", fall ? "YES" : "NO");
    
    // Position 10: separator
    // Position 11+: ECG data (5120 bytes in hex format)
    printf("ECG Data: %d bytes received\n", data_offset - 11);
    printf("Total packet size: %d bytes\n", data_offset);
    
    printf("==========================================\n\n");
}

int main() {
    stdio_init_all();
    
    printf("=== P2P Sensor Receiver (Peer) ===\n");
    printf("Initializing Wi-Fi...\n");
    
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }
    
    cyw43_arch_enable_sta_mode();
    printf("Connecting to WiFi: %s...\n", WIFI_SSID);
    
    int connect_attempts = 0;
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                        CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Connection attempt %d failed. Retrying...\n", ++connect_attempts);
        if (connect_attempts >= 3) {
            printf("Failed to connect to WiFi\n");
            cyw43_arch_deinit();
            return -1;
        }
    }
    
    printf("✓ Connected to WiFi\n");
    
    // Get local IP
    struct netif *netif = &cyw43_state.netif[CYW43_ITF_STA];
    printf("Local IP: %s\n", ip4addr_ntoa(&netif->ip_addr));
    
    // Parse peer IP
    ip4addr_aton(PEER_IP, &peer_addr);
    printf("Peer IP: %s:%d\n", PEER_IP, PEER_PORT);
    printf("Local listening port: %d\n\n", LOCAL_PORT);
    
    // Create UDP socket for sending ACK
    send_pcb = udp_new();
    if (!send_pcb) {
        printf("Failed to create send PCB\n");
        cyw43_arch_deinit();
        return -1;
    }
    
    // Create UDP socket for receiving
    recv_pcb = udp_new();
    if (!recv_pcb) {
        printf("Failed to create receive PCB\n");
        cyw43_arch_deinit();
        return -1;
    }
    
    if (udp_bind(recv_pcb, IP_ADDR_ANY, LOCAL_PORT) != ERR_OK) {
        printf("Failed to bind receive socket\n");
        cyw43_arch_deinit();
        return -1;
    }
    
    udp_recv(recv_pcb, recv_callback, NULL);
    printf("✓ Listening for peer data on port %d\n\n", LOCAL_PORT);
    printf("Waiting for sensor data...\n");
    
    uint32_t last_status_time = 0;
    
    // Main loop
    while (true) {
        cyw43_arch_poll();
        
        // Print status every 5 seconds
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_status_time > 5000) {
            printf("Status: %d chunks received, %d bytes buffered\n", 
                   chunks_received, data_offset);
            last_status_time = current_time;
        }
        
        // If we have complete data, parse and reset
        if (data_complete) {
            parse_sensor_data();
            
            // Reset for next transmission
            memset(received_data, 0, BUF_SIZE);
            data_offset = 0;
            chunks_received = 0;
            data_complete = false;
            expected_crc = 0;
            
            printf("Ready for next transmission...\n");
        }
        
        sleep_ms(100);
    }
    
    // Cleanup
    if (send_pcb) udp_remove(send_pcb);
    if (recv_pcb) udp_remove(recv_pcb);
    cyw43_arch_deinit();
    
    return 0;
}
