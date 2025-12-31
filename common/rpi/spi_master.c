#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#define DEVICE "/dev/spidev0.0"

#define SPI_HEADER   0xAA   // RPi → STM
#define RESP_HEADER  0x55   // STM → RPi

#pragma pack(push, 1)
typedef struct {
    uint8_t  header;
    uint8_t  detected;
    uint8_t  class_ID;
    float    timestamp;
    float    distance;
    float    bbox_x;
    float    bbox_y;
    float    bbox_h;
    float    bbox_w;
    uint8_t  checksum;
} SPI_Packet_t;
#pragma pack(pop)

#define PACKET_SIZE ((int)sizeof(SPI_Packet_t))

static uint8_t xor_checksum(const uint8_t *p, size_t n)
{
    uint8_t x = 0;
    for (size_t i = 0; i < n; i++) x ^= p[i];
    return x;
}

int main(void)
{
    int fd = open(DEVICE, O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 50000;

    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    SPI_Packet_t tx_pkt, rx_pkt;

    printf("RPi SPI START\n");

    while (1)
    {
        memset(&tx_pkt, 0, sizeof(tx_pkt));
        memset(&rx_pkt, 0, sizeof(rx_pkt));

        tx_pkt.header    = SPI_HEADER;
        tx_pkt.detected  = 1;
        tx_pkt.class_ID  = 2;
        tx_pkt.timestamp = 1.23f;
        tx_pkt.distance  = 90.34f;
        tx_pkt.bbox_x    = 319.8999999f;
        tx_pkt.bbox_y    = 50.0f;
        tx_pkt.bbox_w    = 50.0f;
        tx_pkt.bbox_h    = 80.0f;
        tx_pkt.checksum  = xor_checksum((uint8_t*)&tx_pkt, PACKET_SIZE - 1);

        struct spi_ioc_transfer tr;
        memset(&tr, 0, sizeof(tr));

        tr.tx_buf = (unsigned long)&tx_pkt;
        tr.rx_buf = (unsigned long)&rx_pkt;
        tr.len = PACKET_SIZE;
        tr.speed_hz = speed;
        tr.bits_per_word = bits;

        ioctl(fd, SPI_IOC_MESSAGE(1), &tr);

        printf("TX: det=%u cls=%u dist=%.2f ts=%.2f cs=%02X\n",
               tx_pkt.detected, tx_pkt.class_ID,
               tx_pkt.distance, tx_pkt.timestamp, tx_pkt.checksum);

        printf("RX: hdr=%02X det=%u cls=%u dist=%.2f ts=%.2f cs=%02X\n\n",
               rx_pkt.header, rx_pkt.detected, rx_pkt.class_ID,
               rx_pkt.distance, rx_pkt.timestamp, rx_pkt.checksum);

        usleep(100000);
    }

    close(fd);
    return 0;
}
