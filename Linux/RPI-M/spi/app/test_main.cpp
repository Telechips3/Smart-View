#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include "../common/protocol.h"

int main() {
    int fd = open("/dev/stm32_spi", O_WRONLY);
    if (fd < 0) { perror("장치 열기 실패"); return -1; }

    while (true) {
        UART_Packet_t p = {0};
        p.header = 0xAA;
        p.distance = 150.75f; // 예시 데이터
        p.class_ID = 1;

        // XOR 체크섬 계산
        uint8_t crc = 0;
        uint8_t* ptr = (uint8_t*)&p;
        for(int i=0; i < (int)PACKET_SIZE-1; i++) crc ^= ptr[i];
        p.checksum = crc;

        write(fd, &p, PACKET_SIZE);
        std::cout << "전송: 거리 " << p.distance << "m" << std::endl;
        usleep(500000); // 0.5초 대기
    }
    close(fd);
    return 0;
}