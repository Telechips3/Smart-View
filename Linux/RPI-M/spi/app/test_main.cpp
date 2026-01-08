#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include "../common/protocol.h"
float x[5] = {16, 15, 14.5, 13.5, 12.5};
float y[10] = {30.0,70.0,120.0,180.0,250.0, 320, 370, 420, 480, 630};
int main() {
	int fd = open("/dev/stm32_spi", O_WRONLY);
	if (fd < 0) { perror("장치 열기 실패"); return -1; }
	int e = 0;

	UART_Packet_t p = {0};
	p.header = 0xAA;
	p.class_ID = 3;
	p.detected = 1;
	for(int j=100; j>0; j-=5)
	{		
		p.distance = j;
	
		// XOR 체크섬 계산
		uint8_t crc = 0;
		uint8_t* ptr = (uint8_t*)&p;
		for(int i=0; i < (int)PACKET_SIZE-1; i++) crc ^= ptr[i];
		p.checksum = crc;
	
		write(fd, &p, PACKET_SIZE);
		std::cout << "전송: 거리 " << p.distance << "m" << std::endl;
		usleep(1000000); // 0.5초 대기


	}
	close(fd);
	return 0;
}
