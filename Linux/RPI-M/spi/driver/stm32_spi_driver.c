#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include "../common/protocol.h"

static struct spi_device *stm_spi_device = NULL;

// User-space에서 write() 호출시 실행되는 함수
// 유저가 보낸 데이터를 SPI 버스를 통해 하드웨어로 전송
static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos) {
    UART_Packet_t packet;
    if (len != PACKET_SIZE) return -EINVAL;
    if (copy_from_user(&packet, buf, len)) return -EFAULT;

    if (stm_spi_device) {
        struct spi_transfer t = { .tx_buf = &packet, .len = PACKET_SIZE };
        struct spi_message m;
        spi_message_init(&m);
        spi_message_add_tail(&t, &m);
        spi_sync(stm_spi_device, &m); // 하드웨어 전송
    }
    return len;
}

// 파일 연산 구조체: /dev/ 인터페이스를 통해 호출될 함수들 매핑
static const struct file_operations stm_fops = {
    .owner = THIS_MODULE,
    .write = dev_write
};

// 미스크 디바이스 설정: 자동으로 /dev/ 디바이스 노드를 생성해줌
static struct miscdevice stm_misc = {
    .minor = MISC_DYNAMIC_MINOR, // 보조 번호 자동 할당
    .name = DEVICE_NAME, // /dev/ 아래에 생성될 이름
    .fops = &stm_fops // 파일 연산자 연결
};

// 장치 트리와 드라이버가 매칭되었을 때 호출
static int stm32_spi_probe(struct spi_device *spi) {
    stm_spi_device = spi; // 나중에 write 함수에서 사용하기 위해 저장
    pr_info("STM32 SPI: Driver Probed!\n");

    // 사용자 인터페이스를 시스템에 등록
    return misc_register(&stm_misc);
}

// 장치가 제거되거나 드라이버 언로드될 때 호출
static void stm32_spi_remove(struct spi_device *spi) {
    misc_deregister(&stm_misc); // 등록했던 디바이스 노드 제거
}

// 장치 트리의 compatible 문자열과 매칭하기 위한 ID 테이블
static const struct of_device_id stm32_ids[] = { { .compatible = "my,stm32-link" }, {} };
MODULE_DEVICE_TABLE(of, stm32_ids);

// SPI 드라이버 본체 설정
static struct spi_driver stm32_driver = {
    .driver = { .name = "stm32_spi", .of_match_table = stm32_ids },
    .probe = stm32_spi_probe, .remove = stm32_spi_remove,
};
module_spi_driver(stm32_driver); // module_init 이랑 module_exit 하나로 합친 매크로
MODULE_LICENSE("GPL");