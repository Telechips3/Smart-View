#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include "../common/protocol.h"

#define MAX_SPI_DEVICES 2

static struct spi_device *stm_spi_device[MAX_SPI_DEVICES];
static int device_count = 0;
static DEFINE_MUTEX(spi_lock); // 안전한 접근을 위한 락

// User-space에서 write() 호출시 실행되는 함수
// 유저가 보낸 데이터를 SPI 버스를 통해 하드웨어로 전송
static ssize_t dev_write(struct file *file, const char __user *buf, size_t len, loff_t *ppos) {
    void * packet;
    if (len != PACKET_SIZE) return -EINVAL;
    packet = memdup_user(buf, len);
    if (IS_ERR(packet)) return PTR_ERR(packet);
    
    int ret;
    
    mutex_lock(&spi_lock);
    
    // 등록된 모든 장치(CS0, CS1)에 루프 돌면서 전송
    for (int i = 0; i < device_count; i++) {
        if (stm_spi_device[i]) {
            dev_info(&stm_spi_device[i]->dev, "Broadcasting bytes to CS%d\n", stm_spi_device[i]->chip_select[0]);
            
            struct spi_transfer t = { .tx_buf = packet, .len = PACKET_SIZE };
                // 전송 실행
            struct spi_message m;
            spi_message_init(&m);
            spi_message_add_tail(&t, &m);
            //print_hex_dump(KERN_INFO, "[DEBUG] SPI TX: ", DUMP_PREFIX_NONE, 16, 1, packet, len, false);
            ret = spi_sync(stm_spi_device[i], &m); // 하드웨어 전송
                
            if (ret < 0)
                dev_err(&stm_spi_device[i]->dev, "Failed to write to CS%d\n", stm_spi_device[i]->chip_select[0]);
        }
    }

    mutex_unlock(&spi_lock);
    kfree(packet);
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
    //stm_spi_device = spi; // 나중에 write 함수에서 사용하기 위해 저장
    
    int ret = 0;
    int cs_idx = spi->chip_select[0];
    mutex_lock(&spi_lock);
    pr_info("STM32 SPI: Driver Probed!\n");

    // CS 번호(0 or 1)에 맞춰 배열에 저장
    if (cs_idx < MAX_SPI_DEVICES) {
        stm_spi_device[cs_idx] = spi;
        dev_info(&spi->dev, "Registered SPI device CS%d\n", cs_idx);
        device_count++;
    } else {
        dev_err(&spi->dev, "Invalid Chip Select %d\n", cs_idx);
        mutex_unlock(&spi_lock);
        return -EINVAL;
    }

    // 첫 번째 장치가 들어올 때만 /dev/stm32_spi 파일을 생성
    if (device_count == 1) {
        ret = misc_register(&stm_misc);
        if (ret) {
            dev_err(&spi->dev, "Failed to register misc device\n");
            // 실패 시 롤백 로직 필요하지만 간단한 예제라 생략
        } else {
            dev_info(&spi->dev, "Created /dev/stm32_spi interface\n");
        }
    }

    mutex_unlock(&spi_lock);
    return ret;
}

// 장치가 제거되거나 드라이버 언로드될 때 호출
static void stm32_spi_remove(struct spi_device *spi) 
{
    int cs_idx = spi->chip_select[0]; // [수정 5]
    mutex_lock(&spi_lock);

    if (cs_idx < MAX_SPI_DEVICES) {
        stm_spi_device[cs_idx] = NULL;
        device_count--;
        dev_info(&spi->dev, "Removed SPI device CS%d\n", cs_idx);
    }

    // 더 이상 연결된 장치가 없으면 /dev 파일도 삭제
    if (device_count == 0) {
        misc_deregister(&stm_misc);
        dev_info(&spi->dev, "Removed /dev/stm32_spi interface\n");
    }

    mutex_unlock(&spi_lock);
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
