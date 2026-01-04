// common.h
#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>
#include <time.h>
#include <fcntl.h>      
#include <unistd.h>     
#include <stdio.h>      
#include <stdlib.h>     
#include <string.h>
#include <sys/mman.h>   
#include <sys/stat.h>   

// POSIX Shared Memory 이름
#define SHM_NUM                 3
#define SHM_NAME_LIDAR          "/lidar_shm"
#define SHM_NAME_FRONT_CAMERA   "/camera_front_shm"
#define SHM_NAME_BACK_CAMERA    "/camera_back_shm"

#define LIDAR_PROC     "lidar"
#define CAMERA_PROC    "camera"
#define MAIN_PROC      "main_controller"

#define path_LIDAR_PROC     "./build/lidar"
#define path_CAMERA_PROC    "./build/camera"
#define path_MAIN_PROC      "./build/main_controller"

#define QUEUE_SIZE          50
#define MAX_LIDAR_POINTS    1440 // 360도 * 4 (0.25도 분해능 가정 시 여유분)
#define MAX_BBOX_OBJS       20   // 한 프레임에 최대 감지할 객체 수

#define UDP_PORT 5005

#define SIZE(X) (sizeof(X))

typedef struct {
    float angle;  // 각도 (Radian or Degree)
    float range;  // 거리 (Meter)
} LidarPoint;

typedef struct {
    float timestamp;      // 시스템 시간
    int count;            // 유효한 포인트 개수
    LidarPoint points[MAX_LIDAR_POINTS];
} LidarItem;

typedef struct {
    LidarItem buffer[QUEUE_SIZE];
    int head, tail;
    
    pthread_mutex_t mutex;
    sem_t sem_empty;
    sem_t sem_full;
} LidarQueue;

// --- [2] Camera 데이터 구조 (BBox)
typedef struct {
    int8_t class_id; // (옵션) 객체 종류 (사람, 차 등)
    float x, y, w, h;
} BBox;

typedef struct {
    float timestamp;
    int obj_count;           // 감지된 객체 수
    BBox objects[MAX_BBOX_OBJS];
} CameraItem;

typedef struct {
    CameraItem buffer[QUEUE_SIZE];
    int head, tail;

    pthread_mutex_t mutex;
    sem_t sem_empty;
    sem_t sem_full;
} CameraQueue;


#endif