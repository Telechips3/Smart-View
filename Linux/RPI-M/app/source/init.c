#include <sys/types.h>
#include <sys/wait.h>
#include "../common.h"

static pid_t child_pids[3];

// 프로세스를 실행시키는 함수
pid_t spawn_process(const char *program_name, const char *path)
{
    pid_t pid = fork();

    if (pid < 0)
    {
        // 1. Fork 실패 시
        perror("Fork failed");
        exit(1);
    }
    else if (pid == 0)
    {
        // 2. 자식 프로세스 영역 (Child Process)
        // 여기서 실제로 다른 프로그램으로 변신(Overlay)합니다.

        // execl(경로, 프로세스이름, 인자..., NULL)
        // 예: ./camera_app 실행
        execl(path, program_name, NULL);

        // execl이 성공하면 아래 코드는 실행되지 않음.
        // 만약 여기까지 왔다면 실행 파일이 없거나 에러가 난 것임.
        perror("Exec failed");
        exit(1);
    }

    // 3. 부모 프로세스 영역 (Parent Process)
    // 부모는 여기서 아무것도 안 하고 그냥 리턴해서 다음 자식을 낳으러 감
    printf("[Init System] Launched %s (PID: %d)\n", program_name, pid);
    return pid;
}

int shared_module_init(const char *const name, int shm_size)
{
    shm_unlink(name);

    int fd = shm_open(name, O_CREAT | O_EXCL | O_RDWR, 0666);
    printf("%s is made\n", name);
    if (ftruncate(fd, shm_size) == -1)
    {
        perror("shared_ftruncate fail");
        exit(1);
    }
    return fd;
}

void shared_init(int *fd, const char *const shm_path[], const int *const shm_size)
{
    for (int i = 0; i < SHM_NUM; i++)
    {
        fd[i] = shared_module_init(shm_path[i], shm_size[i]);
        if (fd[i] == -1)
        {
            perror("shared_init fail");
            exit(1);
        }
    }
}

void shared_close(int *fd, const char *const shm_path[])
{
    for (int i = 0; i < SHM_NUM; i++)
    {
        close(fd[i]);
        shm_unlink(shm_path[i]);
    }
}

void inner_semaphore_mutex_init(pthread_mutex_t *mutex, sem_t *sem_empty, sem_t *sem_full)
{
    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(mutex, &mattr);
    pthread_mutexattr_destroy(&mattr);

    sem_init(sem_empty, 1, QUEUE_SIZE);
    sem_init(sem_full, 1, 0);
}

void semaphore_mutex_init(int *fd, int *shm_size)
{
    LidarQueue *ptr;
    ptr = (LidarQueue *)mmap(NULL, shm_size[0], PROT_READ | PROT_WRITE, MAP_SHARED, fd[0], 0);
    if (ptr == MAP_FAILED)
    {
        perror("0 mmap failed");
        return;
    }
    inner_semaphore_mutex_init(&ptr->mutex, &ptr->sem_empty, &ptr->sem_full);
    ptr->head = 0;
    ptr->tail = 0;
    munmap(ptr, shm_size[0]);

    // back
    CameraQueue *bptr = (CameraQueue *)mmap(NULL, shm_size[1], PROT_READ | PROT_WRITE, MAP_SHARED, fd[1], 0);
    if (bptr == MAP_FAILED)
    {
        perror("1 mmap failed");
        return;
    }
    inner_semaphore_mutex_init(&bptr->mutex, &bptr->sem_empty, &bptr->sem_full);
    bptr->head = 0;
    bptr->tail = 0;
    munmap(bptr, shm_size[1]);

    // front
    CameraQueue *fptr = (CameraQueue *)mmap(NULL, shm_size[2], PROT_READ | PROT_WRITE, MAP_SHARED, fd[2], 0);
    if (fptr == MAP_FAILED)
    {
        perror("2 mmap failed");
        return;
    }
    inner_semaphore_mutex_init(&fptr->mutex, &fptr->sem_empty, &fptr->sem_full);
    fptr->head = 0;
    fptr->tail = 0;
    munmap(fptr, shm_size[2]);
}

int main()
{
    printf("=== Smart-View System Initialization ===\n");

    // [Step 1] 공통 초기화 작업 (여기서 다 수행)
    // 예: 공유 메모리 생성, DB 연결, 하드웨어 핀 설정 등
    printf("[Init] Setting up Shared Memory...\n");
    printf("[Init] Checking Network Interface...\n");
    printf("[Init] Initialization Complete.\n");

    int fd[SHM_NUM];
    int shm_size[SHM_NUM] = {
        SIZE(LidarQueue),
        SIZE(CameraQueue),
        SIZE(CameraQueue)};

    const char *shm_path[SHM_NUM] = {
        SHM_NAME_LIDAR,
        SHM_NAME_BACK_CAMERA,
        SHM_NAME_FRONT_CAMERA};

    shared_init(fd, shm_path, shm_size);
    semaphore_mutex_init(fd, shm_size);

    // [Step 2] 각 프로세스 실행 (Fork & Exec)
    // 실제 실행 파일 경로를 적어주세요
    child_pids[0] = spawn_process(LIDAR_PROC, path_LIDAR_PROC);
    child_pids[1] = spawn_process(CAMERA_PROC, path_CAMERA_PROC);
    child_pids[2] = spawn_process(MAIN_PROC, path_MAIN_PROC);

    // [Step 3] 모니터링 (부모 프로세스의 역할)
    // 자식들이 죽지 않고 잘 돌아가는지 감시합니다.
    while (1)
    {
        int status;
        pid_t died_pid = waitpid(-1, &status, WNOHANG);
        if (died_pid > 0)
        {
            printf("[Alert] Child process %d died!\n", died_pid);
        }

        // 2. 키 입력 확인 (표준 입력 감시)
        // 주의: 이 방식은 Enter를 눌러야 입력이 넘어옵니다.
        // 실시간 키 1개 감지를 원하면 termios 설정을 추가해야 합니다.
        char input[10];
        if (fgets(input, sizeof(input), stdin) != NULL)
        {
            if (input[0] == 'q' || input[0] == 'Q')
            {
                printf("[Init System] Sending SIGINT to all children...\n");

                for (int i = 0; i < 3; i++)
                {
                    if (child_pids[i] > 0)
                    {
                        kill(child_pids[i], SIGINT); // 자식들에게 SIGINT 전송
                        printf("Sent SIGINT to PID %d\n", child_pids[i]);
                    }
                }
                break; // 부모도 종료 루틴으로
            }
        }

        usleep(100000); // CPU 점유율 방지 (100ms)
    }

    shared_close(fd, shm_path);
    return 0;
}