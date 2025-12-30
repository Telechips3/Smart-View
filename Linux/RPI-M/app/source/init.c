#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

// 프로세스를 실행시키는 함수
void spawn_process(const char* program_name, const char* path) {
    pid_t pid = fork();

    if (pid < 0) {
        // 1. Fork 실패 시
        perror("Fork failed");
        exit(1);
    } 
    else if (pid == 0) {
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
}

int main() {
    printf("=== Smart-View System Initialization ===\n");

    // [Step 1] 공통 초기화 작업 (여기서 다 수행)
    // 예: 공유 메모리 생성, DB 연결, 하드웨어 핀 설정 등
    printf("[Init] Setting up Shared Memory...\n");
    printf("[Init] Checking Network Interface...\n");
    printf("[Init] Initialization Complete.\n");

    // [Step 2] 각 프로세스 실행 (Fork & Exec)
    // 실제 실행 파일 경로를 적어주세요
    spawn_process("camera_proc", "./camera_app");
    spawn_process("lidar_proc",  "./lidar_app");
    spawn_process("main_ctrl",   "./main_controller");

    // [Step 3] 모니터링 (부모 프로세스의 역할)
    // 자식들이 죽지 않고 잘 돌아가는지 감시합니다.
    while (1) {
        int status;
        // wait(): 자식 중 하나라도 종료될 때까지 대기
        pid_t child_pid = wait(&status);

        if (child_pid > 0) {
            printf("[Alert] Child process %d died!\n", child_pid);
            
            // 여기서 죽은 프로세스를 다시 살리는(Respawn) 로직을 넣을 수도 있습니다.
            // 예: if (child_pid == camera_pid) spawn_process(..., "./camera_app");
        }
    }

    return 0;
}