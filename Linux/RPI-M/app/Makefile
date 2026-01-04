#-lrt -lpthread -lcjson 이런 애들 추가 되어야 함.

# 1. 컴파일러 및 플래그 설정
CC = gcc
CXX = g++
COMMON_FLAGS = -Wall -I.
LIBS = -lrt -lpthread -lcjson

CFLAGS = $(COMMON_FLAGS)
CXXFLAGS = $(COMMON_FLAGS) -std=c++11

# 2. 경로 및 공통 파일 설정
SRC_DIR = source
BUILD_DIR = build

# 3. 파일 자동 탐색 (wildcard 사용)
# source 폴더의 모든 .c와 .cpp를 찾되, 공통 파일인 init.c는 제외합니다.
ALL_C_SRCS   := $(wildcard $(SRC_DIR)/*.c)
ALL_CPP_SRCS := $(wildcard $(SRC_DIR)/*.cpp)

# 4. 실행 파일 이름 결정
# 파일 경로와 확장자를 제거하고 build/ 폴더 경로를 붙입니다.
# 예: source/camera.c -> build/camera
TARGETS_C   := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%, $(ALL_C_SRCS))
TARGETS_CPP := $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%, $(ALL_CPP_SRCS))

ALL_TARGETS = $(TARGETS_C) $(TARGETS_CPP)

# 기본 타겟
all: prepare $(ALL_TARGETS)

prepare:
	@mkdir -p $(BUILD_DIR)

# 6. 패턴 규칙: 모든 C 소스 파일을 실행 파일로 빌드
$(BUILD_DIR)/%: $(SRC_DIR)/%.c
	@echo "빌드 중 (C): $@"
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# 7. 패턴 규칙: 모든 C++ 소스 파일을 실행 파일로 빌드
$(BUILD_DIR)/%: $(SRC_DIR)/%.cpp
	@echo "빌드 중 (C++): $@"
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LIBS)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all prepare clean