# 1. 컴파일러 및 플래그 설정
CC = gcc
CXX = g++

# OpenCV 설정 추가 (pkg-config 이용)
OPENCV_CFLAGS = $(shell pkg-config --cflags opencv4)
OPENCV_LIBS = $(shell pkg-config --libs opencv4)

COMMON_FLAGS = -Wall -I.
# 기존 라이브러리에 OpenCV 라이브러리 추가
LIBS = -lrt -lpthread -lcjson $(OPENCV_LIBS)

CFLAGS = $(COMMON_FLAGS)
# C++ 컴파일 시에만 OpenCV 헤더 경로 추가
CXXFLAGS = $(COMMON_FLAGS) -std=c++11 $(OPENCV_CFLAGS)

# 2. 경로 및 공통 파일 설정
SRC_DIR = source
BUILD_DIR = build

# 3. 파일 자동 탐색
ALL_C_SRCS   := $(wildcard $(SRC_DIR)/*.c)
ALL_CPP_SRCS := $(wildcard $(SRC_DIR)/*.cpp)

# 4. 실행 파일 이름 결정
TARGETS_C   := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%, $(ALL_C_SRCS))
TARGETS_CPP := $(patsubst $(SRC_DIR)/%.cpp, $(BUILD_DIR)/%, $(ALL_CPP_SRCS))

ALL_TARGETS = $(TARGETS_C) $(TARGETS_CPP)

# 기본 타겟
all: prepare $(ALL_TARGETS)

prepare:
	@mkdir -p $(BUILD_DIR)

# 6. 패턴 규칙: C (OpenCV를 C에서 쓸 일은 거의 없지만 LIBS에 포함되어 있으니 빌드는 됨)
$(BUILD_DIR)/%: $(SRC_DIR)/%.c
	@echo "빌드 중 (C): $@"
	$(CC) $(CFLAGS) $^ -o $@ $(LIBS)

# 7. 패턴 규칙: C++ (OpenCV 헤더와 라이브러리 모두 사용)
$(BUILD_DIR)/%: $(SRC_DIR)/%.cpp
	@echo "빌드 중 (C++): $@"
	$(CXX) $(CXXFLAGS) $^ -o $@ $(LIBS)

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all prepare clean