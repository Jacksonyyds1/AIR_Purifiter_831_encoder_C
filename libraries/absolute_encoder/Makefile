# Makefile for Absolute Encoder C Library

# 编译器设置
CC = gcc
CFLAGS = -Wall -Wextra -Werror -std=c99 -O2 -g
INCLUDES = -I.

# 源文件
SOURCES = absolute_encoder.c \
          encoder_map.c \
          smart_search_manager.c \
          position_tracker.c \
          input_preprocessor.c

# 头文件
HEADERS = absolute_encoder.h \
          absolute_encoder_logger.h \
          encoder_map.h \
          smart_search_manager.h \
          position_tracker.h \
          input_preprocessor.h

# 目标文件
OBJECTS = $(SOURCES:.c=.o)

# 库文件名
LIBRARY = libabsolute_encoder_c.a

# 测试程序
TEST_PROG = test_example
TEST_SOURCE = test_example.c

# 默认目标
all: $(LIBRARY)

# 生成静态库
$(LIBRARY): $(OBJECTS)
	ar rcs $@ $^
	@echo "Library $(LIBRARY) created successfully"

# 编译测试程序
test: $(LIBRARY) $(TEST_SOURCE)
	$(CC) $(CFLAGS) $(INCLUDES) $(TEST_SOURCE) -L. -labsolute_encoder_c -lm -o $(TEST_PROG)
	@echo "Test program $(TEST_PROG) created successfully"

# 运行测试
run-test: test
	./$(TEST_PROG)

# 编译目标文件
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# 清理
clean:
	rm -f $(OBJECTS) $(LIBRARY) $(TEST_PROG)
	@echo "Clean completed"

# 安装（复制到系统目录，需要root权限）
install: $(LIBRARY)
	cp $(LIBRARY) /usr/local/lib/
	cp $(HEADERS) /usr/local/include/
	@echo "Installation completed"

# 测试编译（检查语法）
check: $(OBJECTS)
	@echo "Syntax check completed successfully"

# 显示帮助
help:
	@echo "Available targets:"
	@echo "  all      - Build the library (default)"
	@echo "  test     - Build the test program"
	@echo "  run-test - Build and run the test program"
	@echo "  clean    - Remove object files and library"
	@echo "  install  - Install library and headers to system"
	@echo "  check    - Check syntax by compiling objects"
	@echo "  help     - Show this help message"

# 伪目标
.PHONY: all clean install check help test run-test

# 依赖关系
absolute_encoder.o: absolute_encoder.c absolute_encoder.h encoder_map.h smart_search_manager.h position_tracker.h input_preprocessor.h absolute_encoder_logger.h
encoder_map.o: encoder_map.c encoder_map.h absolute_encoder_logger.h
smart_search_manager.o: smart_search_manager.c smart_search_manager.h encoder_map.h absolute_encoder_logger.h
position_tracker.o: position_tracker.c position_tracker.h encoder_map.h absolute_encoder_logger.h
input_preprocessor.o: input_preprocessor.c input_preprocessor.h absolute_encoder_logger.h
