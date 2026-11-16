CFLAGS := -std=c11 -Wall -Wextra -Werror -I../firmware/common
test: test_can_messages
	./test_can_messages
test_can_messages: test_can_messages.c ../firmware/common/can_messages.h
	$(CC) $(CFLAGS) $< -o $@
clean:
	rm -f test_can_messages
.PHONY: test clean
