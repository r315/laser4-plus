
all: l4p bp

norule:
	@echo "specify board"
	@echo "Available boards:"
	@echo "	l4p - Laser4 plus"
	@echo "	bp - bluepill"

clean:
	@${RM} -rf build

l4p:
	@"$(MAKE)" -C target/laser4_plus PROJECT_DIR=$(CURDIR)

bp:
	@"$(MAKE)" -C target/laser4_plus BUILD_DIR=$(BUILD_DIR)/bluepill

l4p-program:
	@"$(MAKE)" -C target/laser4_plus BUILD_DIR=$(BUILD_DIR)/laser4_plus

bp-program:
	@"$(MAKE)" -C target/laser4_plus BUILD_DIR=$(BUILD_DIR)/bluepill


.PHONY: