# Build and execute executable
run: build execute

.PHONY: build
build:
	@echo "Attempting build..."
	if [ ! -d "build" ] ; then mkdir build ; fi
	cd build; cmake ..
	cd build; cmake --build .

.PHONY: rebuild
rebuild:
	@echo "Rebuilding..."
	rm -rf build/
	mkdir build
	cd build; cmake ..
	cd build; cmake --build .

.PHONY: execute
execute:
	@echo "Executing executable..."
	@./build/sim

.PHONY: restore
restore:
	@if [ -d "build" ] ; then echo "Removing build directory"; rm -rf build/ ; fi
