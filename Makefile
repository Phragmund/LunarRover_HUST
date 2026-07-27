.PHONY: validate test build

validate:
	python3 tools/validate_workspace.py

test: validate
	colcon test --event-handlers console_direct+
	colcon test-result --verbose

build:
	./scripts/build.sh
