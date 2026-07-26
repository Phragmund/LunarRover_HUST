.PHONY: validate test build

validate:
	python3 tools/validate_workspace.py

test: validate
	PYTHONPATH=src/lunar_rover_localization pytest -q src/lunar_rover_localization/test

build:
	./scripts/build.sh

