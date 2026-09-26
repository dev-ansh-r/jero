.PHONY: test lint submodules

submodules:
	git submodule update --init --depth 1 upstream/Open_Duck_Mini_Runtime

test:
	python -m pytest -q jero_link/tests tests

lint:
	ruff check .
	shellcheck robot/*.sh tools/*.sh training/*.sh
