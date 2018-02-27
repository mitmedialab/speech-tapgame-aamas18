init:
	@echo "Installing dependencies from requirements.txt"
	pip install -r requirements.txt
.PHONY: init

test:
	python -m scripts.run_unit_tests
.PHONY: test

lint:
	@echo "Running PyLint on all TapGameController Files"
	pylint TapGameController/*.py TapGameController/**/*.py
.PHONY: lint

curriculum: GameUtils/Curriculum.py
	@echo "Checking to see if GameUtils/Curriculum.py is there"

lev_matrix: curriculum
	@echo "Generating lev_matrix from curriculum"
	python -m scripts.generate_levenshtein_weight_matrix

glove_matrix: curriculum
	@echo "Generating glove_matrix from curriculum"
	python -m scripts.generate_glove_matrix

cov_matrix: lev_matrix glove_matrix
	@echo "Generating covariance matrix from glove_matrix and lev_matrix"
	python -m scripts.generate_covariance_matrix