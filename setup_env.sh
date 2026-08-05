#!/usr/bin/env bash

set -e

# Usage: ./setup.sh [conda|venv] [env_name] [python_version]
# Check input arguments
if [ $# -lt 1 ] || [ $# -gt 3 ]; then
    echo "Usage: $0 [conda|venv] [env_name] [python_version]"
    exit 1
fi

# Configuration
ENV_TYPE=$1
ENV_NAME="${2:-time_optimal_traj_quad}"
PYTHON_VERSION="${3:-3.14.6}"
ENV_FILE="environment.yml"
REQUIREMENTS="requirements.txt"

if [ ! -f "$REQUIREMENTS" ]; then
    echo "Error: $REQUIREMENTS not found"
    exit 1
fi

echo "Creating environment type: $ENV_TYPE"
echo "Python version: $PYTHON_VERSION"

if [ "$ENV_TYPE" == "conda" ]; then
    echo "Creating Conda environment: $ENV_NAME"

    if ! command -v conda &> /dev/null; then
        echo "Error: conda not found"
        exit 1
    fi

    if [ ! -f "$ENV_FILE" ]; then
        echo "Error: $ENV_FILE not found"
        exit 1
    fi

    echo "Creating environment from $ENV_FILE..."

    conda env create \
        -f "$ENV_FILE" \
        -n "$ENV_NAME"

    echo ""
    echo "Done."
    echo "Activate with:"
    echo "conda activate $ENV_NAME"


elif [ "$ENV_TYPE" == "venv" ]; then

    echo "Creating Python virtual environment..."

    # Check python version
    if ! command -v python$PYTHON_VERSION &> /dev/null; then
        echo "Error: python$PYTHON_VERSION not found"
        echo "Install Python $PYTHON_VERSION first"
        exit 1
    fi

    python$PYTHON_VERSION -m venv .venv

    echo "Activating virtual environment..."

    source .venv/bin/activate

    echo "Upgrading pip..."
    pip install --upgrade pip

    echo "Installing dependencies..."
    pip install -r "$REQUIREMENTS"

    echo ""
    echo "Done."
    echo "Activate with:"
    echo "source .venv/bin/activate"

else
    echo "Invalid option: $ENV_TYPE"
    echo "Usage: $0 [conda|venv]"
    exit 1
fi