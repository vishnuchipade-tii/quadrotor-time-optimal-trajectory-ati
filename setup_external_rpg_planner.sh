#!/usr/bin/env bash

set -e

################################################################################
# Configuration
################################################################################

RPG_REPO="https://github.com/vishnuchipade-tii/rpg_time_optimal_extended.git"

DEFAULT_INSTALL_DIR="../rpg_time_optimal"

DEFAULT_ENV_TYPE="conda"
DEFAULT_ENV_NAME="rpg_env"

CONFIG_DIR="config/external"
CONFIG_FILE="${CONFIG_DIR}/rpg_time_opt_traj_planner.yaml"

################################################################################
# Arguments
################################################################################

INSTALL_DIR="${1:-$DEFAULT_INSTALL_DIR}"
ENV_TYPE="${2:-$DEFAULT_ENV_TYPE}"
ENV_NAME="${3:-$DEFAULT_ENV_NAME}"

echo "============================================================"
echo "Installing RPG Time Optimal Trajectory Planner"
echo "============================================================"

echo "Install directory : $INSTALL_DIR"
echo "Environment type  : $ENV_TYPE"
echo "Environment name  : $ENV_NAME"

###############################################################################
# Clone
###############################################################################

if [ ! -d "$INSTALL_DIR" ]; then
    git clone "$RPG_REPO" "$INSTALL_DIR"
else
    echo "Repository already exists."
fi

###############################################################################
# Setup environment
###############################################################################

echo "Running RPG planner python environment setup..."

pushd "$INSTALL_DIR"

chmod +x setup_env.sh

./setup_env.sh "$ENV_TYPE" "$ENV_NAME"

popd > /dev/null

###############################################################################
# Detect environment
###############################################################################

mkdir -p "$CONFIG_DIR"

if [ -f "${INSTALL_DIR}/.venv/bin/python" ]; then

cat > "$CONFIG_FILE" <<EOF
environment:
  type: venv

project:
  root: ${INSTALL_DIR}

python:
  executable: ${INSTALL_DIR}/.venv/bin/python

script:
  entry: main.py
EOF

elif conda env list | grep -q "^rpg_env"; then

cat > "$CONFIG_FILE" <<EOF
environment:
  type: conda
  name: rpg_env

project:
  root: ${INSTALL_DIR}

python:
  executable: python

script:
  entry: main.py
EOF

else

    echo "Unable to determine RPG environment."
    echo "Please edit ${CONFIG_FILE} manually."
    exit 1

fi

echo
echo "Configuration written to:"
echo "  ${CONFIG_FILE}"