#!/bin/bash
set -e

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}[1/3] Checking Acados Installation...${NC}"

if [ ! -d "/opt/acados" ]; then
    echo "Acados not found in /opt/acados."
    echo "This script assumes you have sudo privileges."
    
    # Clone
    sudo git clone https://github.com/acados/acados.git /opt/acados
    cd /opt/acados
    sudo git submodule update --recursive --init

    # Build
    echo -e "${GREEN}Building Acados Core...${NC}"
    mkdir -p build
    cd build
    cmake -DACADOS_WITH_QPOASES=ON ..
    make -j4
    sudo make install
    
    # Python Interface
    echo -e "${GREEN}Installing Python Interface...${NC}"
    cd /opt/acados/interfaces/acados_template
    pip3 install .
else
    echo -e "${GREEN}Acados found.${NC}"
fi

# Environment
export ACADOS_SOURCE_DIR="/opt/acados"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/opt/acados/lib"

echo -e "${GREEN}[2/3] Generating Altair NMPC Solver...${NC}"
cd ~/altair_ws/src/altair_controller/scripts
python3 generate_altair_nmpc.py

echo -e "${GREEN}[3/3] Building Controller...${NC}"
cd ~/altair_ws
colcon build --packages-select altair_controller

echo -e "${GREEN}DONE! Controller is ready.${NC}"
