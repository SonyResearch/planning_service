#!/bin/bash
install_libraries=true

# Check for arguments
for arg in "$@"; do
    if [[ "$arg" == "-n" ]]; then
        install_libraries=false
    fi
done

if [ "$install_libraries" = true ]; then
    echo "Installing doxygen"
    sudo apt-get update
    sudo apt-get install -y python3-pip
    sudo apt-get install -y doxygen graphviz
fi

rm -rf docs/website
mkdir -p docs/website
mkdir -p docs/website/protos/
mkdir -p docs/website/cpp/

doxygen docs/config_cpp.txt
mv docs/website/cpp/html/* docs/website/
doxygen docs/config_protos.txt
mv docs/website/protos/html/* docs/website/protos/
