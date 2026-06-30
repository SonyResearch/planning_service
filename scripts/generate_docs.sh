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
    pip install mkdocs
    pip install mkdocs-material
fi

rm -rf docs/_out
rm -rf docs/planning_service_website

# Build mkdocs docs for the website
pushd docs/website/
rm -rf site
mkdocs build
popd
mkdir docs/planning_service_website
mkdir docs/planning_service_website/api
mv docs/website/site/* docs/planning_service_website/

# Build doxygen docs for the API
doxygen docs/config.txt
mv docs/_out/html/* docs/planning_service_website/api

# Build client docs
echo "Building client docs"
pushd client/
# echo current directory
echo "setting up venv"
rm -rf client_docs_env
python -m venv client_docs_env
echo "activating venv"
source client_docs_env/bin/activate
echo "installing dependencies"
pip install --upgrade pip pip==23.0.0 # Peg version as venv is not propograted to subprocess.run call in 23.1+ in setup.py
pip install -r client/python/planning_service_client/requirements.txt  # Install dependencies
pip install -U sphinx
pip install sphinx_rtd_theme
echo "cleaning"
rm -rf build
rm -rf dist
rm -rf *.egg-info
# pip install . -v
# make stop for 10 seconds
echo "building docs"
sphinx-build docs/source docs/build
popd
# Back to root
echo "moving docs"
deactivate
# If code coverage directory exists, move it to the website
if [[ -d coverage ]]; then
    echo "Adding code coverage to website"
    mv coverage docs/planning_service_website/coverage
fi
mkdir docs/planning_service_website/client_python
mv client/docs/build/* docs/planning_service_website/client_python/

echo "Website generation complete!"
