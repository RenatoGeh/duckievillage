# Exits script on error and print offending line.
function exit_with_error {
  echo "Error at [${BASH_SOURCE[1]##*/}]->${FUNCNAME[1]}:${BASH_LINENO[0]}"
  exit 1
}

printf "Check if git is installed... "
if ! command -v git &> /dev/null; then
  echo "Git not found! Install git first."
fi
echo "OK"

# Save current working directory and cd to Duckievillage.
pushd . || exit_with_error
cd "$(dirname "$0")" || exit_with_error

echo "Updating Duckievillage... "
git pull || exit_with_error
popd || exit_with_error

printf "Updating Duckietown... "
cd duckietown || exit_with_error
git pull || exit_with_error
cd ..

printf "Check if Anaconda is installed... "
if ! command -v conda &> /dev/null; then
  echo "Anaconda not found! Assuming Duckievillage was installed another way. Not updating dependencies."
else
  echo "OK"
  printf "Check if Duckietown's environment exists... "
  if conda env list | grep "duckietown"; then
    echo "OK"
    printf "Setting up conda.sh... "
    conda_path="$(conda config --show root_prefix | awk '{print $2};')"
    [ ! -z $conda_path ] || exit_with_error
    source "${conda_path}/etc/profile.d/conda.sh" || exit_with_error
    echo "OK"

    echo "Updating dependencies..."
    cd duckietown || exit_with_error
    conda activate duckietown || exit_with_error
    conda env update --file environment.yaml --prune || exit_with_error
    cd .. || exit_with_error
  fi
  echo "Doesn't exist! Assuming Duckievillage was installed another way. Not updating dependencies."
fi

printf "Updating assignments... "
cd assignments || exit_with_error
git pull || exit_with error
cd .. || exit_with_error

echo "---"
echo "All done."
