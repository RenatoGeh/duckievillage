# Exits script on error and print offending line.
function exit_with_error {
  echo "Error at [${BASH_SOURCE[1]##*/}]->${FUNCNAME[1]}:${BASH_LINENO[0]}"
  exit 1
}

# Get current shell.
function get_shell_rc {
  echo "${HOME}/.$(basename $(readlink /proc/$$/exe))rc"
}

# Print help.
if [[ "$*" == *--help* ]] || [[ "$*" == *-h* ]]; then
  echo "Usage: $0 [-h|--help] rcfile"
  echo "  Flags:"
  echo "    -h | --help  - Prints this help message."
  echo "  Positional arguments:"
  echo "    rcfile       - Path to your rc file. If none is given, assume $(get_shell_rc)"
  exit 0
fi

# Set shell rc path.
if [ -z "$1" ]; then
  _shell="$(get_shell_rc)"
  echo "No rcfile given. Assuming ${_shell} as rcfile."
else
  _shell="$1"
fi

printf "Check if Anaconda is installed... "
if ! command -v conda &> /dev/null; then
  echo "Anaconda not found! Install anaconda (or miniconda) first."
  exit 1
fi
echo "OK"

printf "Setting up conda.sh... "
conda_path="$(conda config --show root_prefix | awk '{print $2};')"
[ ! -z $conda_path ] || exit_with_error
source "${conda_path}/etc/profile.d/conda.sh" || exit_with_error
echo "OK"

printf "Check if git is installed... "
if ! command -v git &> /dev/null; then
  echo "Git not found! Install git first."
fi
echo "OK"

echo "Installing..."
# First, set the current directory to Duckievillage's.
pushd . || exit_with_error
cd "$(dirname "$0")" || exit_with_error
# Next, clone the Duckietown fork. Skip if directory already exists.
printf "Check if Duckietown is cloned... "
if [ ! -d "duckietown" ]; then
  echo "not cloned."
  git clone "https://github.com/RenatoGeh/gym-duckietown.git" duckietown || exit_with_error
else
  echo "OK"
fi
# Create environment for Duckietown.
echo "Creating environment..."
conda env create -f duckietown/environment.yaml --name duckietown || exit_with_error
# Add Duckietown to PYTHONPATH.
echo "Adding path to ${_shell}..."
echo "export PYTHONPATH=\"\${PYTHONPATH}:$(pwd)/duckietown/src\"" >> ${_shell}
echo "Activating environment..."
conda activate duckietown || exit_with_error
# Manually add dependencies which are not in the Anaconda repositories.
echo "Installing pip dependencies..."
pip install -r duckietown/requirements.txt || exit_with_error

# Clone assignments.
printf "Check if assignments is cloned... "
if [ ! -d "assignments" ]; then
  echo "not cloned."
  git clone "https://github.com/RenatoGeh/duckievillage-assignments.git" assignments || exit_with_error
else
  echo "OK"
fi

# Run test.
echo ""
echo "Do you want to run a test to see if everything is working?"
select opt in "Yes" "No"; do
  case $opt in
    Yes ) source ${_shell} && python3 assignments/manual/manual.py; break;;
    No ) break;;
  esac
done

echo "---"
echo "To run Duckievillage, you'll first need to run the following command:"
echo "       conda activate duckietown"
echo "This has to be done for every shell session before using Duckievillage."

# Revert working directory changes.
popd || exit_with_error
echo "---"
echo "All done."


