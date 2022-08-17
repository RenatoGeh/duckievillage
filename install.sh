#!/dev/null
# This /dev/null is to force the user to run with *their* shell. Run as:
# $(echo "$SHELL") ./install.sh

# Uncomment to debug
# set -o xtrace

# TF = Terminal Formatting
TF_BOLD=$(tput bold)
TF_RED=$(tput setaf 1)
TF_GREEN=$(tput setaf 2)
TF_YELLOW=$(tput setaf 3)
TF_BLUE=$(tput setaf 4)
TF_RESET=$(tput sgr0)
TF_ERR="${TF_BOLD}${TF_RED}"
TF_WARN="${TF_BOLD}${TF_YELLOW}"
TF_STEP="${TF_BOLD}${TF_BLUE}"
TF_OK="${TF_BOLD}${TF_GREEN}"

PROTO_PREFIX_SSH="ssh://git@"
PROTO_PREFIX_HTTPS="https://"

#PROTO_PREFIX_GIT="${PROTO_PREFIX_SSH}"
# Uncomment to use HTTPS instead of SSH
PROTO_PREFIX_GIT="${PROTO_PREFIX_HTTPS}"

# Exits script on error and print offending line.
function exit_with_error {
  echo "${TF_ERR}Error at [${BASH_SOURCE[1]##*/}]->${FUNCNAME[1]}:${BASH_LINENO[0]}${TF_RESET}"
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
  echo "${TF_WARN}No rcfile given. Assuming ${_shell} as rcfile.${TF_RESET}"
else
  _shell="$1"
fi

printf "${TF_STEP}Check if Anaconda is installed...${TF_RESET} "
if ! command -v conda &> /dev/null; then
  echo "${TF_ERR}Anaconda not found! Install anaconda (or miniconda) first.${TF_RESET}"
  exit 1
fi
echo "${TF_OK}OK${TF_RESET}"

printf "${TF_STEP}Setting up conda.sh...${TF_RESET} "
conda_path="$(conda config --show root_prefix | awk '{print $2};')"
[ ! -z $conda_path ] || exit_with_error
source "${conda_path}/etc/profile.d/conda.sh" || exit_with_error
echo "${TF_OK}OK${TF_RESET}"

printf "${TF_STEP}Check if git is installed...${TF_RESET} "
if ! command -v git &> /dev/null; then
  echo "${TF_ERR}Git not found! Install git first.${TF_RESET}"
fi
echo "${TF_OK}OK${TF_RESET}"

echo "${TF_STEP}Installing...${TF_RESET}"
# First, set the current directory to Duckievillage's.
pushd . || exit_with_error
cd "$(dirname "$0")" || exit_with_error
# Next, clone the Duckietown fork. Skip if directory already exists.
printf "${TF_STEP}Check if Duckietown is cloned...${TF_RESET} "
if [ ! -d "duckietown" ]; then
  echo "${TF_WARN}not cloned.${TF_RESET}"
  git clone "https://github.com/RenatoGeh/gym-duckietown.git" duckietown || exit_with_error
else
  echo "${TF_OK}OK${TF_RESET}"
fi
# Create environment for Duckietown.
# echo "${TF_STEP}Removing old environment (if exists)...${TF_RESET}"
# conda env remove -n duckietown
echo "${TF_STEP}Creating environment...${TF_RESET}"
conda env create -f duckietown/environment.yaml --name duckietown || conda env update --prune -f duckietown/environment.yaml --name duckietown || exit_with_error
# Add Duckietown to PYTHONPATH.
echo "${TF_STEP}Adding path to ${_shell}...${TF_RESET}"
echo "export PYTHONPATH=\"\${PYTHONPATH}:$(pwd)/duckietown/src\"" >> ${_shell}
echo "${TF_STEP}Activating environment...${TF_RESET}"
conda activate duckietown || exit_with_error
# Manually add dependencies which are not in the Anaconda repositories.
echo "${TF_STEP}Installing pip dependencies...${TF_RESET}"
pip install -r duckietown/requirements.txt || exit_with_error

# Clone assignments.
printf "${TF_STEP}Check if assignments is cloned...${TF_RESET} "
if [ ! -d "assignments" ]; then
  echo "${TF_WARN}not cloned.${TF_RESET}"
  until git clone "${PROTO_PREFIX_GIT}gitlab.uspdigital.usp.br/mac0318-2021/assignments.git" assignments; do
    echo "${TF_ERR}Something went wrong. Try again.${TF_RESET}"
    echo "${TF_ERR}  (Exit at any time with Ctrl+C. If you do, we recommend running uninstall.sh before retrying.)${TF_RESET}"
  done
else
  echo "${TF_OK}OK${TF_RESET}"
fi

# Run test.
echo ""
echo "${TF_STEP}Do you want to run a test to see if everything is working?${TF_RESET}"
select opt in "Yes" "No"; do
  case $opt in
    Yes ) echo "Reloading ${_shell}"; source ${_shell} && conda activate duckietown && python3 assignments/manual/manual.py; break;;
    No ) break;;
  esac
done

echo "${TF_STEP}---${TF_RESET}"
echo "${TF_OK}To run Duckievillage, you'll first need to run the following command:"
echo "       conda activate duckietown"
echo "This has to be done for every shell session before using Duckievillage.${TF_RESET}"

# Revert working directory changes.
popd || exit_with_error
echo "${TF_STEP}---${TF_RESET}"
echo "${TF_OK}All done.${TF_RESET}"
