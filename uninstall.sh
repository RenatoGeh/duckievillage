# Exits script on error and print offending line.
function exit_with_error {
  echo "Error at [${BASH_SOURCE[1]##*/}]->${FUNCNAME[1]}:${BASH_LINENO[0]}"
  exit 1
}

# Print help.
if [[ "$*" == *--help* ]] || [[ "$*" == *-h* ]]; then
  echo "Usage: $0 [-h|--help]"
  echo "  Flags:"
  echo "    -h | --help  - Prints this help message."
  exit 0
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

echo "Uninstalling..."
pushd . || exit_with_error
cd "$(dirname "$0")" || exit_with_error
if [ -d "duckietown" ]; then
  echo "Delete the Duckietown repository?"
  select opt in "Yes" "No"; do
    case $opt in
      Yes ) rm -rf ./duckietown; break;;
      No ) break;;
    esac
  done
fi

echo "Delete the Anaconda Duckietown environment?"
select opt in "Yes" "No"; do
  case $opt in
    Yes ) conda remove --name duckietown --all; break;;
    No ) break;;
  esac
done

echo "Clean up unused Anaconda packages?"
select opt in "Yes" "No"; do
  case $opt in
    Yes ) conda clean -a -t -p; break;;
    No ) break;;
  esac
done

echo "Done!"
echo "You might want to do some cleaning up on the following:"
echo "  1. Remove the PYTHONPATH line from your rcfile;"
echo "  2. Delete this script together with Duckievillage;"
echo "  3. Completely delete unused Anaconda environment and package files."
echo "Bye!"
