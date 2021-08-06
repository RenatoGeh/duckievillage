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

printf "Updating Duckietown... "
cd duckietown || exit_with_error
git pull || exit_with_error
cd .. || exit_with_error

printf "Updating assignments... "
cd assignments || exit_with_error
git pull || exit_with error
cd .. || exit_with_error

echo "Updating Duckievillage... "
git pull || exit_with_error
popd || exit_with_error

echo "---"
echo "All done."
