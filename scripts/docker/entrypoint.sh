#!/bin/sh
set -e

# Important: ensure /etc/machine-id isn't empty OR set TR_INSTALL_HASH at docker run time.
# (see Run section below)

if [ ! -e "$HOME/.transitive/.installation_complete" ]; then
  cp -r /transitive-preinstalled/. "$HOME/.transitive"
  rm -rf /transitive-preinstalled
fi

cd "$HOME/.transitive"
bash start_agent.sh
