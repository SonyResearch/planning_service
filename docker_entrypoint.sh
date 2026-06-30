#!/bin/bash
set -e
# set permissions of built image and run as non-root
# If we are running as root (default) then is a user_uid specified move
# user to that uid and set permissions.
# This lets us use mounted volumes as the specified uid to match the
# running host containers permissions.
USER_NAME=planning
if [ "$2" = '/usr/share/planning_service/docker_start.sh' -a "$(id -u)" = '0' ]; then
  # Now check user_uid is set and error out if not
  if [ -n "${USER_UID}" ]; then
    echo "Setting up permissions...."
    usermod -u ${USER_UID} ${USER_NAME}
    groupmod -g ${USER_UID} ${USER_NAME}
    chown -R ${USER_NAME}:${USER_NAME} /root /data /logs
  else
    echo "Could not up permissions, run export=USER_UID=\$UID to set the env variable?"
    exit 1
  fi
  echo "Done setting up permissions"
  echo "exec gosu ${USER_NAME} $@"
  exec gosu ${USER_NAME} "$@"
  exit 0
fi
# Allow to run as other user
exec "$@"
