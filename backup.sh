#!/bin/bash

#BACKUP_DEST="${HOME}/data/tmp/backup"
#TARGET_DIRS="${HOME}/data/tmp/scripts ${HOME}/data/tmp/tex"

BACKUP_DEST="${HOME}/backups"
TARGET_DIRS="${HOME}/catkin_ws/src/beginner_tutorials/scripts ${HOME}/exp3_ws/src/exp3/map ${HOME}/Documents ${HOME}/Downloads ${HOME}/Desktop"
TEMPLATE_DESKTOP_DIR="${HOME}/phyexp3_setup/Desktop"

#ECHO="echo"
ECHO=""

${ECHO} sudo chmod 777 ${BACKUP_DEST}

${ECHO} mkdir -p ${BACKUP_DEST}
TIMESTAMP=`date +"%Y%m%d-%H%M%S"`;
CURRENT_DEST_BASE=${BACKUP_DEST}/${TIMESTAMP};
${ECHO} mkdir -p ${CURRENT_DEST_BASE}

for d in ${TARGET_DIRS}; do
  BASEDIR=${d%/*};
  DIRNAME=${d##*/};
  DIR_WITHOUT_ROOT=${d#/};
  DIR_DOT_NAMED=${DIR_WITHOUT_ROOT//\//.};
  DEST_DIR=${CURRENT_DEST_BASE}/${DIR_DOT_NAMED};
  echo "backup [${d}] into [${DEST_DIR}]";
  ${ECHO} mv ${d} ${DEST_DIR};
  ${ECHO} mkdir -p ${d};
#  echo rsync -a ${d}.orig/ ${d};
done

${ECHO} cp -r ${TEMPLATE_DESKTOP_DIR}/* ${HOME}/Desktop

${ECHO} sudo chmod 700 ${BACKUP_DEST}
