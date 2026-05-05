#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -lt 2 ]; then
  echo "Usage: $0 /path/to/server.jar /path/to/level.lvl"
  exit 1
fi
SERVER="$1"
LEVEL="$2"
cd "$(dirname "$0")/../searchclient_java"
javac searchclient/*.java
java -jar "$SERVER" -c "java -Xmx8g searchclient.SearchClient -project" -l "$LEVEL" -t 180 -g
