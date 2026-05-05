#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../searchclient_java"
javac searchclient/*.java
