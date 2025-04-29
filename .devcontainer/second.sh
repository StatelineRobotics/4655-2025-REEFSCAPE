#!/bin/bash
source "$HOME/.sdkman/bin/sdkman-init.sh"

echo "install java version 21"
sdk install java 21.0.2-open

echo "install gradle version 8.11"
sdk install gradle 8.11

chmod +rwx gradlew