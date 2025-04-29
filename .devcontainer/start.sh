#!/bin/bash

echo "install sdkman"
curl -s "https://get.sdkman.io" | bash
chmod +rwx /root/.sdkman/bin/sdkman-init.sh
/root/.sdkman/bin/sdkman-init.sh

chmod +rwx second.sh

./.devcontainer/second.sh