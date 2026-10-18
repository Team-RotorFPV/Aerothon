#!/usr/bin/env bash
# Install the early UART router and the web host with automatic sensor connection.
set -euo pipefail
test "$(id -u)" = 0
workspace=/home/pi5/aerothon_ws
deploy=/home/pi5/aerothon-deploy
install -d -o pi5 -g pi5 "$deploy" /home/pi5/.aerothon /home/pi5/.local/state/aerothon /home/pi5/.local/state/aerothon/roslogs
for file in run_container.sh run_router.sh console.launch.py Dockerfile Dockerfile.dependencies profile_router.py stop_runtime.py; do
    install -o pi5 -g pi5 -m 0644 "$workspace/scripts/pi_runtime/$file" "$deploy/$file"
done
chmod 0755 "$deploy/run_container.sh" "$deploy/run_router.sh"
[[ -f "$deploy/runtime.env" ]] || install -o pi5 -g pi5 -m 0644 "$workspace/scripts/pi_runtime/runtime.env" "$deploy/runtime.env"
cat >/etc/systemd/system/aerothon-gcs.service <<'EOF'
[Unit]
Description=AeroTHON Pi webpage and managed production runtime
Requires=docker.service
After=docker.service network.target aerothon-router.service
Conflicts=aerothon-console.service aerothon-stack.service
[Service]
ExecStart=/bin/bash /home/pi5/aerothon-deploy/run_container.sh web
ExecStop=/bin/bash -c '/usr/bin/python3 /home/pi5/aerothon-deploy/stop_runtime.py && /usr/bin/docker stop --time 30 aerothon-runtime'
Restart=on-failure
RestartSec=5
KillMode=process
TimeoutStopSec=65
[Install]
WantedBy=multi-user.target
EOF
cat >/etc/systemd/system/aerothon-router.service <<'EOF'
[Unit]
Description=AeroTHON single FC UART owner
DefaultDependencies=no
After=local-fs.target systemd-udev-trigger.service
Before=network-pre.target network.target docker.service aerothon-gcs.service shutdown.target
Conflicts=shutdown.target
[Service]
User=pi5
EnvironmentFile=/home/pi5/aerothon-deploy/runtime.env
ExecStart=/bin/bash /home/pi5/aerothon-deploy/run_router.sh
Restart=on-failure
RestartSec=1
NoNewPrivileges=true
[Install]
WantedBy=multi-user.target
EOF
cat >/etc/systemd/system/aerothon-console.service <<'EOF'
[Unit]
Description=AeroTHON ground console, without mission or actuator nodes
Requires=docker.service
After=docker.service network.target aerothon-router.service
Conflicts=aerothon-stack.service
[Service]
ExecStart=/bin/bash /home/pi5/aerothon-deploy/run_container.sh console
ExecStop=-/usr/bin/docker stop --time 30 aerothon-console
Restart=on-failure
RestartSec=5
TimeoutStopSec=45
KillSignal=SIGINT
[Install]
WantedBy=multi-user.target
EOF
cat >/etc/systemd/system/aerothon-stack.service <<'EOF'
[Unit]
Description=AeroTHON hardware mission stack, manual integration start
Requires=docker.service
After=docker.service network.target aerothon-router.service
Conflicts=aerothon-console.service
[Service]
ExecStart=/bin/bash /home/pi5/aerothon-deploy/run_container.sh mission
ExecStop=-/usr/bin/docker stop --time 30 aerothon-mission
TimeoutStopSec=45
KillSignal=SIGINT
EOF
systemctl daemon-reload
systemctl disable aerothon-stack.service 2>/dev/null || true
systemctl disable aerothon-console.service 2>/dev/null || true
systemctl enable aerothon-gcs.service aerothon-router.service
systemctl start aerothon-router.service aerothon-gcs.service
