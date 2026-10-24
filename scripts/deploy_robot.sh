#!/usr/bin/env bash
# Transfer a built image. This script does not start the hardware.
set -euo pipefail
target="${1:-jetson@10.10.1.220}"
image="${2:-robis:jazzy-basic}"
profile="${3:-basic}"
case "$profile" in basic|navigation) ;; *) echo 'Perfil deve ser basic ou navigation.' >&2; exit 2 ;; esac
if [[ ! "$target" =~ ^([a-zA-Z0-9_][a-zA-Z0-9_.-]*@)?[a-zA-Z0-9_][a-zA-Z0-9_.-]*$ ]]; then
    echo 'Informe alias SSH, usuario@hostname ou usuario@IPv4, sem opções adicionais.' >&2
    exit 2
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
docker image inspect "$image" >/dev/null
architecture="$(docker image inspect --format '{{.Architecture}}' "$image")"
if [[ "$architecture" != arm64 ]]; then
    echo "A imagem é $architecture; a Jetson Nano requer arm64." >&2
    exit 2
fi
# Docker permissions on the Nano must already be configured for this SSH user.
ssh "$target" 'docker info >/dev/null && mkdir -p "$HOME/robis"'
docker save "$image" | gzip | ssh "$target" 'gzip -d | docker load'
scp "$script_dir/run_robot.sh" "$target:robis/run_robot.sh"
ssh "$target" 'chmod +x "$HOME/robis/run_robot.sh"'
printf 'Imagem carregada. Para iniciar depois da validação: ssh %s "ROBIS_ROBOT_PROFILE=%s ROBIS_ROBOT_IMAGE=%s ~/robis/run_robot.sh"\n' "$target" "$profile" "$image"
