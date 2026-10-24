# robis_navigation

Contém as configurações e launches de SLAM, localização AMCL e navegação Nav2.

Entra nos perfis `navigation` do robô e `simulation`; a imagem básica não
inclui navegação. Os dois perfis usam NavFn e Regulated Pure Pursuit.

Os parâmetros ficam em `config/`; `maps/` contém mapas de exemplo. Mapas
gerados durante o uso ficam em `data/navigation/`, preservados pelo volume
de dados do container. Veja [como salvar e carregar mapas](maps/README.md).

| Launch | Responsabilidade |
| --- | --- |
| `slam.launch.py` | SLAM Toolbox para construir o mapa |
| `localization.launch.py` | Mapa salvo, AMCL e seu gerenciador de ciclo de vida |
| `navigation.launch.py` | Planejador, controlador, comportamentos e suavização de velocidades do Nav2 |

O launch de navegação precisa de localização e TFs fornecidas pelos outros
componentes. O bringup com `stack:=navigation` combina AMCL e Nav2;
`make sim-nav` seleciona esse modo na simulação. `make sim-slam` inicia SLAM.
No robô, use o bringup físico com `use_sim_time=false`, conforme seu guia.

A localização inicia mapa, AMCL e seu gerenciador de ciclo de vida diretamente.
O relógio é configurado em todos os nós e costmaps. Veja
[o ensaio da simulação](../../docs/navigation.md) e
[o perfil físico](../../docs/real-navigation.md).

O ensaio automatizado `make nav-check` pertence às ferramentas locais
`extras/`; seus mapas e relatórios ficam em `extras/data/navigation/`.
Para o clone público, veja o [README principal](../../README.md) e
[a entrega com simulação](../../docs/initial-release.md).
