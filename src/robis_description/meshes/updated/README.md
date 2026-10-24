# Visual atualizado do robIS

`robis.glb` é a montagem completa fornecida em outubro/2026.
`body.dae` contém as 110 peças estáticas, em metros e Z-up no referencial
`base_link`, com materiais incorporados. Rodas motrizes e casters mantêm os
visuais articulados do Xacro; o CAD estático não é usado como colisão.
O vidro usa a representação de transparência disponível no formato COLLADA;
ela não reproduz toda a refração PBR do GLB/Blender.

Os índices de vértices, normais e UVs são intercalados com offsets distintos.
Isso evita que o carregador COLLADA do Gazebo trunque as malhas exportadas
pelo Assimp quando vários atributos compartilham o mesmo offset.

Fontes, Blender, peças individuais e manifestos originais estão em
`extras/assets/updated-2026-10-18/`. A revisão anterior fica em
`extras/assets/previous-2026-10-18/`. A conversão é uma ferramenta local de
desenvolvimento em `extras/`; os arquivos já exportados acompanham o clone
público. Para converter novamente, após arquivar os recursos que serão substituídos:

```bash
python3 extras/scripts/import_robis_visual.py \
  extras/assets/updated-2026-10-18/robis.glb --replace
```

A conversão não altera articulações, colisões, massas, inércias ou TF dos
sensores. Dimensões inferidas do modelo visual não são novas calibrações.
No Gazebo, a origem óptica é configurada em `sensors.camera.optical_origin_*`,
à frente da lente visual; esses valores não substituem as extrínsecas físicas.
